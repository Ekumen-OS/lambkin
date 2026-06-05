# Copyright 2026 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Benchmark loop decorator for lambkin.

Provides the @benchmark decorator, which drives the execution loop over all
variants and iterations. It collects CLI option definitions registered by
@option, parses them once before the loop using an internal click parser, and
injects the resulting values into a Context class on each (variant, iteration)
pair.

Input hooks registered via "@nominal.input" are managed by "InputRegistry"
instance and resolved before the benchmark function runs on each iteration,
injecting their return values into "ctx.inputs".

Raises ValueError if variants is empty.
"""

import functools
import inspect
import logging
import sys
import time

import click
import yaml
from click.formatting import HelpFormatter

from lambkin.common import defaults
from lambkin.core.ctx.context import Context
from lambkin.core.ctx.source import Source
from lambkin.core.decorators.input import InputRegistry
from lambkin.logger import configure_logging
from lambkin.sdk_options import SDK_OPTIONS

logger = logging.getLogger(__name__)


def _format_elapsed(seconds: float) -> str:
    """Format an elapsed time into a human-readable string.

    Scales the output unit to the duration so the result is always easy to
    read at a glance, from sub-second runs to multi-day sweeps:

    - Under 60 s:  ``'0.0023s'``
    - Under 1 h:   ``'45m 03s'``
    - Under 1 day: ``'2h 15m 07s'``
    - 1 day or more: ``'2d 03h 15m 07s'``

    Sub-second precision is kept only for runs under 60 seconds; longer
    durations are truncated to whole seconds.

    Args:
        seconds (float): Elapsed time in seconds, as returned by ``time.monotonic()``.

    Returns:
        A human-readable elapsed time string.
    """
    if seconds < 60:
        return f"{seconds:.4f}s"
    total = int(seconds)
    d, remainder = divmod(total, 86400)
    h, remainder = divmod(remainder, 3600)
    m, s = divmod(remainder, 60)
    if d > 0:
        return f"{d}d {h:02d}h {m:02d}m {s:02d}s"
    if h > 0:
        return f"{h}h {m:02d}m {s:02d}s"
    return f"{m}m {s:02d}s"


def _show_options(fn) -> None:
    """Print all options registered via @lambkin.option on fn."""
    user_options = getattr(fn, "__lambkin_options__", [])
    formatter = HelpFormatter()
    with formatter.section("SDK Options"):
        formatter.write_dl([(opt.opts[0], opt.help or "") for opt in SDK_OPTIONS])
    with formatter.section("Custom Options"):
        if not user_options:
            formatter.write_text("No options registered in this script.")
        else:
            formatter.write_dl([
                (
                    opt.opts[0],
                    (opt.help or "")
                    + (
                        f"  [default: {opt.default}]" if opt.default is not None else ""
                    ),
                )
                for opt in user_options
            ])
    click.echo(formatter.getvalue(), nl=False)


def _list_variants(variants) -> None:
    """Print all variants with their var_N number to stdout."""
    width = len(str(len(variants)))
    click.echo("Available variants:")
    for i, variant in enumerate(variants):
        prefix = f"  [{i + 1:{width}}]  "
        indent = " " * len(prefix)
        pairs = list(variant.items())
        click.echo(f"{prefix}{pairs[0][0]}={pairs[0][1]}")
        for k, v in pairs[1:]:
            click.echo(f"{indent}{k}={v}")
        click.echo()


def _parse_index_list(raw: str, label: str) -> set[int]:
    """Parse a comma-separated string of var_N folder numbers into a set.

    Tokens can be plain numbers or ranges in start:end format (inclusive on
    both ends), e.g. "3:5,42" expands to {3, 4, 5, 42}.

    Args:
        raw: Comma-separated numbers or ranges matching folder names, e.g.
            "3:5,42" to select var_3/, var_4/, var_5/, var_42/.
        label: Human-readable name used in error messages (e.g. "variant").

    Returns:
        The parsed numbers.

    Raises:
        click.BadParameter: If any token is not a positive integer or a valid
            start:end range.
    """
    result = set()
    for token in raw.split(","):
        token = token.strip()
        if ":" in token:
            parts = token.split(":")
            if len(parts) != 2 or not all(p.isdigit() for p in parts):
                raise click.BadParameter(
                    f"Invalid {label} range {token!r}: must be start:end "
                    f"with positive integers (e.g. 3:5).",
                    param_hint=f"--{label}s",
                )
            start, end = int(parts[0]), int(parts[1])
            if start < 1 or end < start:
                raise click.BadParameter(
                    f"Invalid {label} range {token!r}: start must be >= 1 "
                    f"and end must be >= start.",
                    param_hint=f"--{label}s",
                )
            result.update(range(start, end + 1))
        else:
            if not token.isdigit() or int(token) < 1:
                raise click.BadParameter(
                    f"Invalid {label} index {token!r}: must be a positive integer.",
                    param_hint=f"--{label}s",
                )
            result.add(int(token))
    return result


def _parse_options(fn, cli_args):
    """Parse CLI options from fn.__lambkin_options__ and return a dict."""
    user_options = getattr(fn, "__lambkin_options__", [])
    all_options = list(SDK_OPTIONS) + user_options
    if not all_options:
        return {}
    cmd = click.Command(name="benchmark", params=all_options)
    click_ctx = cmd.make_context("benchmark", list(cli_args))
    return click_ctx.params


def benchmark(variants, num_iterations):
    """Drive the benchmark execution loop over all variants and iterations.

    Parses CLI options registered by @lambkin.option once before the loop, then
    creates a Context for each (variant, iteration) pair and calls the decorated
    function with it. Input hooks registered via @nominal.input are resolved
    before each call, injecting their return values into ctx.inputs.

    Args:
        variants (list[dict]): Sequence of variant dicts to sweep over. Each dict
           is exposed as attributes on ``ctx.variant``.
        num_iterations (int): Number of times to repeat each variant. Controls the
            ``iter_<N>`` subfolders under each variant directory.

    Returns:
        A decorator that wraps the benchmark function with the execution loop.

    Raises:
        ValueError: If variants is empty
    """
    if not variants:
        raise ValueError(
            "You have provided an empty variants list; therefore, no "
            "benchmarking iterations will be executed."
        )

    def decorator(fn):
        inputs = InputRegistry()

        @functools.wraps(fn)
        def wrapper(args=None, base_dir=None):
            cli_args = sys.argv[1:] if args is None else args
            options = _parse_options(fn, cli_args)
            if options.get("show_options"):
                _show_options(fn)
                sys.exit(0)
            if options.get("list_variants"):
                _list_variants(variants)
                sys.exit(0)
            log_level = options.get("log_level", defaults.LOG_LEVEL)
            cli_variants = options.get("variants")

            # Configure the logging level for the SDK.
            configure_logging(log_level)

            # Resolve variant and iteration filters from CLI options.
            # Numbers match the var_N / iter_N folder names on disk.
            selected_variants = (
                _parse_index_list(cli_variants, "variant") if cli_variants else None
            )

            logger.info(
                "Starting benchmark: %d variant(s), %d iteration(s) each.",
                len(variants), num_iterations,
            )
            if selected_variants:
                selected_count = len(selected_variants)
                logger.info(
                    "Selected variants: %s (%d/%d).",
                    ", ".join(f"var_{i}" for i in sorted(selected_variants)),
                    selected_count,
                    len(variants),
                )
            else:
                selected_count = len(variants)
            total_runs = selected_count * num_iterations
            current_run = 0

            # Create Source object
            source = Source(path=inspect.getfile(fn))

            # Determine base_dir for all benchmark outputs.
            base_dir = (
                base_dir
                if base_dir
                else source.path.parent / defaults.BENCHMARKS_DIRNAME
            )

            # The base context is used to resolve inputs and write variants.yaml.
            # A side effect is that creates a directory for variant 1 / iteration 1
            # containing a metadata file with the information available at this
            # point in time. However, this directory will later be overwritten
            # with the actual data collected for variant 1 / iteration 1 during
            # execution.
            with Context(
                variant={},
                iteration=0,
                options=options,
                source=source,
                base_dir=base_dir,
                variant_index=0,
            ) as base_ctx:
                # TODO(teresa-ortega): Consider an alternative approach for managing
                # the base context.
                resolved_inputs = inputs.resolve(base_ctx)
                variants_map = {
                    f"var_{i + 1}": variant for i, variant in enumerate(variants)
                }
                variants_map_path = base_ctx.paths.base_dir / "variants.yaml"
                with open(variants_map_path, "w") as f:
                    yaml.dump(
                        variants_map, f, default_flow_style=False, sort_keys=False
                    )

            # Calculate start time
            start_time = time.monotonic()

            # Loop over variants and iterations, creating a new Context for each run.
            # variant_index is always the original 0-based position in the full
            # variants list so that output folder numbers (var_N) are stable
            # regardless of which subset is selected at the CLI.
            for variant_index, variant in enumerate(variants):
                if selected_variants and (variant_index + 1) not in selected_variants:
                    continue
                for iteration in range(num_iterations):
                    # Log the current run number and total runs to track progress.
                    current_run += 1
                    logger.info("Benchmark run %d/%d", current_run, total_runs)
                    with Context(
                        variant=variant,
                        iteration=iteration,
                        options=options,
                        source=source,
                        base_dir=base_dir,
                        inputs=resolved_inputs,
                        variant_index=variant_index,
                    ) as ctx:
                        fn(ctx)

            # Calculate total elapsed time
            logger.info(
                "Benchmark finished in %s.",
                _format_elapsed(time.monotonic() - start_time),
            )

        wrapper.input = inputs.register
        return wrapper

    return decorator
