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

Provides the @benchmark decorator, which drives the execution loopover all
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
import sys

import click

from lambkin.core.ctx.context import Context
from lambkin.core.ctx.source import Source
from lambkin.core.decorators.input import InputRegistry
from lambkin.sdk_options import SDK_OPTIONS


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

    Parameters
    ----------
    variants : iterable of dict
        Sequence of variant dicts to sweep over. Each dict is exposed as
        attributes on ctx.variant.
    num_iterations : int
        Number of times to repeat each variant. Controls the iter_<N> subfolders
        under each variant directory.

    Raises:
    ------
    ValueError
        If variants is empty
    """
    if not variants:
        raise ValueError(
            "You have provided an empty variants list; therefore, no "
            "benchmarking iterations will be executed."
        )

    def decorator(fn):
        inputs = InputRegistry()

        @functools.wraps(fn)
        def wrapper(args=None, output_dir=None):
            cli_args = sys.argv[1:] if args is None else args
            options = _parse_options(fn, cli_args)
            source = Source(path=inspect.getfile(fn))
            # The base context creates a directory for variant 1 / iteration 1
            # containing a metadata file with the information available at this
            # point in time.
            # This directory will later be overwritten with the
            # actual data collected for variant 1 / iteration 1 during
            # execution.
            base_ctx = Context(
                variant={},
                iteration=0,
                options=options,
                source=source,
                variant_index=0,
                output_dir=output_dir,
            )
            # TODO(teresa-ortega): Consider an alternative approach for managing
            # the base context.
            resolved_inputs = inputs.resolve(base_ctx)
            for variant_index, variant in enumerate(variants):
                for iteration in range(num_iterations):
                    ctx = Context(
                        variant=variant,
                        iteration=iteration,
                        options=options,
                        source=source,
                        inputs=resolved_inputs,
                        variant_index=variant_index,
                        output_dir=output_dir,
                    )
                    fn(ctx)

        wrapper.input = inputs.register
        return wrapper

    return decorator
