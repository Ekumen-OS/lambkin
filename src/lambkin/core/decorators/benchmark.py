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
injects the resulting values into a Context class on each (variation, iteration)
pair.

"""

import inspect
import sys

import click

from lambkin.core.ctx.context import Context
from lambkin.core.ctx.source import Source


def _parse_options(fn, cli_args):
    """Parse CLI options from fn.__lambkin_options__ and return a dict."""
    registered = getattr(fn, "__lambkin_options__", [])
    if not registered:
        return {}
    cmd = click.Command(name="benchmark", params=registered)
    click_ctx = cmd.make_context("benchmark", list(cli_args))
    return click_ctx.params


def benchmark(variants, num_iterations):
    """Drive the benchmark execution loop over all variants and iterations.

    Parses CLI options registered by @lambkin.option once before the loop,
    then creates a Context for each (variants, iteration) pair and calls
    the decorated function with it.

    Parameters
    ----------
    variants : iterable of dict
        Sequence of variation dicts to sweep over. Each dict is exposed
        as attributes on ctx.variation.
    num_iterations : int
        Number of times to repeat each variation. Controls the iter_<N>
        subfolders under each variation directory.
    """

    def decorator(fn):
        def wrapper(args=None):
            cli_args = sys.argv[1:] if args is None else args
            options = _parse_options(fn, cli_args)
            source = Source(path=inspect.getfile(fn))
            for variant_index, variant in enumerate(variants):
                for iteration in range(num_iterations):
                    ctx = Context(
                        variant=variant,
                        iteration=iteration,
                        output_dir=".",
                        # TODO(teresa-ortega): change the output_dir
                        # implementation.
                        options=options,
                        source=source,
                        variant_index=variant_index,
                    )
                    fn(ctx)

        return wrapper

    return decorator
