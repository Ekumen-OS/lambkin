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

"""Input decorator for lambkin."""

import functools


def input(hook):
    """Wrap a benchmark function to inject an input into ``ctx.inputs``.

    Uses :func:`inspect.getfile` to read the hook name and injects
    the return value into ``ctx.inputs.<name>`` before the benchmark runs.

    Parameters
    ----------
    hook : callable
        Input resolver function. Must accept a single ``ctx`` argument
        and return the input value to inject into ``ctx.inputs``.
    """

    def decorator(benchmark_fn):
        @functools.wraps(benchmark_fn)
        def wrapper(ctx):
            setattr(ctx.inputs, hook.__name__, hook(ctx))
            return benchmark_fn(ctx)

        wrapper.__wrapped__ = benchmark_fn
        return wrapper

    return decorator
