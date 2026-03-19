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

"""CLI option registration for benchmark.

Provides the @option decorator, modeled directly after click.option. Instead of
parsing arguments immediately, it creates a class click.Option object and stores
it on "fn.__lambkin_options__" so that @benchmark can collect and parse them later and
inject the resulting values into "ctx.options".

Flag names are normalized by click: "--clock-rate" becomes "clock_rate".
"""

from click import Option


def option(*param_decls, **attrs):
    """Registers a CLI option on the benchmark function.

    Example:
        @lambkin.option("--clock-rate", default=100.0)
        @lambkin.option("--sensor-topic", default="/scan")
    """
    for decl in param_decls:
        if not decl.startswith("-"):
            raise ValueError(
                f"Invalid flag name: {decl!r}. Must start with '-' or '--'."
            )

    def decorator(fn):
        if not hasattr(fn, "__lambkin_options__"):
            fn.__lambkin_options__ = []
        fn.__lambkin_options__.append(Option(param_decls, **attrs))
        return fn

    return decorator
