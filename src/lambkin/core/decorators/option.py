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

Provides the option decorator. Instead of parsing arguments immediately, it
stores option definitions on the function so that later can parse and inject
them into "ctx.options".

The @benchmark orchestrator later collects these definitions to construct a
unified CLI parser and inject the resulting values into ctx.options.
"""


def option(*param_decls, **attrs):
    """Registers a CLI option on the benchmark function.

    Example:
        @lambkin.option("--clock-rate", default=100.0)
        @lambkin.option("--sensor-topic", default="/scan")
    """

    def decorator(fn):
        if not hasattr(fn, "_options"):
            fn._options = []
        fn._options.append((param_decls, attrs))
        return fn

    return decorator
