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

"""Input decorator for lambkin.

Provides the class "InputRegistry" class, which manages the registration and
resolution of input hooks for a benchmark function. Hooks are registered via the
"InputRegistry.register" method and resolved before the benchmark function runs,
injecting their return values into "ctx.inputs" under the hook's function name.
"""


class InputRegistry:
    """Manages the registration and resolution of input hooks for a benchmark."""

    def __init__(self):
        """Initialize the InputRegistry."""
        self._hooks = []

    def register(self, hook_fn):
        """Decorator used to register a function as an input provider."""
        self._hooks.append(hook_fn)
        return hook_fn

    def resolve(self, ctx):
        """Resolve all registered input hooks.

        Executes all registered hooks and dynamically maps their return values
        to the "ctx.inputs" namespace using the original function's name.
        """
        for hook in self._hooks:
            result = hook(ctx)
            setattr(ctx.inputs, hook.__name__, result)
