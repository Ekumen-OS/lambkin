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

import inspect


def _validate_result(hook_fn, result) -> None:
    """Validate the return value of a hook function."""
    if result is None:
        raise ValueError(
            f"Hook '{hook_fn.__name__}' returned None or did not return a value."
        )
    if isinstance(result, str) and not result.strip():
        raise ValueError(
            f"Hook '{hook_fn.__name__}' returned an empty or blank string."
        )


def _validate_hook_signature(hook_fn) -> None:
    """Validate that the hook function accepts a single 'ctx' parameter."""
    params = list(inspect.signature(hook_fn).parameters.keys())

    if len(params) != 1:
        raise ValueError(f"Hook '{hook_fn.__name__}' must have exactly 1 parameter, ")


class InputRegistry:
    """Manages the registration and resolution of input hooks for a benchmark."""

    def __init__(self):
        """Initialize the InputRegistry."""
        self._hooks = []

    def register(self, hook_fn):
        """Decorator used to register a function as an input provider."""
        existing_names = [h.__name__ for h in self._hooks]
        if hook_fn.__name__ in existing_names:
            raise ValueError(
                f"Hook name conflict: '{hook_fn.__name__}' is already registered."
            )
        self._hooks.append(hook_fn)
        return hook_fn

    def resolve(self, ctx):
        """Resolve all registered input hooks."""
        for hook in self._hooks:
            _validate_hook_signature(hook)
            result = hook(ctx)
            _validate_result(hook, result)
            setattr(ctx.inputs, hook.__name__, result)
