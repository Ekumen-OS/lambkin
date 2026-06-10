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

"""Output decorator for lambkin.

Provides the "OutputRegistry" class, which manages the registration and
execution of output hooks for a benchmark function. Hooks are registered via the
"OutputRegistry.register" method and executed once after the full benchmark
loop completes, receiving the context of the last iteration.
"""

import inspect


def _validate_hook_signature(hook_fn) -> None:
    """Validate that the hook function accepts a single 'ctx' parameter."""
    params = list(inspect.signature(hook_fn).parameters.keys())

    if len(params) != 1:
        raise ValueError(f"Hook '{hook_fn.__name__}' must have exactly 1 parameter.")


class OutputRegistry:
    """Manages the registration and execution of output hooks for a benchmark.

    Warning:
        Output hooks receive the context of the last benchmark iteration,
        but by the time they run, the iteration cgroup has already been torn
        down. Do not call ``ctx.shell`` or launch any processes inside an
        output hook — it will fail with a confusing error. Output hooks are
        intended for reading paths and artifacts from disk only, for example
        via ``ctx.paths.base_dir``.
    """

    def __init__(self):
        """Initialize the OutputRegistry."""
        self._hooks = []

    def register(self, hook_fn):
        """Decorator used to register a function as an output handler.

        Args:
            hook_fn: function to register as an output hook. Must accept
                a single ``ctx`` argument.

        Returns:
            The original function, unchanged.

        Raises:
            ValueError: if the hook signature is invalid or its name is already
                registered.
        """
        _validate_hook_signature(hook_fn)
        existing_names = [h.__name__ for h in self._hooks]
        if hook_fn.__name__ in existing_names:
            raise ValueError(
                f"Hook name conflict: '{hook_fn.__name__}' is already registered."
            )
        self._hooks.append(hook_fn)
        return hook_fn

    def run(self, ctx):
        """Run all registered output hooks with the last iteration context.

        Calls each registered hook in registration order, passing ``ctx`` as
        the sole argument. If no hooks are registered, this method does nothing.

        Args:
            ctx: Context of the last benchmark iteration, passed as-is to
                each hook.

        Raises:
            ValueError: if ctx is None.
        """
        if ctx is None:
            raise ValueError("Cannot run output hooks: context is None.")
        for hook in self._hooks:
            hook(ctx)
