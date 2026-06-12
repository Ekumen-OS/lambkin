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

Provides the ``InputRegistry`` class, which manages the registration and
resolution of scoped input hooks for a benchmark function.

Hooks are registered via ``InputRegistry.register`` and resolved against
the appropriate context by calling ``registry.resolve(ctx)``. The registry
dispatches on ``ctx.scope`` to determine which hooks to run. Merging with
parent scope inputs is delegated to the context's ``inputs`` setter.

Hooks can be scoped to one of three lifecycle levels:

- ``"benchmark"`` (default): resolved once before the variant loop, against
  a ``BenchmarkContext``. Use for inputs that do not depend on the current
  variant or iteration (e.g. downloading a shared dataset).
- ``"variant"``: resolved once per variant, against a ``VariantContext``.
  Use for inputs that depend on ``ctx.variant`` but not ``ctx.iteration``
  (e.g. selecting a dataset file by sensor model).
- ``"iteration"``: resolved once per iteration on a cache miss, against an
  ``IterationContext``. Use for inputs that depend on both ``ctx.variant``
  and ``ctx.iteration`` (e.g. computing a per-iteration random seed).

Hook names must be unique across all scopes. Registering two hooks with
the same name — regardless of scope — raises a ``ValueError`` at decoration
time.
"""

import inspect
from types import SimpleNamespace

_VALID_SCOPES = frozenset({"benchmark", "variant", "iteration"})


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
    """Validate that the hook function accepts a single parameter."""
    params = list(inspect.signature(hook_fn).parameters.keys())
    if len(params) != 1:
        raise ValueError(f"Hook '{hook_fn.__name__}' must have exactly 1 parameter.")


class InputRegistry:
    """Manages the registration and resolution of scoped input hooks.

    Hooks are bucketed by scope (``benchmark``, ``variant``, ``iteration``)
    and resolved by calling ``registry.resolve(ctx)``, which dispatches on
    ``ctx.scope``. Merging with parent scope inputs is handled by the
    context's ``inputs`` setter.

    All hook names must be unique across all scopes.
    """

    def __init__(self) -> None:
        """Initialize the InputRegistry."""
        self._hooks: dict[str, list] = {
            "benchmark": [],
            "variant": [],
            "iteration": [],
        }

    def _all_names(self) -> set[str]:
        """Return all registered hook names across all scopes."""
        return {h.__name__ for hooks in self._hooks.values() for h in hooks}

    def _do_register(self, fn, scope: str):
        """Register a single hook function under the given scope.

        Args:
            fn: The hook function to register.
            scope: The scope to register the hook under.

        Returns:
            The hook function unchanged.

        Raises:
            ValueError: If the hook signature is invalid or its name conflicts
                with an already-registered hook in any scope.
        """
        _validate_hook_signature(fn)
        if fn.__name__ in self._all_names():
            raise ValueError(
                f"Hook name conflict: '{fn.__name__}' is already registered "
                f"in another scope. Hook names must be unique across all scopes."
            )
        self._hooks[scope].append(fn)
        return fn

    def register(self, hook_fn=None, *, scope: str = "benchmark"):
        """Register a function as a scoped input provider.

        Supports two calling conventions::

            @nominal.input
            def dataset(ctx): ...                      # benchmark scope

            @nominal.input(scope="variant")
            def dataset(ctx): ...                      # variant scope

        Args:
            hook_fn: The function to register. When the decorator is used
                without arguments, this is the decorated function. When called
                with arguments (e.g. ``scope="variant"``), this is ``None``
                and a decorator is returned instead.
            scope: Lifecycle scope for this hook. One of ``"benchmark"``,
                ``"variant"``, or ``"iteration"``. Defaults to ``"benchmark"``.

        Returns:
            The hook function unchanged (when used as a plain decorator), or
            a decorator (when called with arguments).

        Raises:
            ValueError: If ``scope`` is not valid, the hook signature is
                invalid, or the hook name is already registered in any scope.
        """
        if scope not in _VALID_SCOPES:
            raise ValueError(
                f"Invalid scope {scope!r}. Must be one of: "
                f"{', '.join(sorted(_VALID_SCOPES))}."
            )
        if hook_fn is not None:
            return self._do_register(hook_fn, scope)

        def decorator(fn):
            return self._do_register(fn, scope)

        return decorator

    def resolve(self, ctx) -> SimpleNamespace:
        """Resolve input hooks for the given context scope.

        Dispatches on ``ctx.scope`` to determine which hooks to run and
        returns the results as a ``SimpleNamespace``. Merging with parent
        scope inputs is delegated to the context's ``inputs`` setter.

        Args:
            ctx: A ``BenchmarkContext``, ``VariantContext``, or
                ``IterationContext`` instance.

        Returns:
            A ``SimpleNamespace`` containing the inputs resolved at this
            scope only. The context's ``inputs`` setter handles merging
            with parent inputs.

        Raises:
            ValueError: If ``ctx.scope`` is not a recognized scope.
        """
        scope = ctx.scope
        if scope not in _VALID_SCOPES:
            raise ValueError(
                f"Unknown context scope {scope!r}. Must be one of: "
                f"{', '.join(sorted(_VALID_SCOPES))}."
            )
        resolved = {}
        for hook in self._hooks[scope]:
            result = hook(ctx)
            _validate_result(hook, result)
            resolved[hook.__name__] = result
        return SimpleNamespace(**resolved)
