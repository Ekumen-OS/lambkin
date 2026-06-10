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

"""Unit tests for the input decorator in lambkin.core.decorators."""

import pytest

from lambkin.core.decorators.output import OutputRegistry


def test_register_returns_original_function():
    """register() returns the original function unchanged."""
    registry = OutputRegistry()

    def plots(ctx):
        pass

    assert registry.register(plots) is plots


def test_run_calls_hook_with_ctx():
    """run() calls the registered hook with the given ctx."""
    registry = OutputRegistry()
    seen = []

    def plots(ctx):
        seen.append(ctx)

    registry.register(plots)
    ctx = object()
    registry.run(ctx)
    assert seen == [ctx]


def test_run_calls_multiple_hooks_in_order():
    """run() calls all registered hooks in registration order."""
    registry = OutputRegistry()
    seen = []

    def first(ctx):
        seen.append("first")

    def second(ctx):
        seen.append("second")

    registry.register(first)
    registry.register(second)
    registry.run(object())
    assert seen == ["first", "second"]


def test_run_with_no_hooks_does_not_raise():
    """run() with no registered hooks completes without error."""
    registry = OutputRegistry()
    registry.run(object())


def test_register_hook_with_no_parameters_raises():
    """A hook with no parameters raises ValueError at register time."""
    registry = OutputRegistry()

    def bad_hook():
        pass

    with pytest.raises(ValueError, match="exactly 1 parameter"):
        registry.register(bad_hook)


def test_register_hook_with_extra_parameters_raises():
    """A hook with more than 1 parameter raises ValueError at register time."""
    registry = OutputRegistry()

    def bad_hook(ctx, extra):
        pass

    with pytest.raises(ValueError, match="exactly 1 parameter"):
        registry.register(bad_hook)


def test_register_duplicate_name_raises():
    """Registering two hooks with the same name raises ValueError."""
    registry = OutputRegistry()

    def plots(ctx):
        pass

    registry.register(plots)

    def plots(ctx):  # noqa: F811
        pass

    with pytest.raises(ValueError, match="already registered"):
        registry.register(plots)
