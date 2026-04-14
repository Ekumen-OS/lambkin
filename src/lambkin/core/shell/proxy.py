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

"""Shell proxy for lambkin benchmark execution.

Provides an abstraction over shell command dispatch, allowing benchmark
processes to be launched and managed through a consistent interface.
"""

from __future__ import annotations

import shlex
from typing import Any


class _CommandProxy:
    """Builds a shell command lazily by chaining attribute access and calls."""

    def __init__(self, parts: list[str]) -> None:
        """Initialize the proxy with the command words accumulated so far."""
        self._parts = parts

    def __getattr__(self, name: str) -> _CommandProxy:
        """Append a new word to the command and return a new proxy."""
        return _CommandProxy(self._parts + [name])

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        """Finalize and print the command.

        Positional args are appended as quoted tokens to handle paths with
        spaces correctly. Keyword args are converted to --flag value pairs,
        with underscores replaced by hyphens. Boolean True values produce a
        standalone flag, False values are ignored.

        This is a dry-run implementation that will be extended to dispatch
        commands to BackgroundProcess for real execution.
        """
        extra = []

        for arg in args:
            extra.append(shlex.quote(str(arg)))

        for key, value in kwargs.items():
            flag = "--" + key.replace("_", "-")
            if value is True:
                extra.append(flag)
            elif value is not False:
                extra.extend([flag, shlex.quote(str(value))])

        command = " ".join(self._parts + extra)
        print(f"[CMD]: {command}")


class ShellProxy:
    """Dry-run mock shell that prints commands instead of executing them."""

    def __getattr__(self, name: str) -> _CommandProxy:
        """Start building a new command from the given top-level tool name."""
        return _CommandProxy([name])
