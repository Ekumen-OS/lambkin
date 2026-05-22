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

"""Base exceptions for the lambkin SDK.

All errors raised by lambkin inherit from LambkinError, allowing callers to
catch SDK-specific failures with a single except clause.
"""


class LambkinError(Exception):
    """Base class for all lambkin SDK errors."""


class LambkinSystemdNotFoundError(LambkinError):
    """Raised when systemd-run is not found on the system."""


class LambkinSystemdScopeTimeoutError(LambkinError):
    """Raised when a systemd scope fails to stop within the timeout period."""


class LambkinProcessDiedUnexpectedlyError(LambkinError):
    """Raised when a background process exits before the context manager does.

    Attributes:
    ----------
    argv : list[str]
        The command that died.
    returncode : int
        The exit code of the process.
    """

    def __init__(self, argv: list[str], returncode: int) -> None:
        """Initialize with the command and its exit code.

        Parameters
        ----------
        argv : list[str]
            The command that died.
        returncode : int
            The exit code of the process.
        """
        self.argv = argv
        self.returncode = returncode
        super().__init__(
            f"Background process {argv[0]!r} died unexpectedly "
            f"with return code {returncode}."
        )
