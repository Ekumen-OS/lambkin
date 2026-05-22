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
"""ROS 2 command proxy specialisation."""

from __future__ import annotations

import os

from lambkin.core.shell.proxy import CommandProxy


class RosLaunchCommand(CommandProxy):
    """Command proxy specialisation for ros2 launch.

    Injects ROS_LOG_DIR into the child process environment so that
    ROS node logs land in the iteration directory alongside all other
    benchmark artefacts.
    """

    def _make_env(self) -> dict:
        """Build the environment for the child process.

        Copies the current environment and sets ROS_LOG_DIR to the iteration
        directory so that ROS node logs are written alongside all other
        benchmark artefacts.

        Returns:
            A copy of the current environment with ROS_LOG_DIR set.
        """
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = str(self._cwd)
        return env

    def __call__(self, *args, **kwargs):
        """Dispatch the command with ROS_LOG_DIR injected into the environment.

        Args:
            *args: Positional arguments appended as tokens to the command.
            **kwargs: Keyword arguments converted to --flag value pairs.

        Returns:
            The CompletedProcess instance returned by the parent class.
        """
        kwargs["env"] = self._make_env()
        return super().__call__(*args, **kwargs)
