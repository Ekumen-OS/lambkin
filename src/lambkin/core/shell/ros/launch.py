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
"""ROS 2 command proxy specialisation.

Provides a specialised command proxy for ros2 launch that injects
ROS_LOG_DIR into the child process environment before execution,
ensuring that ROS node logs are written to the iteration directory
alongside all other benchmark artefacts.
"""

from __future__ import annotations

import os
import subprocess

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

    def _make_popen(self, argv, stdout, stderr) -> subprocess.Popen:
        """Launch the process with ROS_LOG_DIR set to the iteration directory.

        Args:
            argv: The command to run.
            stdout: stdout stream configuration.
            stderr: stderr stream configuration.

        Returns:
            The running process.
        """
        return subprocess.Popen(
            argv,
            cwd=self._cwd,
            stdout=stdout,
            stderr=stderr,
            env=self._make_env(),
        )

    def build_env(self) -> dict:
        """Build the environment for the child process.

        Returns a copy of the current environment with ROS_LOG_DIR set to
        the iteration directory, so that ROS node logs land alongside all
        other benchmark artefacts rather than in the default ~/.ros/log.

        Returns:
            A copy of the current environment with ROS_LOG_DIR set.
        """
        return self._make_env()


CommandProxy.register(("ros2", "launch"), RosLaunchCommand)
