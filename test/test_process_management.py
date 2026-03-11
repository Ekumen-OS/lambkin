#!/usr/bin/env python3

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

"""Unit tests for the process management functions in lambkin.py."""

import subprocess
import time
import unittest

from examples.lambkin_benchmarking import execute_background_process, wait_for_processes


class TestProcessManagement(unittest.TestCase):
    """Test suite for process management functions in the lambkin module."""

    def test_execute_background_process(self):
        """Tests if execute_background_process correctly starts a process.

        It verifies the process runs in the background without blocking
        the main thread.
        """
        process = execute_background_process(["sleep", "3"])

        self.assertIsNone(
            process.poll(), "The process should be running in background."
        )

        process.terminate()
        process.wait()

    def test_wait_for_processes(self):
        """Tests if wait_for_processes properly blocks until waitlist completes.

        It verifies that the function waits for the processes in the waitlist
        and subsequently terminates the processes in the termination_list.
        """
        p_play = subprocess.Popen(["sleep", "2"])
        p_infinite = subprocess.Popen(["sleep", "infinity"])
        start_time = time.time()
        wait_for_processes(waitlist=[p_play], termination_list=[p_infinite])
        end_time = time.time()
        total_time = end_time - start_time

        self.assertGreaterEqual(
            total_time, 2.0, "The function did not wait for the waitlist to finish."
        )
        self.assertLess(
            total_time, 5.0, "The function got stuck waiting for the infinite process."
        )
        self.assertIsNotNone(
            p_infinite.poll(),
            "The process in the termination_list was not closed properly.",
        )


if __name__ == "__main__":
    unittest.main()
