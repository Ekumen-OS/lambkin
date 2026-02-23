#!/usr/bin/env python3
"""Unit tests for the process management functions in lambkin.py."""

import os
import subprocess
import sys
import time
import unittest

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from lambkin import execute_background_process, wait_for_processes


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
        p_record = subprocess.Popen(["sleep", "10"])

        start_time = time.time()
        wait_for_processes(waitlist=[p_play], termination_list=[p_record])
        end_time = time.time()

        total_time = end_time - start_time

        self.assertGreaterEqual(
            total_time, 2.0, "The function did not wait for the waitlist to finish."
        )
        self.assertLess(
            total_time, 5.0, "The function got stuck waiting for the infinite process."
        )
        self.assertIsNotNone(
            p_record.poll(),
            "The process in the termination_list was not closed properly.",
        )


if __name__ == "__main__":
    unittest.main()
