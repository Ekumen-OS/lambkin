# Copyright 2022 Ekumen, Inc.
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

import numpy as np

from itertools import product

from robot.api import TestSuite

from typing import Any
from typing import Generator
from typing import Iterable
from typing import List
from typing import Tuple


def linspace(start: float, end: float, num: int) -> Generator[float, None, None]:
    """Generates evenly spaced numbers over a specified interval."""
    yield from np.linspace(start, end, num)


def sequence(*args: Any) -> Generator[Any, None, None]:
    """Generates a sequence consisting of the provided values."""
    yield from args


class TestCaseEvaluation:
    """A RobotFramework listener that expands
    generator expressions in test case arguments."""

    ROBOT_LISTENER_API_VERSION = 3

    def __init__(self):
        self.ROBOT_LIBRARY_LISTENER = self

    def start_suite(self, suite: TestSuite, _) -> None:
        """Removes all test cases from the suite and creates
        a new list of test cases expanding their arguments."""
        tests = suite.tests
        suite.tests = None  # Remove existing test cases
        for test_case in tests:
            if len(test_case.body) != 1:
                raise AssertionError('Test case body must have one keyword.')
            keyword = test_case.body[0]
            for num, args in enumerate(self._generate_args(keyword.args), start=1):
                # TODO(nahuel): find a better naming system
                new_test_case = suite.tests.create(name=f'{test_case.name} {num}')
                new_test_case.body.create_keyword(name=keyword.name, args=args)

    @staticmethod
    def _generate_args(args: Iterable[str]) -> Generator[Tuple[Any], None, None]:
        """Generates new argument tuples with the cartesian
        product of evaluated expressions."""
        iterators: List[Iterable[Any]] = []
        for arg in args:
            try:
                result = eval(arg)
                if not isinstance(result, Iterable):
                    raise TypeError('Evaluated expression does not produce an iterable')
                iterators.append(result)
            except Exception:
                # If the argument can't be evaluated, keep it as is
                iterators.append([arg])
        yield from product(*iterators)
