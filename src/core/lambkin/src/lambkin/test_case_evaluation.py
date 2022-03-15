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

from __future__ import annotations

import numpy as np

from dataclasses import dataclass, field
from itertools import product

from robot.model.testcase import TestCase
from robot.model.testsuite import TestSuite
from robot.result.model import TestSuite as TestResult
from typing import Any
from typing import Generator
from typing import Iterable
from typing import List
from typing import Tuple


@dataclass
class TestData:
    """Represents test case data in a way that is easier to handle."""

    name: str
    keyword: str
    args: List[str] = field(default_factory=list)

    @staticmethod
    def from_test_case(test_case: TestCase) -> TestData:
        """Creates a instance of TestData from a given TestCase."""
        if len(test_case.body) != 1:
            raise AssertionError(f'Test case body must have one keyword.')
        keyword = test_case.body[0]
        return TestData(
            name=test_case.name,
            keyword=keyword.name,
            args=list(keyword.args))


def linspace(start: float, end: float, num: int) -> Generator[float, None, None]:
    """Generates evenly spaced numbers over a specified interval."""
    for value in np.linspace(start, end, num):
        yield value


def sequence(*args: Any) -> Generator[Any, None, None]:
    """Generates a sequence consisting of the provided values."""
    for value in args:
        yield value


class TestCaseEvaluation:
    """A RobotFramework listener that expands
    generator functions in test case arguments."""

    ROBOT_LISTENER_API_VERSION = 3

    def __init__(self):
        self.ROBOT_LIBRARY_LISTENER = self

    def start_suite(self, suite: TestSuite, result: TestResult) -> None:
        """Removes all test cases from the suite and creates
        a new list of test cases expanding their arguments."""
        tests = [TestData.from_test_case(test) for test in suite.tests]
        suite.tests = None

        for test in tests:
            for num, args in enumerate(self._generate_args(test), start=1):
                # TODO(nahuel): find a better naming system
                test_case = suite.tests.create(name=f'{test.name} {num}')
                test_case.body.create_keyword(name=test.keyword, args=args)

    @staticmethod
    def _generate_args(test: TestData) -> Generator[Tuple[Any], None, None]:
        """Generates new argument tuples with the cartesian
        product of evaluated expressions."""
        iterators: List[Iterable[Any]] = []
        for arg in test.args:
            try:
                result = eval(arg)
                if not isinstance(result, Iterable):
                    raise TypeError('Evaluated expression does not produce an iterable')
                iterators.append(result)
            except Exception:
                # If the argument can't be evaluated, keep it as is
                iterators.append([arg])

        for args in product(*iterators):
            yield args
