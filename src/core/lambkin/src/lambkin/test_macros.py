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
import re

from abc import ABC, abstractmethod
from copy import deepcopy
from dataclasses import dataclass, field

from robot.model.testcase import TestCase
from robot.model.testsuite import TestSuite
from robot.result.model import TestSuite as TestResult
from typing import Generator
from typing import Iterable
from typing import List
from typing import Optional
from typing import Type


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

    def add_to_suite(self, suite: TestSuite):
        """Creates a new test case in a suite based on TestData."""
        test_case = suite.tests.create(name=self.name)
        test_case.body.create_keyword(name=self.keyword, args=self.args)


class MacroExpander(ABC):
    """Represents an object that can take a list of tests
    and expand its arguments when they match a certain macro."""

    subclasses: List[Type[MacroExpander]] = []
    macro: Optional[str] = None

    def __init_subclass__(cls, *args, **kwargs):
        """Registers subclasses."""
        super().__init_subclass__(*args, **kwargs)
        cls.subclasses.append(cls)

    def _match(self, arg: str) -> bool:
        """Returns true if this argument can be expanded by this class."""
        return re.match(f'^{self.macro}\(.*\)', arg) is not None

    def _get_options(self, arg: str) -> List[str]:
        """Returns the options provided with the macro in list form."""
        return re.search(r'\((.*?)\)', arg).group(1).replace(' ', '').split(',')

    def _count_expansions(self, tests: Iterable[TestData]) -> int:
        """Calculates how many iterations it would take to process all the
        macro appearances in different test arguments."""
        return max(sum(self._match(arg) for arg in test.args) for test in tests)

    def _expand_test(self, test: TestData) -> Generator[TestData, None, None]:
        """Generates test cases expanding macro arguments from a given
        test once."""
        for index, arg in enumerate(test.args):
            # Find the index of the argument to be replaced
            if self._match(arg):
                options = self._get_options(arg)
                break
        else:
            # Consider this test 'expanded' if there is nothing to replace
            yield test
            return

        try:
            for num, value in enumerate(self.generate(*options)):
                new_test = deepcopy(test)
                # TODO(nahuel): find a better naming system
                new_test.name = f'{test.name} {num + 1}'
                new_test.args[index] = value
                yield new_test
        except TypeError as error:
            raise TypeError(
                f"Expansion macro {self.macro}{tuple(options)} "
                 "has incorrect arguments") from error

    def expand(self, tests: List[TestData]) -> List[TestData]:
        """Process all macro appearances and returns a list of
        expanded test cases."""
        for _ in range(self._count_expansions(tests)):
            tests = [new_test
                for test in tests
                for new_test in self._expand_test(test)]
        return tests

    @abstractmethod
    def generate(self, *args: str) -> Generator[str, None, None]:
        """Generates values based on the options provided with
        the macro."""
        raise NotImplementedError


class LinspaceExpander(MacroExpander):
    """Linspace expander subclass.
    See https://numpy.org/doc/stable/reference/generated/numpy.linspace.html"""

    macro: str = 'LINSPACE'

    def generate(self, start: str, end: str, num: str) -> Generator[str, None, None]:
        """Generates evenly spaced numbers over a specified interval."""
        for value in np.linspace(float(start), float(end), int(num)):
            # TODO(nahuel): fix 0.07500000000000001 type values
            yield str(value)


class SequenceExpander(MacroExpander):
    """Sequence expander subclass."""

    macro: str = 'SEQUENCE'

    def generate(self, *args: str) -> Generator[str, None, None]:
        """Generates a sequence consisting of the provided values."""
        for value in args:
            yield value


class TestMacros:
    """A RobotFramework listener that expands
    generator macros in test case arguments."""

    ROBOT_LISTENER_API_VERSION = 3

    def __init__(self):
        self.ROBOT_LIBRARY_LISTENER = self

    def start_suite(self, suite: TestSuite, result: TestResult) -> None:
        """Removes all test cases from the suite and creates
        a new list of test cases expanding their macro arguments."""
        tests = [TestData.from_test_case(test) for test in suite.tests]
        suite.tests = None

        for expander_type in MacroExpander.subclasses:
            tests = expander_type().expand(tests)

        for test in tests:
            test.add_to_suite(suite)
