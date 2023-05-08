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

"""This module supplements RobotFramework hooks library resource."""

from robot.api import logger
from robot.api.deco import keyword
from robot.model.keyword import Keyword
from robot.libraries.BuiltIn import BuiltIn
from typing import Dict, List


class ExtensionPoints:
    """
    A RobotFramework library implementation for dynamic keyword-based hooks.

    This allows separating control flow definition from execution.
    """

    def __init__(self) -> None:
        """Instantiate library."""
        self._keywords: Dict[str, List[Keyword]] = {}

    def register_keyword(self, hook: str, name: str, *args: str) -> None:
        """
        Register a new keyword to a given `hook`.

        Positional and named arguments can be used. Keywords with positional
        list parameters followed by named parameters are not supported.
        """
        kw = Keyword(name, args=args)
        if hook not in self._keywords:
            self._keywords[hook] = []
        self._keywords[hook].append(kw)

        logger.info(f"'{kw}' registered to '{hook}' hook.")

    def run_registered_keywords(self, hook: str) -> None:
        """
        Run all keywords registered to `hook`.

        Not having any keyword to run is not an error.
        """
        if hook in self._keywords:
            builtin = BuiltIn()
            for kw in self._keywords[hook]:
                builtin.run_keyword(kw.name, *kw.args)
        else:
            logger.info(f"No registered keywords to run in '{hook}' hook.")

    def clear_registered_keywords(self, hook: str) -> None:
        """Remove all keywords registered to `hook`."""
        del self._keywords[hook]


class SuiteExtensionPoints(ExtensionPoints):
    """A RobotFramework library providing extension points at suite scope (with suite lifetime)."""

    ROBOT_LIBRARY_SCOPE = 'SUITE'

    @keyword('Register Keyword On Current Suite')
    def register_keyword_on_current_suite(self, *args: str) -> None:
        """Register a new keyword to a given ``hook`` in suite scope."""
        self.register_keyword(*args)

    @keyword('Run Keywords Registered On Current Suite')
    def run_keywords_registered_on_current_suite(self, *args: str) -> None:
        """Run all keywords registered to `hook` in suite scope."""
        self.run_registered_keywords(*args)

    @keyword('Clear Keywords Registered On Current Suite')
    def clear_keywords_registered_on_current_suite(self, *args: str) -> None:
        """Remove all keywords registered to `hook` in suite scope."""
        self.clear_registered_keywords(*args)


class TestExtensionPoints(ExtensionPoints):
    """A RobotFramework library providing extension points at test scope (with test lifetime)."""

    ROBOT_LIBRARY_SCOPE = 'TEST'

    @keyword('Register Keyword On Current Test')
    def register_keyword_on_current_test(self, *args: str) -> None:
        """Register a new keyword to a given `hook` in test scope."""
        self.register_keyword(*args)

    @keyword('Run Keywords Registered On Current Test')
    def run_keywords_registered_on_current_test(self, *args: str) -> None:
        """Run all keywords registered to `hook` in test scope."""
        self.run_registered_keywords(*args)

    @keyword('Clear Keywords Registered On Current Test')
    def clear_keywords_registered_on_current_test(self, *args: str) -> None:
        """Remove all keywords registered to `hook` in test scope."""
        self.clear_registered_keywords(*args)
