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

from dataclasses import dataclass, field
from robot.api import logger
from robot.api.deco import keyword
from robot.libraries.BuiltIn import BuiltIn
from typing import Dict, List


@dataclass
class Keyword:
    args: List[str] = field(default_factory=list)
    executed: bool = field(default=False, repr=False)

    def run(self):
        self.executed = True
        return BuiltIn().run_keyword(*self.args)


class Hooks:
    """A RobotFramework library that implements keyword hook handling to
    separate control flow definition from execution."""

    ROBOT_LIBRARY_SCOPE = 'TEST'

    def __init__(self):
        self.hooks: Dict[str, List[Keyword]] = {}

    @keyword('Register Keyword')
    def register_keyword(self, hook: str, *args: str) -> None:
        """Registers a new keyword to a given hook name.

        The keyword and arguments in ``kw`` will be registered to ``hook`` name
        as long as the same keyword is not already registered to that ``hook``.

        Positional and named arguments can be used. Keywords with positional
        list parameters followed by named parameters are not supported.
        """
        kw = Keyword(args)

        if hook in self.hooks:
            if kw in self.hooks[hook]:
                logger.info(f"{kw} already exists in '{hook}' hook.")
                return
            self.hooks[hook].append(kw)
        else:
            self.hooks[hook] = [kw]

        logger.info(f"{kw} registered to '{hook}' hook.")

    @keyword('Run Registered Keywords')
    def run_registered_keywords(self, hook: str) -> None:
        """Runs all keywords registered to ``hook``.

        Not having any keyword to run is not an error.
        """
        if hook in self.hooks and len(self.hooks[hook]) > 0:
            for kw in self.hooks[hook]:
                kw.run()
        else:
            logger.info(f"No registered keywords to run in '{hook}' hook.")

    @keyword('Clear Registered Keywords')
    def clear_registered_keywords(self, hook: str) -> None:
        """Removes all keywords registered to ``hook``."""
        self.hooks[hook].clear()

    @keyword('Registered Keywords Should Have Run')
    def registered_keywords_should_have_run(self, hook: str) -> None:
        """Fails if a keyword registered to ``hook`` was not called."""
        if hook in self.hooks:
            for kw in self.hooks[hook]:
                if not kw.executed:
                    raise AssertionError(f"{kw} registered to '{hook}' hook has not been called.")

    @keyword('All Registered Keywords Should Have Run')
    def all_registered_keywords_should_have_run(self) -> None:
        """Fails if any registered keyword was not called."""
        for hook in self.hooks.keys():
            self.registered_keywords_should_have_run(hook)
