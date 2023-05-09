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

"""This module provides RobotFramework listeners to augment its capabilities."""

import re

from itertools import product

from typing import Any, Dict, Iterable, List, Optional, Tuple, Union

from robot.api.deco import keyword
from robot.api.exceptions import Error

from robot.running.arguments import EmbeddedArguments
from robot.running.context import EXECUTION_CONTEXTS
from robot.running.model import TestCase, Keyword
from robot.result.model import TestCase as TestCaseResult
from robot.result.model import TestSuite as TestSuiteResult
from robot.model import SuiteVisitor


class InlineExpressionExpander(SuiteVisitor):
    """
    A RobotFramework visitor that expands arbitrary inline expressions.

    Inline expressions may be found within keyword call arguments, embedded or not.
    Inline expressions follow the ${{...}} form. On expansion, keyword calls are
    substituted in-place for each interpolation by each combination in the cartesian
    product of all sequences that result from each inline expansion (at least one element).
    """

    def visit_keyword(self, keyword: Keyword):
        """Substitute `keyword` in-place upon inline expression expansion."""
        if keyword not in keyword.parent.body:
            return
        index = keyword.parent.body.index(keyword)
        iterators = [self._expand_expressions(keyword.name)] + [
            self._expand_expressions(arg) for arg in keyword.args]
        for i, (name, *args) in enumerate(product(*iterators)):
            keyword.parent.body.insert(
                index + i + 1, Keyword(
                    name=name, args=args,
                    assign=keyword.assign,
                    parent=keyword.parent,
                    type=keyword.type,
                    lineno=keyword.lineno
                )
            )
        del keyword.parent.body[index]

    EXPRESSION_PATTERN = re.compile(r'\$\{\{(?P<inline_expression>.*?)\}\}')

    @classmethod
    def _expand_expressions(cls, unevaluated_expression: str) -> Iterable[Any]:
        """Evaluate and interpolate all inline expressions."""
        namespace = EXECUTION_CONTEXTS.current.namespace
        local_scope = \
            namespace.variables.as_dict(decoration=False)
        global_scope = {
            library.name: library.get_instance()
            for library in namespace.libraries
        }
        matches = list(cls.EXPRESSION_PATTERN.finditer(unevaluated_expression))
        if not matches:
            return [unevaluated_expression]
        expansions = []
        index = matches[0].start()
        if index > 0:
            expansions.append([unevaluated_expression[:index]])
        for match in matches:
            if match.start() > index:
                expansions.append([unevaluated_expression[index:match.start()]])
            values = eval(
                match.group('inline_expression'), global_scope, local_scope)
            if isinstance(values, str) or not isinstance(values, Iterable):
                values = [values]
            expansions.append(values)
            index = match.end()
        if index < len(unevaluated_expression):
            expansions.append([unevaluated_expression[index:]])
        if len(expansions) == 1:
            return expansions[0]
        concat = (lambda values: ''.join(map(str, values)))
        return map(concat, product(*expansions))


class ParameterizationSupport:
    """
    A RobotFramework library that enables benchmark case parameterization.

    This is achieved via listeners, test templates, and inline expressions.
    """

    ROBOT_LISTENER_API_VERSION = 3

    def __init__(self) -> None:
        """Instantiate library."""
        self.ROBOT_LIBRARY_LISTENER = self
        self.expressions_expander = InlineExpressionExpander()
        self.active_template: Optional[EmbeddedArguments] = None

    @keyword(tags=['imperative', 'internal'])
    def extract_template_arguments(self, instance: str) -> Dict[str, Any]:
        """
        Extract arguments from an instance of a test case template.

        For internal use only.

        :param instance: test case template instance with embedded arguments.
        :return: test case arguments as a name-value mapping.
        :raises Error: if not invoked from within a templated test case.
        """
        if self.active_template is None:
            raise Error('Not in a templated test case')
        match = self.active_template.match(instance)
        return dict(self.active_template.map(match.groups()))

    def start_test(self, test_case: TestCase, _) -> None:
        """Bookkeep the active test case template."""
        if test_case.template is not None:
            self.active_template = \
                EmbeddedArguments.from_name(test_case.template)
        else:
            self.active_template = None
        test_case.visit(self.expressions_expander)


class TracebackSupport:
    """A RobotFramework listener that provides tracebacks on error."""

    ROBOT_LISTENER_API_VERSION = 3

    class KeywordCallTracer:
        """A RobotFramework listener to trace keyword calls."""

        ROBOT_LISTENER_API_VERSION = 2

        class Traceback:
            """A RobotFramework traceback object."""

            def __init__(self, name: Optional[str] = None) -> None:
                """Instantiate traceback with an optional `name`."""
                self._name = name
                self._calls: List[Tuple[str, List[str], str, Union[str, int]]] = []

            def append(
                self, name: str, args: List[str],
                source: str, lineno: Union[str, int]
            ) -> None:
                """
                Add keyword call site to traceback.

                :param name: keyword name.
                :param args: keyword args.
                :param source: source file where call site is located.
                :param lineno: call site location within source file.
                """
                self._calls.append((name, args, source, lineno))

            def __str__(self) -> str:
                lines = ['Traceback' + f' ({self._name})' if self._name else '']
                if self._calls:
                    for name, _, source, lineno in reversed(self._calls[1:]):
                        lines.append(f'  File "{source}", line {lineno}, in {name}')
                    name, args, source, lineno = self._calls[0]
                    lines.append(f'  File "{source}", line {lineno}, in')
                    lines.append(f'      {name} {" ".join(args)}')
                return '\n'.join(lines)

            def __bool__(self) -> bool:
                return bool(self._calls)

        def __init__(self) -> None:
            """Instantiate keyword call tracer."""
            self.suite_traces: List[TracebackSupport.KeywordCallTracer.Traceback] = []
            self.test_traces: List[TracebackSupport.KeywordCallTracer.Traceback] = []
            self._traces: List[TracebackSupport.KeywordCallTracer.Traceback] = self.suite_traces
            self._active_traces: List[TracebackSupport.KeywordCallTracer.Traceback] = []

        def start_suite(self, *_) -> None:
            """Listen to suite starts."""
            self.suite_traces.clear()
            self._active_traces.clear()
            self._traces = self.suite_traces

        def start_test(self, *_) -> None:
            """Listen to test starts."""
            self.test_traces.clear()
            self._active_traces.clear()
            self._traces = self.test_traces

        def start_keyword(self, name: str, attributes: Dict[str, Any]) -> None:
            """Listen to keyword call starts."""
            if attributes['type'] == 'SETUP':
                self._active_traces.append(self.Traceback('setup'))
            elif attributes['type'] == 'TEARDOWN':
                self._active_traces.append(self.Traceback('teardown'))
            elif not self._active_traces:
                self._active_traces.append(self.Traceback('body'))

        def end_keyword(self, name: str, attributes: Dict[str, Any]) -> None:
            """Listen to keyword call ends."""
            if attributes['status'] == 'FAIL':
                name = attributes['kwname']
                if attributes['type'] not in ('KEYWORD', 'SETUP', 'TEARDOWN'):
                    name = f"{attributes['type']} {name}"
                self._active_traces[-1].append(
                    name.strip(), attributes['args'],
                    attributes['source'] or '<unknown>',
                    attributes['lineno'] or '?')
            if attributes['type'] in ('SETUP', 'TEARDOWN'):
                if not self._active_traces:
                    return
                trace = self._active_traces.pop()
                if trace:
                    self._traces.append(trace)

        def end_test(self, *_) -> None:
            """Listen to test ends."""
            if any(self._active_traces):
                self._traces.extend(filter(
                    bool, self._active_traces))
            self._active_traces.clear()
            self._traces = self.suite_traces

        def flush(self) -> None:
            """
            Flush active traces.

            Useful to decouple invocations within test scope.
            """
            if any(self._active_traces):
                self._traces.extend(filter(
                    bool, self._active_traces))
            self._active_traces.clear()

    def __init__(self) -> None:
        """Instantiate library."""
        self.keyword_call_tracer = self.KeywordCallTracer()
        self.ROBOT_LIBRARY_LISTENER = [self.keyword_call_tracer, self]

    @keyword('Flush Traceback')
    def flush_traceback(self) -> None:
        """Flush current traceback."""
        self.keyword_call_tracer.flush()

    def end_suite(self, _, result: TestSuiteResult) -> None:
        """Listen to suite ends."""
        if result.status == 'FAIL':
            traces = map(str, self.keyword_call_tracer.suite_traces)
            result.message = '\n\n'.join([*traces, result.message])

    def end_test(self, _, result: TestCaseResult) -> None:
        """Listen to test ends."""
        if result.status == 'FAIL':
            traces = map(str, self.keyword_call_tracer.test_traces)
            result.message = '\n\n'.join([*traces, result.message])
