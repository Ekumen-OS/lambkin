import contextlib
import fnmatch
import functools
import glob
import importlib
import os

import unittest.mock as mock

from types import ModuleType
from typing import Any, Dict, Iterable, List, Optional, Tuple, Type, Union

from jinja2.sandbox import SandboxedEnvironment

from robot.api.parsing import get_resource_model, ModelVisitor
from robot.libdoc import libdoc
from robot.libdocpkg.builder import RESOURCE_EXTENSIONS

from docutils import frontend, utils
from docutils.nodes import Node
from docutils.statemachine import StringList
from docutils.parsers.rst import Directive, Parser
from sphinx.application import Sphinx
from sphinx.util.template import SphinxTemplateLoader
from sphinx.util.typing import OptionSpec
from sphinx.util import logging, rst


logger = logging.getLogger(__name__)


DirectiveArgs = Tuple[str, List[str], Dict[str, Any], StringList, int, int, str]


def mock_directive(prototype: Directive, returns: Optional[List[Node]] = None) -> Type[Directive]:
    """
    Create a mock :mod:`docutils` directive class after the given `prototype`.

    This mock directive class tracks invocations in its ``call_args_list`` attribute.

    :param prototype: directive class to gather mock directive configuration from.
    :param returns: optional return value when running the mock directive.
      The `prototype` directive class will be run if none is provided.
    :returns: mock directive class
    """

    class MockDirective(Directive):

        _prototype = prototype

        required_arguments: int = getattr(prototype, 'required_arguments', 0)
        optional_arguments: int = getattr(prototype, 'optional_arguments', 0)
        final_argument_whitespace: bool = \
            getattr(prototype, 'final_argument_whitespace', False)
        option_spec: Optional[OptionSpec] = getattr(prototype, 'option_spec', None)
        has_content: bool = getattr(prototype, 'has_content', False)

        call_args_list: List[DirectiveArgs] = []
        return_value: Optional[List[Node]] = returns

        def run(self):
            self.call_args_list.append((
                self.name, self.arguments, self.options, self.content,
                self.lineno, self.content_offset, self.block_text))
            if self.return_value is None:
                return self._prototype(
                    self.name, self.arguments, self.options, self.content,
                    self.lineno, self.content_offset, self.block_text,
                    self.state, self.state_machine)
            return self.return_value

    return MockDirective


@contextlib.contextmanager
def patch_directives(returns: Optional[List[Node]] = None):
    """
    Patch :mod:`docutils` directives factory to mock them.

    To be used as a context manager, when entering scope
    it yields the cache hodling all mock directives created
    upon request by `docutils` internals.

    :param returns: forwarded to :func:`mock_directive`.
    """
    cache: Dict[str, Directive] = {}

    def decorator(spec):
        @functools.wraps(spec)
        def _wrapper(directive_name, language_module, document):
            messages = []
            if directive_name not in cache:
                directive, messages = spec(
                    directive_name, language_module, document)
                cache[directive_name] = mock_directive(directive, returns)
            return cache[directive_name], messages
        return _wrapper

    target = 'docutils.parsers.rst.directives.directive'
    with mock.patch(target, new_callable=decorator, spec=True):
        yield cache


def find_directives(
    app: Sphinx, name: str, source: Optional[str] = None
) -> Iterable[DirectiveArgs]:
    """
    Find all directives in an ReST document.

    :param app: current Sphinx application.
    :param name: target document name.
    :param source: optional target document sources,
      read from document file if not provided.
    :returns: an iterable over directive run arguments.
    """
    if source is None:
        path = app.builder.env.doc2path(name)
        if not os.path.isfile(path):
            return
        encoding = app.config.source_encoding
        with open(path, mode='r', encoding=encoding) as f:
            source = f.read()
    settings = frontend.get_default_settings(Parser)  # type: ignore
    settings.ensure_value('env', app.env)
    settings.report_level = 5
    with patch_directives([]) as cache:
        document = utils.new_document(name, settings)
        Parser().parse(source, document)
        for mock_directive in cache.values():
            for args in mock_directive.call_args_list:
                yield args


def has_robot_keywords(module: ModuleType) -> bool:
    """Check if a `module` defines RobotFramework keywords."""
    return any(
        hasattr(getattr(module, name), 'robot_name')
        for name in dir(module) if not name.startswith('__'))


EXTENSIONS_PATH = os.path.dirname(os.path.abspath(__file__))
EXTENSION_TEMPLATES_PATH = [os.path.join(
    EXTENSIONS_PATH, 'templates', 'autorobot')]


def prepare_template_engine(app: Sphinx) -> SandboxedEnvironment:
    """
    Instantiate :mod:`sphinx` template engine.

    :returns: a :mod:`jinja2` environment.
    """
    loader = SphinxTemplateLoader(
        app.srcdir,
        app.config.templates_path,
        EXTENSION_TEMPLATES_PATH)

    env = SandboxedEnvironment(loader=loader)
    env.filters['escape'] = rst.escape

    def _underline(title: str, line: str = '=') -> str:
        return title + '\n' + line * len(title)
    env.filters['underline'] = _underline
    return env


def find_robot_resources(
    module: Union[ModuleType, str], recursive: bool = False
) -> Iterable[Tuple[str, str]]:
    """
    Find all RobotFramework resource files at `module` location.

    Python source files are ignored.

    :param module: either a module object or a module name.
    :param recursive: whether to recurse into submodules while
      searching or not.
    :returns: an iterable over tuples of search path and file path.
    """
    if isinstance(module, ModuleType):
        module = module.__name__
    spec = importlib.util.find_spec(module)
    if spec is None:
        return
    if spec.submodule_search_locations is None:
        return
    for search_path in spec.submodule_search_locations:
        pattern = '**/*' if recursive else '*'
        for ext in RESOURCE_EXTENSIONS:
            pattern = os.path.join(search_path, f'{pattern}.{ext}')
            for source_path in glob.iglob(pattern, recursive=recursive):
                yield search_path, os.path.relpath(source_path, search_path)


def fully_qualified_name(path: str, package: Optional[str] = None) -> str:
    """
    Compute a fully qualified name for a "module" given its `path`.

    Note that this function will use Python module name semantics
    regardless of whether `path` points a Python source file or not.

    :param path: path to "module".
    :param package: optional base Python package
      if `path` points to a "submodule".
    :returns: fully qualified name (ie. dot separators).
    """
    basename, _ = os.path.splitext(path)
    module_name = basename.replace('/', '.')
    if package is not None:
        module_name = package + '.' + module_name
    return module_name


def new_static_file(app: Sphinx, name: str) -> str:
    """
    Register a new static file with the given `app`.

    This function will take care of all the necessary
    configuration and filesystem manipulation.

    :param name: file name (can be a relative path).
    :returns: path relative to :mod:`sphinx` project root.
    """
    path = os.path.join('_static', name)
    os.makedirs(os.path.dirname(path), exist_ok=True)

    html_static_path: List[str] = getattr(app.config, 'html_static_path', [])
    if '_static' not in html_static_path:
        html_static_path.append('_static')
        setattr(app.config, 'html_static_path', html_static_path)

    return path


def generate_robot_documentation(app: Sphinx):
    """
    Generate RobotFramework documentation files for the given `app`.

    To do this, it will look for recursive ``autosummary`` directives
    (see :mod:`sphinx.ext.autosummary` for further reference) and for
    each RobotFramework source file found within the specified Python
    modules, it will invoke :func:`robot.libdoc.libdoc` to generate
    HTML documentation and add a new ReST document to the `app` to
    embed it.

    This function is to be connected to ``'builder-inited'`` events.
    It only supports HTML output builders.
    """
    if app.builder.format != 'html':
        msg = '[autorobot] %s output not supported, ignoring'
        logger.info(msg, app.builder.format)
        return

    env = prepare_template_engine(app)
    template = env.get_template('resource.rst')

    encoding = app.config.source_encoding
    exclude_patterns = app.config.autorobot_exclude_patterns

    for docname in app.builder.env.found_docs:
        dirpath = os.path.dirname(app.builder.env.doc2path(docname))
        for directive in find_directives(app, docname):
            name, _, options, content, *_ = directive
            if name != 'autosummary':
                continue
            if 'recursive' not in options:
                continue
            genpath = options.get('toctree', dirpath).strip()
            for name in filter(None, content):
                for root, path in find_robot_resources(name, recursive=True):
                    fullpath = os.path.join(root, path)
                    if any(fnmatch.fnmatch(fullpath, p)
                           for p in exclude_patterns):
                        continue

                    fullname = fully_qualified_name(path, package=name)
                    html_doc_path = \
                        new_static_file(app, os.path.join(
                            'libdoc', fullname + '.html'))

                    libdoc(
                        fullpath, outfile=html_doc_path, name='Library',
                        docformat=app.config.autorobot_docformat
                    )

                    html_doc = os.path.relpath(html_doc_path, genpath)
                    context = {'fullname': fullname, 'html_doc': html_doc}

                    rst_doc_path = os.path.join(genpath, fullname + '.rst')
                    with open(rst_doc_path, mode='w', encoding=encoding) as f:
                        f.write(template.render(context))


class DocumentationCollector(ModelVisitor):
    """A RobotFramework AST model visitor to collect top level documentation."""

    def __init__(self):
        """Initialize visitor."""
        self.documentation = ''

    def generic_visit(self, node):
        """
        Visit any unspecified node.

        This function ignores unspecified traversal paths.
        """
        pass

    def visit_File(self, node):
        """
        Visit file node.

        This function traverses file body.
        """
        super().generic_visit(node)

    def visit_SettingSection(self, node):
        """
        Visit settings section node.

        This function traverses settings statements.
        """
        super().generic_visit(node)

    def visit_Documentation(self, node):
        """Visit documentation node."""
        self.documentation = node.value


def fetch_robot_documentation(path: str) -> str:
    """
    Retrieve RobotFramework resource file documentation.

    :param path: path to resource file.
    :returns: resource file documentation.
    """
    model = get_resource_model(path)
    reader = DocumentationCollector()
    reader.visit(model)
    return reader.documentation.replace('\n', ' ')


def summarize_robot_documentation(app: Sphinx, docname: str, source: List[str]):
    """
    Summarize RobotFramework resource files in Python module documentation.

    To do this, it will look for ``automodule`` directives
    (see :mod:`sphinx.ext.autodoc` for further reference),
    replace them with embedded HTML documentation generated
    :func:`robot.libdoc.libdoc` if any RobotFramework keyword
    is found in the referenced module, and add links to documentation
    for any RobotFramework resource file found in the aforementioned
    module.

    This function is to be connected to ``'source-read'`` events.
    It only supports HTML output builders.
    """
    if app.builder.format != 'html':
        return

    source_path = app.builder.env.doc2path(docname)
    source[0] = '\n'.join(line.rstrip() for line in source[0].splitlines())

    env = prepare_template_engine(app)
    for directive in find_directives(app, docname, source[0]):
        name, arguments, options, content, *_, block_text = directive
        if name != 'automodule':
            continue

        module = importlib.import_module(arguments[0])
        if has_robot_keywords(module):
            html_doc_path = \
                new_static_file(app, os.path.join(
                    'libdoc', module.__name__ + '.html'))

            libdoc(
                module.__file__,
                outfile=html_doc_path, name='Library',
                docformat=app.config.autorobot_docformat
            )

            html_doc = os.path.relpath(
                html_doc_path, os.path.dirname(source_path))
            context = {'fullname': module.__name__, 'html_doc': html_doc}
            template = env.get_template('inline_resource.rst')
            addendum = template.render(context)

            source[0] = source[0].replace(block_text, addendum)

        resources = [
            (fully_qualified_name(path, package=module.__name__),
             fetch_robot_documentation(os.path.join(root, path)))
            for root, path in find_robot_resources(module.__name__)
            if not any(fnmatch.fnmatch(os.path.join(root, path), p)
                       for p in app.config.autorobot_exclude_patterns)
        ]
        if resources:
            template = env.get_template('resource_summary.rst')
            addendum = template.render({'resources': resources})
            source[0] = '\n'.join([source[0], addendum])

        encoding = app.config.source_encoding
        with open(source_path, mode='w', encoding=encoding) as f:
            f.write(source[0])


def setup(app: Sphinx) -> Dict[str, Any]:
    """Set up ``autorobot`` extension."""
    app.setup_extension('sphinx.ext.autosummary')  # a dependency
    app.connect('builder-inited', generate_robot_documentation)
    app.connect('source-read', summarize_robot_documentation)
    app.add_config_value('autorobot_docformat', 'ROBOT', 'env')
    app.add_config_value('autorobot_exclude_patterns', [], 'env')
    return {'version': '0.1', 'parallel_read_safe': False}
