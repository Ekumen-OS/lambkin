# -*- coding: utf-8 -*-
#
# Copyright 2023 Ekumen, Inc.
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
#
# Configuration file for the Sphinx documentation builder.

import os

import ament_index_python

# -- Project information -----------------------------------------------------

project = 'Nominal Beluga AMCL Benchmark Report'
copyright = '2024, Ekumen Inc.'
author = 'Ekumen Inc.'

version = '0.1.0'
release = '0.1.0'

# -- General configuration ---------------------------------------------------

extensions = [
    'linuxdoc.rstFlatTable',
    'sphinxcontrib.datatemplates',
    'sphinxcontrib.repl'
]

# The suffix(es) of source filenames.
source_suffix = '.rst'

# The master toctree document.
master_doc = 'index'

# The language for content autogenerated by Sphinx.
language = 'en'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
exclude_patterns = ['build']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = None

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '10pt',
    'extraclassoptions': 'openany,oneside'
}

latex_table_style = []

# Grouping the document tree into LaTeX files.
latex_documents = [
    # (source, target name, title, author, documentclass)
    (master_doc, 'report.tex', project, author, 'manual'),
]

def setup(app):
    app.add_config_value('sysroot', os.path.relpath('/', app.srcdir), rebuild=False)
