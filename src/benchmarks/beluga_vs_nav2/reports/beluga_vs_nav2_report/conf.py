# -*- coding: utf-8 -*-
#
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
#
# Configuration file for the Sphinx documentation builder.

import os
import string

# -- Project information -----------------------------------------------------

project = 'LAMBKIN Benchmark Report'
copyright = '2022, Ekumen Inc.'
author = 'Ekumen Inc.'

version = '0.1.0'
release = '0.1.0-alpha'

# -- General configuration ---------------------------------------------------

extensions = [
    'matplotlib.sphinxext.plot_directive'
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

# Code that should be executed before each plot.
plot_pre_code = """
import lambkin.shepherd as lks
import pandas as pd
import seaborn as sns
from matplotlib import pyplot as plt
"""

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '10pt',
}

# Grouping the document tree into LaTeX files.
latex_documents = [
    # (source, target name, title, author, documentclass)
    (master_doc, 'report.tex', project, author, 'manual'),
]

def interpolate(app, docname, source):
    source[0] = string.Template(source[0]).substitute({})

def setup(app):
    app.connect('source-read', interpolate)
