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

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
import sphinx_rtd_theme

# Add the parent directory to the path so that Sphinx can find the package
sys.path.insert(0, os.path.abspath("./_extension"))
sys.path.insert(0, os.path.abspath("../src"))

# -- Project information -----------------------------------------------------

project = 'LAMBKIN Shepherd'
copyright = '2023, Ekumen Labs'
author = 'Ekumen Labs'
release = "0.1.0"
version = "0.1"

# -- General configuration ---------------------------------------------------

extensions = [
    "autorobot",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "_extension", "Thumbs.db", ".DS_Store"]
html_theme = "sphinx_rtd_theme"

# -- Options for HTML output -------------------------------------------------

html_theme_options = {
    "style_nav_header_background": "#007acc",
}

# -- Options for autodoc extension -------------------------------------------

autodoc_member_order = "bysource"
autodoc_default_options = {
    "members": True,
    "member-order": "bysource",
    "undoc-members": True,
    "private-members": True,
    "special-members": "__init__",
    "exclude-members": "",
}

# -- Options for napoleon extension ------------------------------------------

napoleon_google_docstring = False
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = True
napoleon_include_special_with_doc = True

# -- Options for viewcode extension ------------------------------------------

viewcode_follow_imported_members = True

# -- Options for robotsummary extension --------------------------------------

autorobot_docformat = 'ROBOT'
autorobot_exclude_patterns = ['**/all.resource']
