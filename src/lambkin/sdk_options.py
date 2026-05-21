# Copyright 2026 Ekumen, Inc.
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

"""SDK-level CLI options exposed natively by lambkin.

Defines the set of CLI options that lambkin provides out of the box,
without requiring the user to declare them via ``@lambkin.option``.
These options are merged with any user-defined options by ``@benchmark``
before parsing, so they are always available on ``ctx.options``.

.. warning::
    The option names defined here are reserved. Declaring any of them
    via ``@lambkin.option`` in user benchmarks will raise a Click error
    at parse time due to duplicate parameter names.

    Reserved options:
        - ``--dry-run``
        - ``--show-options``

Note:
    ``--show-options`` behaves like ``--help``: it prints registered
    options and exits immediately, never reaching the benchmark body.
"""

import click

from lambkin.common import defaults

SDK_OPTIONS: tuple[click.Option, ...] = (
    click.Option(
        ["--dry-run"],
        is_flag=True,
        show_default=True,
        default=defaults.DRY_RUN,
        help="Run the benchmark in dry-run mode: commands are logged but not executed.",
    ),
    click.Option(
        ["--show-options"],
        is_flag=True,
        default=False,
        help="List all options registered via @lambkin.option.",
    ),
    click.Option(
        ["--log-output"],
        type=click.Choice(["file", "console", "both"]),
        show_default=True,
        default=None,
        help="Where to route process output: file, console, or both.",
    ),
)
