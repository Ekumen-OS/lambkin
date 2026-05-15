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

Typical usage::

    # ctx.options.dry_run is always available, no @lambkin.option needed
    def my_benchmark(ctx):
        ctx.options.dry_run
"""

import click

SDK_OPTIONS: list[click.Option] = [
    click.Option(
        ["--dry-run"],
        is_flag=True,
        default=False,
        help="Print commands without executing them.",
    ),
]
