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

"""Logging configuration utilities for the lambkin SDK."""

import logging
import sys

from lambkin.common import defaults


def configure_logging(level: str = defaults.LOG_LEVEL) -> None:
    """Configure the lambkin logger.

    Sets up a dedicated handler on the 'lambkin' logger with propagation
    disabled, so lambkin output never touches the root logger or any
    user-configured handlers. Safe to call multiple times — existing
    handlers are cleared before adding the new one.

    Args:
        level: Logging level as a string, e.g. 'debug', 'info', 'warning',
            'error'. Case-insensitive. Defaults to the SDK default log level.
    """
    int_level = getattr(logging, level.upper())
    logger = logging.getLogger("lambkin")
    logger.propagate = False
    logger.handlers.clear()
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(int_level)
    handler.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(handler)
    logger.setLevel(int_level)
