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

"""Partial restart cache for lambkin benchmarks.

Provides hash-based caching to skip (variant, iteration) pairs that have
already completed successfully. The cache key is derived from the variant
parameters, relevant options, and the benchmark source file contents, so
that changing any of these invalidates only the affected iterations.

Completion is persisted to disk via metadata.yaml, so it survives process
restarts, crashes, and interruptions.
"""

from __future__ import annotations

import hashlib
import json
import logging
from pathlib import Path

import yaml

from lambkin.sdk_options import SDK_OPTIONS

logger = logging.getLogger(__name__)

# Derive excluded options from SDK_OPTIONS so this set stays in sync
# automatically whenever new SDK options are added.
_HASH_EXCLUDED_OPTIONS = {opt.name for opt in SDK_OPTIONS}


def compute_run_hash(
    variant: dict,
    iteration: int,
    options: dict,
) -> str:
    """Derive a stable hash from the inputs of a (variant, iteration) run.

    The hash covers variant parameters, the iteration index, relevant options
    (excluding flags that do not affect outputs). Changing any of these
    invalidates the cached result.

    Args:
        variant: The variant parameters for this run.
        iteration: Zero-based iteration index within this variant.
        options: The resolved options dict for this run.

    Returns:
        A hex digest string identifying this run's inputs.
    """
    relevant_options = {
        k: v for k, v in options.items() if k not in _HASH_EXCLUDED_OPTIONS
    }
    # TODO: evaluate if we want to invalidate the cache for changes in the source file
    # that don't affect the benchmark function (e.g. logs, comments, etc.).
    # If so, we could try to extract just the benchmark function's source code using
    # the inspect module, but this is non-trivial and may not be robust to all valid
    # Python syntax. For now, we take the simpler approach of not accouting for changes
    # in the source file, so that we do not invalidate the cache.
    # source_contents = source_path.read_bytes()
    payload = json.dumps(
        {
            "variant": variant,
            "iteration": iteration,
            "options": relevant_options,
        },
        sort_keys=True,
    ).encode()
    digest = hashlib.sha256(payload).hexdigest()
    return digest


def is_completed(metadata_path: Path, run_hash: str) -> bool:
    """Return True if this iteration completed successfully with matching inputs.

    Reads the metadata file at the given path and checks that both
    completed_at and run_hash are present and that run_hash matches the
    hash computed from the current inputs.

    Args:
        metadata_path: Path to the iteration metadata file.
        run_hash: The hash computed from the current run inputs.

    Returns:
        True if the iteration is complete and inputs match, False otherwise.
    """
    if not metadata_path.exists():
        return False
    try:
        with open(metadata_path) as f:
            metadata = yaml.safe_load(f)
    except Exception:
        logger.warning(
            "Could not read %s, treating iteration as incomplete.", metadata_path
        )
        return False
    if not metadata.get("completed_at"):
        return False
    if metadata.get("run_hash") != run_hash:
        return False
    return True
