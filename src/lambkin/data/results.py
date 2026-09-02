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

"""Reader and writer for the result archives evo tools write.

The archive is a flat zip holding ``info.json``, ``stats.json`` and one ``.npy``
per array, each named after the array it holds. Which arrays are present depends
on the metric and the alignment mode, so none of them is guaranteed.

Written from observation of archives the evo tools produced, not from evo's
source. evo is not imported and is not a dependency: its tools run as external
processes and only their output is parsed. ``test/data/fixtures/`` holds real
archives the tests read back.
"""

import dataclasses
import io
import json
import logging
import zipfile
from pathlib import Path
from typing import Any, Final

import numpy as np
import numpy.typing as npt

logger = logging.getLogger(__name__)

INFO_MEMBER: Final[str] = "info.json"
STATS_MEMBER: Final[str] = "stats.json"
ARRAY_SUFFIX: Final[str] = ".npy"

# Members are stored uncompressed, so anything past this is a zip bomb rather
# than a trajectory.
MAX_UNCOMPRESSED_BYTES: Final[int] = 512 * 1024 * 1024


class ResultFormatError(ValueError):
    """Raised when a result archive is malformed or cannot be parsed."""


@dataclasses.dataclass(frozen=True)
class Result:
    """The contents of a single result archive.

    Attributes:
        stats: Aggregate statistics, e.g. ``rmse``. May be non-finite.
        np_arrays: Per-sample arrays keyed by name, e.g. ``error_array``. Which
            ones are present depends on the metric and the alignment mode, so
            treat any given one as optional.
        info: Presentation metadata, e.g. ``title``. Opaque; don't parse it.
    """

    stats: dict[str, float]
    np_arrays: dict[str, npt.NDArray[Any]]
    info: dict[str, Any] = dataclasses.field(default_factory=dict)


def read_result(path: Path | str) -> Result:
    """Read a result archive.

    Members other than the two JSON ones and ``*.npy`` are ignored, so archives
    from a newer evo still load. A missing ``path`` raises the usual
    :class:`FileNotFoundError`, to tell "no result" from "unreadable result".

    Args:
        path: Path to the archive.

    Returns:
        The parsed :class:`Result`.

    Raises:
        ResultFormatError: If the archive is not a zip, lacks a required
            member, holds malformed JSON or arrays, or is implausibly large.
    """
    path = Path(path)
    try:
        archive = zipfile.ZipFile(path)
    except zipfile.BadZipFile as exc:
        raise ResultFormatError(f"{path}: not a valid zip archive") from exc

    with archive:
        size = sum(entry.file_size for entry in archive.infolist())
        if size > MAX_UNCOMPRESSED_BYTES:
            raise ResultFormatError(
                f"{path}: unpacks to {size} bytes, "
                f"over the {MAX_UNCOMPRESSED_BYTES} byte limit"
            )
        info = _read_json(archive, path, INFO_MEMBER)
        raw_stats = _read_json(archive, path, STATS_MEMBER)
        np_arrays = {
            name[: -len(ARRAY_SUFFIX)]: _read_array(archive, path, name)
            for name in archive.namelist()
            if name.endswith(ARRAY_SUFFIX)
        }

    try:
        stats = {str(key): float(value) for key, value in raw_stats.items()}
    except (TypeError, ValueError) as exc:
        raise ResultFormatError(f"{path}: {STATS_MEMBER} is not numeric") from exc

    if not np_arrays:
        logger.debug("%s: archive holds no arrays", path)
    return Result(stats=stats, np_arrays=np_arrays, info=info)


def write_result(path: Path | str, result: Result) -> None:
    """Write ``result`` to a result archive.

    evo's own tools read the output, but it is not a byte-for-byte copy of
    what evo writes: values, names and dtypes round-trip, bytes do not.

    Args:
        path: Destination path, overwritten if it exists.
        result: The result to serialize.

    Raises:
        ResultFormatError: If an array cannot be stored without pickling.
    """
    path = Path(path)
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.writestr(INFO_MEMBER, json.dumps(result.info))
        archive.writestr(STATS_MEMBER, json.dumps(result.stats))
        for name, array in result.np_arrays.items():
            buffer = io.BytesIO()
            try:
                np.save(buffer, np.asarray(array), allow_pickle=False)
            except ValueError as exc:
                raise ResultFormatError(
                    f"{path}: array {name} needs pickling to store"
                ) from exc
            archive.writestr(f"{name}{ARRAY_SUFFIX}", buffer.getvalue())


def _read_json(archive: zipfile.ZipFile, path: Path, member: str) -> dict[str, Any]:
    """Read a required JSON object member."""
    try:
        raw = archive.read(member)
    except KeyError as exc:
        raise ResultFormatError(f"{path}: missing required member {member}") from exc
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ResultFormatError(f"{path}: {member} is not valid UTF-8 JSON") from exc
    if not isinstance(payload, dict):
        raise ResultFormatError(f"{path}: {member} must hold a JSON object")
    return payload


def _read_array(archive: zipfile.ZipFile, path: Path, member: str) -> npt.NDArray[Any]:
    """Read one .npy member, refusing anything that would need unpickling."""
    try:
        array = np.load(io.BytesIO(archive.read(member)), allow_pickle=False)
    except (ValueError, EOFError, OSError) as exc:
        raise ResultFormatError(f"{path}: {member} is not a readable array") from exc
    if not isinstance(array, np.ndarray):
        raise ResultFormatError(f"{path}: {member} does not hold an array")
    return array
