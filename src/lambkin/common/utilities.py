"""Common utility functions for lambkin."""

import itertools
from collections.abc import Iterable
from typing import Any


def safe_merge(base, head):
    """Perform a shallow merge of head into base.

    Args:
        base: target mapping for merge operation.
        head: source mapping for merge operation.

    Returns:
        Merged mapping.

    Raises:
        ValueError: whenever a key would be overwritten.
    """
    for key in head:
        if key in base:
            raise ValueError(f"{key} would be overwritten by merge")
    return {**base, **head}


def enforce_nonempty(
    it: Iterable[Any], msg: str = "iterable is empty"
) -> Iterable[Any]:
    """Enforce iterable is not empty.

    Args:
        it: iterable to check.
        msg: error message if empty.

    Returns:
        The iterable.

    Raises:
        ValueError: if the iterable is empty.
    """
    it = iter(it)
    try:
        item = next(it)
    except StopIteration:
        raise ValueError(msg) from None
    return itertools.chain([item], it)
