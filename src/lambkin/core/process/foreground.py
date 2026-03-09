"""Blocking process execution for lambkin benchmarks.

Wraps subprocess.run to execute processes in the foreground,
waiting for completion before continuing.Useful for commands
that must finish before the next benchmark step proceeds.
"""
