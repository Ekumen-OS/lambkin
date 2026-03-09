"""Non-blocking process execution for lambkin benchmarks.

Wraps subprocess.Popen to launch processes in the background,
allowing the benchmark to continue while the process runs.
Useful for starting long-running services or ROS nodes that
must run alongside the benchmark.
"""
