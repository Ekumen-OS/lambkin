"""Trajectory evaluation via the evo library.

Reads trajectory files produced during a benchmark run
and computes standard accuracy metrics using evo:

- APE (Absolute Pose Error): measures global trajectory drift
- RPE (Relative Pose Error): measures local pose-to-pose consistency
- RSE (evo_res): processes and displays evaluation results across multiple runs,
enabling comparison and aggregated reporting

Evaluation results and reports are written out after each run,
providing a consistent record of benchmark performance.
"""
