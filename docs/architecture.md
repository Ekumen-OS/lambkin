# Architecture

The packages found in LAMBKIN fall into one of these three categories:

- [Core packages](../src/core) (e.g. `lambkin-shepherd`) with common tools shared by all benchmarks.
- [Tool packages](../src/tools) not needed during benchmarking (i.e. dataset format convertion tools).
- [Benchmark packages](../src/benchmarks) that target a specific SLAM system (i.e. Cartographer ROS, SLAM Toolbox).

TBD

## Benchmark sequence diagram

The following diagram shows a typical SLAM benchmark running inside a docker container:

```mermaid
sequenceDiagram
  participant C as Docker Container
  participant F as Robot Framework
  participant P as Play Process
  participant S as SLAM System
  participant R as Record Process
  C->>F: Start benchmark
  activate F
  loop for each test case
    loop x10
      Note over F: Bring up ROS core
      F->>R: Start recording
      activate R
      F->>S: Bring up SLAM system
      activate S
      F->>+P: Start playing dataset
      deactivate F
      P-->>-F: Finished playing dataset
      activate F
      F->>S: Shutdown SLAM system
      deactivate S
      F->>R: Stop recording
      deactivate R
      Note over F: Shutdown ROS core
    end
  end
  Note over F: Generate report
  F-->>C: Write report and logs
  deactivate F
```
