# LAMBKIN `shepherd`

The main package for benchmark definition and orchestration.

To do so, `lambkin.shepherd` leverages many existing technologies such as:

- [ROS](https://www.ros.org/) for data exchange with localization and mapping systems
- [Robot Framework](https://robotframework.org/) for benchmark definition and orchestration
- [evo](https://michaelgrupp.github.io/evo/) for localization performance instrumentation
- [timem](https://timemory.readthedocs.io/en/develop/features.html#command-line-tools) for computational performance instrumentation
- [pandas](https://pandas.pydata.org/) and [seaborn](https://seaborn.pydata.org/) for metrics analysis visualization

And more. Most dependencies are optional though, and will not be imported if not required.
