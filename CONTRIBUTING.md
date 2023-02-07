# How to contribute to Beluga

Thank you for investing your time in contributing to this project!

## Contributions

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](./LICENSE):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).

## Getting started

### Run benchmarks

This is an example on how to run an existing benchmark using the provided `slam_toolbox_benchmark` package.

1. Build `lambkin` and `slam_toolbox_benchmark` docker images.

    ```bash
    make -C <PATH_TO_LAMBKIN_INSTALLATION>/src/core/lambkin/docker/
    make -C <PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/
    ```

2. Change to a directory containing the required datasets in ROS 1 bag format. For this example, you'll need the Magazino robot
[hallway_return.bag](https://storage.googleapis.com/cartographer-public-data/bags/toru/hallway_return.bag) dataset from
[Cartographer's Public Data repository](https://google-cartographer-ros.readthedocs.io/en/latest/data.html#magazino).

    ```bash
    cd <PATH_TO_FOLDER_WITH_DATASET>
    ```

    The current directory will be bound to `/root/work` inside the container.

3. Execute the benchmark pipeline with this command:

    ```bash
    <PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot
    ```

4. After the run, you should see the following folder structure in your current directory:

    ```bash
    ├── magazino_benchmark/
    │    ├── tests/                  # LAMBKIN output artifacts
    │    │    └── hallway_return/
    │    │         ├── runs/
    │    │         │    ├── 1/
    │    │         │    ├── 2/
    │    │         │    ├── 3/
    │    │         │    └── ...
    │    │         ├── metadata.json # Test case metadata
    │    │         └── ...
    │    ├── report.pdf              # LAMBKIN benchmark report
    │    ├── output.xml              # RobotFramework test execution information
    │    ├── report.html             # RobotFramework test report
    │    ├── log.html                # RobotFramework logs
    │    └── ...
    └── hallway_return.bag           # Input dataset
    ```

### Issues

#### Create a new issue

If you spot a problem or have a feature request you'd like to discuss, [search if an issue already exists](https://docs.github.com/en/github/searching-for-information-on-github/searching-on-github/searching-issues-and-pull-requests#search-by-the-title-body-or-comments).
If a related issue doesn't exist, you can [open a new issue](https://github.com/ekumenlabs/beluga/issues/new/choose).

### Make changes

If you want to change the benchmark definition (amount of iterations, duration of each run, SLAM system parameters, etc.)
or the way benchmarking tools are implemented, in principle, you will need to rebuild the corresponding docker images.

In order to make the development process easier and skip the image rebuilding step, we provide an option to bind mount any modified package using [docker volumes](https://docs.docker.com/storage/volumes/).
For instance, if you are changing the `lambkin` package itself, you could use the following command to run the previous benchmark with your changes:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot --rebuild --bind lambkin:<PATH_TO_LAMBKIN_INSTALLATION>/src/core/lambkin
```

The `--rebuild` option will rebuild the `catkin` workspace for the changes to take effect before execution.

There is a shortcut to bind mount the same benchmark package you are running:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot --rebuild --rebind
```

And you can mount more than one package at your convenience:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run magazino_benchmark.robot ---rebuild --rebind --bind lambkin:<PATH_TO_LAMBKIN_INSTALLATION>/src/core/lambkin
```

#### Make changes locally

1. Clone the repository.
   ```bash
   git clone git@github.com:ekumenlabs/lambkin.git
   ```

2. Create a working branch and start with your changes. The suggested branch name convention is `<user_name>/<feature_name>`.

3. Push your changes and [create a PR](https://github.com/ekumenlabs/lambkin/compare)!

4. At the time a feature branch is squashed-and-merged into `main`, the commit message should adhere to the following good practices:
   - Limit the subject line to 50 characters.
   - Capitalize the subject line.
   - Do not end the subject line with a period.
   - Use the imperative mood in the subject line.
   - Wrap the body at 72 characters.
   - Use the body to explain _what_ and _why_ vs. _how_.
   - See https://cbea.ms/git-commit/ for more information and the reasoning behind this.

#### Using the interactive mode

When developing, we found useful to test commands before adding them to the automation pipeline. Instead of running a `RobotFramework` script,
you can use the `--interactive` flag, which gives you access to a command-line interface attached to a container already configured to run GUI apps. Running the following command on different terminal windows will give you access to the same container:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run --interactive
```

You might want to combine this option with [package binding](#changing-an-existing-benchmark) to use your preferred IDE in the host while testing your changes in the running container:

```bash
<PATH_TO_LAMBKIN_INSTALLATION>/src/benchmarks/slam_toolbox_benchmark/docker/run --interactive --rebuild --rebind --bind lambkin:<PATH_TO_LAMBKIN_INSTALLATION>/src/core/lambkin
```

Once inside the container, you can run a benchmark using the `rosrun` command:

```
root@lambkin_slam_toolbox:~/work# rosrun slam_toolbox_benchmark magazino_benchmark.robot
```

Use this command to rebuild the `catkin` workspace for your changes to take effect:

```
root@lambkin_slam_toolbox:~/work# catkin_make_isolated --install -C /root/ws
```