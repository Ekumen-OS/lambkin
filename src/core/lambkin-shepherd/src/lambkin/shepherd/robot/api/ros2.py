# Copyright 2023 Ekumen, Inc.
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

"""This module supplements RobotFramework ROS 2 library resource."""

from argparse import Namespace
from typing import Any, Iterable

import os.path
import yaml

from robot.api import logger
from robot.api.deco import keyword

from importlib import import_module

from lambkin.shepherd.utilities import deepdict


def get_node_parameters(
    *, node, node_name: str, parameter_names: Iterable[str]
) -> Iterable[str]:
    """
    Get multiple parameter values from an specific node.

    :param node: local node to query the ROS 2 graph.
    :param node_name: name of the target node.
    :param parameter_names: the names of the parameters to fetch.
    :returns: an iterable over parameter values, in the same order
      parameter names were given.
    """
    ros2param = import_module('ros2param.api')
    response = ros2param.call_get_parameters(
        node=node, node_name=node_name,
        parameter_names=parameter_names)
    return [
        ros2param.get_value(
            parameter_value=parameter_value)
        for parameter_value in response.values]


def get_node_parameter(
    *, node, node_name: str, parameter_name: str
) -> Any:
    """
    Get a single parameter value from an specific node.

    See `get_node_parameters` documentation for further reference.
    """
    return next(iter(get_node_parameters(
        node=node, node_name=node_name,
        parameter_names=[parameter_name])))


def list_nodes_with_parameters(*, node) -> Iterable[str]:
    """
    List the names of all the nodes in the ROS 2 graph that declare parameters.

    :param node: local node to query the ROS 2 graph.
    :returns: an iterable over node names.
    """
    ros2node = import_module('ros2node.api')
    ros2service = import_module('ros2service.api')
    service_names = {name for name in ros2service.get_service_names(node=node)}
    return [
        n.full_name for n in ros2node.get_node_names(node=node)
        if f'{n.full_name}/get_parameters' in service_names]


@keyword('Dump ROS 2 Parameters')
def dump_ros_2_parameters(
    file_path: str = 'parameters.yaml', cwd: str = '.', **kwargs: Any
) -> None:
    """
    Write ROS 2 parameters from each node to a YAML file.

    :param file_path: path to file to write to.
    :param cwd: optional working directory.
    """
    ros2cli = import_module('ros2cli.node.strategy')
    ros2param = import_module('ros2param.api')
    with ros2cli.NodeStrategy(Namespace(**kwargs)) as node:
        output = deepdict()
        for node_name in list_nodes_with_parameters(node=node):
            parameter_names = (
                ros2param.call_list_parameters(
                    node=node, node_name=node_name))
            parameter_values = get_node_parameters(
                node=node, node_name=node_name,
                parameter_names=parameter_names)
            node_output = deepdict()
            for name, value in zip(parameter_names, parameter_values):
                node_output[name.split(ros2param.PARAMETER_SEPARATOR_STRING)] = value
            output[node_name, 'ros__parameters'] = node_output
        with open(os.path.join(cwd, file_path), 'w') as f:
            yaml.dump(output, f, default_flow_style=False)


@keyword('Set ROS 2 Parameter')
def set_ros_2_parameter(
    name: str, value: Any, *node_names: str, **kwargs: Any
) -> None:
    """
    Set a ROS 2 parameter on a set of nodes.

    :param name: name of the parameter.
    :param value: value of the parameter.
    :param node_names: names of the nodes to target.
      All nodes known will be targeted if none is specified.
    """
    ros2cli = import_module('ros2cli.node.strategy')
    ros2param = import_module('ros2param.api')
    rcl_interfaces = import_module('rcl_interfaces.msg')
    parameter = rcl_interfaces.Parameter(name=name)
    parameter.value = ros2param.get_parameter_value(string_value=str(value))
    with ros2cli.NodeStrategy(Namespace(**kwargs)) as node:
        known_node_names = list_nodes_with_parameters(node=node)
        if not node_names:
            node_names = tuple(known_node_names)
        for node_name in node_names:
            response = ros2param.call_set_parameters(
                node=node, node_name=node_name, parameters=[parameter])
            result = response.results[0]
            assert result.successful, (
                "failed to set parameter on "
                f"{node_name} due to {result.reason}")


@keyword('Warn If ROS 2 Nodes Are Not Using Sim Time')
def warn_if_ros_2_nodes_are_not_using_sim_time(*node_names: str, **kwargs: Any) -> bool:
    """Log a warning to console if a given ROS 2 node is not using simulation time."""
    ros2cli = import_module('ros2cli.node.strategy')
    with ros2cli.NodeStrategy(Namespace(**kwargs)) as node:
        known_node_names = list_nodes_with_parameters(node=node)
        if not node_names:
            node_names = tuple(known_node_names)
        all_are_using_sim_time = True
        for node_name in node_names:
            if node_name in known_node_names:
                use_sim_time = get_node_parameter(
                    node=node, node_name=node_name,
                    parameter_name='use_sim_time')
                if not use_sim_time:
                    logger.console(f'\n[!] {node_name} is not using sim time')
                    all_are_using_sim_time = False
            else:
                logger.console(f'\n[!] {node_name} cannot be found')
                all_are_using_sim_time = False
        return all_are_using_sim_time


@keyword('Warn If ROS 2 Bag Is Compressed')
def warn_if_ros_2_bag_is_compressed(bag_path: str) -> bool:
    """
    Log a warning to console if the given ROS 2 bag is compressed.

    :param bag_path: path to ROS 2 bag to be checked.
    """
    rosbag2_py = import_module('rosbag2_py')
    metadata = rosbag2_py.Info().read_metadata(bag_path, "")
    compression_mode = metadata.compression_mode
    if compression_mode != "":
        logger.console(
            f'\n[!] {bag_path} has {compression_mode}'
            ' compression, performance may be degraded')
        return True
    return False
