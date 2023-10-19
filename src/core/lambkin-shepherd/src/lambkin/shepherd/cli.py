# Copyright 2022 Ekumen, Inc.
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

"""This module contains the lambkin CLI."""

import argparse

import os
import pathlib
import shutil


def robot(args):
    """Robotframework CLI as a subcommand."""
    if '--' in args.remaining_args:
        args.remaining_args.remove('--')
    if '-d' not in args.remaining_args and \
       '--outputdir' not in args.remaining_args:
        args.remaining_args.extend(['-d', args.file.stem])
    if args.skip_all:
        args.remaining_args.extend(['-v', 'BENCHMARK.SUITE.SKIP:True'])
    path_to_executable = shutil.which('robot')
    args = [path_to_executable] + args.remaining_args + [args.file]
    os.execv(path_to_executable, args)


def main(argv=None):
    """Entrypoint for lambkin CLI."""
    parser = argparse.ArgumentParser(
        description='Localization And Mapping Benchmarking Toolkit CLI')
    parser.set_defaults(subcommand=None)
    subparsers = parser.add_subparsers()
    robot_subparser = subparsers.add_parser(
        'robot', description='Wrapper for RobotFramework CLI')
    robot_subparser.add_argument(
        '-f', '--file', type=pathlib.PurePath, required=True,
        help='Path to robot file, useful in shebang lines.')
    robot_subparser.add_argument(
        '--skip-all', action='store_true', default=False,
        help='Whether to skip all benchmark work (and e.g. only report)')
    robot_subparser.set_defaults(subcommand=robot)

    args, unknown = parser.parse_known_args(argv)
    args.remaining_args = unknown
    if not args.subcommand:
        parser.print_help()
        return 0
    return args.subcommand(args)
