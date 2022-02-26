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

import argparse

import os
import pathlib
import shutil

import lambkin.aggregate as aggregate

def robot(args):
    if '--' in args.remaining_args:
        args.remaining_args.remove('--')
    if '-d' not in args.remaining_args:
        args.remaining_args.extend(['-d', args.file.stem])
    path_to_executable = shutil.which('robot')
    args = [path_to_executable] + args.remaining_args + [args.file]
    os.execv(path_to_executable, args)

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Localization And Mapping Benchmarking CLI')
    subparsers = parser.add_subparsers()
    robot_subparser = subparsers.add_parser('robot')
    robot_subparser.add_argument(
        '-f', '--file', type=pathlib.PurePath, required=True,
        help='Path to robot file, useful in shebang lines.')
    robot_subparser.set_defaults(subcommand=robot)

    aggregate_subparser = subparsers.add_parser(
        'aggregate', parents=[aggregate.get_parser()], add_help=False)
    aggregate_subparser.set_defaults(subcommand=aggregate.main)

    args, unknown = parser.parse_known_args(argv)
    args.remaining_args = unknown
    return args.subcommand(args)
