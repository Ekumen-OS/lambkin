import os

from rocker.extensions import RockerExtension


class Local(RockerExtension):

    name = 'local'

    @classmethod
    def get_name(cls):
        return cls.name

    def precondition_environment(self, cli_args):
        pass

    def validate_environment(self, cli_args):
        pass

    def get_preamble(self, cli_args):
        return ''

    def get_snippet(self, cli_args):
        return ''

    def get_docker_args(self, cli_args):
        cwd = os.getcwd()
        return ' '.join(['', '-v', f'{cwd}:{cwd}', '-w', cwd])

    @staticmethod
    def register_arguments(parser):
        parser.add_argument(
            '--local', action='store_true', default=False,
            help='run on working directory within container')
