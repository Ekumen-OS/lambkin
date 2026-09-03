# Copyright 2026 Ekumen, Inc.
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

{ pkgs }:

let
  inherit (pkgs) lib;
  fs = lib.fileset;

  # Filters by exclusion rather than picking .py files: test/data reads binary
  # evo archives out of test/data/fixtures/. pyproject.toml carries
  # [tool.pytest.ini_options].
  testSource = fs.toSource {
    root = ../.;
    fileset = fs.unions [
      ../pyproject.toml
      (fs.difference ../test (fs.fileFilter (file: file.hasExt "pyc") ../test))
    ];
  };

  pytestEnv = pkgs.python3.withPackages (ps: [
    ps.lambkin
    ps.pytest
    ps.pytest-repeat
  ]);
in
{
  # Building the packages exercises the uv_build patch and every import check.
  inherit (pkgs) lambkin evo;

  # Click handles --help before the CLI touches a cgroup, so this is sandbox-safe.
  lambkin-cli = pkgs.runCommand "lambkin-cli-smoke" { } ''
    ${lib.getExe pkgs.lambkin} --help > /dev/null
    touch $out
  '';

  # Also asserts the GPL-3 obligations are met by the output itself.
  evo-cli = pkgs.runCommand "evo-cli-smoke" { } ''
    export HOME="$TMPDIR"
    export MPLBACKEND=Agg
    ${pkgs.evo}/bin/evo_ape --help > /dev/null
    ${pkgs.evo}/bin/evo_traj --help > /dev/null
    test -s ${pkgs.evo}/share/doc/evo/LICENSE
    test -s ${pkgs.evo}/share/doc/evo/evo-${pkgs.evo.version}-source.tar.gz
    touch $out
  '';

  # Only the cgroup-free subset; the rest stays with the podman pytest workflow,
  # which sets up a writable cgroup v2 hierarchy.
  pytest = pkgs.runCommand "lambkin-pytest" { } ''
    cp -r ${testSource}/. work
    chmod -R u+w work
    cd work
    export HOME="$TMPDIR"
    ${pytestEnv}/bin/pytest -q --no-header -p no:cacheprovider \
      test/common test/utils test/data
    touch $out
  '';
}
