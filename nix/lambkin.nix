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

{
  lib,
  buildPythonPackage,
  pythonOlder,
  uv-build,
  click,
  numpy,
  pyyaml,
}:

let
  fs = lib.fileset;

  # README.md and LICENSE are build inputs: pyproject.toml declares both.
  # examples/, test/ and doc/ are never named, so they are not shipped and
  # editing them does not rebuild this.
  source = fs.toSource {
    root = ../.;
    fileset = fs.unions [
      ../pyproject.toml
      ../README.md
      ../LICENSE
      # Filtering by extension drops __pycache__ without risking
      # src/lambkin/core/shell/ros/, which has no __init__.py.
      (fs.fileFilter (file: file.hasExt "py" || file.hasExt "md") ../src)
    ];
  };
in
buildPythonPackage {
  pname = "lambkin";
  version = "0.0.0";
  pyproject = true;

  disabled = pythonOlder "3.10";

  src = source;

  # nixpkgs ships uv-build 0.10.0; pyproject.toml asks for >=0.10.3,<0.11.0 and
  # pypaBuildHook enforces it. pythonRelaxDeps cannot help - it rewrites the
  # built wheel's METADATA, not [build-system].requires.
  postPatch = ''
    substituteInPlace pyproject.toml \
      --replace-fail '"uv_build>=0.10.3,<0.11.0"' '"uv_build"'
  '';

  build-system = [ uv-build ];

  # Declared in pyproject.toml but imported nowhere under src/.
  # TODO(xfranv8): remove once pyproject.toml stops declaring it.
  pythonRemoveDeps = [ "pydantic" ];

  dependencies = [
    click
    numpy
    pyyaml
  ];

  # The test suite needs a writable cgroup v2 hierarchy, which the sandbox has
  # not. The cgroup-free subset runs as a flake check; see checks.nix.
  doCheck = false;

  pythonImportsCheck = [
    "lambkin"
    "lambkin.cli"
    # Guards the __init__.py-less namespace directory against being dropped;
    # otherwise breakage only surfaces at ctx.shell.ros2.launch.
    "lambkin.core.shell.ros.launch"
  ];

  meta = {
    description = "A programmatic SLAM benchmarking SDK";
    homepage = "https://github.com/Ekumen-OS/lambkin";
    license = lib.licenses.asl20;
    mainProgram = "lambkin";
    platforms = lib.platforms.linux;
  };
}
