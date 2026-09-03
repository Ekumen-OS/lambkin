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

# evo is GPL-3.0-or-later and LAMBKIN is Apache-2.0, so it lives in its own
# derivation and its own top-level attribute: nothing in LAMBKIN's closure
# refers to it. LAMBKIN invokes it as an external process and never imports it.
#
# The header above covers this expression; the software it packages is licensed
# as recorded in meta.license below.
{
  lib,
  buildPythonPackage,
  fetchFromGitHub,
  hatchling,
  argcomplete,
  colorama,
  matplotlib,
  natsort,
  numexpr,
  numpy,
  packaging,
  pandas,
  pillow,
  pygments,
  pyyaml,
  rosbags,
  scipy,
  seaborn,
}:

buildPythonPackage rec {
  pname = "evo";
  # evo/__init__.py says "v1.35.2"; hatchling strips the leading "v".
  version = "1.35.2";
  pyproject = true;

  src = fetchFromGitHub {
    owner = "MichaelGrupp";
    repo = "evo";
    tag = "v${version}";
    hash = "sha256-m1hqD8kPNO22N4H8iIlG3kLI4gKpgh72FCPXCBsL1f0=";
  };

  build-system = [ hatchling ];

  # v1.35.2 asks for rosbags>=0.11.0, exactly what nixpkgs carries. Do not bump
  # to v1.36 or later until nixpkgs' rosbags reaches 0.11.1.
  dependencies = [
    argcomplete
    colorama
    matplotlib
    natsort
    numexpr
    numpy
    packaging
    pandas
    pillow
    pygments
    pyyaml
    rosbags
    scipy
    seaborn
  ];

  # evo.tools.settings writes Path.home() / ".evo" at import time, and the
  # sandbox HOME does not exist. preFixup rather than preCheck, because
  # pythonImportsCheckPhase runs even though doCheck is false.
  preFixup = ''
    export HOME="$TMPDIR"
  '';
  env.MPLBACKEND = "Agg";

  # Upstream's suite needs its own trajectory fixtures; checks.nix smoke-tests
  # the CLI instead, which is all LAMBKIN relies on.
  doCheck = false;

  pythonImportsCheck = [
    "evo"
    "evo.tools.settings"
    "evo.main_ape"
  ];

  # GPL-3 sections 3 and 6: the complete corresponding source and the licence
  # ship alongside the binaries. Built from the exact tree that was compiled,
  # with reproducible tar flags so the derivation stays deterministic.
  postInstall = ''
    install -Dm644 LICENSE README.md -t "$out/share/doc/evo"

    tar -czf "$out/share/doc/evo/evo-${version}-source.tar.gz" \
      --sort=name --owner=0 --group=0 --numeric-owner --mtime='@1' \
      --exclude=dist --exclude='__pycache__' --exclude='*.pyc' \
      --transform "s,^source,evo-${version}," \
      -C "$NIX_BUILD_TOP" source
  '';

  meta = {
    description = "Python package for the evaluation of odometry and SLAM";
    homepage = "https://github.com/MichaelGrupp/evo";
    changelog = "https://github.com/MichaelGrupp/evo/releases/tag/v${version}";
    # Per-file headers grant "version 3 ... or any later version".
    license = lib.licenses.gpl3Plus;
    mainProgram = "evo";
    platforms = lib.platforms.unix;
  };
}
