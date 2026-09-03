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

# Plain overlay, importable without flakes:
#
#     nixpkgs.overlays = [ (import ./nix/overlay.nix) ];
final: prev: {
  # Library forms, on every Python version, so downstream can compose them into
  # an environment of its own.
  pythonPackagesExtensions = prev.pythonPackagesExtensions ++ [
    (python-final: _python-prev: {
      lambkin = python-final.callPackage ./lambkin.nix { };
      evo = python-final.callPackage ./evo.nix { };
    })
  ];

  # Separate top-level attributes on purpose: LAMBKIN only invokes evo_* as
  # external processes on PATH, so evo's GPL-3 closure never meets LAMBKIN's.
  lambkin = final.callPackage ./lambkin-cli.nix { };
  evo = final.python3.pkgs.toPythonApplication final.python3.pkgs.evo;
}
