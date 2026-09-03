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
  description = "LAMBKIN - Localization And Mapping BenchmarKINg SDK";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";

  outputs =
    { self, nixpkgs }:
    let
      inherit (nixpkgs) lib;

      # LAMBKIN needs cgroup v2; there is no Darwin story.
      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      pkgsFor =
        system:
        import nixpkgs {
          inherit system;
          overlays = [ self.overlays.default ];
        };

      eachSystem = f: lib.genAttrs systems (system: f (pkgsFor system));
    in
    {
      overlays.default = import ./nix/overlay.nix;

      packages = eachSystem (pkgs: {
        inherit (pkgs) lambkin evo;
        default = pkgs.lambkin;
      });

      apps = eachSystem (pkgs: rec {
        lambkin = {
          type = "app";
          program = lib.getExe pkgs.lambkin;
          inherit (pkgs.lambkin) meta;
        };
        evo = {
          type = "app";
          program = lib.getExe pkgs.evo;
          inherit (pkgs.evo) meta;
        };
        default = lambkin;
      });

      devShells = eachSystem (pkgs: {
        # Mirrors the documented uv workflow. No evo: that stays a separate,
        # explicit install.
        default = pkgs.mkShell {
          packages = [
            pkgs.uv
            pkgs.python312
            pkgs.pre-commit
          ];
          env = {
            UV_PYTHON_DOWNLOADS = "never";
            UV_PYTHON = "${pkgs.python312}/bin/python3.12";
          };
          # A leaked PYTHONPATH would shadow uv's .venv.
          shellHook = ''
            unset PYTHONPATH
          '';
        };
      });

      checks = eachSystem (pkgs: import ./nix/checks.nix { inherit pkgs; });

      formatter = eachSystem (pkgs: pkgs.nixfmt-tree);
    };
}
