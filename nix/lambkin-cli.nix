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
  python3,
  writeShellScriptBin,
  # Extra Python packages that benchmark scripts should be able to import, e.g.
  #
  #     pkgs.lambkin.override { extraPythonPackages = ps: [ ps.matplotlib ]; }
  extraPythonPackages ? (_: [ ]),
}:

let
  pythonEnv = python3.withPackages (ps: [ ps.lambkin ] ++ extraPythonPackages ps);
in
# lambkin re-execs the benchmark script with sys.executable. Going through a
# withPackages environment makes that interpreter self-sufficient, so imports
# resolve from its site-packages rather than from a PYTHONPATH that would leak
# into ros2, evo_ape and everything else lambkin shells out to.
#
# prog_name is explicit because `python -c` leaves sys.argv[0] as "-c", which
# Click would print in usage and error messages.
(writeShellScriptBin "lambkin" ''
  exec ${pythonEnv}/bin/python \
    -c 'from lambkin.cli import main; main(prog_name="lambkin")' "$@"
'').overrideAttrs
  (old: {
    meta = (old.meta or { }) // {
      inherit (python3.pkgs.lambkin.meta)
        description
        homepage
        license
        platforms
        ;
      mainProgram = "lambkin";
    };
    passthru = { inherit pythonEnv; };
  })
