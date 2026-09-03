# Nix packaging

`flake.nix` exposes two packages:

| Output | Licence | Notes |
|---|---|---|
| `packages.<system>.lambkin` | Apache-2.0 | The SDK and its `lambkin` CLI. |
| `packages.<system>.evo` | GPL-3.0-or-later | [`evo`](https://github.com/MichaelGrupp/evo). Not in nixpkgs, so the derivation lives in [`evo.nix`](evo.nix). |

They are independent on purpose: LAMBKIN only invokes `evo_ape` / `evo_rpe` /
`evo_traj` as external processes on `PATH`, so installing one never pulls in the
other.

```bash
nix build .#lambkin
nix build .#evo
nix flake check
```

[`overlay.nix`](overlay.nix) is a plain `final: prev:` function, so it also works
without flakes:

```nix
nixpkgs.overlays = [ (import ./nix/overlay.nix) ];
```

It adds `lambkin` and `evo` as top-level CLIs, plus their library forms to every
Python version in the package set (`python312.pkgs.lambkin`, and so on).

## Verifying without Nix installed

[`docker/`](docker) holds a pinned, Nix-capable image, so anyone with Docker can
reproduce the build. Nothing is installed on the host.

```bash
./nix/docker/run.sh image                            # build the harness image
./nix/docker/run.sh build .#lambkin
./nix/docker/run.sh build .#evo
./nix/docker/run.sh flake check --all-systems
./nix/docker/run.sh flake lock --no-update-lock-file # assert flake.lock is current
./nix/docker/run.sh smoke                            # build both, run --help on each
./nix/docker/run.sh shell                            # interactive bash
```

`run.sh` is a thin wrapper. The equivalent raw invocation is:

```bash
docker build -t lambkin-nix:2.35.2 -f nix/docker/Dockerfile nix/docker

docker run --rm -it \
  -v lambkin-nix-store-2.35.2-native:/nix \
  -v lambkin-nix-cache-2.35.2-native:/root/.cache/nix \
  -v "$PWD:/workspace" \
  -w /workspace \
  lambkin-nix:2.35.2 \
  nix build --no-link --print-out-paths .#lambkin
```

The `/nix` volume persists the store; without it every build re-downloads its
whole closure. Reclaim it with `./nix/docker/run.sh gc`, or drop the volumes with
`./nix/docker/run.sh clean`.

CI runs this same script, so a change that breaks it fails the `nix` workflow.

## Gotchas

* **Untracked files are invisible to the flake.** Nix reads the git tree, so a new
  `flake.nix` or `nix/*.nix` must be `git add`ed first. Staging is enough.
* **`result` symlinks.** The commands above pass `--no-link --print-out-paths`. A
  plain `nix build` creates `./result` as root, pointing into the container's
  `/nix`, so it dangles on the host. It is gitignored.
* **Root-owned files.** `nix flake lock` writes `flake.lock` as root into the bind
  mount; `./nix/docker/run.sh fix-ownership` hands the worktree back.
* **The build sandbox is off by default**, because it needs `CAP_SYS_ADMIN`.
  `LAMBKIN_NIX_SANDBOX=1` re-enables it under `--privileged`, as CI does.
* **Architecture.** The harness runs natively. `LAMBKIN_NIX_PLATFORM=linux/amd64`
  reproduces CI's store paths, at the cost of qemu emulation.
* **Running benchmarks needs cgroup v2 delegation**, which a plain `docker run`
  does not provide. Use `--privileged --cgroupns=private`, or the setup in
  [`examples/beluga`](../examples/beluga).
