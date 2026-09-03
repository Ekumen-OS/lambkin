#!/usr/bin/env bash
# Run Nix inside a pinned, Nix-capable container against this worktree.
#
# Usage:
#   ./nix/docker/run.sh image                 Build (or rebuild) the harness image
#   ./nix/docker/run.sh build .#lambkin       Build a flake output
#   ./nix/docker/run.sh build .#evo
#   ./nix/docker/run.sh flake check           Evaluate and build every flake check
#   ./nix/docker/run.sh shell                 Interactive bash inside the container
#   ./nix/docker/run.sh smoke                 Build both packages and run their CLIs
#   ./nix/docker/run.sh gc                    Garbage-collect the persistent store
#   ./nix/docker/run.sh fix-ownership         Reclaim files root created in the mount
#   ./nix/docker/run.sh clean                 Delete the persistent volumes
#   ./nix/docker/run.sh <args...>             Anything else runs as `nix <args...>`
#
# Environment overrides:
#   LAMBKIN_NIX_IMAGE      image tag to use       (default: lambkin-nix:<version>)
#   LAMBKIN_NIX_PLATFORM   docker platform        (default: the host's native one)
#   LAMBKIN_NIX_SANDBOX    1 enables Nix's build sandbox; implies --privileged

set -euo pipefail

NIX_VERSION=2.35.2
IMAGE=${LAMBKIN_NIX_IMAGE:-lambkin-nix:${NIX_VERSION}}
REPO_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)

# Native by default: forcing linux/amd64 puts arm64 developers behind qemu. Set
# LAMBKIN_NIX_PLATFORM=linux/amd64 to reproduce CI's store paths exactly.
PLATFORM=${LAMBKIN_NIX_PLATFORM:-}

# The persisted /nix holds the nix binary's own store path and arch-specific
# outputs, so volumes must not be shared across Nix versions or platforms.
platform_tag=$(printf '%s' "${PLATFORM:-native}" | tr '/:' '--')
STORE_VOLUME=lambkin-nix-store-${NIX_VERSION}-${platform_tag}
CACHE_VOLUME=lambkin-nix-cache-${NIX_VERSION}-${platform_tag}

platform_args=()
if [ -n "${PLATFORM}" ]; then
  platform_args=(--platform "${PLATFORM}")
fi

build_image() {
  docker build \
    "${platform_args[@]}" \
    --tag "${IMAGE}" \
    --file "${REPO_ROOT}/nix/docker/Dockerfile" \
    "${REPO_ROOT}/nix/docker"
}

run() {
  docker image inspect "${IMAGE}" >/dev/null 2>&1 || build_image

  local tty_args=()
  if [ -t 0 ] && [ -t 1 ]; then
    tty_args=(--interactive --tty)
  fi

  local sandbox_args=()
  if [ "${LAMBKIN_NIX_SANDBOX:-0}" = "1" ]; then
    # Needs CAP_SYS_ADMIN to unshare a mount namespace. Narrower alternative:
    #   --cap-add SYS_ADMIN --security-opt seccomp=unconfined \
    #   --security-opt apparmor=unconfined
    sandbox_args=(--privileged --env 'NIX_CONFIG=sandbox = true')
  fi

  docker run --rm \
    "${tty_args[@]}" \
    "${platform_args[@]}" \
    "${sandbox_args[@]}" \
    --volume "${STORE_VOLUME}:/nix" \
    --volume "${CACHE_VOLUME}:/root/.cache/nix" \
    --volume "${REPO_ROOT}:/workspace" \
    --workdir /workspace \
    "${IMAGE}" \
    "$@"
}

case "${1-}" in
  image)
    build_image
    ;;
  shell)
    shift
    run bash "$@"
    ;;
  smoke)
    run bash -euo pipefail -c '
      lambkin=$(nix build --no-link --print-out-paths .#lambkin)
      evo=$(nix build --no-link --print-out-paths .#evo)
      echo "lambkin -> $lambkin"
      echo "evo     -> $evo"
      "$lambkin/bin/lambkin" --help
      "$evo/bin/evo_ape" --help
    '
    ;;
  gc)
    shift
    run nix store gc "$@"
    ;;
  fix-ownership)
    # nix runs as root in the bind mount; hand what it wrote back to the host.
    run chown -R "$(id -u):$(id -g)" /workspace
    ;;
  clean)
    docker volume rm --force "${STORE_VOLUME}" "${CACHE_VOLUME}"
    ;;
  "" | -h | --help | help)
    sed -n '2,23p' "${BASH_SOURCE[0]}"
    exit 2
    ;;
  *)
    run nix "$@"
    ;;
esac
