# Nix-capable image used to build and verify LAMBKIN's flake outputs.

FROM docker.io/nixos/nix:2.35.2@sha256:7a007c766426c1877758ddc5cb87a965ac131fc78c582ce0083d922d51ae945c

# Rewritten in full rather than appended to, so the whole configuration is
# visible in one place. sandbox = false is inherited on purpose: the sandbox
# needs CAP_SYS_ADMIN, which a plain `docker run` does not grant. Builds still
# run as the unprivileged nixbld users. LAMBKIN_NIX_SANDBOX=1 re-enables it.
RUN printf '%s\n' \
      'experimental-features = nix-command flakes' \
      'sandbox = false' \
      'build-users-group = nixbld' \
      'substituters = https://cache.nixos.org' \
      'trusted-public-keys = cache.nixos.org-1:6NCHdD59X431o0gWypbMrAURkbJ16ZPMQFGspcDShjY=' \
      'accept-flake-config = false' \
    > /etc/nix/nix.conf

RUN git config --global --add safe.directory '*'

WORKDIR /workspace

CMD ["bash"]
