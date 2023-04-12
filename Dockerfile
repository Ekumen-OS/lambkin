# syntax=docker/dockerfile:1.4
# Copyright 2023 Ekumen, Inc.
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

ARG baseimage=baseimage
FROM ${baseimage}

# Install bare minimum dependencies
RUN if command -v apt-get > /dev/null; then \
    apt-get update; \
    apt-get install -y adduser bash coreutils curl git python3 python3-pip sed tar; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration locales; \
    locale-gen en_US.UTF-8; \
    # Python 3.x build dependencies follow:
    apt-get install -y build-essential pkg-config libbz2-dev libffi-dev libgdbm-dev \
      libgdbm-compat-dev liblzma-dev libncurses5-dev libreadline6-dev libsqlite3-dev \
      libssl-dev lzma lzma-dev tk-dev uuid-dev zlib1g-dev; \
else \
    echo "Unknown package manager"; \
    exit 1; \
fi

# Install pyenv for root
RUN curl https://pyenv.run | bash
RUN cat <<EOF >> ~/.bash_profile
[ -f "~/.profile" ] && . ~/.profile
export PYENV_ROOT=\$HOME/.pyenv
export PATH="\$PYENV_ROOT/bin:\$PATH"
eval "\$(pyenv init -)"
EOF

# Switch to bash as shell
SHELL ["/bin/bash", "--login", "-c"]

# Install Python 3.9
RUN pyenv install 3.9

# Install pip package manager
RUN curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py && \
    pyenv shell 3.9 && python3.9 /tmp/get-pip.py

# Install Ansible
RUN pyenv shell 3.9 && pip3.9 install ansible

# Ensure host-to-container uid/gid compatibility
ENV USERNAME lambkin
RUN addgroup --gid 1000 $USERNAME && \
    adduser --uid 1000 --ingroup $USERNAME \
      --home /home/$USERNAME --shell /bin/bash $USERNAME && \
    echo "$USERNAME:$USERNAME" | chpasswd && adduser $USERNAME sudo && \
    echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.5.1/fixuid-0.5.1-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && chmod 4755 /usr/local/bin/fixuid && mkdir -p /etc/fixuid && \
    echo -e "user: $USERNAME\ngroup: $USERNAME\npaths: ['/home/$USERNAME']" | cat - > /etc/fixuid/config.yml

# Provision container as a lambkin runner
ARG configuration=
ARG ansible_flags=
RUN --mount=type=bind,source=.,target=/tmp/context,readonly pyenv shell 3.9 && \
    cd /tmp/context && ansible-galaxy collection install .ansible && \
    ansible-playbook -u $USERNAME -e "repository_uri=file:///tmp/context" \
    -e "${configuration}" ${ansible_flags} ekumenlabs.lambkin.local_runner

USER $USERNAME
ENV HOME /home/$USERNAME
WORKDIR $HOME

ENTRYPOINT ["fixuid", "-q", "/bin/bash", "--login", "-c"]
