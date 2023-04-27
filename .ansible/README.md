# LAMBKIN Collection for Ansible

An Ansible collection to provision a platform for `lambkin` development and usage.

## Contributing to this collection

## Tested with Ansible

Ansible 7.4.0 (ansible-core 2.14.4)

## External requirements

This collection assumes that `python3` and `pip3` are available in the remote host.

Currently, only POSIX OSes are supported. It is assumed that `sudo` is available for privilege escalation.

## Included content

### Playbooks

#### [ekumenlabs.lambkin.local_runner](playbooks/local_runner.yml)

This playbook provisions the local host to run LAMBKIN-powered benchmarks.

### Roles

#### [ekumenlabs.lambkin.runner](roles/runner)

This role provisions the remote host to run LAMBKIN-powered benchmarks.

By default, all available components are installed. These components may be explicitly included
and excluded via `include_components` and `exclude_components` (list) variables, respectively.

## Using this collection

### Installing from source

```sh
git clone https://github.com/ekumenlabs/lambkin
cd lambkin && ansible-galaxy collection install .ansible
```

### Provisioning local host

```sh
ansible-playbook ekumenlabs.lambkin.local_runner
```

## More information

- [Ansible Collection overview](https://github.com/ansible-collections/overview)
- [Ansible User guide](https://docs.ansible.com/ansible/devel/user_guide/index.html)
- [Ansible Developer guide](https://docs.ansible.com/ansible/devel/dev_guide/index.html)
- [Ansible Community code of conduct](https://docs.ansible.com/ansible/devel/community/code_of_conduct.html)
- [News for Maintainers](https://github.com/ansible-collections/news-for-maintainers)

## Licensing

Copyright 2023 Ekumen, Inc.

Apache License 2.0

See [LICENSE](../LICENSE) to see full text.
