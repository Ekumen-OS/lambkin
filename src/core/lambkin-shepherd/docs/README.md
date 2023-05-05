# `lambkin.shepherd` documentation

**Note**: only HTML format documentation is supported.

## Usage

1. Install documentation dependencies:

  ``` sh
  cd path/to/lambkin-shepherd
  poetry install --with docs --no-root
  ```

2. Build HTML documentation:

  ```sh
  cd docs
  make html
  ```

3. Browse documentation:

  ```sh
  xdg-open _build/html/index.html
  ```
