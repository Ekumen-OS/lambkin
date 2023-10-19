# `lambkin.clerk` documentation

## Usage

1. Install documentation dependencies:

  ``` sh
  cd path/to/lambkin-clerk
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
