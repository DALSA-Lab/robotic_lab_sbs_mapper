
# Project library for C++ and Python Packages

In this template, there are two self-contained dummy packages:

- **`cpp_pkg`** — a C++ package
- **`py_pkg`** — a Python package

The goal is to provide a structured starting point for projects that combine **C++ performance** with **Python flexibility**, while maintaining clean separation, modularity, and self-containmenet. Having both types of packages at the end is not mandatory, but the ability to do so without putting any extra effort into setup provides flexibility when needed.

---

## Why Combine C++ and Python?

Combining C++ and Python in the same project allows you to:
- Use **C++** for performance-critical components (e.g., algorithms, device control).
- Use **Python** for scripting, orchestration, and data processing.
- Share logic across languages via bindings or interfaces.

---

## Package Structure

### `cpp_pkg`

```
cpp_pkg
├─ CMakeLists.txt
├─ include
├─ src
├─ tests
└─ README.md
```

This follows a typical C++ project layout:
- `CMakeLists.txt` for package configuration
- `include/` for headers (`.hpp`)
- `src/` for implementation (`.cpp`)
- `tests/` for testing
- `README.md` for package introductions

### `py_pkg`

```
py_pkg
├─ pyproject.toml
├─ src/
│  └─ py_pkg/
├─ tests/
└─ README.md
```

This follows a typical Python project layout: (`poetry new py_pkg` was used to create the layout)

- `pyproject.toml`  for package configuration
- `src/` for implementation (actual Python package lives in `src/py_pkg/`)
- `tests/` for testing
- `README.md` for package introductions

---

> Remember, you can customize the layout of your package as much as needed to suit your specific requirements. The only requirement is that each package you create must be **modular** and **self-contained**.

A ***modular*** package is designed to encapsulate a specific functionality or domain, making it easy to understand, maintain, and reuse independently or as part of a larger system. It should expose clear interfaces and avoid unnecessary coupling with other packages.

A ***self-contained*** package means it includes everything necessary to build, run, test, and document itself without relying on external parts of the repository. This typically includes:
- Its own source code
- Build or configuration files (e.g., `CMakeLists.txt` or `pyproject.toml`)
- Tests
- Documentation (e.g., `README.md`)
- Instructions or mechanisms for installing dependencies

Together, being ***modular*** and ***self-contained*** ensures that each package can be developed, tested, and deployed independently, which improves maintainability, scalability, and collaboration across teams.

---

## Template's Way of Building and Maintaining

### C++ (`cpp_pkg`)
- **Build System**: CMake
- **Compiler**: Clang (installed via LLVM)
- **Build Tool**: Ninja
- **Formatting**: `clang-format`
- **Linting**: `clang-tidy`

> If you want to try building the same we did, make sure to install CMake, Clang and Ninja in your machine.

Assuming you are in the root directory of the package:

To format:
```bash
clang-format -i src/*.cpp include/*.hpp
```

To build (with linting):
```bash
cmake  -S . -B build -G "Ninja" -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_CLANG_TIDY="clang-tidy;-header-filter=.;-checks=*;"
cmake --build build
```

---

### Python (`py_pkg`)
- **Packaging and Dependency Management Tool**: Poetry
- **Configuration Tool**: pyproject.toml
- **Formatting & Linting**: Ruff

> Remember that with Python no build step is required.

Assuming you are in the root directory of the package and have a Python envioment to work with:

To install the package:
```bash
pip install -e .
```

> It is recommended to create a virtual environment and install the package, because it helps isolate dependencies and avoid conflicts between different Python projects.

You can create a virtual enviroment like this, in case you need one:
```bash
python -m venv myenv
```

And then activate it (on Windows via CMD):
```bash
"myenv/Scripts/activate.bat"
```

To format and lint:
```bash
ruff format .
ruff check .
```

You can also build your project into a distributable format (wheel and source tarball):
```bash
poetry build
```

---

## Documentation

- **C++**: Use **Doxygen-style comments** in headers and source files.
- **Python**: Use **NumPy-style docstrings**.

This allows automatic documentation generation via **Sphinx** and **GitHub Actions**. For more, check [Docs](/docs/).

---

## Testing

Write tests using your preferred framework. This template does not enforce any strict requirements for testing, allowing developers to focus more on writing and improving the core functionality. However, it is strongly recommended to test the most critical and foundational parts of your packages. In this way, in case of contributions, any new code added should not break existing functionality or introduce regressions. Well-tested core components act as a safety net, ensuring that changes are reliable and maintainable. They also help contributors understand the expected behavior and integration points of your package.

> **Important**: Code must be tested before merging into `main`. If new code is added, corresponding tests must be created. This ensures structured contributions and avoids regressions.

---

## Development Golden Rules

- Always keep your packages **modular** and **self-contained**.
- Document your code.
- Format and lint (This step can make you to refactor your code).
- Write tests for any new functionality you add and check whether it behaves as expected.
- (Advacnced) Use GitHub Actions to enforce functional gates.

---

## DevOps

This template integrates GitHub Actions to automate common DevOps tasks for both C++ and Python packages. Changes to main must be made via pull requests so CI can run the automated jobs that format, lint, and validate code and documentation before merging.

How it works
- On every pull request the workflow searches the repository for packages:
  - A C++ package is identified by a `CMakeLists.txt` file.
  - A Python package is identified by a `pyproject.toml` file.
- For each discovered package the jobs:
  - Format source files (`clang-format` for C++, `ruff format` for Python).
  - Run linters (`clang-tidy` for C++, `ruff check` for Python).
    - *This also includes checking documentation completeness and correctness. Note that Clang does not have a straight forward way to do so; however, Doxygen has. So, Doxygen is used to produce documentation, and warnings like documentation is not available are treated as errors.*
- If linting find errors, the related status checks fail and GitHub will block merging the pull request until the issues are fixed.

Why this matters
- Prevents regressions by enforcing checks on every change.
- Keeps main stable and ready for releases.
- Ensures contributors address any issues early in the review cycle rather than after merging.

Recommendation
- Do not push directly to main. Require pull requests and configure branch protection rules, so merges are only allowed when the required status checks pass.
- Ensure each package in the repo is modular and self-contained.