# C++/Python Project Template

This is a standardized C++/Python project template for DALSA projects that will be hosted on GitHub, designed to help project owners quickly set up new projects with a consistent structure and DevOps features. Its goal is to ensure maintainability and collaboration by enforcing standards that make future contributions and usage seamless. The template integrates **Sphinx** + **Doxygen** for unified documentation of both C++ and Python packages, and uses GitHub Actions to automate the generation of web-based documentation, code rewrites to follow a consistent style (formatting), and code analysis (linting).
This template introduces a certain structure so that users and contributors can quickly understand your project.

--- 

## Project Structure

    .
    ├── lib/               # Library for C++ and Python packages
    │   ├── cpp_pkg/       # Dummy C++ package
    │   └── py_pkg/        # Dummy Python package 
    ├── docs/              # Documentation built by Sphinx + Doxygen via Breathe
        ├── architecture/  # Component-based documentation
        ├── code/          # Code-based documentation
        └── usage/         # Examples and tutorials

All code is stored in `lib/` and organized into packages, either C++ or Python packages.\
All documentation, on the other hand, is stored in `docs/`. Documentation is a crucial aspect of this template. Using **Sphinx** with **Doxygen** via **Breathe**, code documentation is automated based on comments/docstrings and enriched with more info provided by the developers using **reStructuredText (reST)**. Together with GitHub Actions, this allows enforcement of consistent documentation practices across C++ and Python codebases. In this way:

*   Documentation stays up-to-date with code changes.
*   Documentation is organized and easily navigable.
*   A nice-looking and intuitive website is generated.
*   Both users and contributors can understand the codebase quickly.
*   The same standards are followed across different projects.

## Main Features

### Documentation Tools
- **Sphinx** — Primary tool for generating project documentation
  - Primarily uses reStructuredText, including its directives and extensions to produce documentation.
  - Supports both **reStructuredText (.rst)** and **Markdown (.md)** formats. 

- **Doxygen** — Specialized for C++ code documentation  
  - Produces **HTML output** for standalone C++ docs (found in `docs/doxygen/html`).
  - Generates **XML output** used by **Breathe**, a bridge that lets **Sphinx** include C++ documentation.

### GitHub Actions
- **Formatting**: Automatic styling (ruff format for Python, clang-format for C++) before merging into main
- **Linting**: Automatic linting (ruff for Python, clang-tidy for C++) before merging into main
- **Documentation**: Automatic build and deployment to GitHub Pages when updating main

## Before Using This Template
If you haven't done so already, check the following:  
- [Docs](/docs/) and learn how our project documentation is structured in `rst` and `md` files.
- [Lib](/lib/) and learn how to work with packages.
- [Docs Webpage](https://dalsa-lab.github.io/cpp_python_project_template/)

Do you think that's not enough? If not, then checking the following is highly recommended:
- [reStructuredText](https://www.sphinx-doc.org/en/master/usage/restructuredtext/index.html) (explore **reStructuredText Primer**, **Roles**, and **Directives**)
- [Sphinx Extensions](https://www.sphinx-doc.org/en/master/usage/extensions/index.html)
- [NumPy Style Guide](https://numpydoc.readthedocs.io/en/latest/format.html)
- [Doxygen Documenting Code Guide](https://www.doxygen.nl/manual/docblocks.html)
- [Clang-Tidy](https://clang.llvm.org/extra/clang-tidy/index.html) and [Clang-Format](https://clang.llvm.org/docs/ClangFormat.html)
- [Ruff Formatter](https://docs.astral.sh/ruff/formatter/) and [Ruff Linter](https://docs.astral.sh/ruff/linter/)

## To Produce Documentation Locally: 
Assuming you are in the root directory of this project and have a Python envioment to work with:
1. Install Python dependencies:  
   ```bash
    pip install -r requirements-docs.txt
   ```
2. Install Doxygen from [here](https://www.doxygen.nl/download.html).
3. Build your documentation
   ```bash
    doxygen Doxyfile # If there are any C++ packages
    sphinx-build -b html docs build/html
   ```

## Maintaining Your Code
When using this template, repository-level rulesets are NOT copied into the new repository. Project owners must configure these protections themselves in the repository settings.

Why protect main?
- Prevents direct pushes and accidental changes to main.
- Ensures CI (formatting, linting, testing, etc) runs and passes before code is merged.
- Keeps main stable for releases and documentation builds.

Template's Linting / formatting behavior
- The template’s GitHub Actions jobs for formatting and linting are intended to run on pull requests.
- Only merge into main after the required status checks pass on the PR.

Recommended minimal branch-protection rule (GitHub: Settings → Branches → Add branch ruleset) on the main branch:
- Restrict deletions
- Require a pull request before merging
  - Require review from Code Owners
  - Require conversation resolution before merging
- Require status checks to pass (select cpp-format-lint and python-format-lint)
  - Require branches to be up to date before merging
  - Do not require status checks on creation
- Block force pushes
- Optionally enable: include administrators, require linear history

At minimum, create a rule that enforces updates to main only via pull requests with passing status checks.

## License
This template is licensed under the terms in `LICENSE`.
