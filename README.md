# Heimdall

[![Build & Tests](https://github.com/juliencombattelli/heimdall/actions/workflows/build.yml/badge.svg)](https://github.com/juliencombattelli/heimdall/actions/workflows/build.yml)

Heimdall, a pun for MDAL, standing for Modern Device Access Layers is a vendor-independent library enabling a consistent and portable access to memory-mapped device registers.

Written using C++20, this library targets efficiency, safety, and simplicity of use.

Most errors can be detected at build time when the corresponding input data are known during the compilation (eg. invalid write access to a read-only device register, invalid value written into a bitfield, etc).

Checkout the full Heimdall documentation on [ReadTheDocs](https://heimdall.readthedocs.io).

## Features

- [Not yet implemented] **MCU header generation tool** — Generate the Heimdall header for a specific MCU using an appropriate generator (based on SVD files from CMSIS if available, and/or generator with custom rules).

## Usage

### Requirements

Heimdall requires CMake 3.18+ and a C++20 compliant compiler.
Building the documentation requires Sphinx.

### Integration

There are multiple ways to integrate Heimdall into a CMake project:

- [Not yet implemented] **HeimdallConfig.cmake** — Install Heimdall into your system and use *find_package(Heimdall)* to locate it
- **FetchContent** — Use CMake's *FetchContent* module to download Heimdall and include it to your project
- **Manual add_subdirectory** (not recommended) — Manually add Heimdall to your project (eg. by copying the sources or as a git-submodule) and use *add_subdirectory()* to include it

For other build systems, the file heimdall.pc can be used with pkg-config.

### Options

- [Not yet implemented] **HEIMDALL_ENABLE_TESTING** — Build the unit tests (default: ON)
- [Not yet implemented] **HEIMDALL_ENABLE_FUZZING** — Build the fuzzy tests (default: OFF, needs HEIMDALL_ENABLE_TESTING=ON)
- [Not yet implemented] **HEIMDALL_ENABLE_COVERAGE** — Build with test coverage analysis (default: OFF, needs HEIMDALL_ENABLE_TESTING=ON)
- [Not yet implemented] **HEIMDALL_ENABLE_DOCUMENTATION** — Generate the html documentation using Sphinx (default: OFF)

## Contributing

If you want to get involved and suggest some additional features, signal a bug or submit a patch, please create a pull request
or open an issue on the [Heimdall Github repository](https://github.com/juliencombattelli/heimdall).
