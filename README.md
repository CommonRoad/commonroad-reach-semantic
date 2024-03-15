## CommonRoad-Reach-Semantic: A Toolbox for Specification-Compliant Reachability Analysis of Automated Vehicles

### System Requirements

The software is written in Python 3.10, and was tested on Ubuntu 20.04 & 22.04.

### Building from Source

> **Note:** Currently there appears to be a bug with boost geometry and newer versions of GCC (this seems to start with
> version 11.4).
> A workaround until this is fixed is to use an older version of GCC (we suggest GCC 10).
> To do so, indicate the path to the older version of GCC in the `CXX` environment variable before building the code (
> e.g. `export CXX=/usr/bin/g++-10`).

> **Note:** The build process automatically includes other internal repositories via Git.
> Thus, an SSH key in your LRZ GitLab account is required.
> See [here](https://docs.gitlab.com/ee/ssh/) for instructions on how to add an SSH key.

#### Third-Party Dependencies

While most of these dependencies are added automatically during the build process, you can install them manually via
your package manager to speed up the build process.

**Manual installation required:**

- [OpenMP](https://www.openmp.org/)
- [Spot](https://spot.lrde.epita.fr/)

**Manual installation recommended to speed up the build:**

- [Boost](https://www.boost.org/)

**Manual installation optional:**

- [CommonRoad-Reach](https://commonroad.in.tum.de/tools/commonroad-reach)
- [Eigen3](https://eigen.tuxfamily.org/)
- [yaml-cpp](https://github.com/jbeder/yaml-cpp)
- [spdlog](https://github.com/gabime/spdlog)
- [pybind11](https://github.com/pybind/pybind11)

**Optional dependencies:**

- [GTest](https://google.github.io/googletest/) (optional: for building unit tests)

The additional Python dependencies are listed in `pyproject.toml`.

#### Building the Code

1. We strongly recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.
If you don't want to use Anaconda for space reasons, consider using [Miniconda](https://docs.conda.io/en/latest/miniconda.html).

2. Install [spot](https://spot.lre.epita.fr/) and its Python bindings.
If you are using Anaconda or Miniconda, you can install spot from conda forge with:
```bash
conda install -c conda-forge spot
```
For the C++ code, you also need to install the spot headers.
See the [spot documentation](https://spot.lre.epita.fr/install.html) for instructions.
If you are using Ubuntu, you can install the spot headers with:
```bash
wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | sudo tee /etc/apt/keyrings/lrde-spot.gpg
echo 'deb [signed-by=/etc/apt/keyrings/lrde-spot.gpg] http://www.lrde.epita.fr/repo/debian/ stable/' | sudo tee -a /etc/apt/sources.list
sudo apt-get update
sudo apt-get install libspot-dev
```

3. Install C++ dependencies:

```bash
sudo apt-get update
sudo apt-get install libomp-dev libboost-all-dev libeigen3-dev libyaml-cpp-dev libspdlog-dev pybind11-dev libgtest-dev libgmock-dev
```

4. Build the C++ extension and install the Python package:
```bash
pip install -v .
```

This will build the Python bindings (pycrreachsem) required for C++-boosted computations.

> **Note**: The `-v` flag (verbose) prints information about the build progress

**Optional:**

- To build the code in Debug mode, add the flag `--config-settings=cmake.build-type="Debug"` to the `pip` command.
- See [here](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#configuring-cmake-arguments-and-defines) for further information on configuring CMake arguments via our build system (`scikit-build-core`).

> **Note**: `scikit-build-core` uses `ninja` for building the C++ extension by default.
> Thus, the build is automatically parallelized using all available CPU cores.
> If you want to explicitly configure the number of build jobs, you can do so by passing the
> flag `--config-settings=cmake.define.CMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS` to the `pip` command, where `$BUILD_JOBS`
> is the number of parallel jobs to use.
> See [here](https://scikit-build-core.readthedocs.io/en/latest/faqs.html#multithreaded-builds) for further details.

> **Note**: Building the package in Debug mode (see above) significantly increases the computation time of the C++
> backend. Please make sure you are building in Release mode (default setting) if you require fast computations.

### Running the Code

Run the example script `main.py` to compute specification-compliant reachable sets.

The outputs will be stored in the `./output/` directory.
Default and scenario-specific configurations are stored in the `./configurations/` directory.
The scenarios themselves are located in the `./scenarios/` directory.

> **Note:** You might need to adjust the `path_root` argument of `SemanticConfigurationBuilder` to your setup.

### Possible installation problems

* `error: 'to_finite' is not a member of 'spot'` during cmake build:
  this is caused by an old version being used for compiling even you have installed the latest one.
  The function `to_finite` is declared in the header `remprop.hh`.
  You can search for `sudo find / -name remprop.hh 2>/dev/null` and then delete the folders that are not under `/usr/`.
* `ImportError: /.../commonroad-reach-semantic/commonroad_reach_semantic/pycrreachs.cpython-310-x86_64-linux-gnu.so: undefined symbol: _ZN4spot9to_finiteESt10shared_ptrIKNS_9twa_graphEEPKc`
  when importing pybind11 bindings:
  you can use [this tool](https://demangler.com) to demangle the symbol name.
  There might still be some old version of `spot` is used from your conda environment.
  Try uninstalling `spot` and see whether the error still appears.
  If yes, delete other spot files in your anaconda folder and reinstall spot.
* `ImportError: /.../commonroad_reach/pycrreach.cpython-310-x86_64-linux-gnu.so: undefined symbol: _ZN3fcl15CollisionObjectIdEdlEPv` when running the example script:
  This is most likely caused by using two different compiler versions for compiling `commonroad-reach` and `commonroad-reach-semantic`.
  Make sure that you use the same compiler version for both.
  Also, ensure that you build everything completely from scratch:
  * Uninstall the `commonroad-reach-semantic`, `commonroad-reach`, and `commonroad-drivability-checker` packages
    via `pip`.
  * Remove the `build` directory of `commonroad-reach-semantic` and `commonroad-reach`.

### Development

Check out the [README_FOR_DEVS](./readme/README_FOR_DEVS.md) for information on setting up your development environment.
