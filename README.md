## CommonRoad-Reach-Semantic: A Toolbox for Specification-Compliant Reachability Analysis of Automated Vehicles

### System Requirements

The software is written in Python 3.10, and was tested on Ubuntu 20.04 & 22.04.

### Building the Code

> **Note:** This repository contains [pybind11](https://github.com/pybind/pybind11) as a submodule, so don't forget to run `git submodule update --init --recursive` after cloning.

* We strongly recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.
If you don't want to use Anaconda for space reasons, consider using [Miniconda](https://docs.conda.io/en/latest/miniconda.html).
* Install [spot](https://spot.lre.epita.fr/) and its Python bindings.
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

* Install [CommonRoad-Reach](https://commonroad.in.tum.de/tools/commonroad-reach) **from source** using the version
  indicated by `GIT_TAG` in [ExternalReach.cmake](cmake/external/ExternalReach.cmake).
Please refer to its [README](https://gitlab.lrz.de/cps/commonroad-reachable-set/-/blob/develop/README.md?ref_type=heads) for instructions.

> **Note:** Currently there appears to be a bug with boost geometry and newer versions of GCC (this seems to start with version 11.4).
> A workaround until this is fixed is to use an older version of GCC (we suggest GCC 10).
> To do so, indicate the path to the older version of GCC in the `CXX` environment variable before building the code (e.g. `export CXX=/usr/bin/g++-10`).
> **Important:** Make sure that you use the same compiler version for building CommonRoad-Reach and CommonRoad-Reach-Semantic.

> **Note:** Using the pip package of CommonRoad-Reach does currently not work when using the C++ extensions, probably due to incompatible compiler versions.
> We will have to check this again, once we release a new version of CommonRoad-Reach (> 2023.1.1).


* Build the C++ extension and install the Python package:
```bash
pip install -v .
```

> **Note**: The verbose flag (`-v`) prints detailed information about the C++ build progress.

**Optional:**

- To build the code in Debug mode, add the flag `--config-settings=cmake.build-type="Debug"` to the `pip` command.
- See [here](https://scikit-build-core.readthedocs.io/en/latest/configuration.html#configuring-cmake-arguments-and-defines) for further information on configuring CMake arguments via our build system (`scikit-build-core`).


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
* `terminate called without an active exception` when running the example script:
  See above
* `Segmentation fault` when running the example script:
  See above

### Development

Check out the [README_FOR_DEVS](./readme/README_FOR_DEVS.md) for information on setting up your development environment.
