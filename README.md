## CommonRoad-Reach-Semantic: A Toolbox for Specification-Compliant Reachability Analysis of Automated Vehicles

### System Requirements

The software is written in Python 3.10, and was tested on Ubuntu 20.04 & 22.04.

### Building the Code

> **Note:** This repository contains [pybind11](https://github.com/pybind/pybind11) as a submodule, so don't forget to run `git submodule update --init --recursive` after cloning.

* We strongly recommend using [Anaconda](https://www.anaconda.com/) to manage your virtual python environment.
If you don't want to use Anaconda for space reasons, consider using [Miniconda](https://docs.conda.io/en/latest/miniconda.html).
* Install Python dependencies:
```bash
pip install -r requirements.txt
```
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

* Install [CommonRoad-Reach](https://commonroad.in.tum.de/tools/commonroad-reach) **from source**.
Please refer to its [documentation](https://commonroad.in.tum.de/docs/commonroad-reach/getting_started.html) for instructions.
  Note that this includes installing
  the [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker) from source.

> **Note:** Currently there appears to be a bug with boost geometry and newer versions of GCC (this seems to start with
> version 11.4).
> A workaround until this is fixed is to use an older version of GCC (we suggest GCC 10).
> To do so, indicate the path to the older version of GCC in the `CXX` environment variable before building the code (
> e.g. `export CXX=/usr/bin/g++-10`).
> Make sure that you use the same compiler version for building the CommonRoad Drivability Checker, CommonRoad-Reach,
> and CommonRoad-Reach-Semantic.

* Build the C++ code and its Python bindings (where your Python version is Python X.Y.Z):
```bash
mkdir build && cd build
cmake -DCRDC_DIR="/path/to/drivability-checker-root" -DCRREACH_DIR="/path/to/reach-root" -DPYTHON_VER="XY" -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```
Make sure to use absolute paths to the root directory of your drivability checker and CommonRoad-Reach installation for `CRDC_DIR` and `CRREACH_DIR`, respectively.
If you are using Anaconda, activate your environment before running cmake.

* Move the resulting Python bindings to `commonroad_reach_semantic`:
```bash
cd ..
mv pycrreachs.*.so commonroad_reach_semantic/
```

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

### Development

Check out the [README_FOR_DEVS](./readme/README_FOR_DEVS.md) for information on setting up your development environment.
