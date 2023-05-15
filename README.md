## CommonRoad-Reach-Semantic: A Toolbox for Specification-Compliant Reachability Analysis of Automated Vehicles

### System Requirements

The software is written in Python 3.10, and was tested on Ubuntu 22.04.

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
* Install [CommonRoad-Reach](https://commonroad.in.tum.de/tools/commonroad-reach).
Please refer to its [documentation](https://commonroad.in.tum.de/docs/commonroad-reach/getting_started.html) for instructions.
Note that this includes installing the [CommonRoad Drivability Checker](https://commonroad.in.tum.de/drivability-checker).
* Build the C++ code and its Python bindings (where your Python version is Python X.Y.Z):
```bash
mkdir build && cd build
cmake -DCRDC_DIR="/path/to/drivability-checker-root" -DCRREACH_DIR="/path/to/reach-root" -DPYTHON_VER="XY" -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```
Make sure to use absolute paths to the root directory of your drivability checker and CommonRoad-Reach installation for `CRDC_DIR` and `CRREACH_DIR`, respectively.

* Move the resulting Python bindings to `commonroad_reach_semantic`:
```bash
cd ..
mv pycrreachs.*.so commonroad_reach_semantic/
```

### Running the Code

Run the exemplary script `main.py` to compute specification-compliant reachable sets.

The outputs will be stored in the `./output/` directory.
Default and scenario-specific configurations are stored in the `./configurations/` directory.
The scenarios themselves are located in the `./scenarios/` directory.

> **Note:** You might need to adjust the `path_root` argument of `SemanticConfigurationBuilder.build_configuration` to your setup.
