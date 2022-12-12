# BiConMP

This repository is the implentation of the BiConMP solver. The BiConMP is an online whole body trajectory optimizer for legged robots. The trajectory optimizer has generated various dyanmic behaviorus on Quadrupeds (Solo12, AnYmal), Humanoids (Talos)...

## Dependencies
### Required
- Pinocchio (Version 2.6.4 or greater)
- Crocoddyl (Version 1.8.1 or greater)
- Pybind11
### Optional
- bullet_utils (needed to run examples and tutorials) : [Installation link](https://github.com/machines-in-motion/bullet_utils)
- Robot_Properties_Solo (needed to run examples and tutorials) : [Installation link](https://github.com/open-dynamic-robot-initiative/robot_properties_solo)
- OsqpEigen (to use OSQP as the solver): [Installation Link](https://github.com/robotology/osqp-eigen)
- Meshcat (To visualize soltutions) : [Installation Link](https://github.com/rdeits/meshcat-python)
- Matplotlib (to plot results)
  
## Installation

### Step 1

```
git clone --recurse-submodules git@github.com:machines-in-motion/biconvex_mpc.git
cd biconvex_mpc
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make install -j16
```

If you want to use OSQP, the cmake instructions should be modified to the following:

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DOSQP=TRUE
```
### Step 2
After building the package add the following lines to your bashrc to ensure that python can find BiconMP. 

```
export PYTHONPATH=$PYTHONPATH:path/to/the/biconvex/build/folder
```

## Authors
- Avadesh Meduri
- Paarth Shah

## Citing

```
@article{meduri2022biconmp,
  title={BiConMP: A Nonlinear Model Predictive Control Framework for Whole Body Motion Planning},
  author={Meduri, Avadesh and Shah, Paarth and Viereck, Julian and Khadiv, Majid and Havoutis, Ioannis and Righetti, Ludovic},
  journal={arXiv preprint arXiv:2201.07601},
  year={2022}
}
```

## Copyrights

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft

## License

BSD 3-Clause License

