# BiConvex Motion Planner

This repo contains the full body centroidal motion planner based on an ADMM solution method for the centroidal dynamics and DDP based non-linear Inverse Kinematics

It is highly recommended you install this in a virtual environment such as Conda. This allows easier use of installed dependencies such as Crocoddyl, OsqpEigen, and PyBind11. 

## Dependencies
```
- Pinocchio
- Crocoddyl (Version 1.8.1 or greater)
- OsqpEigen (Optional, to use OSQP as the solver) [Link here: https://github.com/robotology/osqp-eigen]
- PyBind11
```

## Installation

```
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make install
pip install ..
```

If you want to use OSQP, the cmake instructions should be modified to the following:

```
cmake .. -DCMAKE_BUILD_TYPE=Release -DOSQP=TRUE
```


## Adding Paths
In order to run and use the libraries, you must add the following paths to your $PYTHONPATH in the ~/.bashrc (or equivalent)

```
export PYTHONPATH=$PYTHONPATH:<WHERE-YOU-INSTALLED-THIS-PACKAGE>/build
export PYTHONPATH=$PYTHONPATH:<WHERE-YOU-INSTALLED-THIS-PACKAGE>
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:<WHERE-YOU-INSTALLED-THIS-PCKAGE>/build
```

If you are using Mac OSX, replace the last line with:

```
export DYLD_LIBRARY_PATH=DYLD_LIBRARY_PATH:<WHERE-YOU-HAVE-INSTALLED-THIS-PACKAGE>/build
```



## Authors
- Avadesh Meduri
- Paarth Shah

## Copyrights

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft

## License

BSD 3-Clause License

