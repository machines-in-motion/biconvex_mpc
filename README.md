# BiConvex Motion Planner

This repo contains the full body centroidal motion planner based on an ADMM solution method for the centroidal dynamics and DDP based non-linear Inverse Kinematics

## Dependencies
```
- Pinocchio
- Crocoddyl (Version 1.8.1 or greater)
- OsqpEigen (Optional, to use OSQP as the solver) [Link here: https://github.com/robotology/osqp-eigen]
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


## Authors
- Avadesh Meduri
- Paarth Shah

## Copyrights

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft

## License

BSD 3-Clause License

