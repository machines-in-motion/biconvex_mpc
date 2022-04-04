# BiConvex Motion Planner

This repo contains the centroidal motion planner based on a Biconvex solver and DDP based non linear IK

## Dependencies
```
- Pinocchio
- Crocoddyl (Version 1.7, Version 1.8 > not supported)
- OsqpEigen (Optional, to use OSQP as the solver) [Link here: https://github.com/robotology/osqp-eigen]
- Matplotlib (Optional, needed to run demos)
- Robot_Properties_Solo (Optional, needed to run demos)
```
## Installation
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


## Authors
- Avadesh Meduri
- Paarth Shah

## Copyrights

Copyright(c) 2019-2020 New York University, Max Planck Gesellschaft

## License

BSD 3-Clause License

