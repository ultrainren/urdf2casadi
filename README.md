# URDF2CASADI
A module for generating the forward kinematics of a robot from a URDF. It can generate the forward kinematics represented as a dual quaternion or a transformation matrix. `urdf2casadi` works both in python 2 and 3, and any platform that supports `CasADi` and `urdf_parser_py`.

## Installation
With ROS:
1. [Get ROS](http://www.ros.org/install/) (actually anything that installs `urdfdom_py`/`urdf_parser_py` will do).
2. [Get CasADi](https://github.com/casadi/casadi/wiki/InstallationInstructions) (e.g. `pip install casadi`).
3. Run `pip install --user .` in the folder.

Without ROS:
1. Change the `urdfdom-py` to `urdf-parser-py` in `requirements.txt` (line 3) and in `setup.py` (line 20).
2. [Get CasADi](https://github.com/casadi/casadi/wiki/InstallationInstructions) (e.g. `pip3 install casadi`).
3.  Run `pip3 install --user .` in the folder (`--user` specifies that it is a local install).

## Usage example
```python
import casadi as cs
from urdf2casadi import converter
fk_dict = converter.from_file("root", "gantry_tool0", "robot_urdf_file_path.urdf")
print fk_dict.keys()
# should give ['q', 'upper', 'lower', 'dual_quaternion_fk', 'joint_names', 'T_fk', 'joint_list', 'quaternion_fk']
forward_kinematics = fk_dict["T_fk"]
print forward_kinematics([0.3, 0.3, 0.3, 0., 0.3, 0.7])
```

## Todo/Implementation status
- [x] Forward kinematics with SE(3) matrix
- [x] Forward kinematics of rotation with quaternion
- [x] Dual Quaternions as alternative to SE(3) matrices
- [ ] Dynamics from links and their inertia tags
- [x] Denavit Hartenberg?
- [ ] unit tests
- [ ] Examples
