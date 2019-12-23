import casadi as cs
from urdf2casadi import converter
from urdf2casadi.casadi_geom import dual_quaternion_to_transformation_matrix
import os  # For current directory


# urdf2casadi uses cs.SX, which can be hard to read as these are sparse matrices.
# This short function just makes it so that the result will be a numpy matrix
# Use for
def cs2np(asd):
    return cs.Function("temp", [], [asd])()["o0"].toarray()


# NOTE: casadi imports numpy as np, so cs.np is numpy


print('Importing UR5 urdf ===========================================')
urdf_path = "./urdf/ur5.urdf"
root_link = "base_link"
end_link = "tool0"
fk_dict = converter.from_file(root_link, end_link, urdf_path)
print(fk_dict.keys())

print('Joint information ===========================================')
# CasADi SX symbol giving the joint symbols:
q = fk_dict["q"]
# Upper limits of the joint values
q_upper = fk_dict["upper"]
# Lower limits of the joint values
q_lower = fk_dict["lower"]
# Joint names
joint_names = fk_dict["joint_names"]
print("Number of joints:", q.size()[0])
print("Upper limits:", q_upper)
print("Lower limits:", q_lower)
print("Joint names:", joint_names)

print('Forward kinematics ===========================================')
# CasADiTransformation matrix function:
T_fk = fk_dict["T_fk"]
# CasADi Dual quaternion function:
Q_fk = fk_dict["dual_quaternion_fk"]
T0 = T_fk([0., 0., 0., 0., 0., 0.])
p0 = T0[:3, 3]
R0 = T0[:3, :3]
print("Transformation matrix:\n", cs2np(T0))
print("Position:\n", "x:", p0[0], " y:", p0[1], " z:", p0[2])
print("Distance from origin:\n", cs.np.linalg.norm(p0), "m")
