import sympy as sp

# Define symbolic variables
q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
a1, a2 = sp.symbols('a1 a2')
alpha0, alpha3, alpha4 = sp.symbols('alpha0 alpha3 alpha4')
d0, d3, d4, d5 = sp.symbols('d0 d3 d4 d5')

# Define DH parameters in symbolic form
a = [0, a1, a2, 0, 0, 0]
alpha = [alpha0, 0, 0, alpha3, alpha4, 0]
d = [d0, 0, 0, d3, d4, d5]
theta = [q1, q2, q3, q4, q5, q6]

# Function to compute the DH transformation matrix
def dh_transformation_matrix(a, alpha, d, theta):
    return sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
                      [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
                      [0, sp.sin(alpha), sp.cos(alpha), d],
                      [0, 0, 0, 1]])

# Compute individual transformation matrices
T01 = dh_transformation_matrix(a[0], alpha[0], d[0], theta[0])
T12 = dh_transformation_matrix(a[1], alpha[1], d[1], theta[1])
T23 = dh_transformation_matrix(a[2], alpha[2], d[2], theta[2])
T34 = dh_transformation_matrix(a[3], alpha[3], d[3], theta[3])
T45 = dh_transformation_matrix(a[4], alpha[4], d[4], theta[4])
T56 = dh_transformation_matrix(a[5], alpha[5], d[5], theta[5])

# Cumulative transformations
T02 = T01 * T12
T03 = T02 * T23
T04 = T03 * T34
T05 = T04 * T45
T06 = T05 * T56

# Position of the end-effector
P = T06[:3, 3]

# z vectors for each frame
z0 = sp.Matrix([0, 0, 1])
z1 = T01[:3, 2]
z2 = T02[:3, 2]
z3 = T03[:3, 2]
z4 = T04[:3, 2]
z5 = T05[:3, 2]

# Origins for each frame
O0 = sp.Matrix([0, 0, 0])
O1 = T01[:3, 3]
O2 = T02[:3, 3]
O3 = T03[:3, 3]
O4 = T04[:3, 3]
O5 = T05[:3, 3]
O6 = P

# Linear velocity part of the Jacobian
Jv1 = z0.cross(O6 - O0)
Jv2 = z1.cross(O6 - O1)
Jv3 = z2.cross(O6 - O2)
Jv4 = z3.cross(O6 - O3)
Jv5 = z4.cross(O6 - O4)
Jv6 = z5.cross(O6 - O5)

# Angular velocity part of the Jacobian
Jw1 = z0
Jw2 = z1
Jw3 = z2
Jw4 = z3
Jw5 = z4
Jw6 = z5

# Jacobian matrix
Jv = sp.Matrix.hstack(Jv1, Jv2, Jv3, Jv4, Jv5, Jv6)
Jw = sp.Matrix.hstack(Jw1, Jw2, Jw3, Jw4, Jw5, Jw6)

# Full Jacobian
Jacobian = sp.Matrix.vstack(Jv, Jw)

Jacobian.simplify()
print(Jacobian)
