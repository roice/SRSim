# Free Vortex Method
#      Cython implementation

import numpy as np
cimport numpy as np

cdef extern from "math.h":
    double sqrt(double)

ctypedef np.float64_t dtype_t

cdef double PI = 3.14159265358979323846

########################### Vortex Filament Method ##########################
#
# update Lagrangian markers
#  simple backward difference, linear segment 
#  Input:
#    pos_markers: positions of markers
#        (markers per fila) x (num rotors) x (num blades) x 3(pos) matrix
#    dir_psi:     direction of rotation, 1 for counter-clockwise, -1 otherwise
#        one dimensional, (num rotors) x 1 matrix
#    delta_time:  delta t (second)
def VF_markers_update_simpleBD(
        np.ndarray[dtype_t, ndim=4] pos_markers,
        np.ndarray[dtype_t, ndim=1] dir_psi,
        double delta_time,
        ):
    cdef int N_m = pos_markers.shape[0] # number of markers
    cdef int N_r = pos_markers.shape[1] # number of rotors, 4 for quadrotor
    cdef int N_b = pos_markers.shape[2] # num_blades not used, currently only support two-blade
    cdef Py_ssize_t index_m,index_r,index_b # index of markers, rotors and blades, for point update
    cdef Py_ssize_t i, j, k # index of markers, rotors and blades, for pointwise induce vel calc
    cdef np.ndarray[dtype_t, ndim=1] a, b, p # point A, B, P
    cdef np.ndarray[dtype_t, ndim=1] ind_v # induced velocity

    for index_m in range(N_m):
        for index_r in range(N_r):
            for index_b in range(N_b):
                p = pos_markers[index_m, index_r, index_b] # update this marker
                ind_v = np.zeros(3)
                for i in range(N_m-1):
                    for j in range(N_r):
                        for k in range(N_b):
                            if dir_psi[j] < 0: # clockwise
                                a = pos_markers[i, j, k] # point A of this segment
                                b = pos_markers[i+1, j, k] # point B of this segment
                            else: # counter-clockwise
                                a = pos_markers[i+1, j, k]
                                b = pos_markers[i, j, k]
                            ind_v += biot_savart(a, b, p)
                pos_markers[index_m, index_r, index_b] += ind_v*delta_time
                            
# Biot-Savart
#  Input:
#    a: position vector of point A
#    b: position vector of point B
#    p: position vector of point P
cdef np.ndarray[dtype_t, ndim=1] biot_savart(
        np.ndarray[dtype_t, ndim=1] a,
        np.ndarray[dtype_t, ndim=1] b,
        np.ndarray[dtype_t, ndim=1] p):

    cdef np.ndarray[dtype_t, ndim=1] ab, ap, bp # vector AB, AP, BP
    cdef np.ndarray[dtype_t, ndim=1] e # unit vector indicating the induced vel direction
    cdef double h # perpendicular distance from P to AB
    cdef double cos_theta_1, cos_theta_2 # cos(theta_1), cos(theta_2)
    cdef double Gamma = 0.001

    # check if P is overlapping with A or B
    if (isclose(a[0], b[0]) and isclose(a[1], b[1]) and isclose(a[2], b[2]))\
            or (isclose(a[0], p[0]) and isclose(a[1], p[1]) and isclose(a[2], p[2]))\
            or (isclose(p[0], b[0]) and isclose(p[1], b[1]) and isclose(p[2], b[2])):
        return 0. # P is overlapping with A or B
    # AP, BP and AB vector
    ap = p - a
    bp = p - b
    ab = b - a
    # cos(theta_1) and cos(theta_2)
    cos_theta_1 = np.dot(ab, ap)/(norm_vector(ab)*norm_vector(ap))
    cos_theta_2 = np.dot(ab, bp)/(norm_vector(ab)*norm_vector(bp))

    # check if cos(theta_1) is 1.0
    if isclose(cos_theta_1, 1.):
        return 0. # h == 0
    elif isclose(cos_theta_1 - cos_theta_2, 0.):
        return 0. # induced vel is zero

    # h, perpendicular distance from P to AB
    h = norm_vector(ap) * sqrt(1 - cos_theta_1**2)
    # e, unit vector indicating the dir of induced velocity
    e = np.cross(ap, bp)
    e = e/norm_vector(e)
    # induced velocity of this segment
    return Gamma/(4*PI)*(h/sqrt(0.001**4+h**4))*(cos_theta_1-cos_theta_2)*e

# compare two decimals
cdef bint isclose(double a, double b, double rel_tol=1e-09, double abs_tol=0.0) except *:
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# 2-norm of a vector
cdef double norm_vector(np.ndarray[dtype_t, ndim=1] vector) except *:
    cdef double norm = 0.
    cdef Py_ssize_t i
    for i in range(vector.shape[0]):
        norm += vector[i]**2
    return sqrt(norm)
