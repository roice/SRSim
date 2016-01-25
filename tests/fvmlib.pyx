# Free Vortex Method
#      Cython implementation

import numpy as np
cimport numpy as np
cimport cython

cdef extern from "math.h":
    double sqrt(double)
    double pow(double, double)

ctypedef np.float64_t dtype_t

cdef double PI = 3.14159265358979323846

# update Vortex Ring Control Points
#  simple backward difference
#  Input:
#    pos_rings:   positions of rings, pos of control points actually
#        (num rings) x (num rotors) x (num control points: 4) x 3(pos) matrix
#    delta_time:  delta t (second)
#    step:        simulation step count
def VR_CP_update_simpleBD(
        np.ndarray[dtype_t, ndim=4] pos_rings,
        double Gamma,
        double delta_time,
        int step
        ):
    cdef int N_ring = pos_rings.shape[0] # number of rings / rotors
    cdef int N_rotor = pos_rings.shape[1] # number of rotors
    #cdef int N_cp = pos_rings.shape[2] # number of control points
    cdef Py_ssize_t index_ring, index_rotor, index_cp # index of rings, rotors and control points
    cdef Py_ssize_t i, j # index of rings and rotors
    cdef np.ndarray[dtype_t, ndim=1] ind_v # induced velocity
    cdef np.ndarray[dtype_t, ndim=3] dir_rings = np.zeros((N_ring,N_rotor,3)) # direction unit vectors of all rings
    cdef np.ndarray[dtype_t, ndim=3] c_rings = np.zeros((N_ring,N_rotor,3)) # center positions of all rings
    cdef np.ndarray[dtype_t, ndim=2] r_rings = np.zeros((N_ring,N_rotor))# radii of all rings
    cdef np.ndarray[dtype_t, ndim=1] op, u_z, u_r
    cdef double op_z, op_r, sq_op_z, sq_r_r, m

    # calculate direction vector, center and radius of all rings
    for index_ring in range(N_ring):
        for index_rotor in range(N_rotor): # for every rings 
            dir_rings[index_ring, index_rotor] = calc_ring_dir_vector(pos_rings[index_ring, index_rotor])
            c_rings[index_ring, index_rotor] = 0.25*(pos_rings[index_ring, index_rotor, 0]\
                    + pos_rings[index_ring, index_rotor, 1]\
                    + pos_rings[index_ring, index_rotor, 2]\
                    + pos_rings[index_ring, index_rotor, 3])
            r_rings[index_ring, index_rotor] = 0.25*(\
                    norm_vector(c_rings[index_ring, index_rotor]-pos_rings[index_ring, index_rotor, 0])\
                    + norm_vector(c_rings[index_ring, index_rotor]-pos_rings[index_ring, index_rotor, 1])\
                    + norm_vector(c_rings[index_ring, index_rotor]-pos_rings[index_ring, index_rotor, 2])\
                    + norm_vector(c_rings[index_ring, index_rotor]-pos_rings[index_ring, index_rotor, 3]))

    # calculate and update pos of all control points
    for index_ring in range(N_ring):
        for index_rotor in range(N_rotor): # for every rings
            for index_cp in range(4): # for every control points
                ind_v = np.zeros(3) # induced_velocity
                # calculate self-induced velocity
                ind_v += Gamma/(2*r_rings[index_ring, index_rotor])*dir_rings[index_ring, index_rotor]
                # calculate mutual-induced velocity
                for i in range(N_ring):
                    for j in range(N_rotor):
                        # except self
                        if (i == index_ring) and (j == index_rotor):
                            continue
                        else:
                            # calculate vector OP
                            op = pos_rings[index_ring, index_rotor, index_cp] - c_rings[i, j]
                            # calculate reletive z of P compared with O, cylindrical coord
                            op_z = dot_vectors(dir_rings[i, j], op)
                            # calculate relative r of P compared with O, cylindrical coord
                            op_r = sqrt((op[0]*op[0]+op[1]*op[1]+op[2]*op[2])-op_z*op_z)
                            # calculate u_z, cylindrical coord
                            sq_op_z = op_z*op_z
                            sq_r_r = pow(op_r+r_rings[i,j], 2)
                            m = 4*op_r*r_rings[i,j]/(sq_op_z+sq_r_r)
                            u_z = Gamma/(2*PI*sqrt(sq_op_z+sq_r_r))\
                                    * (complete_elliptic_int_first(m)\
                                    + complete_elliptic_int_second(m)\
                                    * (pow(r_rings[i,j],2)-op_r*op_r-op_z*op_z)\
                                    /(op_z*op_z+pow(r_rings[i,j]-op_r,2)))
                            ind_v += u_z*dir_rings[i,j] # map to cartesian coord
                            # calculate u_r, cylindrical coord
                            u_r = Gamma*op_z/(2*PI*op_r*sqrt(op_z*op_z+pow(op_r+r_rings[i,j],2)))\
                                    * (complete_elliptic_int_first(m)\
                                    - complete_elliptic_int_second(m)\
                                    * (pow(r_rings[i,j],2)+op_r*op_r+op_z*op_z)\
                                    /(op_z*op_z+pow(r_rings[i,j]-op_r,2)))
                            ind_v += u_r*(op - dot_vectors(dir_rings[i,j], op)*dir_rings[i,j]) # map to cartesian coord
                # Backward Difference to update pos of this control point
                pos_rings[index_ring, index_rotor, index_cp] += ind_v*delta_time
    # update simulation step
    step += 1

# Vortex Ring Control Point, unit vector indicating the dir of ring
cdef np.ndarray[dtype_t, ndim=1] calc_ring_dir_vector(np.ndarray[dtype_t, ndim=2] pos_cps):
    cdef np.ndarray[dtype_t, ndim=1] u_vector, temp_1, temp_2
    temp_1 = pos_cps[2]-pos_cps[0]
    temp_2 = pos_cps[1]-pos_cps[3]
    u_vector = cross_vectors(temp_1, temp_2)/\
            sqrt(dot_vectors(temp_1, temp_1)*dot_vectors(temp_2, temp_2))
    return u_vector

# calculate complete elliptic integral of the first kind
#  using power series method
cdef double complete_elliptic_int_first(double k) except *:
    return PI/2.*(1. + 0.5*0.5*k*k + 0.5*0.5*0.75*0.75*k*k*k*k)

# calculate complete elliptic integral of the second kind
#  using power series method
cdef double complete_elliptic_int_second(double k) except *:
    return PI/2.*(1. - 0.5*0.5*k*k - 0.5*0.5*0.75*0.75*k*k*k*k/3.)

# calculate Gamma
#def double calculate_gamma(double radius_rotor, double mass_uav)

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
@cython.boundscheck(False)
@cython.wraparound(False)
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
                            ind_v += biot_savart_normal(a, b, p)
                pos_markers[index_m, index_r, index_b] += ind_v*delta_time
                            
# Biot-Savart
#  Input:
#    a: position vector of point A
#    b: position vector of point B
#    p: position vector of point P
@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.ndarray[dtype_t, ndim=1] biot_savart_normal(
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

# Biot-Savart
#  Input:
#    a: position vector of point A
#    b: position vector of point B
#    p: position vector of point P
@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.ndarray[dtype_t, ndim=1] biot_savart_fast(
        np.ndarray[dtype_t, ndim=1] a,
        np.ndarray[dtype_t, ndim=1] b,
        np.ndarray[dtype_t, ndim=1] p):

    cdef np.ndarray[dtype_t, ndim=1] ab, ap, bp # vector AB, AP, BP
    cdef np.ndarray[dtype_t, ndim=1] e # unit vector indicating the induced vel direction
    cdef double sq_h # perpendicular distance^2 from P to AB
    cdef double sq_cos_theta_1, sq_cos_theta_2, cos_theta_1_cos_theta_2 # cos(theta_1)^2, cos(theta_2)^2
    cdef double Gamma = 0.001
    cdef double dot_abab, dot_apap, dot_bpbp, dot_abap, dot_abbp

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
    dot_abab = dot_vectors(ab, ab)
    dot_apap = dot_vectors(ap, ap)
    dot_bpbp = dot_vectors(bp, bp)
    dot_abap = dot_vectors(ab, ap)
    dot_abbp = dot_vectors(ab, bp)
    sq_cos_theta_1 = pow(dot_abap, 2)/dot_abab*dot_apap
    sq_cos_theta_2 = pow(dot_abbp, 2)/dot_abab*dot_bpbp
    cos_theta_1_cos_theta_2 = dot_abap*dot_abbp/(dot_abab*sqrt(dot_apap*dot_bpbp))
    # h, perpendicular distance from P to AB
    sq_h = dot_apap * (1 - sq_cos_theta_1)
    # e, unit vector indicating the dir of induced velocity
    e = cross_vectors(ap, bp)
    e = e/norm_vector(e)
    # induced velocity of this segment
    return Gamma*sqrt(sq_h/(0.001*0.001*0.001*0.001+sq_h*sq_h)/(16*9.86960440108936)\
            *(sq_cos_theta_1 + sq_cos_theta_2 - 2*cos_theta_1_cos_theta_2))*e

# compare two decimals
cdef bint isclose(double a, double b, double rel_tol=1e-09, double abs_tol=0.0) except *:
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# 2-norm of a vector
@cython.boundscheck(False)
@cython.wraparound(False)
cdef double norm_vector(np.ndarray[dtype_t, ndim=1] vector) except *:
    cdef double norm = 0.
    cdef Py_ssize_t i
    for i in range(vector.shape[0]):
        norm += vector[i]**2
    return sqrt(norm)

# dot product of two vectors
@cython.boundscheck(False)
@cython.wraparound(False)
cdef double dot_vectors(np.ndarray[dtype_t, ndim=1] vector_1,\
        np.ndarray[dtype_t, ndim=1] vector_2) except *:
    cdef double dot_product = 0.
    cdef Py_ssize_t i
    for i in range(vector_1.shape[0]):
        dot_product += vector_1[i] * vector_2[i]
    return dot_product

# cross product of two vectors
@cython.boundscheck(False)
@cython.wraparound(False)
cdef np.ndarray[dtype_t, ndim=1] cross_vectors(np.ndarray[dtype_t, ndim=1] vector_1,\
        np.ndarray[dtype_t, ndim=1] vector_2):
    cdef np.ndarray[dtype_t, ndim=1] cross_product
    cross_product = np.array([ vector_1[1]*vector_2[2] - vector_1[2]*vector_2[1],\
            vector_1[2]*vector_2[0] - vector_1[0]*vector_2[2],\
            vector_1[0]*vector_2[1] - vector_1[1]*vector_2[0] ])
    return cross_product
