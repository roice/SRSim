from __future__ import print_function
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.animation as animation

import fvmlib

import scipy.io as sio
import os, sys # for abs path & directory
import time

class AeroOlfactEnvPlot:

    # Helicopter
    # central positon of the helicopter e.g., [x m, y m, z m]
    hc_pos = [0.,0., 3.] # meter
    # attitude [yaw, pitch, roll]
    hc_attitude = [0., 0., 0.] # degree
    # azimuth angle step
    delta_psi = 30. # degree
    # rotational speed
    rotor_rpm = 3000 # rounds per minite
    # vortex circulation
    Gamma = 0.011938486*2


    # init fuction when create an instance of this class
    def __init__(self, command):
        # switch for drawing
        self.draw_vortex_fila = False
        self.draw_flow_vectors = False
        # check validation of plot command, 'command' is a string list, e.g., ['wake_fila', '...']
        for order in command:
            # check if command is valid
            if order == 'octo_copter': # UAV configuration is quad rotor
                self.helicopter_type = 'octo'
            elif order == 'hexa_copter':
                self.helicopter_type = 'hexa'
            elif order == 'quad_copter':
                self.helicopter_type = 'quad'
            elif order == 'tri_copter':
                self.helicopter_type = 'tri'
            elif order == 'bi_copter':
                self.helicopter_type = 'bi'
            elif order == 'single_copter':
                self.helicopter_type = 'single'
            elif order == 'vortex_fila':
                self.draw_vortex_fila = True
            elif order == 'flow_vectors':
                self.draw_flow_vectors = True
            else:
                exit('Error: plot command "' + str(order) + \
                    '" is not supported')
        # init parameters related to copter configuration
        if self.helicopter_type == 'quad':
            # positions of rotors when the helicopter is heading north (y-axis) and centered at the origin
            #  if the helicopter is a quadrotor, then
            #   hc_rotors_pos = [[x_1,y_1,0],[x_2,y_2,0],[x_3,y_3,0],[x_4,y_4,0]] unit: meter
            self.hc_rotors_pos = [ [-0.45/1.4142135, 0.45/1.4142135, 0.],\
                [-0.45/1.4142135, -0.45/1.4142135, 0.],\
                [0.45/1.4142135, -0.45/1.4142135, 0.],\
                [0.45/1.4142135, 0.45/1.4142135, 0.] ]# quad-rotor with wheelbase of 0.45 m, X
            # Assume that all rotors are two-bladed
            #  Right-Hand direction
            #   init azimuth angles of blades, degree
            #    start from y-axis
            self.psi_init = [0., 0., 0., 0.] # quad-rotor
            # rotor evolve direction
            self.psi_dir = [1., -1., 1., -1.] # quad-rotor
            self.rotor_radius = 0.15
        elif self.helicopter_type == 'tri':
            self.hc_rotors_pos = [ [0., 0.2, 0.],\
                [-math.cos(30*np.pi/180)*0.2, -math.sin(30*np.pi/180)*0.2, 0.],\
                [math.cos(30*np.pi/180)*0.2, -math.sin(30*np.pi/180)*0.2, 0.] ] # tri-rotor with wheelbase/2 of 0.2 m
            self.psi_init = [0., 0., 0.] # tri-rotor
            self.psi_dir = [1., -1., 1.] # tri-rotor
            self.rotor_radius = 0.15
        elif self.helicopter_type == 'single':
            self.hc_rotors_pos = [ [0., 0., 0.] ]
            self.psi_init = [0.]
            self.psi_dir = [1.]
            self.rotor_radius = 0.15
        # check if configurations are right
        if not ( (len(self.hc_rotors_pos) == len(self.psi_init))\
                and (len(self.psi_init) == len(self.psi_dir)) ):
            exit('Error: helicopter not appropriately configured')
        # init computation
        self.compute_init()

    # displays dynamically the aero-olfactory environment of a UAV
    def init(self):
        # create an animating figure
        fig = plt.figure()
        self.ax = axes3d.Axes3D(fig)
        # init plot wake vortex fila
        if self.draw_vortex_fila: # if order to draw fila
            self.plot_vortex_fila = [] # list containing 4x2 fila plot handlers
            for i in range(len(self.hc_rotors_pos)):
                x = self.vortex_markers_pos[:, i, 0, 0]
                y = self.vortex_markers_pos[:, i, 0, 1]
                z = self.vortex_markers_pos[:, i, 0, 2]
                b1 = self.ax.plot3D(x, y, z)[0]
                x = self.vortex_markers_pos[:, i, 1, 0]
                y = self.vortex_markers_pos[:, i, 1, 1]
                z = self.vortex_markers_pos[:, i, 1, 2]
                b2 = self.ax.plot3D(x, y, z)[0]
                self.plot_vortex_fila.append([b1, b2])
        # init plot wake flow vectors
        if self.draw_flow_vectors: # if order to draw flow vectors
            # calculate flow vectors, mesh grid 0.1 m
            area = [1., 1., 1.] # helicopter is at the center of this area
            self.mesh_flow_vectors = np.mgrid[\
                    self.hc_pos[0]-area[0]/2:self.hc_pos[0]+area[0]/2:0.2,\
                    self.hc_pos[1]-area[1]/2:self.hc_pos[1]+area[1]/2:0.2,\
                    self.hc_pos[2]-area[2]/2:self.hc_pos[2]+area[2]/2:0.2 ]
            vectors = self.calc_flow_vectors(self.mesh_flow_vectors)
            if vectors == None:
                x, y, z = self.mesh_flow_vectors
                u, v, w = np.zeros_like(self.mesh_flow_vectors)
                self.plot_flow_vectors = self.ax.quiver3D(x, y, z, u, v, w, length=0.05)
        # preserve equal ratio of axis coordinates
        plt.axis('equal')
        # setup animation
        ani = animation.FuncAnimation(fig, self.plot_update, self.plot_data_generator,\
                blit=False, interval=10, repeat=False)
        # set close handle
        fig.canvas.mpl_connect('close_event', self.handle_close)
        # GUI loop
        plt.show()

    # data generator for animation
    def plot_data_generator(self):
        while True:
            self.compute_update_wake()
            yield self.vortex_markers_pos

    # animation plot update function
    def plot_update(self, plot_data):
        # adjust axis ranges
        self.ax.set_xlim3d([self.hc_pos[0] - 0.3, self.hc_pos[0] + 0.3])
        self.ax.set_ylim3d([self.hc_pos[1] - 0.3, self.hc_pos[1] + 0.3])
        self.ax.set_zlim3d([self.hc_pos[2] - 0.3, self.hc_pos[2] + 0.3])
        # plot wake vortex fila
        if self.draw_vortex_fila: # if order to draw fila
            for i in range(len(self.hc_rotors_pos)):
                for j in range(2): # 2 blades
                    x = plot_data[:, i, j, 0]
                    y = plot_data[:, i, j, 1]
                    z = plot_data[:, i, j, 2]
                    self.plot_vortex_fila[i][j].set_data(x, y)
                    self.plot_vortex_fila[i][j].set_3d_properties(z)
        # plot wake flow vectors
        if self.draw_flow_vectors: # if order to draw flow vectors
            x, y, z = self.mesh_flow_vectors
            u, v, w = self.calc_flow_vectors(self.mesh_flow_vectors)
            self.ax.collections.remove(self.plot_flow_vectors)
            self.plot_flow_vectors = self.ax.quiver3D(x, y, z, u, v, w, length=0.05)

    # compute init
    def compute_init(self):
        # pos of markers, [pos_x, pos_y, pos_z]
        # vortex_markers_pos is a iterative_steps(N_markers/8) x 4(rotors) x 2(blades) x 3(pos) matrix
        # calculate init vortex markers
        self.vortex_markers_pos = np.array([self.position_blade_tips(self.hc_pos, self.hc_attitude, self.psi_init)])
        # calculate rotation speed, assume speed of all rotors are equal
        self.Omega = self.rotor_rpm*2*np.pi/60 # rad/s
        # calculate delta time
        self.delta_t = self.delta_psi/self.Omega # s
        # init compute step
        self.step = 1
        # at least 3 markers before update
        self.release_new_marker()
        self.release_new_marker()
        # create a directory named as: srsim_record_[date]_[time]
        p = sys.path[0] # dir of this script
        self.record_dir = p[0:p.index('SRSim/tests')] + 'vf_record_' + \
                time.strftime('%Y-%m-%d_%H%M%S',time.localtime(time.time()))
        print('dir='+str(self.record_dir))
        os.mkdir(self.record_dir)

    # wake computation
    def compute_update_wake(self):
        ############### Simple Backward Difference #################
        # update positions of markers
        fvmlib.VF_markers_update_PIPC(self.vortex_markers_pos, np.array(self.psi_dir), self.delta_t)
        # release new marker
        self.release_new_marker()
        # delete oldest marker
        N_m = len(self.vortex_markers_pos)
        if N_m > (360/self.delta_psi)*20: # 20 rounds
            self.vortex_markers_pos = np.delete(self.vortex_markers_pos, 0, axis=0)
        # create a file containing data of this moment/step
        #  name: vortex_fila_[step].mat
        sio.savemat(self.record_dir + '/vortex_fila_' + str(self.step) + '.mat', \
                {'vortex_fila': self.vortex_markers_pos})

    def release_new_marker(self):
        # release new markers
        psi = np.array(self.psi_init) +\
                self.step*np.array([self.delta_psi*self.psi_dir[i] for i in range(len(self.psi_dir))])
        pos_tips = self.position_blade_tips(self.hc_pos, self.hc_attitude, psi)
        self.vortex_markers_pos = np.append(self.vortex_markers_pos, np.array([pos_tips]), axis=0)
        # update step index
        self.step += 1

    # calculate the position of the blade tips
    def position_blade_tips(self, hc_pos, hc_attitude, psi):
        # get number of rotors
        N_r = len(self.hc_rotors_pos)
        tips = []
        for rotor in range(N_r):
            pos_tip_b1 = self.rotate_vector(\
                self.rotate_vector(np.array([0, self.rotor_radius, 0]), psi[rotor], 0, 0)\
                + np.array(self.hc_rotors_pos[rotor]),\
                hc_attitude[0], hc_attitude[1], hc_attitude[2])\
                + hc_pos
            pos_tip_b2 = self.rotate_vector(\
                self.rotate_vector(np.array([0, self.rotor_radius, 0]), psi[rotor]+180, 0, 0)\
                + np.array(self.hc_rotors_pos[rotor]),\
                hc_attitude[0], hc_attitude[1], hc_attitude[2])\
                + hc_pos
            tips.append([pos_tip_b1, pos_tip_b2])
        return np.array(tips)

    # rotate a vector with angles yaw pitch roll (degree)
    # Right-Hand(counterclockwise)
    #
    #              cos(yaw)  -sin(yaw)  0
    # R_z(yaw)   = sin(yaw)  cos(yaw)   0
    #                 0         0       1
    #
    #              cos(pitch)   0     sin(pitch)
    # R_y(pitch) =     0        1         0
    #              -sin(pitch)  0     cos(pitch)
    #
    #                  1        0         0
    # R_x(roll)  =     0    cos(roll)  -sin(roll)
    #                  0    sin(roll)   cos(roll)
    #
    # R(yaw, pitch, roll) = R_z(yaw)R_y(pitch)R_x(roll)
    def rotate_vector(self, vector, yaw, pitch, roll):
        # calculate rotation matrix
        sin_yaw = math.sin(yaw*np.pi/180.)
        cos_yaw = math.cos(yaw*np.pi/180.)
        sin_pitch = math.sin(pitch*np.pi/180.)
        cos_pitch = math.cos(pitch*np.pi/180.)
        sin_roll = math.sin(roll*np.pi/180.)
        cos_roll = math.cos(roll*np.pi/180.)
        R_z = np.array([[cos_yaw, -sin_yaw, 0],\
                [sin_yaw, cos_yaw, 0], [0, 0, 1]])
        R_y = np.array([[cos_pitch, 0, sin_pitch],\
                [0, 1, 0], [-sin_pitch, 0, cos_pitch]])
        R_x = np.array([[1, 0, 0],\
                [0, cos_roll, -sin_roll], [0, sin_roll, cos_roll]])
        return np.dot(np.dot(np.dot(R_z, R_y), R_x), vector)

    # Induced velocity calculation function
    # compute the induced velocity at a point
    def calc_induced_vel(self, point):
        markers = np.array(self.vortex_markers)
        N_m = len(markers) # number of straight-line vortex segments
        N_r = len(self.hc_rotors_pos) # number of rotors
        N_b = 2 # only two-blade is considered
        p = point # position of this point
        ind_v = np.array([0., 0., 0.]) # overall induced velocity at this point
        for index_m in range(N_m-1):
            for index_r in range(N_r):
                for index_b in range(N_b):
                    if self.psi_dir[index_r] > 1: # counter-clockwise
                        a = markers[index_m, index_r, index_b] # point A of segment
                        b = markers[index_m+1, index_r, index_b] # point B of segment
                    else: # clockwise
                        a = markers[index_m+1, index_r, index_b]
                        b = markers[index_m, index_r, index_b]
                    # AP, BP and AB vector
                    ap = p - a
                    bp = p - b
                    ab = b - a

                    if (np.isclose(a, b) == np.array([True, True, True])).all()\
                            or (np.isclose(a, p) == np.array([True, True, True])).all()\
                            or (np.isclose(b, p) == np.array([True, True, True])).all():
                        continue

                    # cos \theta_1 and cos \theta_2
                    cos_theta_1 = np.dot(ab, ap)/(np.linalg.norm(ab)*np.linalg.norm(ap))
                    cos_theta_2 = np.dot(ab, bp)/(np.linalg.norm(ab)*np.linalg.norm(bp))
                    if cos_theta_1 - cos_theta_2 <= 0.0000000001:
                        continue
                    # h, perpendicular distance from P to AB
                    h = np.linalg.norm(ap) * math.sqrt(1 - math.pow(cos_theta_1, 2))
                    # e, unit vector indicating the dir of induced velocity
                    e = np.cross(ap, bp)
                    e = e/np.linalg.norm(e)
                    # induced velocity of this segment
                    ind_v += self.Gamma/(4*np.pi)*(h/math.sqrt(math.pow(0.001,4)+math.pow(h,4)))*(cos_theta_1-cos_theta_2)*e
        return ind_v

    # non-critical
    # calculation of flow vectors, for display
    def calc_flow_vectors(self, mesh):
        markers = np.array(self.vortex_markers)
        if len(markers) >= 2: # at least 2 markers
            vectors = np.zeros_like(mesh)
            N_m = len(markers) # number of straight-line vortex segments
            N_r = len(self.hc_rotors_pos) # number of rotors
            N_b = 2 # only two-blade is considered
            for i in range(len(mesh[0])): # x index
                for j in range(len(mesh[0, 0])): # y index
                    for k in range(len(mesh[0, 0, 0])): # z index
                        p = mesh[:, i, j, k] # position of this grid
                        ind_v = np.array([0., 0., 0.]) # overall induced velocity at this point
                        for index_m in range(N_m-1):
                            for index_r in range(N_r):
                                for index_b in range(N_b):
                                    if self.psi_dir[index_r] > 1: # counter-clockwise
                                        a = markers[index_m, index_r, index_b] # point A of segment
                                        b = markers[index_m+1, index_r, index_b] # point B of segment
                                    else: # clockwise
                                        a = markers[index_m+1, index_r, index_b]
                                        b = markers[index_m, index_r, index_b]
                                    # AP, BP and AB vector
                                    ap = p - a
                                    bp = p - b
                                    ab = b - a
                                    # cos \theta_1 and cos \theta_2
                                    cos_theta_1 = np.dot(ab, ap)/(np.linalg.norm(ab)*np.linalg.norm(ap))
                                    cos_theta_2 = np.dot(ab, bp)/(np.linalg.norm(ab)*np.linalg.norm(bp))
                                    # h, perpendicular distance from P to AB
                                    h = np.linalg.norm(ap) * math.sqrt(1 - math.pow(cos_theta_1, 2))
                                    # e, unit vector indicating the dir of induced velocity
                                    e = np.cross(ap, bp)
                                    e = e/np.linalg.norm(e)
                                    # induced velocity of this segment
                                    ind_v += self.Gamma/(4*np.pi*h)*(cos_theta_1-cos_theta_2)*e
                        vectors[:, i, j, k] = ind_v
            return vectors
        else:
            return None

    def handle_close(self, evt):
        print('Closed, bye~')

###############################################################
# Execute if running this script
if __name__ == '__main__':
    # -- plotting
    plot = AeroOlfactEnvPlot(['single_copter', 'vortex_fila'])
    plot.init()
