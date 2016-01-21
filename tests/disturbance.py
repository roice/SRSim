from __future__ import print_function
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.animation as animation

class AeroOlfactEnvPlot:

    # QuadRotor
    # init pos, central positon of the quadrotor
    qr_init_pos = [2, 2, 2] # meter
    # wheelbase of quadrotor, the distance of two diagonal rotors
    qr_wb = 0.45 # meter
    rotor_radius = 0.15
    # attitude [yaw, pitch, roll]
    qr_init_attitude = [0, 0, 0] # degree

    # Assume that all rotors are two-bladed
    # Right-Hand direction
    # init angles of blades
    psi_init = [0, 0, 0, 0] # degree
    # rotor evolve direction
    psi_dir = [1, -1, 1, -1]


    # init fuction when create an instance of this class
    def __init__(self, command):
        # switch for drawing
        self.draw_vortex_fila = False
        self.draw_flow_vectors = False
        # check validation of plot command, 'command' is a string list, e.g., ['wake_fila', '...']
        for order in command:
            # check if command is valid
            if order == 'vortex_fila':
                self.draw_vortex_fila = True
            elif order == 'flow_vectors':
                self.draw_flow_vectors = True
            else:
                exit('Error: plot command "' + str(order) + \
                    '" is not supported')
        # init computation
        self.compute_init()

    # displays dynamically the aero-olfactory environment of a UAV
    def init(self):
        # create an animating figure
        fig = plt.figure()
        self.ax = axes3d.Axes3D(fig)
        # init plot wake vortex fila
        if self.draw_vortex_fila:
            x = np.array(self.vortex_markers)[:, 0]
            y = np.array(self.vortex_markers)[:, 1]
            z = np.array(self.vortex_markers)[:, 2]
            self.plot_vortex_fila, = self.ax.plot3D(x, y, z)

        # set axes properties
        #self.ax.set_xlim3d([0, 100])
        #self.ax.set_ylim3d([0, 100])
        #self.ax.set_zlim3d([0, 100])

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
            yield self.vortex_markers

    # animation plot update function
    def plot_update(self, plot_data):
        # plot wake vortex fila
        if self.draw_vortex_fila:
            x = np.array(plot_data)[:, 0]
            y = np.array(plot_data)[:, 1]
            z = np.array(plot_data)[:, 2]
            self.plot_vortex_fila.set_data(x, y)
            self.plot_vortex_fila.set_3d_properties(z)

    # compute init
    def compute_init(self):
        # info of markers, [pos_x, pos_y, pos_z]
        # vortex_markers is a iterative_steps(N_markers/4) x 4(rotors) x 3(pos) matrix
        self.vortex_markers = []
        # calculate init vortex markers
        #  calc the position of the init marker of 1st rotor
        Rot_psi = np.array([ [math.cos(self.psi_init[0]), -math.sin(self.psi_init[0]), 0],\
                [math.sin(self.psi_init[0]), math.cos(self.psi_init[0]), 0], [0, 0, 1] ])
        pos_marker_r1 = self.rotate_vector(\
                np.dot(Rot_psi, np.array([0, self.rotor_radius, 0]))\
                + self.qr_wb/2./1.4142135*np.array([-1, 1, 0]),\
                self.qr_init_attitude[0], self.qr_init_attitude[1], self.qr_init_attitude[2])\
                + self.qr_init_pos
        self.vortex_markers.append(pos_marker_r1)
        print(str(self.vortex_markers))
        # init compute step
        self.step = 1

    # wake computation
    def compute_update_wake(self):
        psi = np.array(self.psi_init) + self.step*np.array([10,10,10,10])
        pos_tip = self.position_blade_tips(self.qr_init_pos, self.qr_init_attitude, psi)
        self.vortex_markers.append(pos_tip[0])
        self.step += 1

    # calculate the position of the blade tips
    def position_blade_tips(self, qr_pos, qr_attitude, psi):
        sin_psi = np.array([ math.sin(psi[0]*np.pi/180), math.sin(psi[1]*np.pi/180),\
                math.sin(psi[2]*np.pi/180), math.sin(psi[3]*np.pi/180) ])
        cos_psi = np.array([ math.cos(psi[0]*np.pi/180), math.cos(psi[1]*np.pi/180),\
                math.cos(psi[2]*np.pi/180), math.cos(psi[3]*np.pi/180) ])
        for rotor in range(1):
            Rot_psi_b1 = np.array([ [cos_psi[rotor], -sin_psi[rotor], 0],\
                    [sin_psi[rotor], cos_psi[rotor], 0], [0, 0, 1]])
            Rot_psi_b2 = np.array([ [-cos_psi[rotor], sin_psi[rotor], 0],\
                    [-sin_psi[rotor], -cos_psi[rotor], 0], [0, 0, 1]])
            pos_tip_b1 = self.rotate_vector(\
                np.dot(Rot_psi_b1, np.array([0, self.rotor_radius, 0]))\
                + self.qr_wb/2./1.4142135*np.array([-1, 1, 0]),\
                qr_attitude[0], qr_attitude[1], qr_attitude[2])\
                + qr_pos
            pos_tip_b2 = self.rotate_vector(\
                np.dot(Rot_psi_b2, np.array([0, self.rotor_radius, 0]))\
                + self.qr_wb/2./1.4142135*np.array([-1, 1, 0]),\
                qr_attitude[0], qr_attitude[1], qr_attitude[2])\
                + qr_pos
        return np.array([pos_tip_b1, pos_tip_b2])

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

    def handle_close(self, evt):
        print('Closed, bye~')

###############################################################
# Execute if running this script
if __name__ == '__main__':
    # -- plotting
    plot = AeroOlfactEnvPlot(['vortex_fila'])
    plot.init()
