from __future__ import print_function
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import matplotlib.animation as animation

class AeroOlfactEnvPlot:

    # Helicopter
    # central positon of the helicopter e.g., [x m, y m, z m]
    hc_pos = [1, 2, 3] # meter
    # attitude [yaw, pitch, roll]
    hc_attitude = [20, 20, 20] # degree
    # azimuth angle step
    delta_psi = 1 # degree


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
            self.hc_rotors_pos = [ [-0.45/1.4142135, 0.45/1.4142135, 0],\
                [-0.45/1.4142135, -0.45/1.4142135, 0],\
                [0.45/1.4142135, -0.45/1.4142135, 0],\
                [0.45/1.4142135, 0.45/1.4142135, 0] ]# quad-rotor with wheelbase of 0.45 m, X
            # Assume that all rotors are two-bladed
            #  Right-Hand direction
            #   init azimuth angles of blades, degree
            #    start from y-axis
            self.psi_init = [0, 0, 0, 0] # quad-rotor
            # rotor evolve direction
            self.psi_dir = [1, -1, 1, -1] # quad-rotor
            self.rotor_radius = 0.15
        elif self.helicopter_type == 'tri':
            self.hc_rotors_pos = [ [0, 0.2, 0],\
                [-math.cos(30*np.pi/180)*0.2, -math.sin(30*np.pi/180)*0.2, 0],\
                [math.cos(30*np.pi/180)*0.2, -math.sin(30*np.pi/180)*0.2, 0] ] # tri-rotor with wheelbase/2 of 0.2 m
            self.psi_init = [0, 0, 0] # tri-rotor
            self.psi_dir = [1, -1, 1] # tri-rotor
            self.rotor_radius = 0.15
        elif self.helicopter_type == 'single':
            self.hc_rotors_pos = [ [0, 0, 0] ]
            self.psi_init = [0]
            self.psi_dir = [1]
            self.rotor_radius = 0.5
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
                x = np.array(self.vortex_markers)[:, i, 0, 0]
                y = np.array(self.vortex_markers)[:, i, 0, 1]
                z = np.array(self.vortex_markers)[:, i, 0, 2]
                b1 = self.ax.plot3D(x, y, z)[0]
                x = np.array(self.vortex_markers)[:, i, 1, 0]
                y = np.array(self.vortex_markers)[:, i, 1, 1]
                z = np.array(self.vortex_markers)[:, i, 1, 2]
                b2 = self.ax.plot3D(x, y, z)[0]
                self.plot_vortex_fila.append([b1, b2])
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
            yield np.array(self.vortex_markers)

    # animation plot update function
    def plot_update(self, plot_data):
        # plot wake vortex fila
        if self.draw_vortex_fila: # if order to draw fila
            for i in range(len(self.hc_rotors_pos)):
                for j in range(2): # 2 blades
                    x = plot_data[:, i, j, 0]
                    y = plot_data[:, i, j, 1]
                    z = plot_data[:, i, j, 2]
                    self.plot_vortex_fila[i][j].set_data(x, y)
                    self.plot_vortex_fila[i][j].set_3d_properties(z)

    # compute init
    def compute_init(self):
        # info of markers, [pos_x, pos_y, pos_z]
        # vortex_markers is a iterative_steps(N_markers/4) x 4(rotors) x 2(blades) x 3(pos) matrix
        self.vortex_markers = []
        # calculate init vortex markers
        self.vortex_markers.append(self.position_blade_tips(self.hc_pos, self.hc_attitude, self.psi_init))
        # init compute step
        self.step = 1

    # wake computation
    def compute_update_wake(self):
        # release new markers
        psi = np.array(self.psi_init) +\
                self.step*np.array([self.delta_psi*self.psi_dir[i] for i in range(len(self.psi_dir))])
        pos_tips = self.position_blade_tips(self.hc_pos, self.hc_attitude, psi)
        self.vortex_markers.append(pos_tips)

        '''
        if len(self.vortex_markers) > 4:
            for i in range(4):
                a = self.vortex_markers[i]-self.vortex_markers[i+1]
                length = math.sqrt(math.pow(a[0], 2) + math.pow(a[1], 2) + math.pow(a[2], 2))
                print('len'+str(i)+'to'+str(i+1)+'is'\
                        +str(length))
        '''

        # adjust axis ranges
        self.ax.set_xlim3d([self.hc_pos[0] - 1, self.hc_pos[0] + 1])
        self.ax.set_ylim3d([self.hc_pos[1] - 1, self.hc_pos[1] + 1])
        self.ax.set_zlim3d([self.hc_pos[2] - 1, self.hc_pos[2] + 1])
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

    def handle_close(self, evt):
        print('Closed, bye~')

###############################################################
# Execute if running this script
if __name__ == '__main__':
    # -- plotting
    plot = AeroOlfactEnvPlot(['single_copter', 'vortex_fila'])
    plot.init()
