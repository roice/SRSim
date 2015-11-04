#from fipy import Grid2D
import numpy as np
from fipy import Grid2D, CellVariable, DiffusionTerm, Viewer
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

class Advection2D():
    # sim area size
    gsize = None # grid size
    xy_n = None  # x,y, grid numbers, e.g., (100, 100) for square sim area
                 # of edge length 100*gsize
    # mean wind vector
    mean_flow = None
    # colored noise process parameters
    wind_damping = None
    wind_bandwidth = None
    G = None
    # simulation period, dt (second)
    dt = None
    # wind vector random increment of vertex points, static variables
    vertexWindVecRanInc = None
    # wind mesh cell centers
    wind_mesh_centers = None
    # wind result /phi
    wind_phi_field = None

    # imcompressible, irrotational, rectangular sim area
    # positions of 4 vertexes:
    #       vertex 0 ------------ vertex 3
    #          |                     |
    #          |                     |
    #          |                     |
    #       vertex 1 ------------ vertex 2
    def icir_rect(self):
        # step 1: Mesh
        mesh = Grid2D(dx=self.gsize, dy=self.gsize, nx=self.xy_n[0], ny=self.xy_n[1])

        # step 2: Equation
        phi = CellVariable(mesh=mesh, name='potential phi', value=0.)
        eqn = (DiffusionTerm(coeff = 1.) == 0.)

        # step 3: Boundary conditions
        # compute flow of 4 vertexes
        # one vertex has 2 components of wind vector, (x, y)
        vertexWindVec = np.array([ [0.0 for i in range(2)] for i in range(4)])
        for i in range(4): # 4 vertexes
            for j in range(2): # 2 components
                vertexWindVec[i, j] = self.mean_flow[j] \
                        + self.colored_noise(self.vertexWindVecRanInc[i][j])
        # interpolate flow vector on sim area edges, and set neumann boundary
        #   conditions, because /grad /phi = V(x,y)
        # get all points which lie on the center of edges of cells
        X, Y = mesh.faceCenters
        # /grad /phi array, of points of mesh face centers
        #   grad_phi_bc[0, :]  /grad/phi_x
        #   grad_phi_bc[1, :]  /grad/phi_y
        grad_phi_bc = np.zeros_like(mesh.faceCenters())
        # p: points on one edge to interpolate, 1 dimension
        # vx, vy: x&y components of interpolated wind vectors of points list p
        # vertex index, for interpolate bc on 4 edges
        # vertexIndex[0] = [1,2], down  boundary, 0<x<nx*dx, y=0
        # vertexIndex[1] = [1,0], left  boundary, x=0, 0<y<ny*dy
        # vertexIndex[2] = [0,3], up    boundary, 0<x<nx*dx, y=ny*dy
        # vertexIndex[3] = [2,3], right boundary, x=nx*dx, 0<y<ny*dy
        vertexIndex = np.array([ [1,2], [1,0], [0,3], [2,3] ])
        for i in range(4): # 4 edges for 2D rect area
            p = np.arange(self.gsize/2, self.gsize*self.xy_n[i%2], self.gsize)
            vx = np.interp(p, [0.0, self.gsize*self.xy_n[i%2]], \
                    [vertexWindVec[vertexIndex[0,0], 0], \
                    vertexWindVec[vertexIndex[0,1], 0]]).T.reshape(1,-1)[0]
            vy = np.interp(p, [0.0, self.gsize*self.xy_n[i%2]], \
                    [vertexWindVec[vertexIndex[0,0], 1], \
                    vertexWindVec[vertexIndex[0,1], 1]]).T.reshape(1,-1)[0]
            print 'vx = ' + str(vx)
            if i == 0: # down boundary
                grad_phi_bc[:, mesh.facesDown()] = np.array([vx, vy])
            elif i == 1: # left boundary
                grad_phi_bc[:, mesh.facesLeft()] = np.array([vx, vy])
            elif i == 2: # up boundary
                grad_phi_bc[:, mesh.facesUp()] = np.array([vx, vy])
            elif i == 3: # right boundary
                grad_phi_bc[:, mesh.facesRight()] = np.array([vx, vy])
        # set neumann boundary condition
        phi.faceGrad.constrain(((grad_phi_bc[0]),(grad_phi_bc[1])), where=mesh.exteriorFaces)

        # step 4: Solve
        eqn.solve(var=phi)
        #print str(phi)
        #print str(type(np.array(phi)))
        self.wind_phi_field = np.array(phi)
        self.wind_mesh_centers = mesh.cellCenters()
        #self.wind_field = np.array(phi).reshape(self.xy_n[0], self.xy_n[1])

    # init class attributes
    def __init__(self):
        self.gsize = 1.0    # float
        self.xy_n = [30, 40]
        self.mean_flow = [10.0, 2.0]
        self.wind_damping = 0
        self.wind_bandwidth = 0
        self.G = 0
        self.dt = 1.0
        # simulation area's shape is rectangular, which has 4 vertexes
        # one vertex has 2 components of wind vector, (x, y)
        # one component of wind vector has 2 elements, e.g., x and x', y and y'
        self.vertexWindVecRanInc = [ [ [0 for i in range(2)] for i in range(2)] for i in range(4)]

    # colored noise process
    #
    # input: u(t), white noise, 'standard normal',
    #        unit variance normal distribution (mean 0 variance 1)
    # output: x(t), wind variance
    #
    # transfer function (G * classical second-order system):
    #
    #                       w_n^2
    # H(s) = G * ---------------------------
    #               s^2 + 2*z*w_n + w_n^2
    #                                       wind_bandwidth^2
    #      = wind_meander*------------------------------------------------------
    #                     s^2 + 2*wind_damping*wind_bandwidth + wind_bandwidth^2
    # Time domain:
    #   d^2                                        d
    #  -----x(t) + 2*wind_damping*wind_bandwidth*----x(t) + wind_bandwidth^2*x(t)
    #  dt^2                                       dt
    #      = G*wind_bandwidth^2*u(t)
    def colored_noise(self, x):
        # sample from standard normal
        u = np.random.randn()
        # init local params
        dx = [0, 0]
        #        d
        # eq 1: ----x(t) = x'
        #        dt
        dx[0] = x[1]
        #        d^2                                        d
        # eq 2: ----x(t) = -2*wind_damping*wind_bandwidth*----x(t)
        #       dt^2                                       dt
        #                 + wind_bandwidth^2*(G*u(t) - x(t))
        dx[1] = -2*self.wind_damping*self.wind_bandwidth*dx[0] \
                + self.wind_bandwidth*self.wind_bandwidth*(self.G*u-x[0])
        #                  d^2
        # eq 3: x' = /int -----x(t)
        #                 dx^2
        x[1] += dx[1] * self.dt
        # eq 4: x = /int x'
        x[0] += x[1] * self.dt
        return x[0]

##############################################################################
# Execute if running this script
if __name__ == '__main__':
    adv = Advection2D()
    adv.icir_rect()
    # convert wind phi field (scalar) to wind vector field
    wind_phi_field = adv.wind_phi_field.reshape(adv.xy_n[1], adv.xy_n[0]).T
    wind_vector_field = np.gradient(wind_phi_field, adv.gsize)
    u, v = wind_vector_field
    print str(u)
    #print str(adv.wind_mesh_centers[0].shape)
    # convert wind phi field to mgrid type for matplotlib plotting
    X, Y = np.mgrid[adv.gsize/2.0:adv.gsize*adv.xy_n[0]:adv.gsize, \
            adv.gsize/2.0:adv.gsize*adv.xy_n[1]:adv.gsize]
    #print str(X.shape)
    U = griddata(np.array(zip(adv.wind_mesh_centers[0], adv.wind_mesh_centers[1])),\
            u.reshape(-1,1), (X,Y), method='nearest')
    V = griddata(np.array(zip(adv.wind_mesh_centers[0], adv.wind_mesh_centers[1])),\
            v.reshape(-1,1), (X,Y), method='nearest')
    #print str(U)
    plt.quiver(X,Y,u,v)
    plt.show()




