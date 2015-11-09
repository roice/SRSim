from mayavi import mlab
import numpy as np
x = [1, 2, 3, 4, 5, 5.1, 7, 8, 9]
y = [0, 0, 0, 0, 0, 0, 0, 0, 0]
z = y
s = [.05, .1, .15, .2, .25, .55, .35, .4, .45]
pts = mlab.points3d(x, y, z, s, color = (0,0,0), Transparenscale_factor = 1)
# Retrieve the LUT of the pts object.
lut = pts.module_manager.scalar_lut_manager.lut.table.to_array()

# The lut is a 255x4 array, with the columns representing RGBA
# (red, green, blue, alpha) coded with integers going from 0 to 255.
# We modify the alpha channel to add a transparency gradient
lut[:, -1] = np.linspace(255, 0, 256)
print 'lut[:,-1] = ' + str(lut[:,-1])
#lut[:, 0] = [int(i) for i in np.linspace(0, 160, 256)]
#lut[:, 1] = [int(i) for i in np.linspace(0, 160, 256)]
#lut[:, 2] = [int(i) for i in np.linspace(0, 160, 256)]
# and finally we put this LUT back in the surface object. We could have
# added any 255*4 array rather than modifying an existing LUT.
pts.module_manager.scalar_lut_manager.lut.table = lut
mlab.show()
