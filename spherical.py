#!/usr/bin/env python
# import matplotlib.pyplot as plt
# from matplotlib import cm, colors
# from mpl_toolkits.mplot3d import Axes3D
# import numpy as np
# from scipy.special import sph_harm
# import seaborn as sns
# import pdb, sys
import matplotlib.animation as animation
from mayavi import mlab
import numpy as np
from numpy import pi, sin, cos, mgrid
N = 40 # color grid
phi = np.linspace(0, np.pi, N)
theta = np.linspace(0, 2*np.pi, N)
phi, theta = np.meshgrid(phi, theta)
#
# # The Cartesian coordinates of the unit sphere
#
x = np.sin(phi) * np.cos(theta)
y = np.sin(phi) * np.sin(theta)
z = np.cos(phi)

dphi, dtheta = pi/250.0, pi/250.0
[phi,theta] = mgrid[0:pi+dphi*1.5:dphi,0:2*pi+dtheta*1.5:dtheta]
m0 = 4; m1 = 3; m2 = 2; m3 = 3; m4 = 6; m5 = 2; m6 = 6; m7 = 4;
r = 0.3
x = r*sin(phi)*cos(theta)
y = r*cos(phi)
z = r*sin(phi)*sin(theta)

#
# cp =  sns.color_palette("hls", z.shape[0])
# cp_fill=[]
# for j in range(N):
#   for i in range(N):
#     cp_fill.append( cp[j % N] + (1.,))
#
# cp_fill = np.array(cp_fill).reshape((N,N,4))
# m, l = 2, 3
#
# # Calculate the spherical harmonic Y(l,m) and normalize to [0,1]
# fcolors = sph_harm(m, l, theta, phi).real
# fmax, fmin = fcolors.max(), fcolors.min()
# fcolors = (fcolors - fmin)/(fmax - fmin)
#
#
#
# # Set the aspect ratio to 1 so our sphere looks spherical
# fig = plt.figure(figsize=plt.figaspect(1.))
# ax = fig.add_subplot(111, projection='3d')
# splt = ax.plot_surface(x, y, z,  rstride=1, cstride=1, facecolors=cp_fill)
# # Turn off the axis planes
# ax.set_axis_off()
# t = range(N)
import time

n_mer, n_long = 6, 11
pi = np.pi
dphi = pi/1000.0
phi = np.arange(0.0, 2*pi + 0.5*dphi, dphi, 'd')
mu = phi*n_mer
x = np.cos(mu)*(1+np.cos(n_long*mu/n_mer)*0.5)
y = np.sin(mu)*(1+np.cos(n_long*mu/n_mer)*0.5)
z = np.sin(n_long*mu/n_mer)*0.5

# View it.
l = mlab.plot3d(x, y, z, np.sin(mu), tube_radius=0.025, colormap='Spectral')

# Now animate the data.
ms = l.mlab_source
for i in range(10):
    x = np.cos(mu)*(1+np.cos(n_long*mu/n_mer +
                                      np.pi*(i+1)/5.)*0.5)
    scalars = np.sin(mu + np.pi*(i+1)/5)
    ms.set(x=x, scalars=scalars)
    time.sleep(0.2)



# fig = mlab.gcf()
# ms = s.mlab_source
# for i in range(5):
#     x, y = np.mgrid[0:3:1.0/(i+2),0:3:1.0/(i+2)]
#     sc = np.asarray(x*x*0.05*(i+1), 'd')
#     ms.reset(x=x, y=y, scalars=sc)
#     fig.scene.reset_zoom()
# surf = mlab.surf(phi,theta,f)
# mlab.show()
# sys.exit()

# def animate(i,t,splt,cp_fill):
#   ax.clear()
#   cp_fill = np.roll(cp_fill,shift = 2 ,axis= 0)
#   N = len(t)
#   phi = np.linspace(0., np.pi, N)
#   theta = np.linspace( 0., 2.*np.pi, N)
#   phi, theta = np.meshgrid(phi, theta)
#   x = np.sin(phi) * np.cos(theta)
#   y = np.sin(phi) * np.sin(theta)
#   z = np.cos(phi)
#
#   # apply rotation here
#   position = np.vstack((x,y,z))
#
#
#   splt = ax.plot_surface(x, y, z,  rstride=1, cstride=1, facecolors=cp_fill)
#   return splt,
# ani = animation.FuncAnimation(fig, animate, fargs=(t,splt,cp_fill),interval = 30, blit=False)
#
# plt.show()
