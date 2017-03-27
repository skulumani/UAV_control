#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib import cm, colors
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.special import sph_harm
import seaborn as sns
import pdb, sys
import matplotlib.animation as animation

N = 40 # color grid
phi = np.linspace(0, np.pi, N)
theta = np.linspace(0, 2*np.pi, N)
phi, theta = np.meshgrid(phi, theta)

# The Cartesian coordinates of the unit sphere

x = np.sin(phi) * np.cos(theta)
y = np.sin(phi) * np.sin(theta)
z = np.cos(phi)

cp =  sns.color_palette("hls", z.shape[0])
cp_fill=[]
for j in range(N):
  for i in range(N):
    cp_fill.append( cp[j % N] + (1.,))

cp_fill = np.array(cp_fill).reshape((N,N,4))
m, l = 2, 3

# Calculate the spherical harmonic Y(l,m) and normalize to [0,1]
fcolors = sph_harm(m, l, theta, phi).real
fmax, fmin = fcolors.max(), fcolors.min()
fcolors = (fcolors - fmin)/(fmax - fmin)



# Set the aspect ratio to 1 so our sphere looks spherical
fig = plt.figure(figsize=plt.figaspect(1.))
ax = fig.add_subplot(111, projection='3d')
splt = ax.plot_surface(x, y, z,  rstride=1, cstride=1, facecolors=cp_fill)
# Turn off the axis planes
ax.set_axis_off()
t = range(N)

def animate(i,t,splt,cp_fill):
  ax.clear()
  cp_fill = np.roll(cp_fill,shift = 0 ,axis= 0)
  N = len(t)
  phi = np.linspace(0., np.pi, N)
  theta = np.linspace( 0., 2.*np.pi, N)
  phi, theta = np.meshgrid(phi, theta)
  x = np.sin(phi) * np.cos(theta)
  y = np.sin(phi) * np.sin(theta)
  z = np.cos(phi)

  # apply rotation here

  splt = ax.plot_surface(x, y, z,  rstride=1, cstride=1, facecolors=cp_fill)
  return splt,
ani = animation.FuncAnimation(fig, animate, fargs=(t,splt,cp_fill),interval = 30, blit=False)

plt.show()
