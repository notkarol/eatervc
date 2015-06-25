#!/usr/bin/env python3

import numpy
from scipy.special import erf
import matplotlib.pyplot as plt

fig, ((ax1, ax2, ax3, ax4)) = plt.subplots(nrows=1, ncols=4)
fig.set_figheight(3)
fig.set_figwidth(15)

s = 2
sizes = [1.2, 1.8]
conds = {'vH4': [[sizes[0], -3.0 * s, -6.5 * s],
                 [sizes[0],  3.0 * s, -6.5 * s],
                 [sizes[1], -3.0 * s, -6.5 * s],
                 [sizes[1],  3.0 * s, -6.5 * s]],
         'vV4': [[sizes[0],  0.0 * s, -5.75 * s],
                 [sizes[0],  0.0 * s, -7.25 * s],
                 [sizes[1],  0.0 * s, -5.75 * s],
                 [sizes[1],  0.0 * s, -7.25 * s]],
         'vA4': [[sizes[0], -3.0 * s, -5.75 * s],
                 [sizes[0],  3.0 * s, -7.25 * s],
                 [sizes[1], -3.0 * s, -7.25 * s],
                 [sizes[1],  3.0 * s, -5.75 * s]],
         'vH6': [[sizes[0], -4.0 * s, -6.5 * s],
                 [sizes[0],  0.0 * s, -6.5 * s],
                 [sizes[0],  4.0 * s, -6.5 * s],
                 [sizes[1], -4.0 * s, -6.5 * s],
                 [sizes[1],  0.0 * s, -6.5 * s],
                 [sizes[1],  4.0 * s, -6.5 * s]],
         'vV6': [[sizes[0],  0.0 * s, -5.25 * s],
                 [sizes[0],  0.0 * s, -6.5 * s],
                 [sizes[0],  0.0 * s, -7.75 * s],
                 [sizes[1],  0.0 * s, -5.25 * s],
                 [sizes[1],  0.0 * s, -6.5 * s],
                 [sizes[1],  0.0 * s, -7.75 * s]],
         'vA6': [[sizes[0], -4.0 * s, -5.75 * s],
                 [sizes[0],  0.0 * s, -7.25 * s],
                 [sizes[0],  4.0 * s, -5.75 * s],
                 [sizes[1], -4.0 * s, -7.25 * s],
                 [sizes[1],  0.0 * s, -5.75 * s],
                 [sizes[1],  4.0 * s, -7.25 * s]],
         'vH8': [[sizes[0], -4.5 * s, -6.5 * s],
                 [sizes[0], -1.5 * s, -6.5 * s],
                 [sizes[0],  1.5 * s, -6.5 * s],
                 [sizes[0],  4.5 * s, -6.5 * s],
                 [sizes[1], -4.5 * s, -6.5 * s],
                 [sizes[1], -1.5 * s, -6.5 * s],
                 [sizes[1],  1.5 * s, -6.5 * s],
                 [sizes[1],  4.5 * s, -6.5 * s]],
         'vA8': [[sizes[0], -4.5 * s, -7.25 * s],
                 [sizes[0], -1.5 * s, -5.75 * s],
                 [sizes[0],  1.5 * s, -7.25 * s],
                 [sizes[0],  4.5 * s, -5.75 * s],
                 [sizes[1], -4.5 * s, -5.75 * s],
                 [sizes[1], -1.5 * s, -7.25 * s],
                 [sizes[1],  1.5 * s, -5.75 * s],
                 [sizes[1],  4.5 * s, -7.25 * s]],
         'vV8': [[sizes[0],  0.0 * s, -8.375 * s],
                 [sizes[0],  0.0 * s, -7.125 * s],
                 [sizes[0],  0.0 * s, -5.875 * s],
                 [sizes[0],  0.0 * s, -4.625 * s],
                 [sizes[1],  0.0 * s, -8.375 * s],
                 [sizes[1],  0.0 * s, -7.125 * s],
                 [sizes[1],  0.0 * s, -5.875 * s],
                 [sizes[1],  0.0 * s, -4.625 * s]],
         'v32': [[x,y,z] for x in sizes for y in numpy.linspace(-6 * s, 6 * s, 4) for z in numpy.linspace(-9 * s, -4 * s, 4)],
         'v00': [[x,y,z] for x in sizes for y in numpy.linspace(-6 * s, 6 * s, 13) for z in numpy.linspace(-9 * s, -4 * s, 6)]}

ms = 9
ax1.set_title('Vertical (V6)')
ax2.set_title('Horizontal (H6)')
ax3.set_title('Alternating (A6)')
ax4.set_title('Testing')

for cond, ax in zip(['vV6', 'vH6', 'vA6', 'v00'], [ax1, ax2, ax3, ax4]):

    for size, x, z in reversed(conds[cond]):
        ax.plot(x, abs(z), 'o', ms = ms * size, color='w',  mfc='none')

    ax.plot([0], [0], 's', ms=ms, color='g', markeredgecolor='g')
    ax.plot([-17, 0], [1, 1], lw=8, color='r')
    ax.plot([0, 17], [1, 1], lw=8, color='r')
    ax.plot([17, 17], [1, 17], lw=8, color='r')
    ax.plot([-17, -17], [1, 17], lw=8, color='r')
    ax.plot([-1], [1], 'o', color='y', markeredgecolor='y')
    ax.plot([1], [1], 'o', color='m', markeredgecolor='m')
    ax.plot([17], [1], 'o', color='c', markeredgecolor='c')
    ax.plot([-17], [1], 'o', color='k', markeredgecolor='k')
    
    ax.set_xlim([-18, 18])
    ax.set_ylim([-1, 23])
    #ax.set_xlabel('X Axis')
    #ax.set_ylabel('Z Axis')
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_xticks([])
    ax.set_yticks([])
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_visible(False)
    ax.spines['left'].set_visible(False)
    #ax.grid()

    ax.set_aspect('equal', 'datalim')
    alpha = 0.15
    dist = 30
    for i in numpy.arange(2.5, 45, 5):
        ang = numpy.pi * i / 180
        ax.plot([0, dist * numpy.cos(ang)], [0, dist * numpy.sin(ang)], alpha=alpha, color='c')
    for i in numpy.arange(47.5, 90, 5):
        ang = numpy.pi * i / 180
        ax.plot([0, dist * numpy.cos(ang)], [0, dist * numpy.sin(ang)], alpha=alpha, color='m')
    for i in numpy.arange(92.5, 135, 5):
        ang = numpy.pi * i / 180
        ax.plot([0, dist * numpy.cos(ang)], [0, dist * numpy.sin(ang)], alpha=alpha, color='y')
    for i in numpy.arange(137.5, 180, 5):
        ang = numpy.pi * i / 180
        ax.plot([0, dist * numpy.cos(ang)], [0, dist * numpy.sin(ang)], alpha=alpha, color='k')

#fig.tight_layout()
#plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
plt.savefig('positions.png', bbox_inches='tight', dpi=200)
plt.savefig('positions.eps', bbox_inches='tight', dpi=200)
#plt.show()



