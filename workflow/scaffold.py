#!/usr/bin/env python3

import numpy
from scipy.special import erf
import matplotlib.pyplot as plt
import sys

fig, ax = plt.subplots(nrows=1, ncols=1)
fig.set_figheight(4)
fig.set_figwidth(8)

ax.set_title("Scaffolding Sensor Contribution")
ax.set_xlabel("Evolutionary Time")
ax.set_ylabel("Contribution of sensor to input neuron")
ax.plot([0, 0.3, 0.9, 1.0], [1, 1, 0, 0], '--', label='p')
ax.plot([0, 0.3, 0.9, 1.0], [0, 0, 1, 1], label='v')
ax.axis([-0.05, 1.05, -0.1, 1.1])
ax.set_xticklabels(['', '0%', '20%', '40%', '60%', '80%', '100%'])
#ax.set_yticklabels(['', '0%', '20%', '40%', '60%', '80%', '100%'])
ax.grid()
plt.legend(loc=5)
plt.savefig('scaffolding_contribution.png', bbox_inches='tight', dpi=200)
sys.exit(0)
eval_fracs = numpy.linspace(0., 1., 512)
sprint_frac = 0.4

s_mult = 2
e_mult = 4


fig, (ax1, ax2, ax3) = plt.subplots(nrows=1, ncols=3)
fig.set_figheight(4)
fig.set_figwidth(12)

p = numpy.zeros(512) + 0.85
v = numpy.zeros(512) + 0.15
ax1.set_title('Melding (M)')
ax1.plot(p, '--', lw=2, label='p')
ax1.plot(v, lw=2, label='v')
ax1.arrow(256, 0.15, 0,  .15, head_width=25, head_length=.075, fc='k', ec='k')
ax1.arrow(256, 0.85, 0, -.15, head_width=25, head_length=.075, fc='k', ec='k')
ax1.grid()
ax1.set_xlim([0, 512])
ax1.set_ylim([-0.1, 1.1])
ax1.set_ylabel('Contribution of modality to input neurons')
ax1.get_yaxis().set_ticklabels(['', '0%', '20%', '40%', '60%', '80%', '100%'])

p = numpy.zeros(512)
p[:350] = 1
v = numpy.zeros(512)
v[350:] = 1
ax2.set_title('Swapping (S)')
ax2.plot(p, '--', lw=2, label='p')
ax2.plot(v, lw=2, label='v')
ax2.arrow(350, 0.5, -100, 0.0, head_width=0.05, head_length=50, fc='k', ec='k')
ax2.grid()
ax2.set_xlim([0, 512])
ax2.set_ylim([-0.1, 1.1])
ax2.set_xlabel('Simulation Timestep')
ax2.get_yaxis().set_ticklabels([])

v = erf((eval_fracs + sprint_frac * s_mult - s_mult / 2) * e_mult - e_mult / 2) * 0.5 + .5
p = 1 - v
ax3.set_title('Sigmoidal (C)')
ax3.plot(p, '--', label='P', lw=2)
ax3.plot(v, label='V', lw=2)
ax3.arrow(350, 0.5, -100, 0.0, head_width=0.05, head_length=50, fc='k', ec='k')
ax3.grid()
ax3.set_xlim([0, 512])
ax3.set_ylim([-0.1, 1.1])
ax3.get_yaxis().set_ticklabels([])

plt.legend(bbox_to_anchor=(1.05, 1), loc=2, borderaxespad=0.)
fig.tight_layout()
plt.savefig('scaffolding.eps', bbox_inches='tight', dpi=200)
plt.savefig('scaffolding.png', bbox_inches='tight', dpi=200)
#plt.show()



