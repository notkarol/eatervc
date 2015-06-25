#!/usr/bin/env python3
from subprocess import Popen, PIPE
import sqlite3
import pickle
import numpy
import sys
import os
import matplotlib as mpl
#mpl.use('Agg')
import matplotlib.pyplot as plt
import scipy.stats
import crossmodal

def main():
    # Load all the trials
    phe_path = sys.argv[1]
    phe_name = os.path.basename(phe_path)
    
    trials = {}
    for filename in os.listdir(phe_path):
        if filename[-2:] != '.p': continue
        trial_phe = filename[:-2]
        
        if trial_phe[2] != '8': continue
        if trial_phe[19] != '6': continue
        if trial_phe[18] != 'H': continue
        trial_path = os.path.join(phe_path, filename)
        with open(trial_path, 'rb') as f:
            trials[trial_phe] = pickle.load(f)
    cond_ids, goals = crossmodal.fetch_condids_goals(phe_path)
    condvaris = crossmodal.fetch_condvaris(phe_path)
    #print(condvaris)
    #return
    # Aggregate
    pos_fits = {}
    phe_fits = {}
    for pos, phe in enumerate(sorted(trials)):
        pos_fits[phe] = numpy.zeros((len(goals)))
        phe_fits[phe] = 0.0
        
        for run_id in sorted(trials[phe]):
            trial_fit = trials[phe][run_id][0][1]
            test_fits = trials[phe][run_id][0][3]
            test_outs = trials[phe][run_id][0][4]

            for i, test_fit in enumerate(test_fits):
                pos_fits[phe][i] += test_fit
            phe_fits[phe] += numpy.mean(test_fits)
            
        pos_fits[phe] /= len(trials[phe])
        phe_fits[phe] /= len(trials[phe])

    for phe in sorted(pos_fits):
        fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)
        test_s = numpy.zeros((6, 13))
        test_l = numpy.zeros((6, 13))
        test_d = numpy.zeros((6, 13))
        for i in range(len(goals)):
            z = int((condvaris[i][2] // -2) - 4)
            x = int((condvaris[i][1] // 2) + 6)

            if goals[i] < 0:
                test_s[z,x] = pos_fits[phe][i]
            else:
                test_l[z,x] = pos_fits[phe][i]
            test_d[z,x] += pos_fits[phe][i] / 2
            #plt.plot(condvaris[i][1], condvaris[i][2], 'o', ms=25 + 15 * goals[i],
            #         color=plt.get_cmap('pink')(1. - pos_fits[phe][i]))
            
        im = ax1.imshow(test_s, cmap=plt.cm.cool, aspect='auto', interpolation='nearest', vmin=0, vmax=1)
        im = ax2.imshow(test_l, cmap=plt.cm.cool, aspect='auto', interpolation='nearest', vmin=0, vmax=1)
        im = ax3.imshow(test_d, cmap=plt.cm.cool, aspect='auto', interpolation='nearest', vmin=0, vmax=1)
        fig.subplots_adjust(right=0.8)
        cbar_ax = fig.add_axes([1.01, 0.15, 0.05, 0.7])
        fig.colorbar(im, cax=cbar_ax)
        cbar_ax.set_xlabel('Error')

        cbar_ax.plot([0,1], [numpy.mean(pos_fits[phe]), numpy.mean(pos_fits[phe])], color='k', lw=2)
        
        ax1.set_title('%sk simulations on sensor %s trained on %s' % (phe[2:4], phe[6], phe[18:20]))
        #ax2.set_title('Small objects above, large objects below')
        ax1.set_xticks([])
        ax1.set_yticks([])
        ax2.set_xticks([])
        ax2.set_yticks([])
        ax3.set_xticks([])
        ax3.set_yticks([])
        #plt.axis([-13, 13, -19, -7])
        #plt.gca().invert_yaxis()
        ax1.invert_yaxis()
        ax2.invert_yaxis()
        ax3.invert_yaxis()
        ax1.set_ylabel('Small Objects\ntested z position')
        ax2.set_ylabel('Large Objects\ntested z position')
        ax3.set_ylabel('Both Objects\ntested z position')
        ax3.set_xlabel('tested x position')
        fig.set_figwidth(8)
        fig.set_figheight(8)
        fig.tight_layout()
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)
        ax1.spines['bottom'].set_visible(False)
        ax1.spines['left'].set_visible(False)
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(False)
        ax2.spines['bottom'].set_visible(False)
        ax2.spines['left'].set_visible(False)
        ax3.spines['top'].set_visible(False)
        ax3.spines['right'].set_visible(False)
        ax3.spines['bottom'].set_visible(False)
        ax3.spines['left'].set_visible(False)
        plt.savefig('%s/workflow/pos_%s.png' % (crossmodal.crossmodal_path(), phe),
                    bbox_inches='tight', dpi=200)
        plt.savefig('%s/workflow/pos_%s.eps' % (crossmodal.crossmodal_path(), phe),
                    bbox_inches='tight', dpi=200)
        plt.close()
        print(phe)

if __name__ == "__main__":
    sys.exit(main())
