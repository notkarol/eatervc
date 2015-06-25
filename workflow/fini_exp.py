#!/usr/bin/env python3
from subprocess import Popen, PIPE
import sqlite3
import pickle
import numpy
import sys
import os
import matplotlib.pyplot as plt

import crossmodal

def main():
    phe_path = sys.argv[1]

    # Load all the trials
    trials = {}
    for filename in os.listdir(phe_path):
        if filename[-2:] != '.p': continue
        trial_phe = filename[:-2]
        trial_path = os.path.join(phe_path, filename)
        with open(trial_path, 'rb') as f:
            trials[trial_phe] = pickle.load(f)

    # Aggregate
    name = os.path.basename(phe_path).split('_')[3]

    condvaris = crossmodal.fetch_condvaris(phe_path)
    cmap=plt.get_cmap('copper')
    fig = plt.figure(figsize=(16, 12), frameon = False)
    axs = plt.subplot(111)
    for phe in sorted(trials):
        ii, world, brain, condition, variator = phe.split('_')
        evals = int(world[1:-1])

        
        test_totals = numpy.zeros(4) # + 2.0
        trial_totals = numpy.zeros(4)
        test_mins = numpy.zeros(4)  + 2.0
        trial_mins = numpy.zeros(4) + 2.0
        for run_id in trials[phe]:
            trial_fits = trials[phe][run_id]['trial_fits']
            test_fits = trials[phe][run_id]['test_fits']

            test_fit = numpy.mean(test_fits[-1])
            if test_fit < 0.3:
                f, axarr = plt.subplots(2, sharey=True)
                axarr[0].set_title('%s with test fitness %.3f' % (phe, test_fit))
                for i in range(len(test_fits[-1])):
                    x = condvaris[i, 1]
                    y = condvaris[i, 2]
                    ax = 0 if condvaris[i, 0] > numpy.mean(condvaris[:, 0]) else 1
                    color = test_fits[-1][i] if test_fits[-1][i] < 1.0 else 1.0
                    color = (color, 1 - color, 0.0)

                    axarr[ax].plot(x, y, 'o', color=color, ms=10.0)
                axarr[1].set_title('Big Objects Above, Small Objects Below')
                plt.xlabel('Starting X Position')
                plt.ylabel('Starting Y Position')
                axarr[0].axis([-15, 15, -7, -21])
                axarr[1].axis([-15, 15, -7, -21])
                f.colorbar(axarr[0], cmap=cmap)
                plt.savefig('%s_%s_%.3f.png' % (phe, run_id, test_fit))
                plt.close()
            
            for i in range(len(trial_fits)):
                trial_fit = trial_fits[i]

                test_totals[i] += test_fit
                trial_totals[i] += trial_fit
                test_mins[i] = test_fit if test_fit < test_mins[i] else test_mins[i]
                trial_mins[i] = trial_fit if trial_fit < trial_mins[i] else trial_mins[i]
                if test_fit < 0.25:
                    s = "%s %s %.3f %.3f %.3f" % (phe, run_id, i / len(trial_fits), trial_fit, test_fit)
                    #print(s)
        test_totals /= len(trials[phe])
        trial_totals /= len(trials[phe])
        if test_totals[-1] < 0.4: #0.3:
            axs.plot(evals, test_totals[-1], 'o', label='%s %.3f %.3f' % (phe, trial_totals[-1], test_totals[-1]))
        elif test_totals[-1] > 0.5:
            axs.plot(evals, test_totals[-1], 'x', label='%s %.3f %.3f' % (phe, trial_totals[-1], test_totals[-1]))
        else:
            axs.plot(evals, test_totals[-1], '.')
        print('%s %3i %.3f %.3f %.3f %.3f' %
              (phe, len(trials[phe]), trial_totals[-1], test_totals[-1], trial_mins[-1], test_mins[-1]))
    plt.title(name)
    plt.xlabel('Evaluations (thousands)')
    plt.ylabel('Test Fitness')
    plt.grid()
    plt.legend(loc=4)
    plt.axis([0, 500, 0, 0.6])
    plt.savefig('%s.png' % name)
    plt.show()
                
if __name__ == "__main__":
    sys.exit(main())
