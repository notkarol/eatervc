#!/usr/bin/env python3
from subprocess import Popen, PIPE
import sqlite3
import pickle
import numpy
import sys
import os
import crossmodal    

def main():
    exp, phe_test, phe_trial = sys.argv[1:4]
    phe_trial_path = "%s/data/%s/%s" % (crossmodal.crossmodal_path(), exp, phe_trial)
    phe_test_path = "%s/data/%s/%s" % (crossmodal.crossmodal_path(), exp, phe_test)
    runs_filename = '%s/%s.p' % (phe_test_path, phe_trial)
    os.chdir(phe_test_path)

    # Cond Ids and goals for the test
    cond_ids, goals = crossmodal.fetch_condids_goals(phe_test_path)

    # Load runs dictionary from file
    runs = {}
    if os.path.exists(runs_filename):
        with open(runs_filename, 'rb') as f:
            runs = pickle.load(f)
                            
    # For each run do the analysis
    for filename in os.listdir(phe_trial_path):
        run = filename[:-2]

        # If this file isn't a pickle file or the run has already been included, pass it
        if filename[-2:] != '.p' or (run in runs and len(runs[run])):
            print("Skipping %s" % filename)
            continue
        runs[run] = []
        print(run)

        # Load the sprints and evals
        with open('%s/%s' % (phe_trial_path, filename), 'rb') as f:
            sprints, evals = pickle.load(f)
        dits = numpy.mean(evals[:,:,0], axis=1)
        fits = numpy.mean(evals[:,:,-1], axis=1)
    
        # Pick weights in the best row in the final quarter of evaluations
        for sprint in [crossmodal.argmin(dits, 0.9, 1.0)]: #, crossmodal.argmin(fits, 0.9, 1.0)]:
            fit = sprints[sprint, 0]
            weights = numpy.array(list(sprints[sprint, 2:]))
            tests, test_fits = crossmodal.run_tests(cond_ids, goals, weights, 0.1)
            runs[run].append([sprint, fit, weights, test_fits, tests])
            print(sprint, numpy.mean(test_fits))
        with open(runs_filename, 'wb') as f:
            pickle.dump(runs, f)

if __name__ == "__main__":
    sys.exit(main())
