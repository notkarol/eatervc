#! /usr/bin/env python3
import sqlite3
import sys
import time
import math
import os
from subprocess import Popen, PIPE

EXP = sys.argv[1]
crossmodal = os.environ.get('CROSSMODAL')
exp_path = '%s/data/%s' % (crossmodal, EXP)

tests = {}
trials = []
for folder in os.listdir(exp_path):
    if not os.path.isdir(os.path.join(exp_path, folder)):
        continue
    conditions = folder.split('_')
    if 'w000' in conditions and conditions[1][1] =='P':
        tests[folder] = []
    elif 'v00' in conditions:# or 'b55f' not in conditions:
        continue
    else:
        files = os.listdir('%s/%s' % (exp_path, folder))
        if len(files) > 1:
            trials.append(folder)

# Match trials with tests based on number of hidden neurons
for test in tests:
    test_conditions = test.split('_')
    for trial in trials:
        trial_conditions = trial.split('_')
        if (trial_conditions[2] == test_conditions[2] and
            trial_conditions[3] == test_conditions[3] and
            (test_conditions[1][1] != 'P' or (trial_conditions[1][1] == test_conditions[1][1]))):
            tests[test].append(trial)

for test in sorted(tests):
    for trial in tests[test]:
        # Submit all the runs to the scheduler
        process = ['qsub', '-d', '%s/%s' % (exp_path, test), '-N', '%s-%s' % (EXP, trial),
                   '-v', 'exp=%s,test=%s,trial=%s' % (EXP, test, trial),
                   '%s/workflow/tester.sh' % crossmodal]
        print(" ".join(process))
        outs, errs = Popen(process, stderr=PIPE, stdout=PIPE).communicate()
        print(outs.decode("utf-8"), end='')
        print(errs.decode("utf-8"))
        #time.sleep(0.1)
