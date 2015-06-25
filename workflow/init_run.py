#! /usr/bin/env python3
import sqlite3
import sys
import time
import math
import os
from subprocess import Popen, PIPE

EXP = sys.argv[1]
PHE = sys.argv[2]
NUM_RUNS = int(sys.argv[3]) if len(sys.argv) > 3 else 100

crossmodal = os.environ.get('CROSSMODAL')
phe_path = "%s/data/%s/%s" % (crossmodal, EXP, PHE)
print(phe_path)
run_ids = []
eval_fields = ""

# Create runs in experiment database
with sqlite3.connect("%s/phe.db" % phe_path) as conn:
    c = conn.cursor()
    
    # Insert a spot for this job
    for _ in range(NUM_RUNS):
        c.execute("INSERT INTO run DEFAULT VALUES")
        run_ids.append(c.lastrowid)

    # Get the number of training conditions 
    c.execute("SELECT count(*) FROM condition")
    num_conditions, = c.fetchone()

# Figure out nodes and cores. Assume that every machine has 12 cores
cpn = 12
cores = num_conditions + 1
nodes = math.ceil(cores / cpn)
ppn = cores if cores < cpn else cpn

print(run_ids)

# Submit all the runs to the scheduler
process = ['qsub', '-d', phe_path,
           '-N', '%s-%s-%i' % (EXP, PHE, num_conditions),
           '-t', "%i-%i" % (min(run_ids), max(run_ids)),
           #'-q', 'workq',
           #'-l', 'walltime=3:00:00',
           '-l', 'nodes=%i:ppn=%i' % (nodes, ppn),
           #'-l', 'nodes=%i:ib:ppn=%i' % (nodes, ppn), '-q', 'ibq', 
           '%s/workflow/runner.sh' % crossmodal]
outs, errs = Popen(process, stderr=PIPE, stdout=PIPE).communicate()
print(outs.decode("utf-8"), end='')
print(errs.decode("utf-8"))

