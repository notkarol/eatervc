#!/usr/bin/env python3
from mpi4py import MPI
import numpy
import matplotlib.pyplot as plt
import sqlite3
import cma
import sys
import time
import pickle

# Calculate the intra and inter category differences for all items on each row. This is for the alternate fitness function
def intra_inter(values, categories):
    intra = 0
    inter = 0
    count_intra = 0
    count_inter = 0
    for fpos in range(values.shape[1]):
        for epos1 in range(values.shape[0]):
            for epos2 in range(epos1 + 1, values.shape[0]):
                if categories[epos1] == categories[epos2]: 
                    intra += numpy.abs(values[epos1, fpos] - values[epos2, fpos])
                    count_intra += 1
                else:
                    inter += numpy.abs(values[epos1, fpos] - values[epos2, fpos])
                    count_inter += 1
    intra /= count_intra * 2
    inter /= count_inter * 2
    return intra, inter

# Fitness function
def exercise(weights, goals, num_intrainter, cond_ids, num_sprints, num_iter, es):
    global SPRINT_ID
    global EVALS
    global SPRINTS
    SPRINT_ID += 1
    if SPRINT_ID >= num_sprints: return 1.0
    SPRINTS[SPRINT_ID, 1] = es.countiter / num_iter
    SPRINTS[SPRINT_ID, 2:] = weights

    # Send everything out first
    for i in range(1, SIZE):
        #print("../gevaluator %i %.3f %s" % (i, SPRINTS[SPRINT_ID, 1], " ".join(list(map(lambda x: "%.3f" % x, weights)))))
        COMM.Send([SPRINTS[SPRINT_ID, 1:], MPI.DOUBLE], dest=i, tag=i)

    # Process in local variables and spit out fitness
    fitness = 0.0 
    for i in range(1, SIZE):
        COMM.Recv(EVALS[SPRINT_ID, i - 1, :], source=i, tag=i)
        fitness += EVALS[SPRINT_ID, i - 1, 0]

    # Calculate our fitness and save it
    fitness /= (SIZE - 1)
    if num_intrainter > 0:
        intra, inter = intra_inter(EVALS[SPRINT_ID, :, 1 : 1 + num_intrainter], goals)
        fitness = fitness + 1 / (1 + intra - inter)
    SPRINTS[SPRINT_ID, 0] = fitness
    return fitness

def main():
    # Initialize sprint id
    global SPRINT_ID
    SPRINT_ID = -1
    run_id = int(sys.argv[1]) if len(sys.argv) >= 2 else 0
    
    # Initialize MPI
    global COMM
    global SIZE
    global RANK
    COMM = MPI.COMM_WORLD
    SIZE = COMM.Get_size()
    RANK = COMM.Get_rank()

    # Fetch conditions from exp.db
    with sqlite3.connect("phe.db") as conn:
        c = conn.cursor()
        c.execute("SELECT sigma, num_evals FROM world")
        sigma, num_evals = c.fetchone()
        c.execute("SELECT id, goal, num_interintra FROM condition")
        cond_ids = []
        goals = []
        for cond_id, goal, num in c.fetchall():
            cond_ids.append(cond_id)
            goals.append(goal)
            num_intrainter = num

        c.execute("SELECT count(*) FROM action WHERE atype=4")
        num_recv, = c.fetchone()
        c.execute("SELECT num_i, num_h, num_o, recurrent FROM brain")
        num_weights = 0
        num_i, num_h, num_o, recurrent = c.fetchone()
        num_weights += ((num_h if recurrent else 0) + num_i + num_o) * num_h
    num_sprints = num_evals // len(cond_ids)

    # Initialize buffers
    weights = numpy.random.uniform(-1, 1 + numpy.spacing(1), num_weights)
    weights[-1 * num_h * num_o : -1 * num_h] = 0

    global RECV
    RECV = numpy.zeros(num_recv, dtype=numpy.float64)
    global SPRINTS
    SPRINTS = numpy.zeros((num_sprints, num_weights + 2))
    global EVALS
    EVALS = numpy.zeros((num_sprints, len(cond_ids), num_recv))

    # Give out each of the evaluators their condition number
    time.sleep(1.0) # make sure other process startup isn't an issue
    for i in range(1, SIZE):
        COMM.Send([numpy.array([cond_ids[i - 1]], dtype='i'), MPI.INT], dest=i, tag=i)
    
    # Run our optimizer
    opts = cma.CMAOptions()
    opts['bounds'] = [-1, 1]
    opts['ftarget'] = 0
    opts['maxfevals'] = num_sprints
    opts['tolfun'] = 0.0
    opts['tolfunhist'] = 0.0
    opts['verb_log'] = 0
    rc = 0
    try: 
        es = cma.CMAEvolutionStrategy(weights, sigma, opts)
        es.optimize(exercise, args=[goals, num_intrainter, cond_ids, num_sprints,
                                    num_sprints // es.popsize, es])
        es.result_pretty()
        print("SAVING PICKLE")
        pickle.dump((SPRINTS, EVALS), open('%04i.p' % run_id, 'wb'))
    except:
        print("EXCEPTION!")
        print(sys.exc_info())
        rc = 1

    print("CLOSING EVALUATORS")
    weights[0] = -1337.0
    for i in range(1, SIZE):
        COMM.Send([weights, MPI.DOUBLE], dest=i, tag=i)

    return rc

if __name__ == "__main__":
    sys.exit(main())
