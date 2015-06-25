#!/usr/bin/env python3
from subprocess import Popen, PIPE
import sqlite3
import pickle
import numpy
import sys
import os
import matplotlib.pyplot as plt
import scipy.special
from matplotlib.collections import LineCollection
from matplotlib.colors import ListedColormap, BoundaryNorm

def crossmodal_path():
    return os.environ.get('CROSSMODAL')

def load_sprint(path):
    with open(path, 'rb') as f:
        sprints, evals = pickle.load(f)
    return sprints, evals

def fitness(guesses, goal):
    return numpy.mean(numpy.abs(guesses - goal))

def argmin(mat, start_percent, end_percent=1.0):
    start = int(start_percent * len(mat))
    end = int(end_percent * len(mat)) + 1
    return numpy.argmin(mat[start : end]) + start

# Colorline provides colorful lines
def clear_frame(ax=None):
    # Taken from a post by Tony S Yu
    if ax is None:
        ax = plt.gca()
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    for spine in ax.spines.itervalues():
        spine.set_visible(False) 
def make_segments(x, y):
    points = numpy.array([x, y]).T.reshape(-1, 1, 2)
    segments = numpy.concatenate([points[:-1], points[1:]], axis=1)
    return segments
def colorline(x, y, z=None, cmap=plt.get_cmap('copper'), norm=plt.Normalize(0.0, 1.0), linewidth=2.0, alpha=0.8):
    if z is None: z = numpy.linspace(0.0, 1.0, len(x))

    # Special case if a single number:
    if not hasattr(z, "__iter__"): z = numpy.array([z])
    z = numpy.asarray(z)

    segments = make_segments(x, y)
    lc = LineCollection(segments, array=z, cmap=cmap, norm=norm, linewidth=linewidth, alpha=alpha)
    ax = plt.gca()
    ax.add_collection(lc)
    return lc

# Activation Function Stuff
def flatten(val):
    return 0.63661977236 * numpy.arctan(6.28318530718 * numpy.power(val, 3));

def edgen(val):
    if not isinstance(val, list) and not isinstance(val, numpy.ndarray):
        val = numpy.array([val], dtype=numpy.float64)
    frac = 0.2
    sign = numpy.array(val >= 0, dtype=numpy.float64)
    sign[sign == 0] = -1
    a = (numpy.abs(val) - frac) / (1 - frac)
    a[a < 0] = 0
    a[a > 1.0] = 1
    return a * sign

def plot_activation_functions():
    x = numpy.arange(-2.5, 2.5, 0.001)
    plt.title('Various Activation Functions')
    plt.plot(x, flatten(x),           label='atan flattening')
    plt.plot(x, scipy.special.erf(x), label='error function')
    plt.plot(x, edgen(x),             label='flat over [-.2,.2]')
    plt.grid()
    plt.axis([-2.1, 2.1, -1.1, 1.1])
    plt.legend(loc=4)
    plt.xlabel("Sum of inputs to neuron")
    plt.ylabel("Output value of neuron")
    plt.show()

# Run the simulation using command line arguments 
def evaluate(cond_id, sprint_dur, weights):
    process = ['../evaluator', str(cond_id), str(sprint_dur)] + list(map(str, weights))
    outs, errs = Popen(process, stderr=PIPE, stdout=PIPE).communicate()
    lines = []
    for line in outs.decode("utf-8").split("\n"):
        try:
            split_line = list(map(float, line.split()))
            if len(split_line): lines.append(split_line)
        except:
            pass
    return numpy.array(lines)

def plot_guesses(cond_ids, goals, sprint_dur, weights, phe_path):
    cwd = os.getcwd()
    os.chdir(phe_path)

    fig = plt.figure(frameon = False)
    plt.axis([0, 512, -1.1, 1.1])
    plt.grid()
    for goal, cond_id in zip(goals, cond_ids):
        outs = evaluate(cond_id, sprint_dur, weights)
        plt.plot(outs[:, -1], lw = (2 if goal > numpy.mean(goals) else 1))
    plt.show()
    os.chdir(cwd)
        

def run_tests(cond_ids, goals, weights, fraction=1.0):
    # Convert NaNs to 2.0
    #nans = numpy.isnan(sprints)
    #sprints[nans] = 2.0
    
    # Store fitnesses of each condition
    sprint_dur = 1.0
    tests = None #
    test_fits = numpy.zeros(len(cond_ids), dtype=numpy.float64)
    for i in range(len(cond_ids)):
        outputs = evaluate(cond_ids[i], sprint_dur, weights)
        if tests is None:
            tests = numpy.zeros((len(cond_ids), 3, outputs.shape[1]), dtype=numpy.float64)
        tests[i, 0, :] = outputs[0, :]
        tests[i, 1, :] = outputs[outputs.shape[0] // 2, :]
        tests[i, 2, :] = outputs[-1, :]
        start = int(outputs.shape[0] - (outputs.shape[0] * fraction))
        test_fits[i] = numpy.mean(numpy.abs(outputs[start:, -1] - goals[i]))
    return tests, test_fits

def fetch_condids_goals(phe_path='.'):
    cond_ids = []
    goals = []
    with sqlite3.connect("%s/phe.db" % phe_path) as conn:
        c = conn.cursor()
        c.execute("SELECT id, goal FROM condition")
        for cond_id, goal in c.fetchall():
            cond_ids.append(cond_id)
            goals.append(goal)
    return cond_ids, goals

def fetch_condvaris(phe_path='.'):
    condvaris = []
    with sqlite3.connect("%s/phe.db" % phe_path) as conn:
        c = conn.cursor()
        c.execute("SELECT cond_id, vari_id, value FROM condvari;")
        for cond_id, vari_id, value in c.fetchall():
            if vari_id == 1: condvaris.append([])
            condvaris[-1].append(value)
    return numpy.array(condvaris)


def fetch_brain(phe_path='.'):
    with sqlite3.connect("%s/phe.db" % phe_path) as conn:
        c = conn.cursor()
        c.execute("SELECT num_i, num_h, num_o, recurrent FROM brain")
        num_i, num_h, num_o, recurrent = c.fetchone()
    return num_i, num_h, num_o, recurrent

def plot_phase(row1, row2, name):
    fig = plt.figure(frameon = False)
    plt.title('Phase %s' % name)
    plt.axis([-1.1, 1.1, -1.1, 1.1])
    plt.grid()
    colorline(row1, row2)
    plt.savefig('%s/workflow/%s.png' % (crossmodal_path(), name), dpi=200)
    plt.close()
    
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

                                                                                    
