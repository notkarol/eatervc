#!/usr/bin/env python3
import os
import sqlite3
import sys
import numpy
import time
from itertools import product
import shutil
from subprocess import Popen, PIPE
import importlib.machinery

# Load experiment file
crossmodal = os.environ.get('CROSSMODAL')
settings_filename = sys.argv[1]
exp = importlib.machinery.SourceFileLoader("module.name", settings_filename).load_module()

# Create raw experiment path
exp_path = "%s/data/%s" % (crossmodal if crossmodal else '..', exp.name)
if os.path.exists(exp_path):
    #os.rename(exp_path, '%s_%i' % (exp_path, int(time.time())))
    #print("Moved existing", exp_path)
    pass
else:
    os.mkdir(exp_path)
    shutil.copy2(settings_filename, "%s/settings.py" % (exp_path))

# Compile code and copy it over
os.chdir("%s/code" % crossmodal)
#outs, errs = Popen("make", stderr=PIPE, stdout=PIPE).communicate()
#if errs: print(errs.decode("utf-8"))
#outs, errs = Popen(["make", "g"], stderr=PIPE, stdout=PIPE).communicate()
#if errs: print(errs.decode("utf-8"))
for executable in ('runner.py', 'evaluator', 'gevaluator'):
    if not os.path.exists("%s/%s" % (exp_path, executable)):
        print(executable)
        shutil.copy2(executable, "%s/%s" % (exp_path, executable))


# Create database for our phenotypes
if not os.path.exists("%s/exp.db" % exp_path):
    print(exp_path)
    with sqlite3.connect("%s/exp.db" % exp_path) as conn:
        c = conn.cursor()
        c.execute("CREATE TABLE IF NOT EXISTS phenotypes (name TEXT PRIMARY KEY, completed_runs INTEGER, avg_trial_fit FLOAT, min_trial_fit FLOAT, avg_test_fit FLOAT,min_test_fit FLOAT, timestamp DATETIME DEFAULT CURRENT_TIMESTAMP);")
        conn.commit()

# For each phenotype
count = 0
for world, actions, variators, sussex, useii, brain, conditions, condvaris in exp.phenotypes:
    # Create the folder
    phe_name = "%s_%s_%s_%s_%s_%s" % (world, actions, brain, sussex, variators, useii)
    phe_path = "%s/%s" % (exp_path, phe_name)
    count += 1
    if os.path.exists(phe_path):
        print('Skipping %s' % phe_path)
        continue
    print(phe_name, count, len(exp.phenotypes))
    os.mkdir(phe_path)

    # Create the database
    with sqlite3.connect("%s/phe.db" % phe_path) as conn:
        c = conn.cursor()

        c.execute("""CREATE TABLE IF NOT EXISTS run (
        id        INTEGER PRIMARY KEY AUTOINCREMENT,
        combined  INTEGER DEFAULT 0,
        timestamp DATETIME DEFAULT CURRENT_TIMESTAMP);""")

        c.execute("""CREATE TABLE IF NOT EXISTS condition (
        id       INTEGER PRIMARY KEY AUTOINCREMENT,
        goal     FLOAT,
        num_interintra INTEGER);""")

        c.execute("""CREATE TABLE IF NOT EXISTS variator (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        obj_type INTEGER,
        obj_elem INTEGER,
        obj_id   INTEGER
        );""")

        c.execute("""CREATE TABLE IF NOT EXISTS condvari (
        cond_id INTEGER,
        vari_id INTEGER,
        value   FLOAT);""")

        c.execute("""CREATE TABLE IF NOT EXISTS world (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        aabb_min_x FLOAT,
        aabb_min_y FLOAT,
        aabb_min_z FLOAT,
        aabb_max_x FLOAT,
        aabb_max_y FLOAT,
        aabb_max_z FLOAT,
        sigma      FLOAT,
        num_evals  INTEGER,
        eval_steps INTEGER,
        eval_dt    FLOAT);""")

        c.execute("""CREATE TABLE IF NOT EXISTS rbody (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        dim_x FLOAT,
        dim_y FLOAT,
        dim_z FLOAT,
        pos_x FLOAT,
        pos_y FLOAT,
        pos_z FLOAT,
        ori_x FLOAT,
        ori_y FLOAT,
        ori_z FLOAT,
        shape INTEGER,
        mass  FLOAT,
        friction_sliding FLOAT,
        friction_rolling FLOAT);""")

        c.execute("""CREATE TABLE IF NOT EXISTS joint (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        rbody_id_0 INTEGER,
        rbody_id_1 INTEGER,
        pos_x FLOAT,
        pos_y FLOAT,
        pos_z FLOAT,
        ori_x FLOAT,
        ori_y FLOAT,
        ori_z FLOAT,
        jtype INTEGER,
        ang_min FLOAT,
        ang_max FLOAT,
        offset FLOAT,
        force FLOAT);""")

        c.execute("""CREATE TABLE IF NOT EXISTS brain (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        num_i INTEGER,
        num_h INTEGER,
        num_o INTEGER,
        recurrent INTEGER,
        connectivity FLOAT);""")

        c.execute("""CREATE TABLE IF NOT EXISTS action (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        atype INTEGER,
        sprint_beg FLOAT,
        sprint_end FLOAT,
        eval_beg FLOAT,
        eval_end FLOAT,
        operation INTEGER,
        position INTEGER,
        parameter FLOAT,
        src_type_0 INTEGER,
        src_elem_0 INTEGER,
        src_id_0 INTEGER,
        src_type_1 INTEGER,
        src_elem_1 INTEGER,
        src_id_1 INTEGER);""")

        c.execute("""CREATE TABLE IF NOT EXISTS sussex (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        src_x FLOAT,
        src_y FLOAT,
        src_z FLOAT,
        radius FLOAT,
        azi FLOAT,
        zen FLOAT,
        rays_x INTEGER,
        rays_y INTEGER,
        spread FLOAT,
        stype INTEGER);""")

        # Insert phenotype into its phe database
        sql = "INSERT INTO world VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);"
        c.execute(sql, exp.worlds[world])
        sql = "INSERT INTO brain VALUES (?, ?, ?, ?, ?, ?);"
        c.execute(sql, exp.brains[brain])
        for rbody in exp.rbodys:
            sql = "INSERT INTO rbody VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);"
            c.execute(sql, rbody)
        for joint in exp.joints:
            sql = "INSERT INTO joint VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);"
            c.execute(sql, joint)
        for sussex in exp.sussexs[sussex]:
            sql = "INSERT INTO sussex VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);"
            c.execute(sql, sussex)
        for action in exp.actions[actions]:
            sql = "INSERT INTO action VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?);"
            c.execute(sql, action)
        for variator in exp.variators:
            c.execute("INSERT INTO variator VALUES (?, ?, ?, ?);", variator)
        for condition in conditions:
            c.execute("INSERT INTO condition VALUES (?, ?, ?);", condition)
        for condvari in condvaris:
            c.execute("INSERT INTO condvari VALUES (?, ?, ?);", condvari)    
        conn.commit()
