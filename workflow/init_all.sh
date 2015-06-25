#!/bin/bash -x

# Our path 
export CROSSMODAL=$(dirname "`pwd -LP`")

# Updata .bashrc if we need to
existing=`grep CROSSMODAL ~/.bashrc`
if [ -z "$existing" ] ; then
    echo "export CROSSMODAL=$CROSSMODAL" >> ~/.bashrc
fi

# Backup existing data as needed
if [ -d "$CROSSMODAL/data" ] ; then
    mv $CROSSMODAL/data $CROSSMODAL/data_`date +%s`
fi

# Create the data directory and associated database
mkdir $CROSSMODAL/data
sqlite3 $CROSSMODAL/data/all.db "CREATE TABLE IF NOT EXISTS experiments (name TEXT PRIMARY KEY, completed_runs INTEGER, avg_trial_fit FLOAT, min_trial_fit FLOAT, avg_test_fit FLOAT,min_test_fit FLOAT, timestamp DATETIME DEFAULT CURRENT_TIMESTAMP);"

# Create the experiments from our settings folder
for exp in `ls --color=never $CROSSMODAL/settings/*.py` ; do 
    $CROSSMODAL/workflow/init_exp.py $exp ; 
done
