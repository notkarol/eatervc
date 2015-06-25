#!/bin/bash -x
#PBS -j oe
#PBS -m n
#PBS -l walltime=0:02:00:00 
#PBS -q sharedq

# Needed for shitty nodes
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/gpfs1/arch/x86_64/gcc4.8.1/lib64
module load mpi/openmpi-gcc481-1.6.5
module load sqlite
module load bullet
module load python/3.4.1

cd ${PBS_O_WORKDIR}
python3 $CROSSMODAL/workflow/tester.py $exp $test $trial
exit $?
