#!/bin/bash -x
#PBS -j oe
#PBS -m n
#PBS -l walltime=0:03:00:00,pmem=1gb
#PBS -q sharedq

#module load gcc/gcc4.9.2 
#module load mpi/openmpi-ib-gcc-1.6.4
module load mpi/openmpi-gcc481-1.6.5 
module load sqlite
module load bullet
module load python/3.4.1
module list

# needed for node424 and 422?
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/gpfs1/arch/x86_64/gcc4.8.1/lib64

exp_id=`echo $PBS_JOBNAME | awk -F '-' '{print $1}'`
phe_id=`echo $PBS_JOBNAME | awk -F '-' '{print $2}'`
nconds=`echo $PBS_JOBNAME | awk -F '-' '{print $3}'`
run_id=`echo $PBS_JOBNAME | awk -F '-' '{print $4}'`

cd ${PBS_O_WORKDIR}
time mpirun -np 1 python3 ${PBS_O_WORKDIR}/../runner.py $run_id : -np $nconds ${PBS_O_WORKDIR}/../evaluator
rc=$?
checkjob -v -v $PBS_JOBID
exit $rc
