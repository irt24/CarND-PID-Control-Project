#!/bin/bash

# Must be in build directory.
cmake .. && make

function run_simulations {
  KP=$1
  KI=$2
  KD=$3
  SETID=$4
  NUMRUNS=$5

  for RUN in `seq 1 $NUMRUNS`;
    do
      # Assumes the term2 simulator is in the same build folder.
      echo "Starting simulator. You'll have to kill it yourself."
      open term2_sim.app

      RUNID="$SETID-$RUN" 
      echo "Set ID is $SETID"
      echo "Starting server with run ID $RUNID"
      ./pid $KP $KI $KD $RUNID
   
      python ../src/process_pid_debug.py --input_filename="/tmp/pid_debug_$RUNID.txt"
    done 
}

# First set of runs: P only.
run_simulations 0.2 0.0 0.0 "p" 3

## Second set of runs: PI. 
run_simulations 0.2 0.0004 0.0 "pi" 3

## Third set of runs: PID. 
run_simulations 0.2 0.0004 3.0 "pid" 3
