#!/bin/sh

usage="Usage: specify the square size of the calibration pattern in meters, optionally specify wether to use proposed optimization or not (0/1)"


if [ $# -eq 0 ]
  then
      printf "%s\n" "$usage"
      exit
fi

squareSz=$1 # default to 0.1m square sizes of the calibration pattern
improve=${2:-1} # default to using the proposed optimization

# set_calibration_poses.m should be configured and run once before calibration
if [ -e ./data/calibration_poses.txt ]
then
    printf "Starting calibration\n"
else
    printf "Configure and run set_calibration_poses.m before performing calibration\n"
    exit
fi

# capture images
printf "Acquiring images\n"

./capture.out

if [ $? -ne 0 ]
then
    printf "Error encountered, exiting\n"
    exit
fi

# perform intrinsic calibration and find the camera poses in the world frame
printf "Performing calibration\n"

matlab -nodesktop -nodisplay -nojvm -r "try world_calibration($squareSz, $improve); catch; end; quit" > /dev/null

printf "Done\n"
