#!/bin/bash


Help() {
  printf "\n  1° argument: chess size (es. '8x6')";
  printf "\n  2° argument: square size in meter (es. '0.04')";
  printf "\n  3° argument: image topic name (es. '/camera/rgb/image_raw')";
  printf "\n  4° argument: camera topic name (es. '/camera/rgb/')";
  printf "\n\n";
}


if [ -z "$1" ]; then
  Help;
else
  if [ -z "$2" ]; then
    Help;
  else
    if [ -z "$3" ]; then
      Help;
    else
      if [ -z "$4" ]; then
        Help;
      else
        rosrun camera_calibration cameracalibrator.py --size $1 --square $2 image:=$3 camera:=$4
      fi
    fi
  fi
fi
