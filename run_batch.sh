#!/bin/bash
cd /workspace/anonymizer

handheld  mini_hercules  quadrupedal_robot  vehicle

list=("handheld/room00/room00" "mini_hercules/campus00/campus00" "mini_hercules/parking00/parking00" "quadrupedal_robot/grass01/grass01" "quadrupedal_robot/room00/room00")

for sequence_name in "${list[@]}"
do
    echo "Processing $sequence_name"
    input="/workspace/data/${sequence_name}.bag"
    output="/workspace/data/${sequence_name}_anonymized.bag"
    echo "Input: $input"
    echo "Output: $output"

    
    # command="python3 shine_batch.py ${config_file}"
    # echo "Run SHINE on $sequence_name"
    # eval $command
    # echo "Done on $sequence_name"
done
