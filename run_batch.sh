#!/bin/bash
cd /workspace/anonymizer

list=("handheld/room00/room00" "mini_hercules/campus00/campus00" "mini_hercules/parking00/parking00" "quadrupedal_robot/grass01/grass01" "quadrupedal_robot/room00/room00")

for sequence_name in "${list[@]}"
do
    echo "Processing $sequence_name"
    input="/workspace/anonymizer/data/${sequence_name}.bag"
    output="/workspace/anonymizer/data/${sequence_name}_anonymized.bag"
    echo "Input: $input"
    echo "Output: $output"
    python anonymizer/bin/anonymize_bag.py --input $input --output $output --weights weights
    echo "Done processing $sequence_name"
done
