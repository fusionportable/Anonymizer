#!/bin/bash
cd /workspace/anonymizer

list=("vehicle/campus01/campus01" "vehicle/downhill00/downhill00" "vehicle/highway01/highway01" "vehicle/multilayer_parking00/multilayer_parking00" "vehicle/street00/street00" "vehicle/tunnel00/tunnel00")

for sequence_name in "${list[@]}"
do
    echo "Processing $sequence_name"
    input="/workspace/anonymizer/data/${sequence_name}.bag"
    output="/workspace/anonymizer/data/${sequence_name}_anonymized.bag"
    echo "Input: $input"
    echo "Output: $output"
    python anonymizer/bin/anonymize_bag.py --input $input --output $output --weights weights --vehicle
    echo "Done processing $sequence_name"
done