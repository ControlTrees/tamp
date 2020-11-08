#!/bin/bash

batchs_folder="batchs_$(date +%Y%m%d_%H%M%S)"
mkdir "${batchs_folder}"

for i in {1..10}
do
 output_folder="${batchs_folder}/${i}"
 mkdir "${output_folder}"
 ./lgp-learn-self-driving ${output_folder}
done
