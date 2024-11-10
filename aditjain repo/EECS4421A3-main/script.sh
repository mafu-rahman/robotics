#!/bin/bash

# Define an array of items
items=("item1" "item2" "item3")

# Iterate over the array using a for loop
for item in "${items[@]}"; do
    # Run your command with the current item
    ros2 interface show rcl_interfaces/msg/${item}
done
