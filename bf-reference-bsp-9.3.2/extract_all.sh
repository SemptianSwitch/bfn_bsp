#!/bin/bash

# Extract all the SDE modules

[ -z ${BSP} ] && echo "Environment variable BSP not set" && exit 1

cd ${BSP}
find -name "*.tgz" -exec tar xzvf '{}' ';'
