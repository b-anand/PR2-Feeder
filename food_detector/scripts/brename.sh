#!/bin/bash

prefix="$1"
shift

base_n="$1"
shift

step="$1"
shift

n=$base_n

for file in "$@" ; do
    formatted_n=$(printf "%03d" $n)
    # re-use original file extension whilke we're at it.
    mv "$file" "${prefix}_${formatted_n}.${file##*.}"
    let n=n+$step
done
