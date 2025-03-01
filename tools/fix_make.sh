#!/bin/bash

# Variables for the strings
REMOVE_STRING="src/dsp"
REPLACE_STRING="src/dsp/MatrixFunctions/MatrixFunctions.c \\\\\nsrc/dsp/BasicMathFunctions/BasicMathFunctions.c \\"

# Use awk to process the file
awk -v remove="$REMOVE_STRING" -v replace="$REPLACE_STRING" '
BEGIN { replaced = 0 }
{
    if ($0 ~ "^" remove) {
        if (!replaced) {
            print replace
            replaced = 1
        }
    } else {
        print
    }
}
' Makefile > Makefile.tmp

mv Makefile.tmp Makefile