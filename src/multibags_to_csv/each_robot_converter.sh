#!/bin/bash

echo 'path directory: ' $1
echo '>>>>>>>>>> position converter start ..... >>>>>>>>>>'
python ConverterPosition.py -p $1
echo '<<<<<<<<<< position converter finish ..... <<<<<<<<<<'

echo '>>>>>>>>>> boundary converter start ..... >>>>>>>>>>'
python ConverterBoundary.py -p $1
echo '<<<<<<<<<< boundary converter finish ..... <<<<<<<<<<'

echo '>>>>>>>>>> runtime converter start ..... >>>>>>>>>>'
python ConverterRuntime.py -p $1
echo '<<<<<<<<<< runtime converter finish ..... <<<<<<<<<<'
