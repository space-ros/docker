#!/usr/bin/bash

mcdc_checker_script=`pwd`/run_mcdc_checker.py
for packagedir in $@; do
	pushd $package
	python3 $mcdc_checker_script `pwd` --sarif_file mcdc-results.txt --verbose
	popd
done

