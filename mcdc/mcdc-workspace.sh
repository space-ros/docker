#!/usr/bin/bash

for packagedir in $@; do
	pushd $package
	mcdc_checker -a 2>&1 | tee mcdc-results.txt
	popd
done

