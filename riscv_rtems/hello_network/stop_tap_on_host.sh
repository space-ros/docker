#!/bin/bash
set -o verbose
set -o errexit
sudo tunctl -d tap0
