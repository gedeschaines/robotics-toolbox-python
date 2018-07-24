#!/bin/bash

# Shell script utilized by Makefile to set its internal VERSION variable.

VERS=`grep -P "version = \'\d+\.\d+\'" ./setup.py | awk '{print $3}'`
VERS=${VERS%\'}
VERS=${VERS#\'}
echo $VERS