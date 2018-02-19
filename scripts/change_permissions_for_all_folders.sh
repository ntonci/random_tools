#!/bin/bash

path=$1

find $path -type d -exec chmod 755 {} \;
