#!/bin/bash

path=$1

find $path -type f -exec chmod 644 {} \;
