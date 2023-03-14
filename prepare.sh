#!/bin/bash

config=
if ! [ -z "$1" ]; then
  board=$(egrep "^${1}=" boards.config)
  if [ -z "$board" ]; then
     echo "Board $1 not found."
     exit 1
  fi
  config=$(echo $board|cut -f2- -d=)
fi
env $config make -j flash term
