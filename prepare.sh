#!/bin/bash

cd $(realpath $(dirname $0))

if ! [ -f boards.config ]; then
  echo "File boards.config is missing."
  echo "Copy boards.config.sample to boards.config and edit as needed"
  exit 1
fi

board=$1
shift
if [ -z "$board" ]; then
  echo "Missing board name."
  echo "Please use one of the boards enumerated in boards.config"
  exit 1
fi

config=$(awk -v s="${board}" 'index($0, s) == 1' boards.config)
if [ -z "$config" ]; then
  echo "Board '$config' not found in boards.config."
  exit 1
fi
config=$(echo $config|cut -f2- -d=)
echo "### Board '$board' - Config $config ###"
echo

env $config make -j$(nproc) $*
