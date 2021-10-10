#!/bin/bash

readarray -d = -t elements <<< $(sudo cat /etc/lsb-release | grep DISTRIB_RELEASE)

versionNumber=${elements[1]}

if [[ $versionNumber == 20* ]] ;
then
    rosDistro="noetic"
fi

if [[ $versionNumber == 18* ]] ;
then
    rosDistro="melodic"
fi

echo $rosDistro