#!/bin/bash          
#
# Script to delay the launch of a roslaunch file
# 
# Koen Lekkerkerker
# Thu 24 Apr 2014 
#
# Check https://answers.ros.org/question/51474/can-i-run-a-bash-script-using-roslaunch/ 
# for more info
#

# Set all the environment variables
echo "setting environment variables"
source $(rospack find noah_behaviour)/scripts/set_env_vars.sh

echo "starting delayed for $1 second[s]"
sleep $1
echo "end wait for $1 second[s]"
shift
    echo "now running 'roslaunch $@'"
    roslaunch $@
fi