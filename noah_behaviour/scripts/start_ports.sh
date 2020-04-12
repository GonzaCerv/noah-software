#!/usr/bin/env bash

# Author:        Gonzalo Cervetti (cervetti.g@gmail.com)
#
# Description:   Starts 2 serial ports communicated between
#                each other (bridge) the service keep running in background
#                in a tmux session.
#                The ports started run 

tmux new-session -d -s socat_virtual_ports 'socat -d -d pty,raw,echo=0,crnl  pty,raw,echo=0,crnl'