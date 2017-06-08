#-------------------------------------------------------------------------------
# Copyright (C) 2017 Francesco Mannella
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#-------------------------------------------------------------------------------
#!/bin/bash

EXEC_NAME=icub_controller

WORLD=$(find $GAZEBO_MODEL_PATH | grep icub_fixed.world) 
[ -z "$WORLD" ] && echo "world $(basename $WORLD) not found" && exit 1

SERVER="gzserver $WORLD" 
CLIENT=gzclient
MAIN_DIR=$(echo $0 | sed -e"s/\/$(basename $0)$//")
SIMULATION="$(readlink -f $MAIN_DIR/)/$EXEC_NAME"

[ ! -z "$(screen -ls | grep $EXEC_NAME)" ] && screen -S $EXEC_NAME -X quit 
screen -dmS $EXEC_NAME
sleep 1
# xterm -fn -*-fixed-*-*-*-*-14-*-*-*-*-*-*-* -e "screen -rdS $EXEC_NAME " &
x-terminal-emulator -e "screen -rdS $EXEC_NAME " &
sleep 1
screen -S $EXEC_NAME -X title yarp
screen -S $EXEC_NAME -X split
sleep 1
screen -S $EXEC_NAME -X focus
screen -S $EXEC_NAME -X screen 
screen -S $EXEC_NAME -X title server

sleep 1
screen -S $EXEC_NAME -X split
sleep 1
screen -S $EXEC_NAME -X focus
screen -S $EXEC_NAME -X screen 
screen -S $EXEC_NAME -X title client

sleep 1
screen -S $EXEC_NAME -X split
sleep 1
screen -S $EXEC_NAME -X focus
screen -S $EXEC_NAME -X screen 
screen -S $EXEC_NAME -X title simulation
sleep 1

screen -S $EXEC_NAME -X -p server stuff "sleep 5 && $SERVER\n"
screen -S $EXEC_NAME -X -p client stuff "sleep 10; $CLIENT\n"
screen -S $EXEC_NAME -X -p simulation stuff "sleep 20; $SIMULATION\n"
screen -S $EXEC_NAME -X -p yarp stuff "yarpserver\n"



