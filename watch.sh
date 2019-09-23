#!/bin/bash
trigger_func()
{
    colcon build --packages-select rmf_traffic && ./build/rmf_traffic/test/test_rmf_traffic
}

if [ -z "$1" ]
  then
    echo "Enter path of file to watch!"
    exit
fi

while :
do
    inotifywait -e close_write $1 |  trigger_func
done