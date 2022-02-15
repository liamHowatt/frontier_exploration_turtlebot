roslaunch frontier_exploration_turtlebot better_explore_irl.launch > /dev/null 2> /dev/null &
python3 wait_til_idle.py
rosnode kill /explore
rostopic pub /move_base_simple/goal $(rostopic type /move_base_simple/goal) '{header: {frame_id: "map"}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' &
jobs -l