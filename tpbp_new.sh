
for ((i=13; i<73; i++))
do
    xterm -e "roscore" &
    sleep 3
    xterm -e "rosparam load config/ane$i.yaml" &
    sleep 2
    xterm -e "rosrun mrpp_sumo sumo_wrapper.py" &
    sleep 3
    xterm -e "rosrun mrpp_sumo $(rosparam get /algo_name).py"  &
    sleep 2
    xterm -e "rosrun mrpp_sumo command_center.py"
    sleep 3
    killall xterm & sleep 5
done
