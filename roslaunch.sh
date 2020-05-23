#/bin/bash
{
    source ~/.bashrc
    pushd $BUILD_PATH
    source devel/setup.bash
    popd
    export ROS_IP=$(getent hosts llp | awk '{ print $1 }')
    export ROS_MASTER_URI=http://${ROS_IP}:11311
    roslaunch astrobee sim.launch dds:=false robot:=sim_pub rviz:=true

    pushd $BUILD_PATH
    source devel/setup.bash
    popd
    export ROS_IP=$(getent hosts llp | awk '{ print $1 }')
    export ROS_MASTER_URI=http://${ROS_IP}:11311
    $ANDROID_PATH/scripts/gs_manager.sh start
    cd $SOURCE_PATH/tools/gds_helper/src
    python gds_simulator.py
}&
{
    gnome-terminal --tab -e ( source ~/.bashrc;  $ANDROID_PATH/scripts/gs_manager.sh start; cd $SOURCE_PATH/tools/gds_helper/src; python gds_simulator.py; bash; )        
}&

wait
echo done