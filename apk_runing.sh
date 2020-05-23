#!/bin/bash
{
    source ~/.bashrc
    cd $ANDROID_PATH/scripts
    ./launch_emulator.sh -n
}&

{
    sleep 10
    source ~/.bashrc
    cd $ANDROID_PATH/core_apks/guest_science_manager
    adb install -g -r activity/build/outputs/apk/activity-debug.apk
    cd $TOHOKU
    adb install -g -r app/build/outputs/apk/app-debug.apk
}&

wait
echo done