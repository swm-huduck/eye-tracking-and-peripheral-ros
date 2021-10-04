#!/bin/bash

echo ''
echo '#############################################################'
echo '################## Start Bluetooth Setting ##################'
echo '#############################################################'
echo ''

echo 'bluetooth_setting'
{
    sudo cp -f ./adv_min_interval /sys/kernel/debug/bluetooth/hci0/adv_min_interval
    sudo cp -f ./adv_max_interval /sys/kernel/debug/bluetooth/hci0/adv_max_interval

    sudo btmgmt -i hci0 power off;

    sudo btmgmt -i hci0 bredr off;
    sudo btmgmt -i hci0 privacy off;
    sudo btmgmt -i hci0 hs off;
    sudo btmgmt -i hci0 sc off;
    sudo btmgmt -i hci0 ssp off;
    sudo btmgmt -i hci0 linksec off;
    sudo btmgmt -i hci0 pairable off;
    sudo btmgmt -i hci0 fast-conn off;
    sudo btmgmt -i hci0 bondable off;

    sudo btmgmt -i hci0 le on;
    sudo btmgmt -i hci0 advertising on;
    sudo btmgmt -i hci0 connectable on;
    sudo btmgmt -i hci0 discov on;

    sudo btmgmt -i hci0 power on;
}

echo ''
echo '#############################################################'
echo '######################### Start ROS #########################'
echo '#############################################################'
echo ''

echo 'ros' {
    source devel/setup.bash;
    roslaunch peripheral peripheral.launch;
}

exit 0;