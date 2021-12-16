#!/bin/sh
USER=ble-receiver
APPLICATION=ble-sensor-receive-daemon
LOGDIR=/var/log/$APPLICATION/

# Add daemon user
sudo adduser --system --no-create-home --group $USER

# Setup systemd integration
sudo setcap 'cap_net_raw,cap_net_admin+eip' `readlink -f \`which python3\``
sudo setcap 'cap_net_raw+ep' `readlink -f \`which hcitool\``

sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python3.8/dist-packages/bluepy/blescan.py
sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python3.8/dist-packages/bluepy/bluepy-helper
sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/local/lib/python3.8/dist-packages/bluepy/bluepy-helper.c

sudo ln -s /opt/$APPLICATION/systemd/$APPLICATION.service /etc/systemd/system/multi-user.target.wants/$APPLICATION.service
sudo ln -s /opt/$APPLICATION/systemd/$APPLICATION.service /lib/systemd/system/$APPLICATION.service 
sudo systemctl enable $APPLICATION.service
sudo systemctl daemon-reload

# Setup log and logrotate integration
sudo mkdir $LOGDIR
sudo chown $USER:$USER $LOGDIR
sudo touch $LOGDIR/$APPLICATION.log
sudo chown $USER:$USER $LOGDIR/$APPLICATION.log

sudo cp logrotate/$APPLICATION /etc/logrotate.d/
