systemctl daemon-reload
systemctl start ethernet_setup.service
systemctl enable ethernet_setup.service
systemctl start control_station.service
systemctl enable control_station.service
shutdown -r now 
