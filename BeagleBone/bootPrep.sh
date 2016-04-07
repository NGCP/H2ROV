systemctl daemon-reload
systemctl start ethernet_setup.service
systemctl enable ethernet_setup.service
shutdown -r now 