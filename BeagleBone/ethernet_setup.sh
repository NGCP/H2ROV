#!/bin/bash
ifconfig eth0 192.168.254.2
route add default gw 192.168.254.1
