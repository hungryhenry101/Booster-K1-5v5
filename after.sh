#!/bin/bash
sudo iptables -D INPUT -p tcp -s 192.168.10.0/24 --dport 22 -j ACCEPT
sudo iptables -D INPUT -p tcp --dport 22 -j DROP
rm -rf /home/booster/Workspace/*