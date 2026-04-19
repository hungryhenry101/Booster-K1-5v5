#!/bin/bash

# 规则1：允许 192.168.10.0/24 的 SSH
if ! sudo iptables -C INPUT -p tcp -s 192.168.10.0/24 --dport 22 -j ACCEPT 2>/dev/null; then
    sudo iptables -A INPUT -p tcp -s 192.168.10.0/24 --dport 22 -j ACCEPT
fi

# 规则2：拒绝所有 SSH
if ! sudo iptables -C INPUT -p tcp --dport 22 -j DROP 2>/dev/null; then
    sudo iptables -A INPUT -p tcp --dport 22 -j DROP
fi

python set_up.py