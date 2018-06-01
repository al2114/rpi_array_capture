#!/bin/bash

ip -o addr show | grep "wlan0    inet " | awk '{split($4,a,"/"); print a[1]}'

