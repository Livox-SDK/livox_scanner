#!/bin/bash

ip link set can0 type can bitrate 1000000
ifconfig can0 up