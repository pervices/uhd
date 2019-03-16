#!/bin/bash

# Varun Venkataramanan
# 20 August 2018
# Crimson Diagnostic Tests
# Tool to diagnose common errors encountered with the Crimson.
# "Is there more than 1 device controlling crimson?"
# -------------------------------------------------------------

# TODO pll locks using status command

usage="$(basename "$0") [options]
Program to diagnose issues with the Per Vices Crimson SDR.

Options:
\t-h, --help     show help text
\t-d, --debug    run commands to automatically fix detected bugs"

autofix=0 # Flag to debug and fix device

while [ "$1" != "" ]; do
    case $1 in
        -d | --debug )
            shift
            autofix=1
            ;;
        -h | --help )
            echo -e "$usage" >&2
            exit
            ;;
        * )
            echo -e "$usage" >&2
            exit 1
    esac
    shift
done

# -------------------------------------------------------------------------

#TODO: Add check/print out of client UHD version

# Ping the device (timeout = 10s)
# Send a single packet to see is the device host is up

#TODO: Get management IP address from uhd_find_devices 

ping -c 1 192.168.10.2 -w 10 > /dev/null
if [ $? -eq 0 ]; then
    echo -e "Ping Device:\tSuccess"
else
    echo -e "Ping Device:\tUnreachable\n\t\tCheck Ethernet Cable"
fi

# Check for Crimson
# Check if the uhd library can find thr Crimson
device=$((uhd_find_devices --args addr=192.168.10.2 type=crimson_tng) 2>&1)
if [[ "$device" == *"No UHD"* ]]; then
    echo -e "Crimson (UHD):\tNot Connected"
else
    echo -e "Crimson (UHD):\tConnected"
fi

# Ping each port (timeout = 10s)
# Ping SFP+ A/B to see if the ports are working
# TODO: Get this from server?
# Same as software ping test

count=0
error_msg=""
sudo ping 10.10.10.2 -c 500000 -f -w 10 > /dev/null
if [ $? -eq  0 ]; then
    count=$((count + 1))
else
    error_msg="\n\t\tCheck SFP+ A Connector."
fi

sudo ping 10.10.11.2 -c 500000 -f -w 10 > /dev/null
if [ $? -eq 0 ]; then
    count=$((count + 1))
else
    error_msg="$error_msg\n\t\tCheck SFP+ B Connector."
fi

echo -e "Ping Test:\t$count of 2 Successful$error_msg"

# Check the sysctl mem settings
# Receive Buffer Size

#TODO: Use interfaces associated with SFP+ addresses
# Or on same subnet
rmem_max=$(sysctl -n net.core.rmem_max)
if [ $rmem_max -ne 50000000 ]; then
    # Only set the buffer size if appropriate flag set
    if [ $autofix -eq 1 ]; then
        sudo sysctl -w net.core.rmem_max=50000000 > /dev/null
        echo -e "Rec Buffer:\t$rmem_max. Should be 50000000.\n\t\tFixed"
    else
        echo -e "Rec Buffer:\t$rmem_max. Should be 50000000.\n\t\tPlease run 'sudo sysctl -w net.core.rmem_max=50000000'"
    fi
else
    echo -e "Rec Buffer:\t$rmem_max"
fi

# Send Buffer Size
wmem_max=$(sysctl -n net.core.wmem_max)
if [ $wmem_max -ne 262144 ]; then
    # Only set the buffer size if appropriate flag set
    if [ $autofix -eq 1 ]; then
        sudo sysctl -w net.core.wmem_max=262144 > /dev/null
        echo -e "Send Buffer:\t$wmem_max. Should be 262144.\n\t\tFixed"
    else
        echo -e "Send Buffer:\t$wmem_max. Should be 262144.\n\t\tPlease run 'sudo sysctl -w net.core.wmem_max=262144'"
    fi
else
    echo -e "Send Buffer:\t$wmem_max"
fi

# SFP MTU=9000
ip=$(ip link show enp1s0f1)
if [[ "$ip" == *"mtu 9000"* ]]; then
    echo -e "SFP:\t\tMTU = 9000"
else
    echo -e "SFP:\t\tMTU must be 9000"
fi


# 10G Linking
# Check if the ethernet is in 10G mode

if sudo ethtool enp1s0f1 | grep -q "Supported link modes:   10000baseT/Full"; then
    echo -e "SFP:\t\t10G Mode On"
else
    echo -e "SFP:\t\tMust be in 10G Mode"
fi

# Board Info
info=$(ssh -q dev0@192.168.10.2 'echo "board -e" | mcu -f t')

# PLL Lock
count=0
if [[ "$info" != *"CHAN: 0x01, PLL Lock Detect: 0x00"* ]]; then
    echo -e "DAC PLL:\tCh 0 not locked"
    count=$((count + 1))
fi
if [[ "$info" != *"CHAN: 0x02, PLL Lock Detect: 0x00"* ]]; then
    echo -e "DAC PLL:\tCh 1 not locked"
    count=$((count + 1))
fi
if [[ "$info" != *"CHAN: 0x04, PLL Lock Detect: 0x00"* ]]; then
    echo -e "DAC PLL:\tCh 2 not locked"
    count=$((count + 1))
fi
if [[ "$info" != *"CHAN: 0x08, PLL Lock Detect: 0x00"* ]]; then
    echo -e "DAC PLL:\tCh 3 not locked"
    count=$((count + 1))
fi
if [ $count -eq 0 ]; then
    echo -e "DAC PLL:\tAll Channels Locked"
elif [ $count -ne 4 ]; then
    echo -e "DAC PLL:\tRemaining $(( 4 - count )) Channels Locked"
fi

# Fuse Farm
if [[ "$info" == *"DAC 1, Fuse Farm: 00"* ]]; then
    echo -e "Fuse Farm:\tDAC 1 Pass"
else
    echo -e "Fuse Farm:\tDAC 1 fail"
fi
if [[ "$info" == *"DAC 4, Fuse Farm: 00"* ]]; then
    echo -e "Fuse Farm:\tDAC 4 Pass"
else
    echo -e "Fuse Farm:\tDAC 4 fail"
fi

# VCO Centred
if [[ "$info" == *"DAC 1, PLL VCO Centred"* ]]; then
    echo -e "PLL VCO:\tDAC 1 Centred"
else
    echo -e "PLL VCO:\tDAC 1 Not Centred"
fi
if [[ "$info" == *"DAC 4, PLL VCO Centred"* ]]; then
    echo -e "PLL VCO:\tDAC 4 Centred"
else
    echo -e "PLL VCO:\tDAC 4 Not Centred"
fi

# Processes
# Check if there are other processes running on Crimson
# Use SFP ip addresses
processes=$(lsof | egrep '10.10.10.2|10.10.11.2')
if [ ! -z "$processes" ]; then
    echo -e "\nThere are processes currently controlling Crimson."
else
    echo -e "\nThere are no processes currently controlling Crimson."
fi

