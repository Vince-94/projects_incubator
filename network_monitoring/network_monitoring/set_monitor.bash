#!/bin/bash

INTERFACE_NAME=wlx00c0cab2f615
MODE=$1

if [[ $MODE == "monitor" ]]; then
    sudo ip link set $INTERFACE_NAME down
    sudo iw $INTERFACE_NAME set type monitor
    sudo ip link set $INTERFACE_NAME up

elif [[ $MODE == "managed" ]]; then
    sudo ip link set $INTERFACE_NAME down
    sudo iw dev $INTERFACE_NAME set type managed
    sudo ip link set $INTERFACE_NAME up

else
    echo "Error: Mode $MODE not handled"

fi

# Check mode type
iw dev $INTERFACE_NAME info | grep type
