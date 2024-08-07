
import socket
import sys
import argparse
import flatbuffers
from ogn.OneMessage import OneMessage

from datetime import datetime
from termcolor import colored, cprint

colors =['white','green','yellow','cyan','magenta','']

if len(sys.argv) > 1:
    UDP_PORT_NO = int(sys.argv[1])
else:
    print("usage: python3 udp_server.py <port>")
    sys.exit(1)


UDP_IP_ADDRESS = "10.0.1.200"

serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))

stations = []

while True:
    data, addr = serverSock.recvfrom(512)

    if addr[0] not in stations:
        stations.append(addr[0]);

    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")

    index = stations.index(addr[0])
    print_c = lambda x: cprint(x, colors[index])
    print_c("%s RAW Message: %s from %s"%(current_time, data.hex(), addr[0]))

    try:
        aircraft = OneMessage.GetRootAsOneMessage(data, 0)
        position = aircraft.AircPos()
        print("Aircraft: %s Lat: %.4f Lon: %.4f Alt: %.4f Timestamp: %d"%(position.Callsign(), position.Lat(), position.Lon(), position.Alt(), position.Timestamp()))
    except Exception as e:
        print(e)
