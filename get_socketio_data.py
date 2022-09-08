#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import socket
from time import sleep
import socketio
import argparse
import json

HOST = '10.100.1.185'
PORT = 5100

sio = socketio.Client()


@sio.on('sensor')
def handleMsg(msg):
    data = json.loads(msg)
    # print('=================', data)
    for sensor in data:
        if sensor['name'] == 'STEERING':
            # print('angle: ', sensor['data'])
            ""
        elif sensor['name'] == 'EVPI_STATE':
            # print('speed: ', sensor['data'])
            ""
        elif sensor['name'] == 'LED':
            # print(sensor['data'])
            if sensor['data']:
                led = json.loads(sensor['data'])


@sio.on('customed_led')
def handleMsg(msg):
    data = json.loads(msg)
    print(data)


@sio.on('ids')
def handleMsg(msg):
    msg = json.loads(msg)
    if msg["Classification"] != "Benign":
        if msg["Attack_type"] == 'DoS':
            print("IDS Alert: DoS Detected!")
        elif msg["Attack_type"] == 'Spoofing':
            print("IDS Alert: Spoofing Detected!")
        else:
            print("Unexpected Alert!")


@sio.on('rsu')
def handleMsg(msg):
    msg = json.loads(msg)
    if 'RSU Cert' in msg:
        if msg['RSU Cert'] == 'Valid':
            light = msg['Traffic_Light']
            remain = msg['Remaining_Time']
        else:
            print('OBU Alert: RSU Invalid!')
    else:
        print('OBU Alert: RSU Invalid!')


@sio.on('drive_status')
def handleRecover(msg):
    data = json.loads(msg)
    print(data)


def process_command():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', '-host', type=str,
                        required=False, help='Set Host', default=HOST)
    parser.add_argument('--port', '-p', type=int,
                        required=False, help='Set Port', default=PORT)
    return parser.parse_args()


if __name__ == '__main__':
    args = process_command()
    host = args.host
    port = args.port

    print('socketio connecting...')
    while (not sio.connected):
        try:
            sio.connect(f"http://{host}:{port}")
        except:
            sleep(3)
            print('try again...')
    print('connected')
