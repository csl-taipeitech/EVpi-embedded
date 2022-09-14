#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from time import sleep
import socketio
import argparse
import json
from can import LEDCan
from threading import Timer

HOST = '192.168.2.100'
PORT = 5100

sio = socketio.Client()
led = LEDCan('can0')

light_status = 'none'
last_light_status = 'none'

def setInterval(func, sec):
    def func_wrapper():
        setInterval(func, sec)
        func()
    t = Timer(sec, func_wrapper)
    t.start()
    return t

@sio.on('sensor')
def handleMsg(msg):
    global light_status
    data = json.loads(msg)
    # print('=================', data)
    for sensor in data:
        if sensor['name'] == 'STEERING':
            if sensor['data'] > 3*3.1415/180:
                # print('Left')
                light_status='left'
            elif sensor['data'] < -3*3.1415/180:
                # print('Right')
                light_status='right'
            else:
                # print('None')
                light_status='none'
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
    global light_status
    msg = json.loads(msg)
    if msg["Classification"] != "Benign":
        if msg["Attack_type"] == 'DoS':
            print("IDS Alert: DoS Detected!")
            light_status='hazzard'
        elif msg["Attack_type"] == 'Spoofing':
            print("IDS Alert: Spoofing Detected!")
            light_status='hazzard'
        else:
            print("Unexpected Alert!")

@sio.on('rsu')
def handleMsg(msg):
    global light_status
    msg = json.loads(msg)
    if 'RSU Cert' in msg:
        if msg['RSU Cert'] == 'Valid':
            light = msg['Traffic_Light']
            remain = msg['Remaining_Time']
        else:
            print('OBU Alert: RSU Invalid!')
            light_status='hazzard'
    else:
        print('OBU Alert: RSU Invalid!')
        light_status='hazzard'

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

def send_can():
    global light_status, led
    # print(light_status)
    if light_status == 'none':
        #led.TestNone()
        led.Non()
    elif light_status == 'left':
        #led.TestLeft()
        led.Left()
    elif light_status == 'right':
        #led.TestRight()
        led.Right()
    elif light_status == 'hazzard':
        led.Hazzard()

def send_can_once():
    global light_status, last_light_status, led
    # print(light_status)
    if light_status == last_light_status:
        return
    if light_status == 'none':
        led.Non()
    elif light_status == 'left':
        led.Left()
    elif light_status == 'right':
        led.Right()
    elif light_status == 'hazzard':
        led.Hazzard()

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

    setInterval(send_can, 0.1)
