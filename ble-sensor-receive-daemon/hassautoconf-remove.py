#!/usr/bin/python3
from time import localtime, strftime, sleep
import queue
import paho.mqtt.client as mqtt
import json
import socket
import os
import sys
import yaml

#--------------------------------------------------
# MQTT to remove all autoconfig
#--------------------------------------------------
HOSTNAME = socket.gethostname()
CLIENT_NAME = HOSTNAME + "-rem-autoconf"

BASE = "homeassistant"

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

def subscribe_topics():
    client.subscribe(BASE + "/#")



# All init of MQTT connection."
def mqtt_init(cfg):
    broker = cfg['broker']
    if 'cert' in cfg:
        auth = 'cert'
    else:
        auth = 'pwd'

    def on_connect(client, userdata, flags, rc):
        if rc==0:
            print("Connected to " + broker)
            subscribe_topics()
            client.connected_flag=True #set flag
        else:
            print("Bad connection Returned code=" + str(rc))

    def on_disconnect(client, userdata, rc):
        print("Disconnected from " + broker + ". Return code: " + str(rc))
        client.connected_flag=False

    def on_subscribe(client, userdata, mid, granted_qos):
        print("Subscribed to topic, mid: " + str(mid))

    def on_message(client, userdata, message):
        #print("Received message on " + message.topic)
        topic = message.topic
        payload = message.payload.decode('utf-8')
        q.put([topic, payload])

    global client # Estabslish global client
    global q
    q = queue.Queue()
    client = mqtt.Client(CLIENT_NAME)
    client.on_connect=on_connect  #bind call back function
    client.on_message=on_message
    client.on_disconnect=on_disconnect
    client.on_subscribe=on_subscribe
    mqtt.Client.connected_flag=False #create flags in class

    if auth == 'cert':
        ca_with_path = os.path.join(SCRIPT_DIR, cfg['cert'])
        client.tls_set(ca_certs=ca_with_path, tls_version=2)
        client.tls_insecure_set(True)
    else:
        client.username_pw_set(cfg['user'], cfg['password'])

    client.loop_start()
    print("Connecting to broker " + broker)

    #Establish connection, and retry until success
    try:
        client.connect(broker)      #connect to broker
    except:
        attempts = 1
        while not client.connected_flag:
            try:
                client.connect(broker)      #connect to broker
            except:
                print("Connection attempt " + str(attempts) + " failed, retrying")
                attempts = attempts + 1
                sleep(5)
    while not client.connected_flag: #Make sure we are connected.
        sleep(1)




def main():

    CFG_FILE = os.path.join(SCRIPT_DIR, 'config.yaml')
    try:
        with open(CFG_FILE) as f:
            cfg = yaml.safe_load(f)

    except FileNotFoundError:
        print('ERROR: Could not find config.yaml in {}'.format(SCRIPT_DIR))
        return -1

    mqtt_init(cfg['mqtt'])

    sleep(3) # Wait for all sensors to register

    while not q.empty():
        message = q.get()
        if "mitemp" in message[1]:
            print("Removing thunderboard sensor in topic " + message[0])
            client.publish(message[0], retain=True)
        #client.publish(message[0], retain=True)
        #print("Removed sensor from: " + message[0])
        if "tb/" in message[1]:
            print("Removing thunderboard sensor in topic " + message[0])
            client.publish(message[0], retain=True)
        if "airthings" in message[1]:
            print("Removing airthings sensor in topic " + message[0])
            client.publish(message[0], retain=True)


    client.loop_stop()    #Stop loop
    client.disconnect() # disconnect



if __name__ == "__main__":
    main()
