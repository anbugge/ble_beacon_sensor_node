#!/usr/bin/python3

from bluepy.btle import Scanner
# from humidity import rh2ah
from datetime import datetime, timedelta
from time import localtime, strftime, sleep
from Crypto.Cipher import AES
from Crypto.Util import Counter
from collections import namedtuple
import hashlib
import queue
import paho.mqtt.client as mqtt
import json
import socket
import os
import sys
import yaml
import hassautoconf as autoconf


#--------------------------------------------------
# Settings TB Sense
#--------------------------------------------------
# Logging
LOGFILE = "/var/log/ble-sensor-receive-daemon/ble-sensor-receive-daemon.log"
verbose = True
DEBUG   = False
TIME_FORMAT = "%Y-%m-%d %H:%M:%S"


# MQTT Server
HOSTNAME = socket.gethostname()
CLIENT_NAME = HOSTNAME + "-ble"

# BLE packets
SECRET_KEY = bytes.fromhex("5d1281ee0cac2c99a5ec7288c7a85f48")
MAGIC_BYTES = "aa5500ff"
PLAINTEXT_BEACON_ID = "aa55"
ENCRYPTED_BEACON_ID = "aa05"

# Availability settings
EXPIRE_AFTER = 4800 # Seconds since last seen is considered offline

# Nodes and topics
NODE_TOPIC = "sensor"

class Sensor:
    name = ''
    topic = ''
    nonce = 0
    last_seen = "2000-01-01 00:00:00"
    availability = ''
    last_pkts = []

TB_SENSORS = dict()

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

#--------------------------------------------------
# Misc debug / parser
#--------------------------------------------------
def debug(text, address=""):
    logfile = open(LOGFILE, "a")
    line = strftime(TIME_FORMAT, localtime()) + " "
    if address != "":
        line += address + " "
    line += text
    if (verbose):
        print(line)
    logfile.write(line + "\n")
    logfile.close()



#--------------------------------------------------
# MQTT Control and Functions
#--------------------------------------------------
def subscribe_topics():
    for address in TB_SENSORS:
        client.subscribe(TB_SENSORS[address].topic + NODE_TOPIC)


# All init of MQTT connection."
def mqtt_init(cfg):
    broker = cfg['broker']
    if 'cert' in cfg:
        auth = 'cert'
    else:
        auth = 'pwd'

    def on_connect(client, userdata, flags, rc):
        if rc==0:
            debug("Connected to {}".format(broker))
            subscribe_topics()
            client.connected_flag=True #set flag
        else:
            debug("Bad connection Returned code=" + str(rc))

    def on_disconnect(client, userdata, rc):
        debug("Disconnected from {}. Return code: {}".format(broker, rc))
        client.connected_flag=False

    def on_subscribe(client, userdata, mid, granted_qos):
        debug("Subscribed to topic, mid: " + str(mid))

    def on_message(client, userdata, message):
        #debug("Received message on " + message.topic)
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
    debug("Connecting to broker " + broker)

    #Establish connection, and retry until success
    try:
        client.connect(broker)      #connect to broker
    except:
        attempts = 1
        while not client.connected_flag:
            try:
                client.connect(broker)      #connect to broker
            except:
                debug("Connection attempt " + str(attempts) + " failed, retrying")
                attempts = attempts + 1
                sleep(5)
    while not client.connected_flag: #Make sure we are connected.
        sleep(1)



#--------------------------------------------------
# Main loop controls
#--------------------------------------------------
def process_message(message):
    topic = message[0]
    payload = message[1]
    try:
        sensor_data = json.loads(payload)
        address = sensor_data["Address"]
        if address in TB_SENSORS:
            stored_nonce = TB_SENSORS[address].nonce
            received_nonce = int(sensor_data["Nonce"].replace(" ", ""),16)
            last_seen = sensor_data["Localtime"]
            
            if stored_nonce == received_nonce:
                if DEBUG:
                    header("Processing MQTT Message", address, TB_SENSORS)
                    debug("Duplicate packet, ignoring.", address)
            if received_nonce > stored_nonce:
                header("Processing MQTT Message", address, TB_SENSORS)
                debug("Updating stored nonce to: " + hex(received_nonce), address)
                TB_SENSORS[address].nonce = received_nonce
                debug("Updating last seen to: " + last_seen, address)
                TB_SENSORS[address].last_seen = last_seen

            if DEBUG: debug("Stored Nonce: " + hex(stored_nonce),address)    
            if DEBUG: debug("Received Nonce: " + hex(received_nonce), address)
        
    except:
        debug("Error processing message from " + topic)


# Returns nice uptime string
def seconds_to_time(seconds):
    s = timedelta(seconds=int(seconds))
    d = datetime(1,1,1) + s
    nice_string = ""
    if   d.year == 2:   nice_string += "1 Year, "
    elif d.year  > 2: nice_string += str(d.year-1) + " Years, "
    if   d.month == 2: nice_string += "1 Month, "
    elif d.month  > 2: nice_string += str(d.month-1) + " Months, "
    if   d.day == 2: nice_string += "1 Day, " 
    elif d.day  > 2: nice_string += str(d.day-1) + " Days, "
    nice_string += '{}:{}:{}'.format(d.hour, d.minute, d.second)
    return nice_string


def toSigned16(n):
    n = n & 0xffff
    return n | (-(n & 0x8000))


def counter(n):
    return n


def stripAddress(addr):
    table = str.maketrans('','',':')
    return addr.translate(table)


def scan_ble_devices(secs):
    scanner = Scanner(0)
    devices = scanner.scan(secs, passive=True)
    for device in devices:
        scanData = device.getScanData()
        for (_, _, value) in scanData:
            address = str(device.addr)
            valueString = str(value)

            # Plaintext beacon
            if valueString[0:4] == "aa55":
                print("")
                print("Plaintext beacon")
                print("addr: " + str(device.addr))
                print(valueString)
              #  processPayload(valueString[4:32])

            # Encrypted beacon
            if valueString[0:4] == ENCRYPTED_BEACON_ID:
                try:
                    location = TB_SENSORS[address].name
                except:
                    debug("",address)
                    debug("Found Encrypted Beacon ID", address)
                    debug("Unknown node, please register", address)
                    debug("",address)
                    break

                sensorData = {}
                sensorData["Label"] = location
                sensorData["Address"] = address

                addrhash = hashlib.sha256(bytes.fromhex(stripAddress(str(device.addr)))).hexdigest()[0:24]
                nonce = (int(addrhash, 16) << 32) + int(valueString[36:44], 16)
                rebootCount = int(valueString[36:44], 16) >> 22
                packetCount = nonce & 0x3FFFFF

                if ( nonce < TB_SENSORS[address].nonce):
                    header("Found Encrypted Beacon", address, TB_SENSORS)
                    debug("Nonce value too low, ignoring", address)
                    debug("Received nonce: " + str(hex(nonce)), address)
                    debug("Stored nonce: " + str(hex(TB_SENSORS[address].nonce)), address)
                elif ( nonce == TB_SENSORS[address].nonce):
                    if DEBUG: debug("Duplicate packet, ignoring.", address)
                else:
                    header("Received Encrypted Beacon", address, TB_SENSORS)
                    debug("Location: " + location, address)
                    if DEBUG: debug("Payload " + valueString, address)
                    debug("Reboot count: " + str(rebootCount), address)
                    debug("Packet counter: " + str(packetCount), address)
                    sensorData["Reboot count"] = rebootCount
                    sensorData["Packet count"] = packetCount

                    ctr = Counter.new(128, initial_value=nonce)
                    aes = AES.new(SECRET_KEY, AES.MODE_CTR, counter=ctr)
                    cipherPayload = bytes.fromhex(valueString[4:36])
                    plainPayload = str(aes.decrypt(cipherPayload).hex())
                    if DEBUG:
                        debug("Nonce: " + str(hex(nonce)), address)
                        debug("Cipher payload: " + str(cipherPayload.hex()), address)
                        debug("Plain payload: " + plainPayload, address)
                    if plainPayload[24:32] == MAGIC_BYTES:
                        current_time = strftime(TIME_FORMAT, localtime())
                        TB_SENSORS[address].nonce = nonce
                        TB_SENSORS[address].last_seen = current_time
                     
                        process_payload(plainPayload, sensorData, address)
                     
                        sensorData["Nonce"] = hex(nonce)[0:26] + " " + hex(nonce)[26:34]
                        sensorData["Localtime"] = current_time
                        sensorData["Gateway"] = HOSTNAME

                        client.publish(TB_SENSORS[address].topic + NODE_TOPIC,
                                        json.dumps(sensorData), 
                                        retain = True)
                    else:
                        debug("Decryption failed. Ignoring.", address)


def header(topic, address, sensor_set):
    debug("", address)
    debug("-----------------------------------------------------------", address)
    debug(topic + ": " + sensor_set[address].name, address)
    debug("-----------------------------------------------------------", address)
    debug("Address: " + address, address)


def process_payload(payload, sensorData, address):
    swVersion = int(payload[22:23],16) + int(payload[23:24],16)/10
    debug("Software Version: " + str(swVersion), address)
    temp = round(toSigned16(int(payload[0:4], 16))/100, 2)
    debug("Temperature: " + str(temp) + " C", address)
    sensorData["Temperature"] = temp
    humidity = round(int(payload[4:8], 16)/100, 2)
    debug("Humidity: " + str(humidity) + " %", address)
    sensorData["Humidity"] = humidity
    voltage = round(int(payload[8:12], 16)/100, 3)
    sensorData["Battery Voltage"] = voltage
    debug("Battery Voltage: " + str(voltage) + " V", address)
    uptime = int(payload[12:20], 16)
    sensorData["Uptime seconds"] = uptime
    sensorData["Uptime"] = seconds_to_time(uptime)
    debug("Uptime: " + str(uptime) + " s", address)
    debug("Uptime: " + seconds_to_time(uptime), address)
    
    #debug("Software version: " + str(swVersion), address)
    sensorData["Software version"] = swVersion


def autoconfig():
    for a in TB_SENSORS:
        for senstype in ["Temperature", "Humidity", "Battery Voltage"]:
    #        device = {"model": "Thunderboard Sense BLE Node",
    #                "manufacturer": "Silicon Labs",
    #                "identifiers": a + "-" + senstype,
    #                "name": TB_SENSORS[a].name + " " + senstype}
            autoconf.register_sensor(client, 
                                     TB_SENSORS[a].topic,
                                     TB_SENSORS[a].name + " " + senstype,
                                     senstype.lower().replace(" ", "-"),
                                     senstype, 
     #                                device=device,
     #                                unique_id=a + "-" + senstype,
                                     expire_after=EXPIRE_AFTER)



#--------------------------------------------------
# Main loop
#--------------------------------------------------
def main():

    CFG_FILE = os.path.join(SCRIPT_DIR, 'config.yaml')
    try:
        with open(CFG_FILE) as f:
            cfg = yaml.safe_load(f)

    except FileNotFoundError:
        print('ERROR: Could not find config.yaml in {}'.format(SCRIPT_DIR))
        return -1

    for name in cfg['sensors']:
        sensor = Sensor()
        sensor.name = name
        sensor.topic = cfg['sensors'][name]['topic'].strip()
        if not sensor.topic.endswith('/'):
            sensor.topic += '/'
        addr = cfg['sensors'][name]['addr']
        TB_SENSORS[addr] = sensor

    mqtt_init(cfg['mqtt'])
    autoconfig()

    # mainloop
    while True:
        # First, make sure we are connected
        while not client.connected_flag: #wait in loop
            sleep(1)

        # Process any waiting messages
        while not q.empty():
            message = q.get()
            process_message(message)

        if DEBUG: debug("Scanning for TB beacons...")
        scan_ble_devices(4)


    # Should never end here, but if so, close nicely
    client.loop_stop()    #Stop loop
    client.disconnect() # disconnect



if __name__ == "__main__":
    main()
