#!/usr/bin/python3

import threading
from bluepy import btle
from datetime import datetime, timedelta
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
import time
import hassautoconf as autoconf


#--------------------------------------------------
# Settings TB Sense
#--------------------------------------------------
# Logging
LOGFILE = "/var/log/ble-sensor-receive-daemon/ble-sensor-receive-daemon.log"
verbose = True
DEBUG   = True
TIME_FORMAT = "%Y-%m-%d %H:%M:%S"


# MQTT Server
HOSTNAME = socket.gethostname()
CLIENT_NAME = HOSTNAME + "-ble"

# BLE packets
SECRET_KEY = bytes.fromhex("5d1281ee0cac2c99a5ec7288c7a85f48")
MAGIC_BYTES = "aa5500ff"
PLAINTEXT_BEACON_ID = "aa55"
ENCRYPTED_BEACON_ID = "aa05"
PLAINTEXT_BEACON_ID_V2 = "aa56"
ENCRYPTED_BEACON_ID_V2 = "aa06"

# Availability settings
EXPIRE_AFTER = 7200 # Seconds since last seen is considered offline

# Nodes and topics
NODE_TOPIC = "sensor"

class Sensor:
    name = ''
    topic = ''
    type = None
    nonce = 0
    last_seen = "2000-01-01 00:00:00"
    availability = ''
    last_pkts = []

SCRIPT_DIR = os.path.dirname( os.path.abspath(__file__) )

#--------------------------------------------------
# Misc debug / parser
#--------------------------------------------------
def debug(text, address=""):
    logfile = open(LOGFILE, "a")
    line = time.strftime(TIME_FORMAT, time.localtime()) + " "
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
class Mqtt:

    # All init of MQTT connection."
    def __init__(self, cfg, sensors):
        self.broker = cfg['broker']
        if 'cert' in cfg:
            auth = 'cert'
        else:
            auth = 'pwd'

        self.sensors = sensors

        self.q = queue.Queue()
        self.client = mqtt.Client(CLIENT_NAME)
        
        # Bind callback functions
        self.client.on_connect=self.on_connect
        self.client.on_disconnect=self.on_disconnect
        self.client.on_message=self.on_message
        self.client.on_subscribe=self.on_subscribe

        self.connected_flag = False

        if auth == 'cert':
            ca_with_path = os.path.join(SCRIPT_DIR, cfg['cert'])
            self.client.tls_set(ca_certs=ca_with_path, tls_version=2)
            self.client.tls_insecure_set(True)
        else:
            self.client.username_pw_set(cfg['user'], cfg['password'])

        self.client.loop_start()
        debug("Connecting to broker " + self.broker)

        #Establish connection, and retry until success
        try:
            self.client.connect(self.broker)      #connect to self.broker
        except:
            attempts = 1
            while not self.connected_flag:
                try:
                    self.client.connect(self.broker)      #connect to broker
                except KeyboardInterrupt as e:
                    raise e
                except Exception as e:
                    debug("Connection attempt " + str(attempts) + " failed, retrying")
                    debug(str(e))
                    attempts = attempts + 1
                    time.sleep(5)
    
    def on_connect(self, client, userdata, flags, rc):
        if rc==0:
            debug("Connected to {}".format(self.broker))
            self.subscribe_topics()
            self.connected_flag = True
            self.autoconfig()
        else:
            debug("Bad connection Returned code=" + str(rc))

    def on_disconnect(self, client, userdata, rc):
        debug("Disconnected from {}. Return code: {}".format(self.broker, rc))
        self.connected_flag = False

    def on_subscribe(self, client, userdata, mid, granted_qos):
        debug("Subscribed to topic, mid: " + str(mid))

    def on_message(self, client, userdata, message):
        #debug("Received message on " + message.topic)
        topic = message.topic
        payload = message.payload.decode('utf-8')
        self.q.put([topic, payload])

    def subscribe_topics(self):
        for address in self.sensors:
            self.client.subscribe(self.sensors[address].topic + NODE_TOPIC)


    def autoconfig(self):
        for (addr, s) in self.sensors.items():
            for senstype in ["Temperature", "Humidity", "Battery Voltage"]:
                senstype_id = senstype.lower().replace(" ", "-")
                device = {"manufacturer": "Silicon Labs",
                          "identifiers": addr,
                          "name": s.name
                          }
                if s.type == 'tbs2':
                    device["model"] = "Thunderboard Sense 2 BLE Node"
                elif s.type == 'tbbg22':
                    device["model"] = "Thunderboard BG22 BLE Node"
                autoconf.register_sensor(client=self.client,
                                        sensor_topic_base=s.topic,
                                        title=senstype,
                                        sensor_type=senstype_id,
                                        key=senstype,
                                        device=device,
                                        unique_id=s.name + "_" + senstype_id,
                                        expire_after=EXPIRE_AFTER)

    def process_messages(self):
        while not self.q.empty():
            message = self.q.get()
            self.process_message(message)

    def process_message(self, message):
        topic = message[0]
        payload = message[1]
        try:
            sensor_data = json.loads(payload)
            address = sensor_data["Address"]
            if address in self.sensors:
                stored_nonce = self.sensors[address].nonce
                received_nonce = int(sensor_data["Nonce"].replace(" ", ""),16)
                last_seen = sensor_data["Localtime"]
                
                if stored_nonce == received_nonce:
                    if DEBUG:
                        header("Processing MQTT Message", address, self.sensors)
                        debug("Duplicate packet, ignoring.", address)
                if received_nonce > stored_nonce:
                    header("Processing MQTT Message", address, self.sensors)
                    debug("Updating stored nonce to: " + hex(received_nonce), address)
                    self.sensors[address].nonce = received_nonce
                    debug("Updating last seen to: " + last_seen, address)
                    self.sensors[address].last_seen = last_seen

                if DEBUG: debug("Stored Nonce: " + hex(stored_nonce),address)    
                if DEBUG: debug("Received Nonce: " + hex(received_nonce), address)
            
        except:
            debug("Error processing message from " + topic)


#--------------------------------------------------
# BLE Scanner
#--------------------------------------------------
class BleScanner():
    def __init__(self, q) -> None:
        self.poll_secs = 5
        self.q = q
        self.scanner = btle.Scanner().withDelegate(ScanDelegate(self.q))

    def start_polling(self):
        if DEBUG:
            debug("BLE: Start polling")
        self.poll_thread = threading.Thread(target=self.__scan_loop)
        self.running = True
        self.poll_thread.start()

    def stop_polling(self):
        if DEBUG:
            debug("BLE: Stop polling")
        self.running = False
        self.poll_thread.join(timeout=self.poll_secs + 1)

    def __scan_loop(self):
        self.scanner.clear()
        self.scanner.start(passive=True)
        
        while self.running:
            if DEBUG:
                debug("BLE: Scan...")
            self.scanner.process(self.poll_secs)
        
        self.scanner.stop()

class ScanDelegate(btle.DefaultDelegate):
    def __init__(self, q):
        self.q = q
        super().__init__()

    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev or isNewData:
            if DEBUG and str(dev.addr).startswith(('90:', '14:b4')):
                debug("ADV: {} {}".format(isNewDev, isNewData), dev.addr)
            self.q.put(dev)

class BleBeacons():

    q: queue.Queue

    def __init__(self, q, mqtt_client, sensors) -> None:
        self.mqtt_client = mqtt_client
        self.sensors = sensors
        self.q = q

    def process_beacons(self, timeout=5):
        try:
            # Wait up to timeout seconds for the first beacon
            device = self.q.get(block=True, timeout=timeout)
            while True:
                scanData = device.getScanData()
                for (_, _, value) in scanData:
                    self.__process_beacon(self.mqtt_client, self.sensors, device, value)
                device = self.q.get(block=False)
        except queue.Empty:
            # Return on timeout or after emptying the queue
            return

    def __process_beacon(self, mqtt_client, sensors, device, data):
        
        address = str(device.addr)
        valueString = str(data)
        beaconId = valueString[0:4]
        encrypted = False
        packetVersion = 0

        if beaconId == PLAINTEXT_BEACON_ID:
            encrypted = False
            packetVersion = 1
        elif beaconId == ENCRYPTED_BEACON_ID:
            encrypted = True
            packetVersion = 1
        elif beaconId == PLAINTEXT_BEACON_ID_V2:
            encrypted = False
            packetVersion = 2
        elif beaconId == ENCRYPTED_BEACON_ID_V2:
            encrypted = True
            packetVersion = 2
        else:
            # Ignore random beacons
            return

        # Plaintext beacon
        if not encrypted:
            print("")
            print("Plaintext beacon")
            print("addr: " + str(device.addr))
            print(valueString)
            # Ignore unencrypted data in production
            #  processPayload(valueString[4:32], packetVersion)
            return

        # Encrypted beacon
        try:
            location = sensors[address].name
        except:
            debug("",address)
            debug("Found Encrypted Beacon ID", address)
            debug("Unknown node, please register", address)
            debug("",address)
            return

        sensorData = {}
        sensorData["Label"] = location
        sensorData["Address"] = address


        (nonce, rebootCount) = self.__parseNonce(address, valueString, packetVersion)
        packetCount = nonce & 0x3FFFFF

        if ( nonce < sensors[address].nonce):
            header("Found Encrypted Beacon", address, sensors)
            debug("Nonce value too low, ignoring", address)
            debug("Received nonce: " + str(hex(nonce)), address)
            debug("Stored nonce: " + str(hex(sensors[address].nonce)), address)
        
        elif ( nonce == sensors[address].nonce):
            if DEBUG:
                debug("Duplicate packet, ignoring.", address)
        
        else:
            header("Received Encrypted Beacon", address, sensors)
            debug("Location: " + location, address)
            debug("RSSI: {}".format(device.rssi), address)
            if DEBUG: debug("Payload " + valueString, address)
            debug("Reboot count: " + str(rebootCount), address)
            debug("Packet counter: " + str(packetCount), address)
            sensorData["Reboot count"] = rebootCount
            sensorData["Packet count"] = packetCount
            sensorData["RSSI"] = device.rssi

            ctr = Counter.new(128, initial_value=nonce)
            aes = AES.new(SECRET_KEY, AES.MODE_CTR, counter=ctr)
            cipherPayload = bytes.fromhex(valueString[4:36])
            plainPayload = str(aes.decrypt(cipherPayload).hex())
            
            if DEBUG:
                debug("Nonce: " + str(hex(nonce)), address)
                debug("Cipher payload: " + str(cipherPayload.hex()), address)
                debug("Plain payload: " + plainPayload, address)
            
            if plainPayload[24:32] != MAGIC_BYTES:
                debug("Decryption failed. Ignoring.", address)
                return

            current_time = time.strftime(TIME_FORMAT, time.localtime())
            sensors[address].nonce = nonce
            sensors[address].last_seen = current_time
            
            self.__process_payload(plainPayload, sensorData, address, packetVersion)
            
            sensorData["Nonce"] = hex(nonce)[0:26] + " " + hex(nonce)[26:34]
            sensorData["Localtime"] = current_time
            sensorData["Gateway"] = HOSTNAME

            if mqtt_client:
                mqtt_client.client.publish(sensors[address].topic + NODE_TOPIC,
                                        json.dumps(sensorData), 
                                        retain = True)

    def __process_payload(self, payload, sensorData, address, packetVersion):

        if packetVersion == 1:
            swVersion = int(payload[22:23],16) + int(payload[23:24],16)/10
            temp = round(self.__toSigned16(int(payload[0:4], 16))/100, 2)
            humidity = round(int(payload[4:8], 16)/100, 2)
            voltage = round(int(payload[8:12], 16)/100, 3)
            uptime = int(payload[12:20], 16)
        else:
            swVersion = int(payload[0],16) + int(payload[1],16)/10
            humidity = round(int(payload[2:4], 16)/2, 3)
            temp = round(self.__toSigned16(int(payload[4:8], 16))/100, 2)
            uptime = int(payload[8:12], 16)
            voltage = round(int(payload[12:14], 16)/50, 2)
            # High bit means we're counting hours instead of seconds
            if uptime & 0x8000:
                uptime &= 0x7FFF
                uptime *= 3600

        debug("Pkt Version:      {}".format(packetVersion), address)   
        debug("Software Version: {}".format(swVersion), address)
        debug("Temperature:      {} C".format(temp), address)
        debug("Humidity:         {} %".format(humidity), address)
        debug("Battery Voltage:  {} V".format(voltage), address)
        debug("Uptime:           {} s".format(uptime), address)
        debug("Uptime:           {}".format(self.__seconds_to_time(uptime)), address)
        
        sensorData["Software version"] = swVersion
        sensorData["Temperature"] = temp
        sensorData["Humidity"] = humidity
        sensorData["Battery Voltage"] = voltage
        sensorData["Uptime seconds"] = uptime
        sensorData["Uptime"] = self.__seconds_to_time(uptime)

    @staticmethod
    def __parseNonce(address, valueString, packetVersion):

        address = address.replace(':', '')
        addrhash = hashlib.sha256(bytes.fromhex(address)).hexdigest()[0:24]
        nonce = (int(addrhash, 16) << 32) + int(valueString[36:44], 16)
        rebootCount = int(valueString[36:44], 16) >> 22

        return (nonce, rebootCount)


    # Returns nice uptime string
    @staticmethod
    def __seconds_to_time(seconds):
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

    @staticmethod
    def __toSigned16(n):
        n = n & 0xffff
        return n | (-(n & 0x8000))

def header(topic, address, sensor_set):
    debug("", address)
    debug("-----------------------------------------------------------", address)
    debug(topic + ": " + sensor_set[address].name, address)
    debug("-----------------------------------------------------------", address)
    debug("Address: " + address, address)


#--------------------------------------------------
# Main loop
#--------------------------------------------------
def main():

    CFG_FILE = os.path.join(SCRIPT_DIR, 'config.yaml')

    bleDebug = False
    if len(sys.argv) > 1 and sys.argv[1] == 'bledebug':
        bleDebug = True

    try:
        with open(CFG_FILE) as f:
            cfg = yaml.safe_load(f)

    except FileNotFoundError:
        print('ERROR: Could not find config.yaml in {}'.format(SCRIPT_DIR))
        return -1

    sensors = {}
    for name, c in cfg['sensors'].items():
        sensor = Sensor()
        sensor.name = name
        sensor.topic = c['topic'].strip()
        if not sensor.topic.endswith('/'):
            sensor.topic += '/'
        addr = c['addr']
        try:
            sensor.type = c['type']
        except KeyError:
            sensor.type = None
        sensors[addr] = sensor

    if bleDebug:
        mqtt_client = None
    else:
        mqtt_client = Mqtt(cfg['mqtt'], sensors)

    bleQueue = queue.Queue()
    scanner = BleScanner(bleQueue)
    beaconHandler = BleBeacons(bleQueue, mqtt_client, sensors)

    scanner.start_polling()
    try:
        # mainloop
        while True:
            # First, make sure we are connected
            if mqtt_client:
                while not mqtt_client.connected_flag: #wait in loop
                    time.sleep(1)

                # Process any waiting messages
                mqtt_client.process_messages()

            # Sleep until beacons are available - process them immediately
            beaconHandler.process_beacons(timeout=5)

    except KeyboardInterrupt:
        if mqtt_client:
            mqtt_client.client.loop_stop()    #Stop loop
            mqtt_client.client.disconnect() # disconnect
        
        scanner.stop_polling()



if __name__ == "__main__":
    main()
