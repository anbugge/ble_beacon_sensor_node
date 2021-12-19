import json

BASE = "homeassistant"

# Assumes that there is a json that contains the "key" in sensor_topic_base/sensor
# If availability is enabled, it assumes a topic sensor_topic_base/status that contains availability
# Key is assumed equal to title unless specified
def register_sensor(client, 
                    sensor_topic_base, 
                    title, 
                    sensor_type, 
                    key=None,
                    unique_id=None,
                    device=None,
                    availability=False,
                    expire_after=0):
    
    if key == None:
        key = title

    config = {}
    config["name"] = title
    delimiter = "/"
    if sensor_topic_base.endswith("/"):
        delimiter = ""
    config["state_topic"] = sensor_topic_base + delimiter + "sensor"
    config["json_attributes_topic"] = sensor_topic_base + delimiter + "sensor"
    config["value_template"] = "{{ value_json['" + key + "'] }}"
    config["expire_after"] = expire_after
    config["state_class"] = 'measurement'

    if device != None:
        if unique_id == None:
            config["unique_id"] = clean_as_topic(title)
        else:
            config["unique_id"] = unique_id

        config["object_id"] = config["unique_id"]
        
        device = dict(device)
        if "identifiers" not in device:
            device["identifiers"] = config["unique_id"]
        config["device"] = device


    if availability:
        config["availability_topic"] = sensor_topic_base + delimiter + "status"
        config["payload_available"] = "Online"
        config["payload_not_available"] = "Offline"
    
    # Set icon etc. etc. based on type
    if sensor_type == "power":
        config["unit_of_measurement"] = "W"
        config["icon"] = "mdi:power-plug"
        config["device_class"] = "power"
    elif sensor_type == "heat":
        config["unit_of_measurement"] = "W"
        config["icon"] = "mdi:radiator"
        config["device_class"] = "power"
    elif sensor_type == "energy" :
        config["unit_of_measurement"] = "kWh"
        config["icon"] = "mdi:flash"
        config["device_class"] = "energy"
    elif sensor_type == "voltage":
        config["unit_of_measurement"] = "V"
        config["icon"] = "mdi:power-plug"
        config["device_class"] = "voltage"
    elif sensor_type == "battery-voltage":
        config["unit_of_measurement"] = "V"
        config["device_class"] = "voltage"
        config["icon"] = "mdi:battery"
    elif sensor_type == "current":
        config["unit_of_measurement"] = "A"
        config["icon"] = "mdi:power-plug"
        config["device_class"] = "current"
    elif sensor_type == "flow":
        config["unit_of_measurement"] = "L/min"
        config["icon"] = "mdi:pipe"
    elif sensor_type == "water":
        config["unit_of_measurement"] = "L"
        config["icon"] = "mdi:water"
    elif sensor_type == "temperature":
        config["unit_of_measurement"] = "°C"
        config["device_class"] = "temperature"
    elif sensor_type == "humidity":
        config["unit_of_measurement"] = "%"
        config["device_class"] = "humidity"
    elif sensor_type == "battery":
        config["unit_of_measurement"] = "%"
        config["device_class"] = "battery"
    else:
        print("ERROR: Unknown sensor type.")
        config = {}


    disctopic = BASE + "/sensor/" + clean_as_topic(title) + "-" + sensor_type + "/config"
    client.publish(disctopic, json.dumps(config, ensure_ascii=False).encode("utf-8"), retain=True)
    print("Registering sensor " + title)

def clean_as_topic(s):
    s = s.lower()
    s = s.replace(" ","_")
    s = s.replace("æ","ae")
    s = s.replace("ø","oe")
    s = s.replace("å","aa")
    return s

def register_camera(client, 
                    camera_topic, 
                    title):
    config = {}
    config["name"] = title
    config["topic"] = camera_topic
    disctopic = BASE + "/camera/" + clean_as_topic(title) + "/config"
    client.publish(disctopic, json.dumps(config), retain=True)
    print("Registering camera " + title)


