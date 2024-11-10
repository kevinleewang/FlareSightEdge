#!/usr/bin/python3
#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import yaml

from edge_ai_class import EdgeAIDemo
import utils
import numpy as np
import time

import paho.mqtt.client as mqtt

# MQTT Broker Settings
BROKER_ADDRESS = "192.168.87.21"  # Replace with your laptop's IP address
BROKER_PORT = 1883                 # Default MQTT port
KEEPALIVE = 60                     # Keepalive interval in seconds

# MQTT Topics to Subscribe To
TOPICS = [
    ("sensor/co", 0),
    ("sensor/co2", 0),   # QoS level 0
    ("sensor/smoke", 0)  # QoS level 0 
]

# Optional: MQTT Authentication
USERNAME = "your_username"  # Replace with your MQTT username if authentication is enabled
PASSWORD = "your_password"  # Replace with your MQTT password if authentication is enabled

SMOKE = None
CO = None
CO2 = None
NUM_SAMPLES = 0

def predict_logistic_regression(input_values):
    # Add bias term to the input values
    weight_smoke = 0.01809941
    weight_co    = -0.03103623
    weight_co2   = -0.01070597
    weights      = np.array([weight_smoke, weight_co, weight_co2])
    bias         = -0.00660117
    
    # Calculate the dot product of input values and weights
    dot_product  = np.dot(input_values, weights)
    
    # Calculate the predicted probability using the logistic function
    prediction   = 1 / (1 + np.exp(-dot_product - bias))
    return prediction

def on_connect(client, userdata, flags, rc):
    """
    Callback when the client receives a CONNACK response from the server.
    """
    if rc == 0:
        with open("log.txt", "a") as file:
            file.write("Connected to MQTT Broker successfully.\n")
        # Subscribe to the specified topics
            for topic, qos in TOPICS:
                client.subscribe(topic, qos)
                file.write(f"Subscribed to topic: {topic} with QoS: {qos}\n")
    else:
        with open("log.txt", "a") as file:
            file.write(f"Failed to connect to MQTT Broker. Return code: {rc}\n")

def on_message(client, userdata, msg):
    """
    Callback when a PUBLISH message is received from the server.
    """
    global CO, CO2, SMOKE, NUM_SAMPLES

    topic = msg.topic
    payload = msg.payload.decode('utf-8')
    if topic == "sensor/co2":
        CO2 = float(payload)
    elif topic == "sensor/smoke":
        SMOKE = float(payload)
    elif topic == "sensor/co":
        CO = float(payload)

    if SMOKE and CO and CO2:
        prediction = predict_logistic_regression([SMOKE, CO, CO2])
        with open("log.txt", "a") as file:
            file.write(f"SMOKE: {SMOKE}, CO: {CO}, CO2: {CO2}\n")
            file.write(f"Prediction: {prediction}\n\n")
        SMOKE = None
        CO = None
        CO2 = None
        f = open(f"sample{NUM_SAMPLES}.npy", "w")
        NUM_SAMPLES += 1

def on_disconnect(client, userdata, rc):
    """
    Callback when the client disconnects from the broker.
    """
    if rc != 0:
        with open("log.txt", "a") as file:
            file.write("Unexpected disconnection from MQTT Broker.\n")
    else:
        with open("log.txt", "a") as file:
            file.write("Disconnected from MQTT Broker.\n")


def main(sys_argv):
    with open("/opt/edgeai-gst-apps/log.txt", "a") as file:
            file.write("App edge ai ran")

    args = utils.get_cmdline_args(sys_argv)

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    try:
    # Connect to the MQTT Broker
        client.connect(BROKER_ADDRESS, BROKER_PORT, KEEPALIVE)

    # Start the Network Loop
        client.loop_forever()

    except Exception as e:
        with open("log.txt", "a") as file:
            file.write(f"An error occurred: {e}\n")

    with open(args.config, "r") as f:
        config = yaml.safe_load(f)

    try:
        demo = EdgeAIDemo(config)
        demo.start()

        if args.verbose:
            utils.print_stdout = True

        if not args.no_curses:
            utils.enable_curses_reports(demo.title)

        demo.wait_for_exit()
    except KeyboardInterrupt:
        demo.stop()
    finally:
        pass

    utils.disable_curses_reports()

    del demo


if __name__ == "__main__":
    main(sys.argv)
