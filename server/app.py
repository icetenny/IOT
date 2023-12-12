import paho.mqtt.publish as publish # pip3 install paho-mqtt
import paho.mqtt.client as mqtt
import time
import json
import random
import ssl
import io
import os
import numpy as np
from PIL import Image
from threading import Lock
from base64 import b64encode
from datetime import datetime
from flask_socketio import SocketIO, emit
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS

"""
Background Thread
"""
thread = None
thread_lock = Lock()

global send_io

port = 1883 # default port
Server_ip = "broker.netpie.io" 
Alias = "Server"

Subscribe_Topic = "@msg/Raspi"
Publish_Topic = "@msg/Server"
# Client_ID = "60ca787e-9f68-4ec4-b4bb-03bc557b8986"
# Token = "mbPcid4ym5uNMyhFYkZd6oUcAjkUipBs"
# Secret = "y1ofkQS8r6BjAxjjfTJJTydYxzp4LGam"

Client_ID = "659e4ffa-8d80-4e37-8ff9-743c1599b833"
Token = "FTnYMzJrsB7UAZq9qH2VMC1UJBAxt6dM"
Secret = "Skp24RH2pfuQpTEcaTixXZo69HBkywPn"

MqttUser_Pass = {"username":Token,"password":Secret}

app = Flask(__name__)
CORS(app)
app.config['SECRET_KEY'] = 'secret key'
socketio = SocketIO(app, cors_allowed_origins='*')

"""
Get current date time
"""
def get_current_datetime():
    now = datetime.now()
    return now.strftime("%m/%d/%Y %H:%M:%S")

"""
Generate random sequence of dummy sensor values and send it to our clients
"""

def callback(data):
    socketio.emit('updateSensorData', {'value_lin': data.linear.x,'value_ang': data.angular.z, "date": get_current_datetime()})

def state_callback(data):
    socketio.emit('updateState', {'value': data.data, "date": get_current_datetime()}) 


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe(Subscribe_Topic)

def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))
    global latest_data
    latest_data =  json.loads(msg.payload.decode('utf-8'))
    print(latest_data)

def background_thread():
    global car_status_data

    client = mqtt.Client(protocol=mqtt.MQTTv311,client_id=Client_ID, clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message

    client.subscribe(Subscribe_Topic)
    client.username_pw_set(Token,Secret)
    client.connect(Server_ip, port)
    client.loop_start()

    car_status = random.choice(['collected','waitting'])
    while True:
            data = {
            "Alias" : Alias,
            "car_status" : car_status
            }
            data_out=json.dumps(data) # encode object to JSON
            client.publish(Publish_Topic, data_out, retain= True)
            car_status_data = data_out
            print("Publish.....")
            time.sleep(2)


@app.route('/', methods=['GET', 'POST'])
def index():
    return render_template('index.html')


@app.route("/get_info_status", methods=["GET", "POST"])
def status():
    try:
        time.sleep(1)
        global latest_data
        recieve_data = latest_data
        return jsonify(recieve_data)
    except Exception as e:
        return jsonify({"error": str(e)})


@app.route("/callingcar", methods=["GET" , "POST"])
def car_status():
    try:
        time.sleep(1)
        global car_status_data
        recieve_data = {'msg': car_status_data}
        return jsonify(recieve_data)
    except Exception as e:
        return jsonify({"error": str(e)})

"""
Decorator for connect
"""
@socketio.on('connect')
def connect():
    global thread
    print('Client connected')

    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_thread)

"""
Decorator for disconnect
"""
@socketio.on('disconnect')
def disconnect():
    print('Client disconnected',  request.sid)
  
if __name__ == '__main__':
    global latest_data
    global car_status_data
    socketio.run(app)