import requests
import os

REQUEST_URL = os.getenv("REQUEST_URL", "http://localhost:8000/data")

def send(data):
    try:
        requests.post(REQUEST_URL, json=data, timeout=2)
    except requests.RequestException as e:
        print(f'Error sending data: {e}')

def callback(self, msg):
    data = {
        'latitude': msg.latitude,
        'longitude': msg.longitude,
        'altitude': msg.altitude,
        'target_latitude': msg.target_latitude,
        'target_longitude': msg.target_longitude,
        'target_altitude': msg.target_altitude,
    }
    send(data)
