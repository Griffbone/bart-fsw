#!/bin/bash
PORT=$1

python3 ./data/sensorFaking/sensorFaking.py ./data/sensorFaking/data/sensor_log.csv ${PORT} 115200
