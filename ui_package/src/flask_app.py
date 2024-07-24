#!/usr/bin/env python3

from flask import Flask, render_template
import yaml
import rospy
app = Flask(__name__)


@app.route('/')
@app.route('/route')
@app.route('/control')
@app.route('/info')
def index():
    return render_template('index.html')


if __name__ == '__main__':
    local_ip = rospy.get_param("appAddress", "127.0.0.1")
    local_port = rospy.get_param("portApp", "50505")
    
    app.run(host=local_ip, port=local_port)
