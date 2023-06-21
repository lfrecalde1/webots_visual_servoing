#!/bin/bash
echo "Installing Python libraries..."
echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
echo "export PYTHONPATH=/usr/local/webots/lib/controller/python:$PYTHONPATH" >> ~/.bashrc
source ~/.bashrc
pip install numpy
pip install scipy==1.7.2
pip install pyyaml
pip install -U rospkg
pip install empy
pip install opencv-contrib-python