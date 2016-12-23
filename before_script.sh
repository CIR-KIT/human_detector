#!/bin/bash
apt-get install -y software-properties-common python-software-properties
yes | apt-add-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl; apt-get update -qq; apt-get install -y libpcl-all;
