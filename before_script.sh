#!/bin/bash
yes | apt-add-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl; apt-get update -qq; apt-get install -y libpcl-all;
