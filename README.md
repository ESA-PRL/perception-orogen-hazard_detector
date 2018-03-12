# Hazard Detector
[![Build Status](https://travis-ci.org/ESA-PRL/perception-orogen-hazard_detector.svg?branch=master)](https://travis-ci.org/ESA-PRL/perception-orogen-hazard_detector)

The hazard detector analyzes an input distance image, compares it to a calibration matrix and outputs whether it detected a hazard.
This can be used to let a rover stop in case there is an obstacle just in front.

The following demo shows the hazard detector's visualization:
![Demo](https://github.com/hdpr-rover/perception-orogen-hazard_detector/blob/master/demo.gif?raw=true)
Green: Area under consideration, Red: Hazard.

This config file lists all available configuration options: https://github.com/hdpr-rover/bundles-hdpr/blob/rock_master/config/orogen/hazard_detector::Task.yml
