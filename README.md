# Hazard Detector
[![Build Status](https://travis-ci.org/ESA-PRL/perception-orogen-hazard_detector.svg?branch=master)](https://travis-ci.org/ESA-PRL/perception-orogen-hazard_detector)

The hazard detector analyzes an input distance image, compares it to a calibration matrix and outputs whether it detected a hazard.
This can be used to let a rover stop in case there is an obstacle just in front.

The following picture presents the hazard detector's visualization:
![roi_visualization](media/roi_visualization.png?s=50)
Green: Area under consideration, Red: Hazard.

From this, we compute a local traversability map in the rover frame. White pixels depict the detected hazard:
![local_traversability](media/local_traversability.png)


This config file lists all available configuration options: https://github.com/esa-prl/bundles-rover/blob/master/config/orogen/hazard_detector::Task.yml
