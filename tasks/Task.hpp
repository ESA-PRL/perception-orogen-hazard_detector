#ifndef HAZARD_DETECTOR_TASK_TASK_HPP
#define HAZARD_DETECTOR_TASK_TASK_HPP

#include <iostream>
#include <fstream>
#include <math.h>
#include "hazard_detector/TaskBase.hpp"
#include "frame_helper/FrameHelper.h"
#include <opencv2/core/mat.hpp>
#include <hazard_detector/HazardDetector.hpp>

namespace hazard_detector {

    class Task : public TaskBase
    {
    friend class TaskBase;
    protected:
        HazardDetector* hazardDetector;
        base::samples::DistanceImage distanceImage; // used to compute location of hazards
        base::samples::frame::Frame cameraFrame;    // used to visualize location of hazards
        base::samples::frame::Frame cvMatToFrame(cv::Mat);
        int calibrate(const base::samples::DistanceImage&);

        hazard_detector::Config config;
        int numCalibrationSamples;
        int curCalibrationSample;
        std::string calibrationPath;
        bool newCalibration;
        std::vector< std::vector<int> > sampleCountPerPixel;
        std::vector< std::vector<float> > calibration;

    public:
        Task(std::string const& name = "hazard_detector::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
