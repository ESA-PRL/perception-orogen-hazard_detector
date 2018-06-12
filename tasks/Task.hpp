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
        HazardDetector* hazard_detector;
        base::samples::DistanceImage distance_image; // used to compute location of hazards
        base::samples::frame::Frame camera_frame;    // used to visualize location of hazards
        base::samples::frame::Frame cvMatToFrame(cv::Mat);
        int calibrate(const base::samples::DistanceImage&);

        hazard_detector::Config config;
        int num_calibration_samples;
        int cur_calibration_sample;
        std::string calibration_path;
        bool new_calibration;
        std::vector< std::vector<int> > sample_count_per_pixel;
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
