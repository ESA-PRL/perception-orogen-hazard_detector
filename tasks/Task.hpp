#ifndef HAZARD_DETECTOR_TASK_TASK_HPP
#define HAZARD_DETECTOR_TASK_TASK_HPP

#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <functional>
#include <string>

#include <opencv2/core/core.hpp>

#include "hazard_detector/TaskBase.hpp"
#include "frame_helper/FrameHelper.h"

#include <hazard_detector/HazardDetector.hpp>

namespace hazard_detector
{

class Task : public TaskBase
{
    friend class TaskBase;

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

    protected:
        HazardDetector* hazard_detector;

        // used to compute location of hazards
        base::samples::DistanceImage distance_image;

        // used to visualize location of hazards
        base::samples::frame::Frame camera_frame;

        base::samples::frame::Frame cvMatToFrame(cv::Mat);

        // Takes a DistanceImage as input and computes (averages)
        // the expected projective distance from the camera to level ground
        // at each pixel.
        // Returns the number of calibration samples already taken into account.
        int calibrate(const base::samples::DistanceImage&);

        hazard_detector::Config config;

        // number of DistanceImage samples which are to be averaged
        int num_calibration_samples;
        int cur_calibration_sample;

        std::string calibration_path;
        bool new_calibration;
        std::vector< std::vector<int> > sample_count_per_pixel;
        std::vector< std::vector<float> > calibration;

        // For consecutive input traversability maps,
        // sum up how many times each pixel was considered a hazard.
        // The result is saved in the global trav_map.
        void accumulateHazardPixels(std::vector<uint8_t> new_trav_map);

        void writeThresholdedTraversabilityMap(const base::Time&);

        int frame_count_while_stopped = 0;

        std::vector<uint8_t> trav_map;
};

}  // namespace hazard_detector

#endif
