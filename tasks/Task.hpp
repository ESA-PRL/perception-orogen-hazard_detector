/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef HAZARD_DETECTOR_TASK_TASK_HPP
#define HAZARD_DETECTOR_TASK_TASK_HPP

#include "hazard_detector/TaskBase.hpp"

namespace hazard_detection {
    class HazardDetector;

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        HazardDetector* hazardDetector;

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
