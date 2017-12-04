/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace hazard_detector;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    hazardDetector = new HazardDetector();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    if (_distance_frame.read(distanceImage) == RTT::NewData)
    {
        _camera_frame.read(cameraFrame);
        std::pair< bool, std::vector<uint8_t> > res = hazardDetector->analyze(distanceImage.data, cameraFrame.image);
        // cameraFrame.image = res.second;
        _hazard_detected.write( res.first );
        _hazard_visualization.write( cameraFrame );
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
