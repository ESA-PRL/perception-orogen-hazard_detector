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
    delete hazardDetector;
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

        cv::Mat visualImage = frame_helper::FrameHelper::convertToCvMat(cameraFrame);
        std::pair< uint16_t, uint16_t > distDims = {distanceImage.height, distanceImage.width};
        std::pair< bool, cv::Mat > res = hazardDetector->analyze(distanceImage.data, distDims, visualImage);
        frame_helper::FrameHelper::copyMatToFrame(res.second, cameraFrame);

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
