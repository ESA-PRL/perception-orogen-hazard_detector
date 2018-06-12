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

    this->config = _config.get();

    hazardDetector = new HazardDetector(config);

    calibrationPath = config.calibrationPath;
    numCalibrationSamples = config.numCalibrationSamples;
    curCalibrationSample = 0;
    newCalibration = config.newCalibration;

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

        // do we need to calibrate or can we just load a calibration?
        if (newCalibration)
        {
            if (calibrate(distanceImage) == numCalibrationSamples)
            {
                newCalibration = false;
                hazardDetector->setCalibration(calibration);
                hazardDetector->saveCalibrationFile(calibrationPath);
            }
        }
        else if (!hazardDetector->isCalibrated())
        {
            hazardDetector->readCalibrationFile(calibrationPath);
        }

        if (!newCalibration)
        {
            std::pair< uint16_t, uint16_t > distDims = {distanceImage.height, distanceImage.width};
            cv::Mat visualImage = frame_helper::FrameHelper::convertToCvMat(cameraFrame);
            bool obstacleDetected = hazardDetector->analyze(distanceImage.data, distDims, visualImage);

            base::Time cur_time = base::Time::now();

            if (obstacleDetected)
            {
                std::vector<uint8_t> trav_map = hazardDetector->getTraversabilityMap();
                int trav_map_height = hazardDetector->getTravMapDims();
                int trav_map_width  = hazardDetector->getTravMapDims();
                base::samples::frame::Frame trav_frame(trav_map_height, trav_map_width, base::samples::frame::MODE_GRAYSCALE);
                trav_frame.setImage(trav_map);
                trav_frame.time = cur_time;
                trav_frame.received_time = cur_time;
                _local_traversability.write(trav_frame);
            }

            //frame_helper::FrameHelper::copyMatToFrame(res.second, cameraFrame);
            cameraFrame = cvMatToFrame(visualImage);
            cameraFrame.time = cur_time;
            cameraFrame.received_time = cur_time;

            _hazard_detected.write( obstacleDetected );
            _hazard_visualization.write( cameraFrame );
        }
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

base::samples::frame::Frame Task::cvMatToFrame(cv::Mat cvmat)
{
    cv::normalize(cvmat, cvmat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::cvtColor(cvmat, cvmat, cv::COLOR_BGR2RGB);

    base::samples::frame::Frame frame(
            cvmat.rows, cvmat.cols, 8,
            base::samples::frame::MODE_GRAYSCALE );
    frame_helper::FrameHelper::copyMatToFrame(cvmat, frame);

    return frame;
}

int Task::calibrate(const base::samples::DistanceImage &distanceImage)
{
    curCalibrationSample++;

    // initialize matrices if necessary
    if (calibration.size() == 0)
    {
        std::vector<float> initValCalib(distanceImage.width, NAN);
        calibration.resize(distanceImage.height, initValCalib);

        std::vector<int> initValCount(distanceImage.width, 0);
        sampleCountPerPixel.resize(distanceImage.height, initValCount);
    }

    // add current readings to calibration matrix,
    // average in last sample
    for (int i = 0; i < distanceImage.height; i++)
    {
        for (int j = 0; j < distanceImage.width; j++)
        {
            int index = i*distanceImage.width + j;

            if (std::isnan(calibration[i][j]) && !std::isnan(float(distanceImage.data[index])))
            {
                calibration[i][j] = float(distanceImage.data[index]);
                sampleCountPerPixel[i][j]++;
            }
            else if (!std::isnan(distanceImage.data[index]))
            {
                calibration[i][j] += float(distanceImage.data[index]);
                sampleCountPerPixel[i][j]++;
            }

            // if calibration is finished, average over samples
            if (curCalibrationSample == numCalibrationSamples)
            {
                calibration[i][j] /= float(sampleCountPerPixel[i][j]);
            }
        }
    }
    return curCalibrationSample;
}
