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
    writeCalibration = false;

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

        if (newCalibration)
        {
            if (calibrate(distanceImage) == numCalibrationSamples)
            {
                newCalibration = false;
                writeCalibration = true;
            }
        }

        if (writeCalibration)
        {
            hazardDetector->saveCalibrationFile(calibrationPath);
            writeCalibration = false;
            //hazardDetector->readCalibration(calibrationPath);
            hazardDetector->setCalibration(calibration);
        }

        if (!newCalibration)
        {
            std::pair< uint16_t, uint16_t > distDims = {distanceImage.height, distanceImage.width};
            cv::Mat visualImage = frame_helper::FrameHelper::convertToCvMat(cameraFrame);
            bool obstacleDetected = hazardDetector->analyze(distanceImage.data, distDims, visualImage);
            //frame_helper::FrameHelper::copyMatToFrame(res.second, cameraFrame);
            cameraFrame = cvMatToFrame(visualImage);

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
