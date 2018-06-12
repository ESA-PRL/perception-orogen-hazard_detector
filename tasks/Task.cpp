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
    delete hazard_detector;
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    this->config = _config.get();

    hazard_detector = new HazardDetector(config);

    calibration_path = config.calibrationPath;
    num_calibration_samples = config.numCalibrationSamples;
    cur_calibration_sample = 0;
    new_calibration = config.newCalibration;

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

    if (_distance_frame.read(distance_image) == RTT::NewData)
    {
        _camera_frame.read(camera_frame);

        // do we need to calibrate or can we just load a calibration?
        if (new_calibration)
        {
            if (calibrate(distance_image) == num_calibration_samples)
            {
                new_calibration = false;
                hazard_detector->setCalibration(calibration);
                hazard_detector->saveCalibrationFile(calibration_path);
            }
        }
        else if (!hazard_detector->isCalibrated())
        {
            hazard_detector->readCalibrationFile(calibration_path);
        }

        if (!new_calibration)
        {
            std::pair< uint16_t, uint16_t > dist_dims = {distance_image.height, distance_image.width};
            cv::Mat visual_image = frame_helper::FrameHelper::convertToCvMat(camera_frame);
            bool obstacle_detected = hazard_detector->analyze(distance_image.data, dist_dims, visual_image);

            base::Time cur_time = base::Time::now();

            if (obstacle_detected)
            {
                std::vector<uint8_t> trav_map = hazard_detector->getTraversabilityMap();
                int height = hazard_detector->getTravMapDims();
                int width  = hazard_detector->getTravMapDims();
                base::samples::frame::Frame trav_frame(height, width, base::samples::frame::MODE_GRAYSCALE);
                trav_frame.setImage(trav_map);
                trav_frame.time = cur_time;
                trav_frame.received_time = cur_time;
                _local_traversability.write(trav_frame);
            }

            //frame_helper::FrameHelper::copyMatToFrame(res.second, camera_frame);
            camera_frame = cvMatToFrame(visual_image);
            camera_frame.time = cur_time;
            camera_frame.received_time = cur_time;

            _hazard_detected.write( obstacle_detected );
            _hazard_visualization.write( camera_frame );
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

int Task::calibrate(const base::samples::DistanceImage &distance_image)
{
    cur_calibration_sample++;

    // initialize matrices if necessary
    if (calibration.size() == 0)
    {
        std::vector<float> init_val_calib(distance_image.width, NAN);
        calibration.resize(distance_image.height, init_val_calib);

        std::vector<int> init_val_count(distance_image.width, 0);
        sample_count_per_pixel.resize(distance_image.height, init_val_count);
    }

    // add current readings to calibration matrix,
    // average in last sample
    for (int i = 0; i < distance_image.height; i++)
    {
        for (int j = 0; j < distance_image.width; j++)
        {
            int index = i*distance_image.width + j;

            if (std::isnan(calibration[i][j]) && !std::isnan(float(distance_image.data[index])))
            {
                calibration[i][j] = float(distance_image.data[index]);
                sample_count_per_pixel[i][j]++;
            }
            else if (!std::isnan(distance_image.data[index]))
            {
                calibration[i][j] += float(distance_image.data[index]);
                sample_count_per_pixel[i][j]++;
            }

            // if calibration is finished, average over samples
            if (cur_calibration_sample == num_calibration_samples)
            {
                calibration[i][j] /= float(sample_count_per_pixel[i][j]);
            }
        }
    }
    return cur_calibration_sample;
}
