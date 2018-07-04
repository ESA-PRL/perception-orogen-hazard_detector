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

    frame_count_while_stopped = 0;

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

    if (_distance_frame.read(distance_image) != RTT::NewData)
    {
        return;
    }

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
        return;
    }
    else if (!hazard_detector->isCalibrated())
    {
        hazard_detector->readCalibrationFile(calibration_path);
    }

    // calibration is done
    if (!new_calibration)
    {
        std::pair< uint16_t, uint16_t > dist_dims = {distance_image.height, distance_image.width};
        cv::Mat visual_image = frame_helper::FrameHelper::convertToCvMat(camera_frame);
        bool obstacle_detected = hazard_detector->analyze(distance_image.data, dist_dims, visual_image);

        base::Time cur_time = base::Time::now();

        camera_frame = cvMatToFrame(visual_image);
        camera_frame.time = cur_time;
        camera_frame.received_time = cur_time;

        _hazard_visualization.write(camera_frame);

        // the rover is only allowed to do point turns in the presence of hazards
        base::commands::Motion2D motion_command;
        _motion_command.read(motion_command);

        if (motion_command.translation == 0 && motion_command.rotation != 0)
        {
            return;
        }
        else if (motion_command.translation > 0)
        {
            frame_count_while_stopped = 0;
        }

        _hazard_detected.write(obstacle_detected);

        if (obstacle_detected)
        {
            if (frame_count_while_stopped < num_frames_while_stopped)
            {
                std::vector<uint8_t> new_trav_map = hazard_detector->getTraversabilityMap();
                if (frame_count_while_stopped == 0)
                {
                    trav_map.resize(new_trav_map.size(), hazard_detector->TRAVERSABLE);
                }
                accumulateHazardPixels(new_trav_map);
                frame_count_while_stopped++;
            }
            else
            {
                writeThresholdedTraversabilityMap(cur_time);
                frame_count_while_stopped = 0;
            }
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

void Task::accumulateHazardPixels(std::vector<uint8_t> new_trav_map)
{
    std::transform(new_trav_map.begin(), new_trav_map.end(), new_trav_map.begin(),
    std::bind(std::divides<uint8_t>(), std::placeholders::_1, hazard_detector->HAZARD));
    std::transform(new_trav_map.begin(), new_trav_map.end(), trav_map.begin(), trav_map.begin(),
    std::plus<uint8_t>());
}

void Task::writeThresholdedTraversabilityMap(const base::Time& cur_time)
{
    std::vector<uint8_t> trav_map = hazard_detector->getTraversabilityMap();
    int height = hazard_detector->getTravMapHeight();
    int width  = hazard_detector->getTravMapWidth();
    base::samples::frame::Frame trav_frame(width, height, base::samples::frame::MODE_GRAYSCALE);

    std::transform(trav_map.begin(), trav_map.end(), trav_map.begin(),
            [&](uint8_t x){return x >= hazard_threshold ? hazard_detector->HAZARD : hazard_detector->TRAVERSABLE;});

    trav_frame.setImage(trav_map);
    trav_frame.time = cur_time;
    trav_frame.received_time = cur_time;
    _local_traversability.write(trav_frame);
}
