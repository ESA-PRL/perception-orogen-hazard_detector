#include "Task.hpp"

namespace hazard_detector
{

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
    if (!TaskBase::configureHook())
        return false;

    hazard_detector = new HazardDetector(_lib_config.get());

    cur_calibration_sample = 0;
    new_calibration = _new_calibration.value();

    frame_count_while_stopped = 0;
    timer = 0;

    return true;
}

bool Task::startHook()
{
    if (!TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if (state() == WAITING_FOR_NEW_PLAN)
    {
        bool new_plan = false;
        if (_new_plan.read(new_plan) != RTT::NewData ||
            new_plan == false)
        {
            return;
        }

        timer = _avoidance_timer.value();
        state(AVOIDANCE_MANEUVER);
    }

    if (_distance_frame.read(distance_image) != RTT::NewData)
    {
        return;
    }

    _camera_frame.read(camera_frame);

    // do we need to calibrate or can we just load a calibration?
    if (new_calibration)
    {
        state(CALIBRATING);
        if (calibrate(distance_image) == _num_calibration_samples.value())
        {
            hazard_detector->setCalibration(calibration);
            hazard_detector->saveCalibrationFile(_calibration_path.value());
            hazard_detector->computeTolerances();
            new_calibration = false;
            state(RUNNING);
        }
        return;
    }
    else if (!hazard_detector->isCalibrated())
    {
        hazard_detector->readCalibrationFile(_calibration_path.value());
        hazard_detector->computeTolerances();
        state(RUNNING);
    }

    // calibration is done
    if (!new_calibration)
    {
        std::pair<uint16_t, uint16_t> dist_dims = {distance_image.height, distance_image.width};
        cv::Mat visual_image = frame_helper::FrameHelper::convertToCvMat(camera_frame);
        bool obstacle_detected = hazard_detector->analyze(distance_image.data, dist_dims, visual_image);

        base::Time cur_time = base::Time::now();

        camera_frame = cvMatToFrame(visual_image);
        camera_frame.time = cur_time;
        camera_frame.received_time = cur_time;

        _hazard_visualization.write(camera_frame);

        // the rover is always allowed to perform point turns
        // and it may also perform an ackermann away from
        // a formerly detected hazard, ignoring that side of its
        // roi for a certain time
        base::commands::Motion2D motion_command;
        _motion_command.read(motion_command);

        // (ackermann while) partially blind
        if (state() == AVOIDANCE_MANEUVER &&
            timer > 0)
        {
            _hazard_detected.write(false);
            trustAvoidanceManeuver(motion_command);


            if (timer == 0)
            {
                hazard_detector->ignoreNothing();
                state(RUNNING);
            }
            return;
        }

        // point turn
        if (motion_command.translation == 0 &&
            motion_command.rotation != 0)
        {
            _hazard_detected.write(false);
            return;
        }

        // normal driving: reset counter
        if (motion_command.translation > 0)
        {
            frame_count_while_stopped = 0;
        }

        _hazard_detected.write(obstacle_detected);

        if (!obstacle_detected)
        {
            state(RUNNING);
            return;
        }

        if (frame_count_while_stopped < _num_frames_while_stopped.value())
        {
            state(FILTERING);
            std::vector<uint8_t> new_trav_map = hazard_detector->getTraversabilityMap();
            if (frame_count_while_stopped == 0)
            {
                trav_map.clear();
                trav_map.resize(new_trav_map.size(), hazard_detector->getValueForTraversable());
            }
            accumulateHazardPixels(new_trav_map);
            frame_count_while_stopped++;
        }
        else
        {
            if (_new_plan.connected())
            {
                state(WAITING_FOR_NEW_PLAN);
            }
            else
            {
                state(RUNNING);
            }
            writeThresholdedTraversabilityMap(cur_time);
            frame_count_while_stopped = 0;
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
            base::samples::frame::MODE_GRAYSCALE);
    frame_helper::FrameHelper::copyMatToFrame(cvmat, frame);

    return frame;
}

int Task::calibrate(const base::samples::DistanceImage& distance_image)
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
            int index = i * distance_image.width + j;
            float distance = static_cast<float>(distance_image.data[index]);

            if (std::isnan(calibration[i][j]) && !std::isnan(distance))
            {
                calibration[i][j] = distance;
                sample_count_per_pixel[i][j]++;
            }
            else if (!std::isnan(distance))
            {
                calibration[i][j] += distance;
                sample_count_per_pixel[i][j]++;
            }

            // if calibration is finished, average over samples
            if (cur_calibration_sample == _num_calibration_samples.value())
            {
                calibration[i][j] /= static_cast<float>(sample_count_per_pixel[i][j]);
            }
        }
    }
    return cur_calibration_sample;
}

void Task::accumulateHazardPixels(std::vector<uint8_t> new_trav_map)
{
    // hazards are represented as value HAZARD (e.g., 255)
    // so we first divide each entry by HAZARD ...
    std::transform(new_trav_map.begin(), new_trav_map.end(), new_trav_map.begin(),
        std::bind(std::divides<uint8_t>(), std::placeholders::_1, hazard_detector->getValueForHazard()));
    // ... before summing them up with the existing counters.
    std::transform(new_trav_map.begin(), new_trav_map.end(), trav_map.begin(), trav_map.begin(),
        std::plus<uint8_t>());
}

void Task::writeThresholdedTraversabilityMap(const base::Time& cur_time)
{
    // thresholding
    std::transform(trav_map.begin(), trav_map.end(), trav_map.begin(),
        [&](uint8_t x){return x >= _hazard_threshold.value() ? hazard_detector->getValueForHazard() : hazard_detector->getValueForTraversable();});

    int height = hazard_detector->getTravMapHeight();
    int width  = hazard_detector->getTravMapWidth();
    base::samples::frame::Frame trav_frame(width, height, base::samples::frame::MODE_GRAYSCALE);

    trav_frame.setImage(trav_map);
    trav_frame.time = cur_time;
    trav_frame.received_time = cur_time;

    _local_traversability.write(trav_frame);
}

void Task::trustAvoidanceManeuver(const base::commands::Motion2D& command)
{
    // Once we started to count down the timer, i.e. started the avoidance
    // maneuver, we do not want to update which part of the RoI to ignore.
    if (timer < _avoidance_timer.value())
    {
        timer--;
        return;
    }

    // Turning left
    if (command.rotation > 0)
    {
        hazard_detector->ignoreRightSide();
        timer--;
    }
    // Turning right
    else if (command.rotation < 0)
    {
        hazard_detector->ignoreLeftSide();
        timer--;
    }
    // Hazard detected in front, but we just keep driving straight.
    // If this happens, something went wrong somewhere else.
    else if (command.translation > 0)
    {
        state(EXCEPTION);
    }
}

}  // namespace hazard_detector
