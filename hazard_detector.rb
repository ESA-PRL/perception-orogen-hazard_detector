#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'

include Orocos

Bundles.initialize

Bundles.run 'camera_bb2::Task' => 'camera_bb2', 'stereo::Task' => 'stereo', 'hazard_detector::Task' => 'hazard_detector' do

    stereo_bb2 = Orocos.name_service.get 'stereo'
    Orocos.conf.apply(stereo_bb2, ['hdpr_bb2'], :override => true)
    stereo_bb2.configure

    camera_bb2 = Orocos.name_service.get 'camera_bb2'
    Orocos.conf.apply(camera_bb2, ['default'], :override => true)
    camera_bb2.configure

    hazard_detector = Orocos.name_service.get 'hazard_detector'
    Orocos.conf.apply(hazard_detector, ['default'], :override => true)
    hazard_detector.configure

    if ARGV.size == 0 then
        log_replay = Orocos::Log::Replay.open( "bb2.log")
    else
        log_replay = Orocos::Log::Replay.open( ARGV[0]+"bb2.log")
    end

    log_replay.use_sample_time = true

    log_replay.camera_firewire_bb2.frame.connect_to camera_bb2.frame_in
    camera_bb2.left_frame.connect_to                stereo_bb2.left_frame
    camera_bb2.right_frame.connect_to               stereo_bb2.right_frame
    camera_bb2.left_frame.connect_to                hazard_detector.camera_frame
    stereo_bb2.distance_frame.connect_to            hazard_detector.distance_frame

    reader = camera_bb2.right_frame.reader

    camera_bb2.start
    stereo_bb2.start
    hazard_detector.start

    log_replay.step

    while !log_replay.eof? do
        if reader.read_new then
            log_replay.step
            # print log_replay.sample_index
            # print ' over '
            # puts log_replay.size
        else
            sleep 0.01
        end
    end

end
