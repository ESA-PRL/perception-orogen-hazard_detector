#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'rock/bundle'
require 'vizkit'

include Orocos

Bundles.initialize

Bundles.run 'camera_bb2::Task' => 'camera', 'stereo::Task' => 'stereo', 'hazard_detector::Task' => 'hazard_detector' do

    stereo = Orocos.name_service.get 'stereo'
    Orocos.conf.apply(stereo, ['hdpr_bb2'], :override => true)
    stereo.configure

    camera = Orocos.name_service.get 'camera'
    Orocos.conf.apply(camera, ['hdpr_bb2'], :override => true)
    camera.configure

    hazard_detector = Orocos.name_service.get 'hazard_detector'
    Orocos.conf.apply(hazard_detector, ['bb2'], :override => true)
    hazard_detector.configure

    if ARGV.size == 0 then
        log_replay = Orocos::Log::Replay.open( "bb2.log")
    else
        log_replay = Orocos::Log::Replay.open( ARGV[0]+"bb2.log")
    end

    log_replay.use_sample_time = true

    log_replay.camera_firewire_bb2.frame.connect_to camera.frame_in
    camera.left_frame.connect_to                stereo.left_frame
    camera.right_frame.connect_to               stereo.right_frame
    stereo.left_frame_sync.connect_to           hazard_detector.camera_frame
    stereo.distance_frame.connect_to            hazard_detector.distance_frame

    reader = camera.right_frame.reader

    camera.start
    stereo.start
    hazard_detector.start

    Vizkit.display hazard_detector.hazard_visualization
    Vizkit.control log_replay
    Vizkit.exec

end
