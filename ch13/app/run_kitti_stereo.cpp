//
// Created by gaoxiang on 19-5-4.
//

#include <iostream>

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
#include <myslam/config.h>

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    
    std::cout << myslam::Config::Get<int>("isw_block") << std::endl;
    
    vo->Run();
    
    std::cout << "[Quit]" << std::endl;

    return 0;
}
