#include <common_utils/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <mbot_lcm_msgs/robot_path_t.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{
    int numTimes = 4;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    mbot_lcm_msgs::robot_path_t path;
    path.path.resize(9);
    
    mbot_lcm_msgs::pose_xyt_t nextPose;
    nextPose.x = 0.61f;
    nextPose.y = 0.61f*0;
    nextPose.theta = 0.0f;
    path.path[0] = nextPose;

    nextPose.x = 0.61f;
    nextPose.y = 0.61f;
    nextPose.theta = 0.0f;
    path.path[1] = nextPose;

    // nextPose.x = 0.61f*2;
    // nextPose.y = 0.61f;
    // nextPose.theta = 0.0f;
    // path.path[2] = nextPose;

    // nextPose.x = 0.61f*2;
    // nextPose.y = 0.61f*-1;
    // nextPose.theta = 0.0f;
    // path.path[3] = nextPose;

    // nextPose.x = 0.61f*3;
    // nextPose.y = 0.61f*-1;
    // nextPose.theta = 0.0f;
    // path.path[4] = nextPose;

    // nextPose.x = 0.61f*3;
    // nextPose.y = 0.61f;
    // nextPose.theta = 0.0f;
    // path.path[5] = nextPose;

    // nextPose.x = 0.61f*4;
    // nextPose.y = 0.61f;
    // nextPose.theta = 0.0f;
    // path.path[6] = nextPose;

    // nextPose.x = 0.61f*5;
    // nextPose.y = 0.61f*0;
    // nextPose.theta = 0.0f;
    // path.path[7] = nextPose;

    // nextPose.x = 0.61f*6;
    // nextPose.y = 0.61f*0;
    // nextPose.theta = 0.0f;
    // path.path[8] = nextPose;


    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
