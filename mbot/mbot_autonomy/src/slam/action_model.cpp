#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
, dx_(0.0f)
, dy_(0.0f)
, dtheta_(0.0f)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    // check motion
    // calculate deltas
    // calculate standard deviations

    std::cout << "prevP.x: " << previousPose_.x << ", odometry.x: " << odometry.x << std::endl;

    bool moved = 0;

    if (previousPose_.x != odometry.x || previousPose_.y != odometry.y) {
        moved = 1;
        std::cout << "robot moved" << std::endl;
    } else {
        std::cout << "robot not moved" << std::endl;
        return moved;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = odometry.theta - previousPose_.theta;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    //sample from normal distribution with previously calculated stdev and apply to particle

    std::random_device rd;
    std::mt19937 gen(rd());

    mbot_lcm_msgs::particle_t newSample = sample;

    std::cout << "applying action" << std::endl;

    if (!ActionModel::updateAction(newSample.pose)) {
        return newSample;
    }

    previousPose_ = sample.pose;
    newSample.parent_pose = sample.pose;
    newSample.pose.utime = sample.pose.utime;

    float ds = sqrt(dx_*dx_ + dy_*dy_);
    float alpha = atan2(dy_, dx_) - previousPose_.theta;

    std::normal_distribution<float> e1_dist(0.0, k1_*fabs(alpha));
    std::normal_distribution<float> e2_dist(0.0, k2_*fabs(ds));
    std::normal_distribution<float> e3_dist(0.0, k1_*fabs(dtheta_ - alpha));

    e1_ = e1_dist(gen);
    e2_ = e2_dist(gen);
    e3_ = e3_dist(gen);

    newSample.pose.x = newSample.parent_pose.x + ((ds + e2_)*cos(newSample.parent_pose.theta + alpha + e1_));
    newSample.pose.y = newSample.parent_pose.y + ((ds + e2_)*sin(newSample.parent_pose.theta + alpha + e1_));
    newSample.pose.theta = newSample.parent_pose.theta + (dtheta_ + e1_ + e3_);

    std::cout << "rnd e1: " << e1_ << ", rnd e2: " <<  e2_ << ", rnd e3: " << e3_ << std::endl;
    std::cout << "cur x: " << sample.pose.x << ", cur y: " << sample.pose.y << ", cur t:" << sample.pose.theta << std::endl;
    std::cout << "new x: " << newSample.pose.x << ", new y: " << newSample.pose.y << ", new t:" << newSample.pose.theta << std::endl;

    return newSample;
}
