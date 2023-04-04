#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>


ActionModel::ActionModel(void)
    : k1_(0.85f), k2_(0.1f), min_dist_(0.0025), min_theta_(0.002), initialized_(false), dx_(0.0f), dy_(0.0f), dtheta_(0.0f)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}

void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    previousPose_ = odometry;
}

bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t &odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    // check motion
    // calculate deltas
    // calculate standard deviations

    bool moved = 0;
    float direction = 1.0; // rotation direction
    
    if (!initialized_) {
        resetPrevious(odometry);
        initialized_ = true;
    }
    
    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);

    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);
    trans_ = std::sqrt(dx_ * dx_ + dy_ * dy_);

    // If the angle traveled is too big for this time step, then it means we went backwards
    if (std::abs(rot1_) > M_PI/2.0)
    {
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(dtheta_, rot1_);

    moved = (trans_ > min_dist_) || (fabs(dtheta_) > min_theta_);

    if (!moved) {
        rot1Std_ = 0;
        transStd_ = 0;
        rot2Std_ = 0;
    } else {
        rot1Std_ = abs(k1_ * rot1_);
        transStd_ = abs(k2_ * trans_);
        rot2Std_ = abs(k1_ * rot2_);
    }

    trans_ *= direction;
    utime_ = odometry.utime;
    previousPose_ = odometry;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t &sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    // sample from normal distribution with previously calculated stdev and apply to particle

    mbot_lcm_msgs::particle_t newSample = sample;

    std::normal_distribution<float> rot1_sample(rot1_, rot1Std_);
    std::normal_distribution<float> trans_sample(trans_, transStd_);
    std::normal_distribution<float> rot2_sample(rot2_, rot2Std_);

    double rot1_hat = rot1_sample(numberGenerator_);
    double trans_hat = trans_sample(numberGenerator_);
    double rot2_hat = rot2_sample(numberGenerator_);

    newSample.pose.x += trans_hat*std::cos(angle_sum(sample.pose.theta, rot1_hat));
    newSample.pose.y += trans_hat*std::sin(angle_sum(sample.pose.theta, rot1_hat));
    newSample.pose.theta = angle_sum(newSample.pose.theta, angle_sum(rot1_hat, rot2_hat));

    newSample.parent_pose = sample.pose;
    newSample.pose.utime = utime_;
    
    return newSample;
}
