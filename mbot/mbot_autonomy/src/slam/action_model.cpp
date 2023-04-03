#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>


ActionModel::ActionModel(void)
: k1_(0.8f)
, k2_(0.4f)
, min_dist_(0.0025)
, min_theta_(0.002)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousOdometry_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////

    // check motion
    // calculate deltas
    // calculate standard deviations

    // std::cout << "prevP.x: " << previousPose_.x << ", odometry.x: " << odometry.x << std::endl;
    // std::cout << "prevP.y: " << previousPose_.y << ", odometry.y: " << odometry.y << std::endl;
    // std::cout << "prevP.t: " << previousPose_.theta << ", odometry.t: " << odometry.theta << std::endl;

    if (!initialized_) {
        ActionModel::resetPrevious(odometry);
        initialized_ = true;
    }

    float deltaX = odometry.x - previousOdometry_.x;
    float deltaY = odometry.y - previousOdometry_.y;
    float deltaTheta = angle_diff(odometry.theta, previousOdometry_.theta);

    trans_ = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousOdometry_.theta);
    float direction = 1.0;

    if (std::abs(rot1_) > M_PI/2.0) {
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);
    // std::cout << "moved test: " << std::sqrt((deltaX*deltaX) + (deltaY*deltaY)) << std::endl;

    // std::cout << "std::abs(deltaTheta) : " << std::abs(deltaTheta) << ": min_theta_" << min_theta_ << std::endl;
    
    // moved_ = (std::sqrt((deltaX*deltaX) + (deltaY*deltaY)) > min_dist_) || (std::abs(deltaTheta) > min_theta_);

    moved_ = (deltaX > min_dist_) || (deltaY > min_dist_) || (std::abs(deltaTheta) > min_theta_);

    rot1Std_ = k1_ * std::fabs(rot1_);
    transStd_ = k2_ * std::fabs(trans_);
    rot2Std_ = k1_ * std::fabs(rot2_);

    trans_ *= direction;
    previousOdometry_ = odometry;
    utime_ = odometry.utime;

    // std::cout << "rot1std: " << rot1Std_ << std::endl;
    // std::cout << "transstd: " << transStd_ << std::endl;
    // std::cout << "rot2std: " << rot2Std_ << std::endl;

    return moved_;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    //sample from normal distribution with previously calculated stdev and apply to particle

    mbot_lcm_msgs::particle_t newSample = sample;

    float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampledTrans*std::cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans*std::sin(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2) ;
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
