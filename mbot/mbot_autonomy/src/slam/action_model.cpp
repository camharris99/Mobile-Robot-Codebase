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
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
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

    // std::cout << "prevP.x: " << previousPose_.x << ", odometry.x: " << odometry.x << std::endl;
    // std::cout << "prevP.y: " << previousPose_.y << ", odometry.y: " << odometry.y << std::endl;
    // std::cout << "prevP.t: " << previousPose_.theta << ", odometry.t: " << odometry.theta << std::endl;

    if (!initialized_) {
        ActionModel::resetPrevious(odometry);
        initialized_ = true;
    }

    float deltaX = odometry.x - previousPose_.x;
    float deltaY = odometry.y - previousPose_.y;
    float deltaTheta = odometry.theta - previousPose_.theta;

    trans_ = std::sqrt(deltaX * deltaX + deltaY * deltaY);
    rot1_ = angle_diff(std::atan2(deltaY, deltaX), previousPose_.theta);
    float direction = 1.0;

    if (std::abs(rot1_) > M_PI/2.0) {
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(deltaTheta, rot1_);
    
    moved_ = (deltaX != 0.0) || (deltaY != 0.0) || (deltaTheta != 0.0);

    if (moved_) {
        rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        transStd_ = std::sqrt(k2_ * std::abs(trans_));
        rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
    }

    trans_ *= direction;
    ActionModel::resetPrevious(odometry);
    utime_ = odometry.utime;

    return moved_;

    // if (sqrt((previousPose_.x - odometry.x)*(previousPose_.x - odometry.x) - (previousPose_.y - odometry.y)*(previousPose_.y - odometry.y)) > min_dist_ || sqrt((previousPose_.y*previousPose_.y) - (odometry.y*odometry.y)) > min_dist_ || fabs(previousPose_.theta - odometry.theta) > min_theta_) {
    //     moved = 1;
    //     // std::cout << "robot moved" << std::endl;
    // } else {
    //     // std::cout << "robot not moved" << std::endl;
    //     return moved;
    // }

    // dx_ = odometry.x - previousPose_.x;
    // dy_ = odometry.y - previousPose_.y;
    // dtheta_ = odometry.theta - previousPose_.theta;

    // return moved_;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    //sample from normal distribution with previously calculated stdev and apply to particle

    // std::random_device rd;
    // std::mt19937 gen(rd());

    mbot_lcm_msgs::particle_t newSample = sample;

    float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampledTrans*cos(sample.pose.theta + sampledRot1);
    newSample.pose.y += sampledTrans*cos(sample.pose.theta + sampledRot1);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2) ;
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;


    // std::cout << "applying action" << std::endl;

    // if (!ActionModel::updateAction(newSample.pose)) {
    //     return newSample;
    // }

    // previousPose_ = sample.pose;
    // newSample.parent_pose = sample.pose;
    // newSample.pose.utime = sample.pose.utime;

    // float ds = sqrt(dx_*dx_ + dy_*dy_);
    // float alpha = atan2(dy_, dx_) - previousPose_.theta;

    // std::normal_distribution<float> e1_dist(0.0, k1_*fabs(alpha));
    // std::normal_distribution<float> e2_dist(0.0, k2_*fabs(ds));
    // std::normal_distribution<float> e3_dist(0.0, k1_*fabs(dtheta_ - alpha));

    // e1_ = e1_dist(gen);
    // e2_ = e2_dist(gen);
    // e3_ = e3_dist(gen);

    // newSample.pose.x = newSample.parent_pose.x + ((ds + e2_)*cos(newSample.parent_pose.theta + alpha + e1_));
    // newSample.pose.y = newSample.parent_pose.y + ((ds + e2_)*sin(newSample.parent_pose.theta + alpha + e1_));
    // newSample.pose.theta = newSample.parent_pose.theta + (dtheta_ + e1_ + e3_);

    // // std::cout << "rnd e1: " << e1_ << ", rnd e2: " <<  e2_ << ", rnd e3: " << e3_ << std::endl;
    // // std::cout << "cur x: " << sample.pose.x << ", cur y: " << sample.pose.y << ", cur t:" << sample.pose.theta << std::endl;
    // // std::cout << "new x: " << newSample.pose.x << ", new y: " << newSample.pose.y << ", new t:" << newSample.pose.theta << std::endl;

    // return newSample;
}
