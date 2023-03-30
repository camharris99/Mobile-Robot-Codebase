#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


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
    // initializing random seed
    std::random_device rd;
    // Mersenne twister PRNG, initialized with seed random device instance
    numberGenerator_ = std::mt19937(rd());

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        previousPose_ = odometry;
        initialized_ = true;
    }
    
    bool moved = 0;

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = angle_diff(odometry.theta,previousPose_.theta);
    
    //printf("%4.2f, %4.2f, %4.2f\n", dx_,dy_,dtheta_);


    moved = (dx_ != 0.0) || (dy_ != 0.0) || (dtheta_ != 0.0);

    if (moved)
    {
        
        alpha_ = angle_diff(atan2(dy_,dx_),previousPose_.theta);
        ds_ =  sqrt(dx_ * dx_ + dy_ * dy_);

        std::normal_distribution<float> e1_dist(0.0, k1_*fabs(alpha_));
        std::normal_distribution<float> e2_dist(0.0, k2_*fabs(ds_));
        std::normal_distribution<float> e3_dist(0.0, k1_*angle_diff_abs(dtheta_,alpha_));

        e1_ = e1_dist(numberGenerator_);
        e2_ = e2_dist(numberGenerator_);
        e3_ = e3_dist(numberGenerator_);
    }

    previousPose_ = odometry;
    utime_ = odometry.utime;

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;


    if (!ActionModel::updateAction(newSample.pose)) {
        return newSample;
    }

    newSample.parent_pose = sample.pose;
    newSample.pose.utime = utime_;

    newSample.pose.x = newSample.parent_pose.x + ((ds_ + e2_)*cos(newSample.parent_pose.theta + alpha_ + e1_));
    newSample.pose.y = newSample.parent_pose.y + ((ds_ + e2_)*sin(newSample.parent_pose.theta + alpha_ + e1_));
    newSample.pose.theta = wrap_to_pi(newSample.parent_pose.theta + (dtheta_ + e1_ + e3_));

    return newSample;
}
