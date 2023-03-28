#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <random>

ActionModel::ActionModel(void)
    : k1_(0.01f), k2_(0.01f), min_dist_(0.0025), min_theta_(0.02), initialized_(false), dx_(0.0f), dy_(0.0f), dtheta_(0.0f)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
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
    printf("%4.2f, %4.2f, %4.2f\n", odometry.x - previousPose_.x,odometry.y - previousPose_.y,odometry.theta - previousPose_.theta);
    if (previousPose_.x != odometry.x || previousPose_.y != odometry.y || previousPose_.theta != odometry.theta)
    {
        moved = 1;
        dx_ = odometry.x - previousPose_.x;
        dy_ = odometry.y - previousPose_.y;
        dtheta_ = odometry.theta - previousPose_.theta;
    }
    else
    {
        moved = 0;
    }

    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t &sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    // sample from normal distribution with previously calculated stdev and apply to particle

    std::random_device rd;
    std::mt19937 gen(rd());

    mbot_lcm_msgs::particle_t newSample = sample;

    if (!updateAction(newSample.pose))
    {
        return newSample;
    }

    previousPose_ = sample.pose;
    newSample.parent_pose = sample.pose;
    // newSample.pose.utime = cur_pico_time;

    float ds = sqrt(dx_ * dx_ + dy_ * dy_);
    float alpha = atan2(dy_, dx_) - previousPose_.theta;

    std::normal_distribution<float> e1_dist(0.0, k1_ * fabs(alpha));
    std::normal_distribution<float> e2_dist(0.0, k2_ * fabs(ds));
    std::normal_distribution<float> e3_dist(0.0, k1_ * fabs(dtheta_ - alpha));

    float e1 = e1_dist(gen);
    float e2 = e2_dist(gen);
    float e3 = e3_dist(gen);
    printf("%4.2f, %4.2f, %4.2f\n", e1,e2,e3);
    newSample.pose.x = newSample.parent_pose.x + ((ds + e2) * cos(newSample.parent_pose.theta + alpha + e1));
    newSample.pose.y = newSample.parent_pose.y + ((ds + e2) * sin(newSample.parent_pose.theta + alpha + e1));
    newSample.pose.theta = newSample.parent_pose.theta + (dtheta_ + e1 + e3);

    return newSample;
}
