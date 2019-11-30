#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////

    alpha1 = 1; //.001;//.05; // were all .001
    alpha2 = 1;
    alpha3 = 1;
    alpha4 = 1;

    pre_odometry.x = 0;
    pre_odometry.y = 0;
    pre_odometry.theta = 0;

    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    var[0] = 0.0;
    var[1] = 0.0;
    var[2] = 0.0;

    delta_rot1 = 0;
    delta_trans = 0;
    delta_rot2 = 0;
    delta_rot1_hat = 0;
    delta_trans_hat = 0;
    delta_rot2_hat = 0;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    delta_rot1 = atan2(odometry.y - pre_odometry.y, odometry.x - pre_odometry.x) - pre_odometry.theta;
    delta_trans = sqrt((pre_odometry.x-odometry.x)*(pre_odometry.x-odometry.x) + (pre_odometry.y-odometry.y)*(pre_odometry.y-odometry.y));
    //delta_trans *= 10; // testing grid theory
    delta_rot2 = odometry.theta - pre_odometry.theta - delta_rot1;

    var[0] = alpha1*delta_rot1*delta_rot1 + alpha2*delta_trans*delta_trans;
    var[1] = alpha3*delta_trans*delta_trans + alpha4*delta_rot1*delta_rot1 + alpha4*delta_rot2*delta_rot2;
    var[2] = alpha1*delta_rot2*delta_rot2 + alpha2*delta_trans*delta_trans;

    std::default_random_engine generator;
    std::normal_distribution<float> p0(0.0,var[0]);
    std::normal_distribution<float> p1(0.0,var[1]);
    std::normal_distribution<float> p2(0.0,var[2]);


    p[0] = p0(generator);
    p[1] = p1(generator);
    p[2] = p2(generator);



    //return false;
    // check if previous odometry is close to new odometry
    float epsilon = .0001;
    float x_dist = (odometry.x - pre_odometry.x)*(odometry.x - pre_odometry.x);
    float y_dist = (odometry.y - pre_odometry.y)*(odometry.y - pre_odometry.y);
    float theta_dist = (odometry.theta - pre_odometry.theta)*(odometry.theta - pre_odometry.theta);
    float distance = sqrt( x_dist + y_dist + theta_dist );

    // set previous odometry values to current odometry values
    pre_odometry.x = odometry.x;
    pre_odometry.y = odometry.y;
    pre_odometry.theta = odometry.theta;

    if (distance > epsilon) {
        return true;
    }
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    particle_t new_sample;
    delta_rot1_hat = delta_rot1 - p[0];
    delta_trans_hat = delta_trans - p[1];
    delta_rot2_hat = delta_rot2 - p[2];

    new_sample.pose.x = sample.parent_pose.x + delta_trans_hat * cos(sample.parent_pose.theta + delta_rot1_hat);
    new_sample.pose.y = sample.parent_pose.y + delta_trans_hat * sin(sample.parent_pose.theta + delta_rot1_hat);
    new_sample.pose.theta = sample.parent_pose.theta + delta_rot1_hat + delta_rot2_hat;

    new_sample.parent_pose = sample.pose;

    //printf("Prev X: %f\n", sample.parent_pose.x);
    //printf("New X: %f\n", new_sample.pose.x);

    return new_sample;
}
