#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <common_utils/geometric/angle_functions.hpp>


struct myclass {
  bool operator() (mbot_lcm_msgs::particle_t i, mbot_lcm_msgs::particle_t j) { return (i.weight>j.weight);}
} myobject;

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    double sampleweight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;
    for(auto &p : posterior_){
        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleweight;

    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(&map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    // OPTIONAL TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    //printf("%i/n",hasRobotMoved);
    if(hasRobotMoved)
    {
        //printf("moved");
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        //proposal = computeProposalDistribution(posterior_);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}



ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    // sample based on weights
    
    ParticleList prior;

    double M = kNumParticles_;
    double M_1 = 1.0/M;
    double r = (((double)rand())/(double)RAND_MAX)*M_1;
    ParticleList Xt;
    double c = posterior_.at(0).weight;
    int i = 0;

    for (int m = 0; m < M; m++) {
        double U = r + m * M_1;
        while (U > c) {
            i++;
            c += posterior_.at(i).weight;
        }
        prior.push_back(posterior_.at(i));
    }

    return prior;
}



ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for(auto pt: prior){
        proposal.push_back(actionModel_.applyAction(pt));
    }
    
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior;
    double sumWeight = 0;
    double minimum = 0.00001;
    for(auto& p :proposal){
        auto weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        if (weighted.weight < minimum) {
            weighted.weight = minimum;
        }

        sumWeight += weighted.weight;
        posterior.push_back(weighted);
    }
    for(auto& p: posterior){
        p.weight /= sumWeight;
    }
    //printf("%4.2f \n",sumWeight);
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    // double xMean = 0.0;
    // double yMean = 0.0;
    // double sinMean = 0.0;
    // double cosMean = 0.0;
    // for(auto& p:posterior){
    //     xMean += p.weight * p.pose.x;
    //     yMean += p.weight * p.pose.y;
    //     sinMean += p.weight * sin(p.pose.theta);
    //     cosMean += p.weight * cos(p.pose.theta);
    // }
    // pose.x = xMean;
    // pose.y = yMean;
    // pose.theta = atan2(sinMean,cosMean);
    //printf("new pose: %4.2f, %4.2f,%4.2f\n", xMean,yMean,pose.theta);
 
    ParticleList sorted_posterior = posterior;

    std::sort(sorted_posterior.begin(), sorted_posterior.end(), myobject);
    sorted_posterior.erase(sorted_posterior.begin()+200,sorted_posterior.end());
    printf("%4.2f, %4.2f, %4.2f, %4.2f  \n",sorted_posterior[0].weight,sorted_posterior[1].weight,sorted_posterior[2].weight,sorted_posterior[3].weight);
    pose = ParticleFilter::computeParticlesAverage(sorted_posterior);


    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    ParticleList particles = particles_to_average;

    int num_particles = particles_to_average.size();

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;

    for (auto& p : particles) {
        p.weight *= kNumParticles_/num_particles;
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
    }

    avg_pose.x = xMean;
    avg_pose.y = yMean;
    avg_pose.theta = std::atan2(sinThetaMean, cosThetaMean);



    return avg_pose;
}
