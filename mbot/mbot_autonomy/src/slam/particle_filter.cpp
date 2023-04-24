#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <algorithm>

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
    double sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;

    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<> dist(0.0, 0.02);

    for (auto& p : posterior_) {
        p.pose.x = posteriorPose_.x + dist(generator);
        p.pose.y = posteriorPose_.y + dist(generator);
        p.pose.theta = wrap_to_pi(posteriorPose_.theta + dist(generator)); // wrap to pi?
        p.pose.utime = posteriorPose_.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    randomPoseGen.update_map(&map);
    double sampleWeight = 1.0 / kNumParticles_;
    std::cout << "initializeFilterRandomly"<< std::endl;
    posteriorPose_ = randomPoseGen.get_pose();
    
    for (auto &p : posterior_)
    {
        p = randomPoseGen.get_particle();
        p.weight = sampleWeight;
    }
    
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

    if (hasRobotMoved) {
        auto prior = resamplePosteriorDistribution(&map);
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // OPTIONAL TODO: Add reinvigoration step
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.

    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
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

    for (auto & part : prior) {
        proposal.push_back(actionModel_.applyAction(part));
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
    double sumWeights = 0.0;
    double minimum = 0.0001;

    for (auto& p : proposal) {
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(p, laser, map);
        if (weighted.weight < minimum) {
            weighted.weight = minimum;
        }
        sumWeights += weighted.weight;
        posterior.push_back(weighted);
    }

    // std::sort(posterior.begin(), posterior.end(), myobject);
    // posterior.erase(posterior.begin()+50, posterior.end());
    // std::cout << posterior.size() << std::endl;

    for (auto& p : posterior) {
        p.weight /= sumWeights;
        // std::cout << "p.weight: " << p.weight << std::endl;
    }

    // std::cout << std::endl;
 
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    ParticleList sorted_posterior = posterior;

    std::sort(sorted_posterior.begin(), sorted_posterior.end(), myobject);
    sorted_posterior.erase(sorted_posterior.begin()+25,sorted_posterior.end());
    pose = ParticleFilter::computeParticlesAverage(sorted_posterior);

    // pose.x = sorted_posterior.at(0).pose.x;
    // pose.y = sorted_posterior.at(0).pose.y;
    // pose.theta = sorted_posterior.at(0).pose.theta;

    // for (auto& p : posterior) {
    //     xMean += p.weight * p.pose.x;
    //     yMean += p.weight * p.pose.y;
    //     cosThetaMean += p.weight * std::cos(p.pose.theta);
    //     sinThetaMean += p.weight * std::sin(p.pose.theta);
    // }

    // pose.x = xMean;
    // pose.y = yMean;
    // pose.theta = std::atan2(sinThetaMean, cosThetaMean);

    // std::cout << "xmean: " << xMean << ", ymean: " << yMean << ", theta: " << pose.theta << std::endl;

    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    ParticleList particles = particles_to_average;

    double xMean = 0.0;
    double yMean = 0.0;
    double cosThetaMean = 0.0;
    double sinThetaMean = 0.0;
    double weightSum = 0.0;

    for (auto& p : particles) {
        xMean += p.weight * p.pose.x;
        yMean += p.weight * p.pose.y;
        cosThetaMean += p.weight * std::cos(p.pose.theta);
        sinThetaMean += p.weight * std::sin(p.pose.theta);
        weightSum += p.weight;
    }

    avg_pose.x = xMean/weightSum;
    avg_pose.y = yMean/weightSum;
    avg_pose.theta = std::atan2(sinThetaMean/weightSum, cosThetaMean/weightSum);

    return avg_pose;
}

