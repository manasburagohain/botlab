#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <chrono>
#include <time.h>
#include <math.h>

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

/*
void ParticleFilter::applyEpsilon(float epsilon) {
    particles_t prior = particles();


    for (int i = 0; i < prior.num_particles; i++) {
        particle_t particle = prior.particles[i];
        
    }
}*/

double ParticleFilter::my_rand(double min, double max) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(min,max);
    return distribution(generator);
}

void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    //for i in kNumParticles, initialize particle to start pose // no need to adjust by epsilon, done in action model
    for (int i = 0; i < kNumParticles_; i++) {
        //set particle poseto initial pose
        posterior_[i].pose = pose;
    }

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution(); // this just resamples. done
        auto proposal = computeProposalDistribution(prior); // this applies action model. done.
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // this uses likelihood sensor model and normalizes. 
        posteriorPose_ = estimatePosteriorPose(posterior_); // from all samples and weights, get pose estimate
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior = posterior_;
    std::vector<particle_t> new_particles;

    // Low variance resampling
    int M = kNumParticles_;
    double r = my_rand(0,1/M);
    float c = prior[0].weight;
    int i = 0; // is 1 in notes

    for (int m = 0; m < M; m++ ) {
        float U = r + m * (1.0/M);
        while (U > c) {
            i++;
            c += prior[i].weight;
        }
        new_particles.push_back(prior[i]);
    }

    return new_particles;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    // all we are doing is applying action model
    for ( auto particle : prior ) {
        auto new_particle = actionModel_.applyAction(particle);
        proposal.push_back(new_particle);
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;

    // Get weights
    double weight_sum = 0;
    for (auto particle : proposal) {
        double w = sensorModel_.likelihood(particle, laser, map);
        particle.weight = w;
        weight_sum += w;
        posterior.push_back(particle);
    }

    weight_sum /= kNumParticles_;

    // Normalize weights
    for (int i = 0; i < kNumParticles_; i++) {
        posterior[i].weight /= weight_sum;
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;

    double x_sum = 0;
    double y_sum = 0;
    double sin_sum = 0;
    double cos_sum = 0;

    for (auto particle : posterior) {
        double w = particle.weight;
        x_sum += w * particle.pose.x;
        y_sum += w * particle.pose.y;
        sin_sum += w * sin(particle.pose.theta);
        cos_sum += w * cos(particle.pose.theta);
    }

    pose.x = x_sum;
    pose.y = y_sum;
    pose.theta = atan2(sin_sum, cos_sum);

    return pose;
}
