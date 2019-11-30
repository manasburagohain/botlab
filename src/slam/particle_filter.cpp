#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <chrono>
#include <time.h>
#include <math.h>

#include <common/grid_utils.hpp>

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

        /*
        Point<double> pose_m = Point<double>(posterior_[i].pose.x,posterior_[i].pose.y);
        Point<int> pose_grid = global_position_to_grid_cell(pose_m, map);
        */

        //set particle pose to initial pose
        posterior_[i].pose = pose;
        posterior_[i].weight = 1.0 / kNumParticles_;
    }

    printf("Initialized particle filter to pose\n");

}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    printf("Made it into update filter\n");
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    printf("Updated action model if robot moved\n");

    if(hasRobotMoved)
    {
        printf("checked if robot has moved\n");
        auto prior = resamplePosteriorDistribution(); // this just resamples. done
        printf("Resampled\n");
        auto proposal = computeProposalDistribution(prior); // this applies action model. done.
        printf("computed proposal\n");
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // this uses likelihood sensor model and normalizes. 
        printf("computed posterior\n");
        posteriorPose_ = estimatePosteriorPose(posterior_); // from all samples and weights, get pose estimate
        printf("estimated new pose\n");
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
    double c = prior[0].weight;
    int i = 0; // is 1 in notes

    for (int m = 0; m < M; m++ ) {
        double U = r + m * (1.0/M);
        //printf("U: %f", U);
        //printf("c: %f", c);
        while (U > c) {
            i++;
            //printf("%d\n",i);
            c += prior[i].weight;
            //printf("C: %f", c);
        }
        printf("%d\n",i);
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
    double weight_sum = 0.0;
    double epsilon = .000001;
    for (auto particle : proposal) {
        //double w = sensorModel_.likelihood(particle, laser, map);
        double w = 1;
        if (w < epsilon) w = epsilon;
        printf("Likelihood: %f\n", w);
        particle.weight = w;
        weight_sum += w;
        posterior.push_back(particle);
    }

    // Normalize weights
    for (int i = 0; i < kNumParticles_; i++) {
        posterior[i].weight = posterior[i].weight / weight_sum;
        printf("Particle x: %f", posterior_[i].pose.x );
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
