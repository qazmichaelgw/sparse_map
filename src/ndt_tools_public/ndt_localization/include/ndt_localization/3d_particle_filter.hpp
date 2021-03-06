/**
 * This is an implementation for 3D (or 6D to be more precise) particle filter.
 * @author Jari Saarinen (jari.p.saarinen@gmail.com)
 */

#ifndef _Particle_filter_3D_h_
#define _Particle_filter_3D_h_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <Eigen/Core>
#include "ndt_localization/particle.hpp"
//#include "ndt_mcl/ownRandom.h"

namespace perception_oru{
  class particle_filter_3d{
  public:
    std::vector<particle> pcloud;	      ///< Particle distribution
    //ownRandom myrand;                 			///< The random number class

    particle_filter_3d(){}

    /**
     * Initializes the filter by sampling from normal distribution with
     * mean in @p0 and variance defined by @variance
     */
    void initializeNormalRandom(unsigned int NumParticles, double mx, double my, double mz, double mroll, double mpitch, double myaw,
                                double vx, double vy, double vz, double vroll, double vpitch, double vyaw);

    /**
     * SIR Update for the filter
     */
    void SIRUpdate();

    unsigned int size() const {return pcloud.size();}

    /**
     * Performs the normalization step
     * i.e. according to updated likelihoods the probability of each
     * particle is calculated and the whole distribution sums up to 1
     */
    void normalize();

    void predict(Eigen::Affine3d Tmotion, double vx, double vy, double vz, double vroll, double vpitch, double vyaw,const Eigen::Affine3d offset=Eigen::Affine3d::Identity());

    Eigen::Affine3d getMean();
    //Eigen::Matrix<double,7,7> getCov();

    /**
     * Helper to convert xyzrpy to eigen affine3d
     */
    inline Eigen::Affine3d xyzrpy2affine(double x, double y, double z, double roll, double pitch, double yaw){
      Eigen::Matrix3d m;
      m = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

      Eigen::Translation3d v(x,y,z);
      return (v*m);
    }
  private:
  };
}

#endif
