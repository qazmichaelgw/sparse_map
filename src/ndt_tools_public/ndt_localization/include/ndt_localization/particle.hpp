#ifndef PARTICLE_HPP
#define PARTICLE_HPP
//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace perception_oru{
  class particle{
  

    double r_,p_,t_,x_,y_,z_;

  public:

      // map publishing function
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      Eigen::Affine3d pose;
    double likelihood;
    double probability;
    particle();
    particle(double roll,double pitch,double yaw,double x, double y, double z);
    particle(Eigen::Affine3d pose_);

    double GetLikelihood();
    double GetProbability();
    void GetXYZ(double &x, double &y, double &z);
    void GetRPY(double &r,double &p,double &y);

    void Set(double roll,double pitch,double yaw,double x, double y, double z);
    void SetLikelihood(double l_);
    void SetProbability(double p_);

    Eigen::Affine3d GetAsAffine();
  };
}
#endif
