#ifndef PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_HPP
//C++ librarys
#include <vector>
#include <iostream>
#include <random>
#include <cmath>
#include <time.h>
//perception_oru
#include "ndt_localization/particle.hpp"
#include "ndt_map/ndt_map.h"
#include "ndt_localization/ndt_histogram_2D.hpp"
#include <pcl/common/transforms.h>
#include <chrono>
#include<Eigen/StdVector>
//
namespace perception_oru{
    // enum init_type{
    //   uniform_map,
    //   normal_guess,
    //   map_initial
    // };

    struct bucket{
        double c_x;
        double c_y;
        double c_th;
        double lik;
        double pro;
    };

    class particle_filter{


    public:
        // map publishing function
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        protected:

        NDTMap *ndtMap;
        bool be2D;
        bool forceSIR;
        //    init_type initializationType;
        int particleCount;
        double varLimit;
        int sirCount;
        int sinceSIR;

        double os;
        double opening;
        histogram_map *h_map;

        ///the scaling factor is here to simulate larger Gaussian when calculating the likelihood. Having a smaller number drives the likelihood up, and MCL is more tolerating of mistakes. How the other hand having a large scaling factor will have more accurate results, but MCL will be more prone to drifting. Being 1 is the same as having no scaling.
        double _motion_model_cov_x, _motion_model_cov_y, _motion_model_cov_yaw, _scaling_factor_gaussian;

        Eigen::Vector3d m_pose;
        std::vector<particle, Eigen::aligned_allocator<particle> > tmp;
        std::vector<double> weights;

        //     perception_oru::NDTMap* ndt_ISSMap;
        double sx,sy;
        void SIRUpdate();
        void Normalize();

        void Predict2D(double x, double y, double th, double sx, double sy, double sth);
        void to2PI(double &a);
        void toPI(double &a);
        void GetRandomPoint(perception_oru::NDTCell* cell,double &x, double &y, double &th);
    public:

        std::vector<particle, Eigen::aligned_allocator<particle> > particleCloud;

        particle_filter(std::string mapFile_, int particleCount_/*, init_type initializationType_*/);
        particle_filter(perception_oru::NDTMap *ndtMap_, int particleCount_/*, init_type initializationType_*/, bool be2D_=true, bool forceSIR_=true,double varLimit_=0, int sirCount_=0);
	
        /** Accessor **/
        void setMotionModelCovX(double x){_motion_model_cov_x = x;}
        void setMotionModelCovY(double y){_motion_model_cov_y = y;}
        void setMotionModelCovYaw(double yaw){_motion_model_cov_yaw = yaw;}
        void setScalingFactorGaussian(double scale){_scaling_factor_gaussian = scale;}
	
        double getMotionModelCovX() const {return _motion_model_cov_x;}
        double getMotionModelCovY() const {return _motion_model_cov_y;}
        double getMotionModelCovYaw() const {return _motion_model_cov_yaw;}
        double getScalingFactorGaussian() const {return _scaling_factor_gaussian;}
        /**************/
	
	
        void UpdateAndPredict(Eigen::Affine3d tMotion,perception_oru::NDTMap* ndtLocalMap_);
	
        /** @brief Update MCL -
         * @param[in] tMotion : motion between last scan and this one. If no motion, MCL is not updated.
         * @param[in] ndtLocalMap_ : The local map used to perform MCL onto the reference map. Attention, the local map will be transformed onto the particule position. It thus need to be centered on the sensor position, compared to the robot base frame but NOT compared to world.
         * @param[in] subsample_level : 1 all cell are used. 0 none
         * @param[in] z_cut : under z_cut on the z axis, gaussian are ignored.
         * @return the position of one of the particules. Used for debugging.
         */
        Eigen::Affine3d UpdateAndPredictEff(Eigen::Affine3d tMotion, perception_oru::NDTMap* ndtLocalMap_, double subsample_level, double z_cut);
        /** @brief same as before but using pointcloud **/
        Eigen::Affine3d UpdateAndPredictEff(Eigen::Affine3d tMotion, const pcl::PointCloud<pcl::PointXYZ>& transformed_cloud, double subsample_level, double z_cut);
        /** @brief same as before but using pointcloud not moved onto the sensor pose **/
        Eigen::Affine3d UpdateAndPredictEff(Eigen::Affine3d tMotion, const pcl::PointCloud<pcl::PointXYZ>& transformed_cloud, const Eigen::Affine3d& sensorpose, double subsample_level, double z_cut);
	
	
        void UpdateAndPredictEffRe(Eigen::Affine3d tMotion, perception_oru::NDTMap* ndtLocalMap_, double subsample_level, double z_cut, double x_var, double y_var, double th_var, double r_x_var, double r_y_var, double r_th_var, int tres);
        void UpdateAndPredict(Eigen::Affine3d tMotion,perception_oru::NDTMap ndtLocalMap_);
        void GetPoseMeanAndVariance2D(Eigen::Vector3d &mean, Eigen::Matrix3d &cov);
        Eigen::Vector3d GetMeanPose2D();
        void Reset();
        void EigenSort( Eigen::Vector3d &eigenvalues,Eigen::Matrix3d &eigenvectors );
        void InitializeFilter();
        void InitializeUniformMap();
        void InitializeNormal(double x, double y, double var);
        void InitializeNormal(double x, double y, double th, double var);
        void InitializeNormal(double x, double y, double z, double th, double var);

        std::vector<std::vector<double> > InitializeNDTH(perception_oru::NDTMap *localNdtMap);

        Eigen::Affine3d getAsAffine(float x, float y, float yaw);



        int SetupNDTH( double te_, int ns_, int np_, std::vector<double> begins_, std::vector<double> ends_, double range_, int os_, double opening_);

            perception_oru::NDTMap* getMap() const {return ndtMap;}
	
        ///@brief Return the cell size aka resolution of the reference ndt map.
        bool getReferenceMapCellSizes(double& cellx, double& celly, double& cellz) const {
            // 		std::cout << "getCell size " << std::endl;
            return ndtMap->getCellSizeInMeters(cellx, celly, cellz);
        }
	
    };
}
#endif
