#include <ndt_fuser/ndt_fuser_hmt.h>

#include <ndt_generic/eigen_utils.h>
#include <ndt_generic/pcl_utils.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_map/pointcloud_utils.h>
#include <tf_conversions/tf_eigen.h>
#include <cstdio>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <algorithm>
#include <boost/program_options.hpp>
#include "graph_map/graph_map_fuser.h"
#include "graph_map/ndt/ndt_map_param.h"
#include "graph_map/ndt/ndtd2d_reg_type.h"
#include "graph_map/ndt/ndt_map_type.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ros/publisher.h"
#include "tf/transform_broadcaster.h"
#include "ndt_generic/eigen_utils.h"
//#include "ndt_mcl/3d_ndt_mcl.h"
#include "ndt_localization/3d_ndt_mcl.hpp"
#include "ndt_generic/io.h"
#include "graph_localization/localization_factory.h"
#include "graph_localization/localization_type.h"
#include "graph_localization/reg_localization_type/reg_localization_type.h"
#include "graph_localization/mcl_ndt/mcl_ndt.h"
#include "graph_map/graph_map_navigator.h"
#include "graph_map/graphfactory.h"
#include "graph_map/reg_type.h"
#include "ndt_offline/readbagfilegeneric.h"
#include <ndt_offline/VelodyneBagReader.h>
#include "ndt_generic/motionmodels.h"
#include "ndt_generic/io.h"
#include "ndt_generic/sensors_utils.h"
#include "ndt_offline/readpointcloud.h"
using namespace perception_oru;
using namespace graph_localization;
using namespace graph_map;
namespace po = boost::program_options;
using namespace std;


std::string map_dir_name="";
std::string output_dir_name="";
std::string output_file_name="";
std::string base_name="";
std::string dataset="";
std::string map_switching_method="";
//map parameters
bool gt_localisation=false;
int itrs=0;
int attempts=1;
int nb_neighbours=0;
int nb_scan_msgs=0;
bool use_odometry_source=true;
bool use_gt_data=false;
bool visualize=true;
bool filter_fov=false;
bool filter_ring_nb=false;
bool step_control=false;
bool alive=false;
bool disable_reg=false, do_soft_constraints=false;
bool save_eval_results=false;
bool keyframe_update=false;
bool hold_start=false;
bool extrapolate_odometry=false;
string registration_type="ndt_d2d_reg";
perception_oru::MotionModel2d::Params motion_params;
std::string base_link_id="", gt_base_link_id="", tf_world_frame="", tf_fuser_frame="fuser";
std::string velodyne_config_file="";
std::string lidar_topic="";
std::string velodyne_frame_id="";
std::string map_file_path="";
std::string tf_topic="";
std::string localisation_type="";
tf::Transform Tsensor_offset_tf;
Eigen::Affine3d Tsensor_offset,fuser_pose;//Mapping from base frame to sensor frame
ros::NodeHandle *n_=NULL;
MapParamPtr mapParPtr=NULL;
GraphMapParamPtr graphParPtr=NULL;
ndt_generic::Vector6d init;
double sensor_time_offset=0;
double resolution_local_factor=0;
unsigned int n_particles=0;
double SIR_varP_threshold=0;
double max_range=0, min_range=0;
double maxRotationNorm_=0;
double interchange_radius_=0;
double maxTranslationNorm_=0;
double rotationRegistrationDelta_=0;
double sensorRange_=100;
double translationRegistrationDelta_=0;
double resolution=0;
double hori_min=0, hori_max=0;
double min_keyframe_dist=0, min_keyframe_rot_deg=0;
double z_filter_min_height=0;
double score_cell_weight=0;
double alpha_distance=0;
unsigned int skip_frame=20;
int attempt =1;
unsigned int nodeid=-1;
unsigned int n_obs_search=5;
double consistency_max_dist, consistency_max_rot;

ros::Publisher *gt_pub,*fuser_pub,*cloud_pub,*odom_pub;
nav_msgs::Odometry gt_pose_msg,fuser_pose_msg,odom_pose_msg;
pcl::PointCloud<pcl::PointXYZ>::Ptr msg_cloud;
LocalisationTypePtr localisation_type_ptr;
LocalisationParamPtr localisation_param_ptr;
GraphMapNavigatorPtr graph_map_;
ReadBagFileGeneric<pcl::PointXYZ> *reader;
/// Set up the sensor link
tf::Transform sensor_link; ///Link from /odom_base_link -> velodyne
std::string bagfilename;
std::string reader_type="velodyne_reader";
bool use_pointtype_xyzir;
int min_nb_points_for_gaussian;
bool forceSIR;
bool keep_min_nb_points;
bool min_nb_points_set_uniform;
plotmarker marker_=plotmarker::point;
template<class T> std::string toString (const T& x)
{
  std::ostringstream o;

  if (!(o << x))
    throw std::runtime_error ("::toString()");

  return o.str ();
}

template <typename PointT> void filter_ring_nb_fun(pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &cloud_nofilter, const std::set<int>& rings) {
  std::cerr << "Can only filter_ring_nb if they are of type velodyne_pointcloud::PointXYZIR" << std::endl;
  cloud = cloud_nofilter;
}

template <> void filter_ring_nb_fun<velodyne_pointcloud::PointXYZIR>(pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud,
                                                                     pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &cloud_nofilter,
                                                                     const std::set<int>& rings) {
  for(int i=0; i<cloud_nofilter.points.size(); ++i) {
    if (rings.find((int)cloud_nofilter[i].ring) != rings.end()) {
      cloud.points.push_back(cloud_nofilter.points[i]);
    }
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
}


std::string transformToEvalString(const Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> &T) {
  std::ostringstream stream;
  stream << std::setprecision(std::numeric_limits<double>::digits10);
  Eigen::Quaternion<double> tmp(T.rotation());
  stream << T.translation().transpose() << " " << tmp.x() << " " << tmp.y() << " " << tmp.z() << " " << tmp.w() << std::endl;
  return stream.str();
}

bool LocateMapFilePath(const std::string &folder_name,std::vector<std::string> &scanfiles){
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (folder_name.c_str())) != NULL) {
    while ((ent = readdir (dir)) != NULL) {
      if(ent->d_name[0] == '.') continue;
      char tmpcname[400];
      snprintf(tmpcname,399,"%s/%s",folder_name.c_str(),ent->d_name);
      std::string tmpfname = tmpcname;
      if(tmpfname.substr(tmpfname.find_last_of(".") + 1) == "MAP"|| tmpfname.substr(tmpfname.find_last_of(".") + 1) == "map") {
        scanfiles.push_back(tmpfname);
      }
    }
    closedir (dir);
  } else {
    std::cerr<<"Could not parse dir name\n";
    return false;
  }
  sort(scanfiles.begin(),scanfiles.end());
  {
    std::cout << "files to be loaded : " << std::endl;
    for (size_t i = 0; i < scanfiles.size(); i++) {
      std::cout << " " << scanfiles[i] << std::flush;
    }
    std::cout << std::endl;
  }
  return true;
}
void ReadAllParameters(po::options_description &desc,int &argc, char ***argv){

  Eigen::Vector3d transl;
  Eigen::Vector3d euler;
  // First of all, make sure to advertise all program options
  desc.add_options()
      ("help", "produce help message")
      ("map-file-path", po::value<std::string>(&map_file_path)->default_value(std::string("")), "file path to .MAP file containing graphMapNavigator")
      ("reader-type", po::value<std::string>(&reader_type)->default_value(std::string("velodyne_reader")), "Type of reader to use when open rosbag e.g. velodyne_reader (config file needed) or pcl_reader when opening pcl2 messages")
      ("bag-file-path", po::value<string>(&bagfilename)->default_value(""), "File path to rosbag to play with maps")
      ("attempts", po::value<int>(&attempts)->default_value(1), "Total retries of localisation, can be used to generate multiple files")
      ("visualize", "visualize the rosbag and fuser estimate/gt")
      ("save-results", "save trajectory for gt, estimation, sensor and odometry")
      ("base-name", po::value<string>(&base_name)->default_value(std::string("mcl")), "prefix for all generated files")
      ("output-dir-name", po::value<string>(&output_dir_name)->default_value(""), "where to save the pieces of the map (default it ./map)")
      ("data-set", po::value<string>(&dataset)->default_value(""), "where to save the pieces of the map (default it ./map)")
      ("localisation-algorithm-name", po::value<string>(&localisation_type)->default_value("reg_localisation_type"), "name of localisation algorihm e.g. mcl_ndt")
      ("map-switching-method", po::value<std::string>(&map_switching_method)->default_value(std::string("")), "Type of reader to use when open rosbag e.g. velodyne_reader (config file needed) or pcl_reader when opening pcl2 messages")
      ("filter-fov", "cutoff part of the field of view")
      ("hori-max", po::value<double>(&hori_max)->default_value(2*M_PI), "the maximum field of view angle horizontal")
      ("hori-min", po::value<double>(&hori_min)->default_value(-hori_max), "the minimum field of view angle horizontal")
      ("filter-ring-nb", "if the number of rings should be reduced")
      ("z-filter-height", po::value<double>(&z_filter_min_height)->default_value(-100000000), "The minimum height of which ndtcells are used for localisation")
      ("score-cell-weight", po::value<double>(&score_cell_weight)->default_value(0.1), "The constant score added to the likelihood by hitting a cell with a gaussian.")
      ("Dd", po::value<double>(&motion_params.Dd)->default_value(1.), "forward uncertainty on distance traveled")
      ("Dt", po::value<double>(&motion_params.Dt)->default_value(1.), "forward uncertainty on rotation")
      ("Cd", po::value<double>(&motion_params.Cd)->default_value(1.), "side uncertainty on distance traveled")
      ("Ct", po::value<double>(&motion_params.Ct)->default_value(1.), "side uncertainty on rotation")
      ("Td", po::value<double>(&motion_params.Td)->default_value(1.), "rotation uncertainty on distance traveled")
      ("Tt", po::value<double>(&motion_params.Tt)->default_value(1.), "rotation uncertainty on rotation")
      ("keyframe-min-distance", po::value<double>(&min_keyframe_dist)->default_value(0.1), "minimum distance traveled before adding cloud")
      ("keyframe-min-rot-deg", po::value<double>(&min_keyframe_rot_deg)->default_value(1), "minimum rotation before adding cloud")
      ("consistency-max-dist", po::value<double>(&consistency_max_dist)->default_value(0.6), "minimum distance traveled before adding cloud")
      ("consistency-max-rot", po::value<double>(&consistency_max_rot)->default_value(M_PI/2.0), "minimum rotation before adding cloud")
      ("tf-base-link", po::value<std::string>(&base_link_id)->default_value(std::string("")), "tf_base_link")
      ("tf-gt-link", po::value<std::string>(&gt_base_link_id)->default_value(std::string("")), "tf ground truth link")
      ("velodyne-config-file", po::value<std::string>(&velodyne_config_file)->default_value(std::string("../config/velo32.yaml")), "configuration file for the scanner")
      ("tf_world_frame", po::value<std::string>(&tf_world_frame)->default_value(std::string("/world")), "tf world frame")
      ("lidar-topic", po::value<std::string>(&lidar_topic)->default_value(std::string("/velodyne_packets")), "velodyne packets topic used")
      ("velodyne-frame-id", po::value<std::string>(&velodyne_frame_id)->default_value(std::string("/velodyne")), "frame_id of the velodyne")
      ("min-range", po::value<double>(&min_range)->default_value(0.6), "minimum range used from scanner")
      ("max-range", po::value<double>(&max_range)->default_value(130), "minimum range used from scanner")
      ("skip-frame", po::value<unsigned int>(&skip_frame)->default_value(20), "sframes to skip before plot map etc.")
      ("tf-topic", po::value<std::string>(&tf_topic)->default_value(std::string("/tf")), "tf topic to listen to")
      ("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("init-x", po::value<double>(&init[0])->default_value(0.00), "init-x")
      ("init-y", po::value<double>(&init[1])->default_value(0.0), "init-y")
      ("init-z", po::value<double>(&init[2])->default_value(0.0), "init-z")
      ("init-ex", po::value<double>(&init[3])->default_value(0.0), "init-ex")
      ("init-ey", po::value<double>(&init[4])->default_value(0.0), "init-ey")
      ("init-ez", po::value<double>(&init[5])->default_value(0.0), "init-ez")
      ("alpha-distance", po::value<double>(&alpha_distance)->default_value(0.), "sensor pose - euler angle vector z")
      ("sensor_time_offset", po::value<double>(&sensor_time_offset)->default_value(0.), "timeoffset of the scanner data")
      ("resolution", po::value<double>(&resolution)->default_value(0.4), "resolution of the map")
      ("n-particles", po::value<unsigned int>(&n_particles)->default_value(270), "Total number of particles to use")
      ("SIR_varP_threshold", po::value<double>(&SIR_varP_threshold)->default_value(0.6), "resampling threshold")
      ("forceSIR", "force Sample Importance Reasampling")
      ("resolution-local-factor", po::value<double>(&resolution_local_factor)->default_value(1.), "resolution factor of the local map used in the match and fusing step")
      ("use_pointtype_xyzir", "If the points to be processed should contain ring and intensity information (velodyne_pointcloud::PointXYZIR)")
      ("min_nb_points_for_gaussian", po::value<int>(&min_nb_points_for_gaussian)->default_value(6), "minimum number of points per cell to compute a gaussian")
      ("n-obs-search", po::value<unsigned int>(&n_obs_search)->default_value(5), "minimum number of points per cell to compute a gaussian")
      ("keep_min_nb_points", "If the number of points stored in a NDTCell should be cleared if the number is less than min_nb_points_for_gaussian")
      ("min_nb_points_set_uniform", "If the number of points of one cell is less than min_nb_points_for_gaussian, set the distribution to a uniform one (cov = Identity)")
      ("step-control", "Step thorugh frames using keyboard input")
      ("gt-localization", "Localisation estimate is set to previous GT each frame, Tmotion is set to TgtMotion")
      ("key-frame-update", "localisation update based on keyframes, update triggeded upon moving a distance from previoussly updated pose")
      ("hold-start", "localisation update based on keyframes, update triggeded upon moving a distance from previoussly updated pose")
      ("disable-localization", "No registration, resampling etc")
      ("no-odometry", "Do not make any odometry based predictions")
      ("reg", "use 3d localization type (not 2d assumption)")
      ("extrapolate-odometry", "Linear extrapolation of motion")
      ("multi-res","multiresaolution in registration")
      ("check-consistency","concistency check of registration result")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, *argv, desc), vm);
  po::notify(vm);
  use_gt_data=gt_base_link_id!="";
  keep_min_nb_points = vm.count("clear_min_nb_points");
  min_nb_points_set_uniform = vm.count("min_nb_points_set_uniform");
  NDTCell::setParameters(0.1, 8*M_PI/18., 1000, min_nb_points_for_gaussian, !keep_min_nb_points, min_nb_points_set_uniform);
  gt_localisation=vm.count("gt-localization");
  keyframe_update=vm.count("key-frame-update");
  disable_reg=vm.count("disable-registration");
  use_odometry_source=!vm.count("no-odometry");
  save_eval_results=vm.count("save-results");
  visualize = vm.count("visualize");
  step_control = vm.count("step-control");
  filter_fov = vm.count("filter-fov");
  filter_ring_nb = vm.count("filter-ring-nb");
  use_pointtype_xyzir = vm.count("use_pointtype_xyzir");
  hold_start=vm.count("hold-start");
  extrapolate_odometry=vm.count("extrapolate-odometry");

  //Check if all iputs are assigned
  if (!vm.count("map-dir-path") && !vm.count("map-file-path")){
    cout << "No .map file specified. Missing map-dir-path and map-file-path.\n";
    cout << desc << "\n";
    exit(0);
  }
  if (vm.count("help")){
    cout << desc << "\n";
    exit(0);
  }

  if( !ndt_generic::GetSensorPose(dataset,transl,euler,Tsensor_offset_tf,Tsensor_offset))
    cout << "no valid dataset specified, will use the provided sensor pose params" << endl;

  localisation_param_ptr=LocalisationFactory::CreateLocalisationParam(localisation_type);
  localisation_param_ptr->visualize=visualize;
  localisation_param_ptr->sensor_pose=Tsensor_offset;
  localisation_param_ptr->switch_map_method=GraphMapNavigatorParam::String2SwitchMethod(map_switching_method);
  localisation_param_ptr->n_obs_search=n_obs_search;
  localisation_param_ptr->enable_localisation=!vm.count("disable-localization");
  cout<<"will use method: "<<GraphMapNavigatorParam::SwitchMethod2String(localisation_param_ptr->switch_map_method)<<endl;
  if(MCLNDTParamPtr parPtr=boost::dynamic_pointer_cast<MCLNDTParam>(localisation_param_ptr )){
    parPtr->n_particles=n_particles;
    parPtr->z_filter_min=z_filter_min_height;
    parPtr->score_cell_weight=score_cell_weight;
    parPtr->SIR_varP_threshold=SIR_varP_threshold;
    parPtr->forceSIR=vm.count("forceSIR");
    GetMotionModel(dataset,parPtr->motion_model);
  }
  if(RegLocalisationParamPtr parPtr=boost::dynamic_pointer_cast<RegLocalisationParam>(localisation_param_ptr)){

    parPtr->keyframe_update=keyframe_update;
    parPtr->min_keyframe_dist=min_keyframe_dist;
    parPtr->min_keyframe_dist_rot_deg=min_keyframe_rot_deg;

    if(graph_map::RegParamPtr reg_par_ptr=GraphFactory::CreateRegParam(registration_type)){
      // if(NDTD2DRegParamPtr ndt_reg_par=boost::dynamic_pointer_cast<NDTD2DRegParam>(reg_par_ptr)){
      reg_par_ptr->sensor_pose=Tsensor_offset;
      reg_par_ptr->sensor_range=max_range;
      reg_par_ptr->enable_registration=!disable_reg;
      reg_par_ptr->check_consistency=vm.count("check-consistency");
      reg_par_ptr->max_translation_norm=consistency_max_dist;
      reg_par_ptr->max_rotation_norm=consistency_max_rot;
      reg_par_ptr->registration2d=!vm.count("locaization3d");
      parPtr->registration_parameters=reg_par_ptr;

      if(NDTD2DRegParamPtr ndt_par_ptr=boost::dynamic_pointer_cast<NDTD2DRegParam>(reg_par_ptr)){
        ndt_par_ptr->resolution_local_factor=resolution_local_factor;
        ndt_par_ptr->multires=vm.count("multi-res");
      }
    }
  }
  return;
}
void initializeRosPublishers(){
  gt_pub=new ros::Publisher();
  odom_pub=new ros::Publisher();
  fuser_pub=new ros::Publisher();
  cloud_pub=new ros::Publisher();
  *gt_pub    =n_->advertise<nav_msgs::Odometry>("/GT", 50);
  *fuser_pub =n_->advertise<nav_msgs::Odometry>("/fuser", 50);
  *odom_pub =n_->advertise<nav_msgs::Odometry>("/odom", 50);
  *cloud_pub = n_->advertise<pcl::PointCloud<pcl::PointXYZ>>("/points2", 1);
  cout<<"initialized publishers"<<endl;
}
void printParameters(){
  cout<<"Output directory: "<<output_dir_name<<endl;
  if(filter_fov)
    cout << "Filtering FOV of sensor to min/max "<<hori_min<<" "<<hori_max<<endl;
  else
    cout<<"No FOV filter."<<endl;

  if(reader_type.compare("velodyne_reader"));
  cout<<"Velodyne config path:"<<velodyne_config_file<<endl;

  cout<<"Bagfile: "<<bagfilename<<endl;
  cout<<"Lidar topic: "<<lidar_topic<<", lidar frame id: "<<velodyne_frame_id<<endl;
  cout<<"World frame: "<<tf_world_frame<<", tf topic"<<tf_topic<<endl;
}

template<typename PointT>
void processData() {
  ndt_generic::StepControl step_controller;
  Eigen::Affine3d Todom_base,odom_pose,Todom_base_prev, Todom_init; //Todom_base =current odometry pose, odom_pose=current aligned with gt, Todom_base_prev=previous pose, Todom_init= first odometry pose in dataset.
  Eigen::Affine3d Tgt_base, Tgt_base_prev, Tgt_init;//Tgt_base=current GT pose,Tgt_base_prev=previous GT pose, Tgt_init=first gt pose in dataset;
  cout<<"process data"<<endl;
  srand(time(NULL));
  tf::TransformBroadcaster br;
  gt_pose_msg.header.frame_id=tf_world_frame;
  fuser_pose_msg.header.frame_id=tf_world_frame;
  odom_pose_msg.header.frame_id=tf_world_frame;


  for(int attempt=1;attempt<=attempts;attempt++){
    std::cerr<<"progress "<<attempt<<"/"<<attempts;
    LoadGraphMap(map_file_path,graph_map_);
    localisation_param_ptr->graph_map_=graph_map_;
    localisation_type_ptr=LocalisationFactory::CreateLocalisationType(localisation_param_ptr);
    GraphPlot::PlotMap(graph_map_->GetCurrentNode()->GetMap(),-1,graph_map_->GetCurrentNodePose(),marker_);
    GraphPlot::PlotPoseGraph(graph_map_);

    if(graph_map_==NULL ||localisation_type_ptr==NULL){
      cout<<"problem opening map"<<endl;
      exit(0);
    }

    cout<<"-------------------------- Map and Localisation parameter ----------------------------"<<endl;
    cout<<localisation_type_ptr->ToString()<<endl;
    cout<<"--------------------------------------------------------"<<endl;
    
    if(localisation_type.compare("reg_localisation_type")==0)
      output_file_name = ndt_generic::removeExtension(map_file_path)+localisation_type+"_lkeyd="+toString(min_keyframe_dist)+"_lkeydeg="+toString(min_keyframe_rot_deg)+"_attempt="+toString(attempt);
    else
      output_file_name = ndt_generic::removeExtension(map_file_path)+localisation_type+"_npart="+toString(n_particles)+"_mpsu="+toString(min_nb_points_set_uniform)+"_mnpfg="+toString(min_nb_points_for_gaussian)+"_attempt="+toString(attempt);
    ndt_generic::CreateEvalFiles eval_files(output_dir_name,output_file_name,save_eval_results);
    int counter = 0;
    ndt_offline::readPointCloud reader(bagfilename, Tsensor_offset, ndt_offline::WHEEL_ODOM, lidar_topic, min_range, max_range, velodyne_config_file, 0, tf_topic, tf_world_frame, gt_base_link_id);

    printParameters();

    pcl::PointCloud<PointT> cloud;

    ros::Time t0,t1,t2,t3,t4,t5;
    t5=ros::Time::now();
    bool no_update=false;
    bool initialized=false;
    while(reader.readNextMeasurement(cloud)){
      counter ++;
      Tgt_base=Todom_base=Eigen::Affine3d::Identity();
      cout<<"process frame"<<endl;

      t0=ros::Time::now();
      if(!n_->ok())
        exit(0);

      if(filter_fov) {
        ndt_generic::filter_height_angle(cloud,hori_min,hori_max,-DBL_MAX, DBL_MAX);
      }
    /*
      if (filter_ring_nb) {
        std::set<int> rings;
        rings.insert(7);
        cloud_nofilter = cloud;
        cloud.clear();
        filter_ring_nb_fun(cloud, cloud_nofilter, rings);
      }
      */

      if (cloud.size() == 0) continue; // Check that we have something to work with depending on the FOV filter here...

      bool odom_valid=false, gt_valid=false;
      if(use_odometry_source)
        odom_valid=reader.GetOdomPose(reader.GetTimeOfLastCloud(), base_link_id, Todom_base);
      if(use_gt_data)
        gt_valid=reader.GetOdomPose(reader.GetTimeOfLastCloud(), gt_base_link_id, Tgt_base);

        if(!(use_gt_data && !gt_valid  ||  use_odometry_source && !odom_valid))//everything is valid
          cout<<"Odometry valid"<<endl;
        else{
          cout<<"skipping this frame"<<endl;
          continue;
        }

        if((!initialized)){
          Todom_init=Todom_base;
          Todom_base_prev = Todom_base;
          Tgt_init=Tgt_base;
          Tgt_base_prev = Tgt_base;
          cout<<"pose gt init: "<<Tgt_init.translation().transpose()<<endl;
          Vector6d variances;
          variances<<0.1,0.1,0.000000,0.0000000,0.0000000,0.001;
          localisation_type_ptr->InitializeLocalization(Tgt_base*ndt_generic::xyzrpyToAffine3d(init[0],init[1],init[2],init[3],init[4],init[5]),variances);
          counter ++;
          initialized=true;
          continue;
        }
        t1=ros::Time::now();
        perception_oru::transformPointCloudInPlace(Tsensor_offset, cloud);
        Eigen::Affine3d Tmotion=Eigen::Affine3d::Identity();
        if(gt_localisation){
          cout<<"gt localisation"<<endl;
          Tmotion = Tgt_base_prev.inverse()*Tgt_base;
         // localisation_type_ptr->SetPose(Tgt_base_prev);
        }
        else if(extrapolate_odometry){
          cout<<"extrapolate"<<endl;
          //Tmotion=localisation_type_ptr->GetVelocity();
        }
        else if(use_odometry_source)
          Tmotion = Todom_base_prev.inverse()*Todom_base;

        bool new_update=localisation_type_ptr->UpdateAndPredict(cloud,Tmotion);
        fuser_pose=localisation_type_ptr->GetPose();
        cout<<"fuser_pose="<<fuser_pose.translation().transpose()<<endl;
        t2=ros::Time::now();

        odom_pose=Tgt_init*Todom_init.inverse()*Todom_base;//Correct for initial odometry
        ros::Time tplot =ros::Time::now();
        if (visualize)
        {
          tf::Transform tf_fuser;
          tf::transformEigenToTF(fuser_pose, tf_fuser);
          br.sendTransform(tf::StampedTransform(tf_fuser,tplot, tf_world_frame,  tf_fuser_frame));
          if (tf_world_frame != "/world") {
            tf::Transform tf_none;
            tf_none.setIdentity();
            br.sendTransform(tf::StampedTransform(tf_none, tplot, "/world", tf_world_frame));
          }
          br.sendTransform(tf::StampedTransform(Tsensor_offset_tf,tplot, tf_fuser_frame, velodyne_frame_id));
        }

        if(visualize && counter % skip_frame==0){ // This is relatively cheap to plot, note also that this is given in the vehicle frame...
          cloud.header.frame_id=tf_fuser_frame;
          pcl_conversions::toPCL(tplot, cloud.header.stamp);
          cloud_pub->publish(cloud);
        }

        if(visualize){
          gt_pose_msg.header.stamp=tplot;
          fuser_pose_msg.header.stamp=gt_pose_msg.header.stamp;
          odom_pose_msg.header.stamp=gt_pose_msg.header.stamp;
          tf::poseEigenToMsg(Tgt_base, gt_pose_msg.pose.pose);
          tf::poseEigenToMsg(fuser_pose, fuser_pose_msg.pose.pose);
          tf::poseEigenToMsg(odom_pose, odom_pose_msg.pose.pose);
          odom_pub->publish(odom_pose_msg);
          gt_pub->publish(gt_pose_msg);
          fuser_pub->publish(fuser_pose_msg);
        }

        bool newnode=Node::DetectNewNode(nodeid,graph_map_->GetCurrentNode());
        if(visualize && (newnode ||counter%1000==300)){
          // GraphPlot::PlotMap(graph_map->GetCurrentNode()->GetMap(),-1,graph_map->GetCurrentNodePose(),marker_);
          GraphPlot::PlotPoseGraph(graph_map_);
          GraphPlot::PlotObservationVector(graph_map_);
          GraphPlot::PlotMap(graph_map_->GetCurrentNode()->GetMap(),-1,graph_map_->GetCurrentNodePose(),plotmarker::point);
        }

        t3=ros::Time::now();
        double diff = (fuser_pose.translation() - Tgt_base.translation()).norm();
        //cout<<"norm between estimated and actual pose="<<diff<<endl;

        Tgt_base_prev = Tgt_base;
        Todom_base_prev = Todom_base;
        cloud.clear();
        //eval_files.Write( reader.getTimeStampOfLastSensorMsg(),Tgt_base*sensor_offset,odom_pose*sensor_offset,fuser_pose*sensor_offset);
        eval_files.Write( reader.GetTimeOfLastCloud(),Tgt_base,odom_pose,fuser_pose,fuser_pose*Tsensor_offset);

        t4=ros::Time::now();
        cout<<"frame: "<<counter<<",  Time: "<<t4-t5<<", t upd: "<<t2-t1<<", t plot: "<<t3-t2<<endl;
        t5=ros::Time::now();
        if(step_control)
          step_controller.Step(counter);
        else
          cout<<"no step control"<<endl;

        localisation_type_ptr->visualize(step_controller.visualize);
      }
      if(save_eval_results)
        SaveObservationVector(output_file_name,graph_map_);

      eval_files.Close();
      graph_map_.reset();
      localisation_type_ptr.reset();
    }

  }


  /////////////////////////////////////////////////////////////////////////////////7
  /////////////////////////////////////////////////////////////////////////////////7
  /// *!!MAIN!!*
  /////////////////////////////////////////////////////////////////////////////////7
  /////////////////////////////////////////////////////////////////////////////////7
  ///

  int main(int argc, char **argv){



    ros::init(argc, argv, "graph_fuser3d_offline");
    po::options_description desc("Allowed options");
    n_=new ros::NodeHandle("~");
    ros::Time::init();
    initializeRosPublishers();
    ReadAllParameters(desc,argc,&argv);

    if (use_pointtype_xyzir)
      processData<velodyne_pointcloud::PointXYZIR>();
    else
      processData<pcl::PointXYZ>();

    cout<<"end of program"<<endl;

  }


