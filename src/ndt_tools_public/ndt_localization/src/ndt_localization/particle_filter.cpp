#include <Eigen/Eigenvalues>
#include "ndt_localization/particle_filter.hpp"
#include "ros/ros.h"

bool myfunction (double i,double j) { return (i>j); }

perception_oru::particle_filter::particle_filter(perception_oru::NDTMap *ndtMap_, int particleCount_ /*, init_type initializationType_*/, bool be2D_, bool forceSIR_, double varLimit_, int sirCount_) : be2D(be2D_), particleCount(particleCount_), forceSIR(forceSIR_), varLimit(varLimit_), sirCount(sirCount_), _motion_model_cov_x(0.5), _motion_model_cov_y(0.7), _motion_model_cov_yaw(0.15), _scaling_factor_gaussian(0.005){
	// 	be2D = be2D_;
	ndtMap = ndtMap_;
	// initializationType=initializationType_;
	// 	particleCount = particleCount_;
	// 	forceSIR = forceSIR_;
	// 	varLimit = varLimit_;
	// 	sirCount = sirCount_;
	weights.resize(particleCount_,0);
}



int perception_oru::particle_filter::SetupNDTH( double te_, int ns_, int np_, std::vector<double> begins_, std::vector<double> ends_, double range_, int os_, double opening_){

	os = os_;
	// ROS_INFO_STREAM(__FILE__<<":"<<__LINE__<<":"<<os);
	opening = opening_;
	//ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
	h_map = new histogram_map(ndtMap, te_, ns_, np_, begins_, ends_, range_, os_, opening_);                                                                                                                                                                                                                                                                  // does range has to be put explicitly? or can we get it from somwhere
	//ROS_INFO_STREAM(__FILE__<<","<<__LINE__);
	//	id = 0;
	//ROS_INFO_STREAM(__LINE__);
}


std::vector<std::vector<double> > perception_oru::particle_filter::InitializeNDTH(perception_oru::NDTMap *localNdtMap){
	Eigen::Vector2d orig;
	//ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
	tmp.resize(particleCount);
	std::vector<double> probability;
	double t_score;
	orig << 0.0, 0.0;
	histogram_cell local_histogram(localNdtMap, h_map->get_te(), h_map->get_ns(), h_map->get_np(), h_map->get_begins(), h_map->get_ends(), h_map->get_cs(), orig);
	h_map->score_map(local_histogram);
	//h_map->confusion_matrix();
	std::vector<std::vector<double> >hm = h_map->getHeatMap();
	//ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
	std::vector<perception_oru::histogram_cell*> a_cc = h_map->getScoredCells();
	std::vector<perception_oru::histogram_cell*> cc;
	std::vector<double> scores;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #ifdef VER_1
// 	//inverting and normalizing weights - version 1
// 	double score_sum = 0;
// 	std::cout << "raw_scores=[\n";
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		scores.push_back(a_cc[i]->cell_score);
// 		score_sum += a_cc[i]->cell_score;
// 		std::cout << a_cc[i]->cell_score << ";\n";
// 	}
// 	std::cout << "];\n probabilities=[\n";
// 	double mean = score_sum / a_cc.size();
// 	double probability_sum = 0;
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		probability.push_back(2 * mean - scores[i]);
// 		probability_sum += probability[i];
// 	}
// 	for(int i = 0; i < probability.size(); ++i){
// 		probability[i] /= probability_sum;
// 		std::cout << probability[i] << ";\n";
// 	}
// 	std::cout << "];\n";
// 	cc = a_cc;
// 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #endif //VER_1
// #ifdef VER_2
// 	///inverting and normalizing weights - version 2
// 	//invertin probabilities
// 	double score_sum = 0;
// 	std::cout << "raw_scores=[\n";
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		scores.push_back(a_cc[i]->cell_score);
// 		score_sum += a_cc[i]->cell_score;
// 		std::cout << a_cc[i]->cell_score << ";\n";
// 	}
// 	//////////////////////////////////////////////////////////////////////////////////////////////
// 	double median;
// 	std::vector<double> scores_to_sort = scores;
//
// 	size_t size = scores.size();
// 	sort(scores_to_sort.begin(), scores_to_sort.end());
// 	if(size  % 2 == 0)
// 		median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
// 	else
// 		median = scores_to_sort[size / 2];
// 	std::vector<double> filtered_scores;
// 	std::vector<perception_oru::histogram_cell*> filtered_cc;
// 	for(int i = 0; i < scores.size(); ++i){
// 		if(scores[i] < median){
// 			filtered_scores.push_back(scores[i]);
// 			filtered_cc.push_back(a_cc[i]);
// 		}
// 	}
// 	a_cc = filtered_cc;
// 	scores = filtered_scores;
// 	//////////////////////////////////////////////////////////////////////////////////////////////
// 	std::cout << "];\n probabilities=[\n";
// 	double mean = score_sum / a_cc.size();
// 	double probability_sum = 0;
// 	int min_id = 0;
// 	double min_val = std::numeric_limits<double>::max();
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		probability.push_back(mean - scores[i]);
// 		probability_sum += probability[i];
// 		if(probability[i] < min_val){
// 			min_val = probability[i];
// 			min_id = i;
// 		}
// 	}
// 	probability_sum -= min_val;
// 	std::vector<double> new_probability;
// 	int min_id_2 = 0;
// 	double min_val_2 = std::numeric_limits<double>::max();
// 	for(int i = 0; i < probability.size(); ++i){
// 		if(i != min_id){
// 			new_probability.push_back(probability[i] / probability_sum);
// 			cc.push_back(a_cc[i]);
// 			if(new_probability[i] < min_val){
// 				min_val_2 = new_probability[i];
// 				min_id_2 = i;
// 			}
// 		}
// 	}
// 	double new_probability_sum = 0;
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] -= min_val_2;
// 		new_probability_sum += new_probability[i];
// 	}
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] /= new_probability_sum;
// 		std::cout << new_probability[i] << "\n";
// 	}
// 	probability = new_probability;
// //find and remove minimum one
//
// 	std::cout << "];\n";
// #endif //VER_2
// #ifdef VER_3
// 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 	///inverting and normalizing weights - version 3
// 	//invertin probabilities
// 	double score_sum = 0;
// 	std::cout << "raw_scores=[\n";
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		scores.push_back(a_cc[i]->cell_score);
// 		score_sum += a_cc[i]->cell_score;
// 		std::cout << a_cc[i]->cell_score << ";\n";
// 	}
// 	//////////////////////////////////////////////////////////////////////////////////////////////
// 	double median;
// 	std::vector<double> scores_to_sort = scores;
//
// 	size_t size = scores.size();
// 	sort(scores_to_sort.begin(), scores_to_sort.end());
// 	if(size  % 2 == 0)
// 		median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
// 	else
// 		median = scores_to_sort[size / 2];
// 	std::vector<double> filtered_scores;
// 	std::vector<perception_oru::histogram_cell*> filtered_cc;
// 	for(int i = 0; i < scores.size(); ++i){
// 		if(scores[i] < median){
// 			filtered_scores.push_back(scores[i]);
// 			filtered_cc.push_back(a_cc[i]);
// 		}
// 	}
// 	a_cc = filtered_cc;
// 	scores = filtered_scores;
// 	//////////////////////////////////////////////////////////////////////////////////////////////
//
// 	std::cout << "];\n probabilities=[\n";
// 	double mean = score_sum / a_cc.size();
// 	double probability_sum = 0;
// 	int min_id = 0;
// 	double min_val = std::numeric_limits<double>::max();
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		probability.push_back(mean - scores[i]);
// 		probability_sum += probability[i];
// 		if(probability[i] < min_val){
// 			min_val = probability[i];
// 			min_id = i;
// 		}
// 	}
// 	probability_sum -= min_val;
// 	std::vector<double> new_probability;
// 	int min_id_2 = 0;
// 	double min_val_2 = std::numeric_limits<double>::max();
// 	for(int i = 0; i < probability.size(); ++i){
// 		if(i != min_id){
// 			new_probability.push_back(probability[i] / probability_sum);
// 			cc.push_back(a_cc[i]);
// 			if(new_probability[i] < min_val){
// 				min_val_2 = new_probability[i];
// 				min_id_2 = i;
// 			}
// 		}
// 	}
// 	double new_probability_sum = 0;
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] -= min_val_2;
// 		new_probability_sum += new_probability[i];
// 	}
// 	for(int i = 0; i < new_probability.size(); ++i)
// 		new_probability[i] /= new_probability_sum;
// 	new_probability_sum = 0;
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] = new_probability[i] * new_probability[i];
// 		new_probability_sum += new_probability[i];
// 	}
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] /= new_probability_sum;
// 		std::cout << new_probability[i] << "\n";
// 	}
// 	probability = new_probability;
// 	//find and remove minimum one
//
// 	std::cout << "];\n";
// #endif //VER_3
// #ifdef VER_4
// 	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 	///inverting and normalizing weights - version 4
// 	//invertin probabilities
// 	double score_sum = 0;
// 	std::cout << "raw_scores=[\n";
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		scores.push_back(a_cc[i]->cell_score);
// 		score_sum += a_cc[i]->cell_score;
// 		std::cout << a_cc[i]->cell_score << ";\n";
// 	}
//
// 	//////////////////////////////////////////////////////////////////////////////////////////////
// 	double median;
// 	std::vector<double> scores_to_sort = scores;
// 	std::vector<double> scores_sorted_filtered;
// 	size_t size = scores.size();
// 	sort(scores_to_sort.begin(), scores_to_sort.end());
// 	if(size  % 2 == 0)
// 		median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
// 	else
// 		median = scores_to_sort[size / 2];
// 	std::vector<double> filtered_scores;
// 	std::vector<perception_oru::histogram_cell*> filtered_cc;
// 	for(int i = 0; i < scores.size(); ++i){
// 		if(scores[i] < median){
// 			filtered_scores.push_back(scores[i]);
// 			filtered_cc.push_back(a_cc[i]);
// 		}
// 	}
//
// 	a_cc = filtered_cc;
// 	scores = filtered_scores;
// 	//////////////////////////////////////////////////////////////////////////////////////////////
//
// 	std::cout << "];\n probabilities=[\n";
// 	double mean = score_sum / a_cc.size();
// 	double probability_sum = 0;
// 	int min_id = 0;
// 	double min_val = std::numeric_limits<double>::max();
// 	for(int i = 0; i < a_cc.size(); ++i){
// 		probability.push_back(mean - scores[i]);
// 		probability_sum += probability[i];
// 		if(probability[i] < min_val){
// 			min_val = probability[i];
// 			min_id = i;
// 		}
// 	}
// 	probability_sum -= min_val;
// 	std::vector<double> new_probability;
// 	int min_id_2 = 0;
// 	double min_val_2 = std::numeric_limits<double>::max();
// 	for(int i = 0; i < probability.size(); ++i){
// 		if(i != min_id){
// 			new_probability.push_back(probability[i] / probability_sum);
// 			cc.push_back(a_cc[i]);
// 			if(new_probability[i] < min_val){
// 				min_val_2 = new_probability[i];
// 				min_id_2 = i;
// 			}
// 		}
// 	}
// 	double new_probability_sum = 0;
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] -= min_val_2;
// 		new_probability_sum += new_probability[i];
// 	}
// 	for(int i = 0; i < new_probability.size(); ++i)
// 		new_probability[i] /= new_probability_sum;
// 	new_probability_sum = 0;
// ///////////////////////////////////////////////////////////////////////////////////////////
// 	std::vector<double> probabilities_cumulative;
// 	std::vector<double> probabilities_sorted = new_probability;
// 	sort(probabilities_sorted.begin(), probabilities_sorted.end());
// 	probabilities_cumulative.push_back(probabilities_sorted[0]);
// 	for(int i = 1; i < probabilities_sorted.size(); ++i)
// 		probabilities_cumulative.push_back(probabilities_cumulative[i - 1] + probabilities_sorted[i]);
// 	for(int i = 0; i < probabilities_cumulative.size(); ++i)
// 		probabilities_cumulative[i] = probabilities_cumulative[i] * probabilities_cumulative[i];
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		for(int j = 0; j < probabilities_sorted.size(); ++j){
// 			if(new_probability[i] == probabilities_sorted[j]){
// 				new_probability[i] = probabilities_cumulative[j];
// 				new_probability_sum += new_probability[i];
// 				break;
// 			}
// 		}
// 	}
//
// ////////////////////////////////////////////////////////////////////////////////////////////
// 	for(int i = 0; i < new_probability.size(); ++i){
// 		new_probability[i] /= new_probability_sum;
// 		std::cout << new_probability[i] << "\n";
// 	}
// 	probability = new_probability;
// 	//find and remove minimum one
// 	std::cout << "];\n";
// #endif // VER_4
//#ifdef VER_5
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///inverting and normalizing weights - version 3
	//invertin probabilities
	double score_sum = 0;
	std::cout << "raw_scores=[\n";
	for(int i = 0; i < a_cc.size(); ++i){
		scores.push_back(a_cc[i]->cell_score);
		//score_sum += a_cc[i]->cell_score;
		std::cout << a_cc[i]->cell_score << ";\n";
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	double median;
	std::vector<double> scores_to_sort = scores;

	size_t size = scores.size();
	sort(scores_to_sort.begin(), scores_to_sort.end());
	double fraction = 0.1;     //top percent
	int it = floor(double(size) * fraction);
	std::cout << size << "\n";
	std::cout << it << "\n";
	double tr = scores_to_sort[it];
	std::cout << tr << "\n";
	// if(size  % 2 == 0)
	//  median = (scores_to_sort[size / 2 - 1] + scores_to_sort[size / 2]) / 2;
	// else
	//  median = scores_to_sort[size / 2];
	std::vector<double> filtered_scores;
	std::vector<perception_oru::histogram_cell*> filtered_cc;
	for(int i = 0; i < scores.size(); ++i){
		if(scores[i] < tr){
			score_sum += scores[i];
			filtered_scores.push_back(scores[i]);
			filtered_cc.push_back(a_cc[i]);
		}
	}
	cc.swap(filtered_cc);
	scores = filtered_scores;
	std::cout << scores.size() << "\n";
	//////////////////////////////////////////////////////////////////////////////////////////////

	std::cout << "];\n probabilities=[\n";
	// double mean = score_sum / a_cc.size();
	// double probability_sum = 0;
	// int min_id = 0;
	// double min_val = std::numeric_limits<double>::max();
	// for(int i = 0; i < filtered_cc.size(); ++i){
	// 	probability.push_back(mean - scores[i]);
	// 	probability_sum += probability[i];
	// 	if(probability[i] < min_val){
	// 		min_val = probability[i];
	// 		min_id = i;
	// 	}
	// }
	// probability_sum -= min_val;
	// std::vector<double> new_probability;
	// int min_id_2 = 0;
	// double min_val_2 = std::numeric_limits<double>::max();
	// for(int i = 0; i < probability.size(); ++i){
	// 	if(i != min_id){
	// 		new_probability.push_back(probability[i] / probability_sum);
	// 		cc.push_back(filtered_cc[i]);
	// 		if(new_probability[i] < min_val){
	// 			min_val_2 = new_probability[i];
	// 			min_id_2 = i;
	// 		}
	// 	}
	// }
	// double new_probability_sum = 0;
	// for(int i = 0; i < new_probability.size(); ++i){
	// 	new_probability[i] -= min_val_2;
	// 	new_probability_sum += new_probability[i];
	// }
	// for(int i = 0; i < new_probability.size(); ++i)
	// 	new_probability[i] /= new_probability_sum;
	//
	// new_probability_sum = 0;
	// for(int i = 0; i < new_probability.size(); ++i){
	// 	new_probability[i] = new_probability[i] * new_probability[i];
	// 	new_probability_sum += new_probability[i];
	// 	std::cout << new_probability[i] << " "<< new_probability_sum << "\n";
	// }
	for(int i = 0; i < scores.size(); ++i){
		probability.push_back(scores[i]/score_sum);
		std::cout << probability[i] << " "<< scores[i]<<" "<<score_sum << "\n";
	}
	//probability = new_probability;
	//find and remove minimum one

	std::cout << "];\n";
//#endif //VER_5
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// sample from the distribution
	std::mt19937 generator(std::chrono::system_clock::now().time_since_epoch().count());
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	std::vector<double> rand_points;
	for(int i = 0; i < particleCount; ++i)
		rand_points.push_back(distribution(generator));
	std::sort(rand_points.begin(), rand_points.end());
	int i = 0;
	int j = 0;
	std::vector<int> counter(probability.size(), 0);
	double current_threshold = probability[j];
	while(i < rand_points.size()){
		if(rand_points[i] < current_threshold){
			counter[j] = counter[j] + 1;
			i++;
		}else {
			j++;
			current_threshold += probability[j];
		}
	}
// std::cout << i<<" "<<counter.size() << "\n";
	// std::cout << "high_variance=[" << "\n";
	// for(int i = 0; i < counter.size(); ++i){
	// 	std::cout << counter[i] << ";\n";
	// 	for(int j = 0; j < counter[i]; ++j){
	// 		double th, x, y;
	// 		cc[i]->getSeed(x, y, th);
	// 		// std::cout << th << "\n";
	// 		std::uniform_real_distribution<double> distribution_x(x - h_map->get_cs() / 2.0, x + h_map->get_cs() / 2.0);
	// 		std::uniform_real_distribution<double> distribution_y(y - h_map->get_cs() / 2.0, y + h_map->get_cs() / 2.0);
	// 		std::uniform_real_distribution<double> distribution_th(th - 5 / 180 * M_PI, th + 5 / 180 * M_PI);
	// 		double r_th = distribution_th(generator);
	// 		double r_x = distribution_x(generator);
	// 		double r_y = distribution_y(generator);
	// 		// std::cout <<x - h_map->get_cs() / 2.0<<" "<< x + h_map->get_cs() / 2.0<<" "<< h_map->get_cs() << "\n";
	// 		//std::cout << "p=" << r_x << " " << r_y << " " << r_th << "\n";
	// 		to2PI(r_th);
	// 		//particleCloud.emplace_back(0.0, 0.0, r_th, r_x, r_y, 0.0);
	// 	}
	// }
	// std::cout << "];" << "\n";
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
// std::cout << cc[i]->cell_score << " " << probability[i] << "\n";
	std::uniform_real_distribution<double> distribution_l(0.0, 1.0 / (double)particleCount);
	double r = distribution_l(generator);
// //std::cout << r << "\n";
	double c = probability[0];
// int
	i = 0;
// //	std::default_random_engine generator;
	// std::cout << "low_variance=[" << "\n";
	for(int m = 0; m < particleCount; m++){
		double U = r + double(m) / double(particleCount);
		while(U > c){
			if(i > probability.size())
				i = 0;
			else
				i++;
			c = c + probability[i];
		}
		//  //draw a point from i cell and add to set of partciles
		double th, x, y;
		// std::cout << i << ";\n";
		cc[i]->getSeed(x, y, th);
		//  // std::cout << th << "\n";
		std::uniform_real_distribution<double> distribution_x(x - h_map->get_cs() / 2.0, x + h_map->get_cs() / 2.0);
		std::uniform_real_distribution<double> distribution_y(y - h_map->get_cs() / 2.0, y + h_map->get_cs() / 2.0);
		std::uniform_real_distribution<double> distribution_th(th - ((M_PI / 180.0) * (360.0 / double(h_map->get_os()))) / 2.0, th + ((M_PI / 180.0) * (360.0 / double(h_map->get_os()))) / 2.0);
		double r_th = distribution_th(generator);
		double r_x = distribution_x(generator);
		double r_y = distribution_y(generator);
		// std::cout <<x - h_map->get_cs() / 2.0<<" "<< x + h_map->get_cs() / 2.0<<" "<< h_map->get_cs() << "\n";
		// std::cout<<"p=" << r_x << " " << r_y << " " << r_th << "\n";
		to2PI(th);
		//particleCloud.emplace_back(0, 0, r_th, r_x, r_y, 0,1.0/double(particleCount),1.0/double(particleCount));
		particleCloud.emplace_back(0, 0, r_th, r_x, r_y, 0);
	}
	// std::cout << "];\n";
	// ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
	// for (int i = 0; i < cc.size(); ++i)
	// {
	// 	ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
	// 	if(cc[i]!=NULL)
	// 		cc[i]=NULL;
	// }
	// cc.clear();
	// ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
	// for (int i = 0; i < a_cc.size(); ++i)
	// {
	// 	ROS_INFO_STREAM(__FILE__ << ":" << __LINE__<<":"<<i);
	// 	if(a_cc[i]!=NULL){
	// 		a_cc[i]=NULL;
	// 	}
	// }
	// a_cc.clear();
	// for (int i = 0; i < filtered_cc.size(); ++i)
	// {
	// 	ROS_INFO_STREAM(__FILE__ << ":" << __LINE__<<":"<<i);
	// 	if(filtered_cc[i]!=NULL)
	// 		filtered_cc[i]=NULL;
	// }
	// filtered_cc.clear();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
//h_map->reset();
//ROS_INFO_STREAM(__FILE__ << ":" << __LINE__);
	return hm;
}







void perception_oru::particle_filter::Reset(){
	tmp.clear();
	// 	delete ndt_ISSMap;
	particleCloud.clear();
}

void perception_oru::particle_filter::InitializeNormal(double x, double y, double th, double var){

	if(particleCloud.size() > 0){
		particleCloud.clear();
		tmp.clear();
	}
	m_pose<<x,y,th;
	tmp.resize(particleCount);
	std::default_random_engine generator;

	for(int parNo = 0; parNo < particleCount; parNo++){
		std::normal_distribution<double> distribution_x(x, var);
		std::normal_distribution<double> distribution_y(y, var);
		std::normal_distribution<double> distribution_t(th, th * var);
		double xx, yy, yawyaw;
		yawyaw = distribution_t(generator);
		xx = distribution_x(generator);
		yy = distribution_y(generator);
		// 		std::cout << "Particule at : " << xx << " " << yy << " " << yawyaw << std::endl;
		particleCloud.emplace_back(0.0, 0.0, yawyaw, xx, yy , 0.0);
	}
}

void perception_oru::particle_filter::InitializeNormal(double x, double y, double z, double th, double var){

	if(particleCloud.size() > 0){
		particleCloud.clear();
		tmp.clear();
	}
	m_pose<<x,y,th;
	tmp.resize(particleCount);
	std::default_random_engine generator;

	for(int parNo = 0; parNo < particleCount; parNo++){
		std::normal_distribution<double> distribution_x(x, var);
		std::normal_distribution<double> distribution_y(y, var);
		// 		std::normal_distribution<double> distribution_z(z, var);
		std::normal_distribution<double> distribution_t(th, th * var);
		double xx, yy, zz, yawyaw;
		yawyaw = distribution_t(generator);
		xx = distribution_x(generator);
		yy = distribution_y(generator);
		// 		zz = distribution_z(generator);
		// 		std::cout << "Particule at : " << xx << " " << yy << " " << yawyaw << std::endl;
		particleCloud.emplace_back(0.0, 0.0, yawyaw, xx, yy , z);
	}
}

void perception_oru::particle_filter::InitializeNormal(double x, double y, double var){

	if(particleCloud.size() > 0){
		particleCloud.clear();
		tmp.clear();
	}
	m_pose<<x,y,0;
	tmp.resize(particleCount);
	std::default_random_engine generator;
	for(int parNo = 0; parNo < particleCount; parNo++){
		std::normal_distribution<double> distribution_x(x, var);
		std::normal_distribution<double> distribution_y(y, var);
		std::uniform_real_distribution<double> distribution_t(0, 2 * 3.1415);
		particleCloud.emplace_back(0.0, 0.0, distribution_t(generator), distribution_x(generator), distribution_y(generator), 0.0);
	}
}


Eigen::Vector3d perception_oru::particle_filter::GetMeanPose2D(){
	double sumX = 0, sumY = 0;
	Eigen::Vector3d pos;
	double sumW = 0;
	double ax = 0, ay = 0;

	for(int i = 0; i < particleCloud.size(); i++){
		double x, y, z, r, p, t;
		particleCloud[i].GetXYZ(x, y, z);
		particleCloud[i].GetRPY(r, p, t);
		sumX += particleCloud[i].GetProbability() * x;
		sumY += particleCloud[i].GetProbability() * y;
		ax += particleCloud[i].GetProbability() * cos(t);
		ay += particleCloud[i].GetProbability() * sin(t);
		sumW += particleCloud[i].GetProbability();
	}
	pos << sumX, sumY, atan2(ay, ax);
	return pos;
}


void perception_oru::particle_filter::GetPoseMeanAndVariance2D(Eigen::Vector3d &mean, Eigen::Matrix3d &cov){
	double sumX = 0, sumY = 0;
	double sumW = 0;
	double ax = 0, ay = 0;

	// 	std::cout << "Particle size " << particleCloud.size() << std::endl;
	assert(particleCloud.size() > 0);
	
	for(int i = 0; i < particleCloud.size(); i++){
		// 		std::cout << "sumX : " << sumX << *" with proba " << particleCloud[i].GetProbability();
		double x, y, z, r, p, t;
		particleCloud[i].GetXYZ(x, y, z);
		// 		std::cout << " and pose " << x << " " << y << " " << z << std::endl;
		particleCloud[i].GetRPY(r, p, t);
		sumX += particleCloud[i].GetProbability() * x;
		sumY += particleCloud[i].GetProbability() * y;
		ax += particleCloud[i].GetProbability() * cos(t);
		ay += particleCloud[i].GetProbability() * sin(t);
		sumW += particleCloud[i].GetProbability();
	}
	mean << sumX, sumY, atan2(ay, ax);

	double xx = 0, yy = 0, xy = 0, aax = 0, aay = 0;
	double cax = cos(atan2(ay, ax));
	double say = sin(atan2(ay, ax));
	double w2 = 0;


	for(int i = 0; i < particleCloud.size(); i++){
		double x, y, z, r, p, t;
		particleCloud[i].GetXYZ(x, y, z);
		particleCloud[i].GetRPY(r, p, t);
		xx += particleCloud[i].GetProbability() * (x - sumX) * (x - sumX);
		yy += particleCloud[i].GetProbability() * (y - sumY) * (y - sumY);
		xy += particleCloud[i].GetProbability() * (x - sumX) * (y - sumY);
		aax += particleCloud[i].GetProbability() * (cos(t) - cax) * (cos(t) - cax);
		aay += particleCloud[i].GetProbability() * (sin(t) - say) * (sin(t) - say);
		w2 += particleCloud[i].GetProbability() * particleCloud[i].GetProbability();
	}

	if(w2 == 1.0){
		fprintf(stderr, "CParticleFilter::getDistributionVariances -- w2=%lf Should not happen!\n", w2);
		w2 = 0.99;
	}
	double wc = 1.0 / (1.0 - w2);
	cov << wc * xx, wc * xy, 0,
		wc * xy, wc * yy, 0,
		0, 0, atan2(wc * aay, wc * aax);
}

void perception_oru::particle_filter::InitializeUniformMap(){
	int pCount_ = particleCount;

	std::vector<perception_oru::NDTCell*> allCells = ndtMap->getAllInitializedCells();
	std::vector<perception_oru::NDTCell*> cells;
	for(int cInd = 0; cInd < allCells.size(); cInd++){
		if(!allCells[cInd]->hasGaussian_){
			cells.push_back(allCells[cInd]);
		}
	}
	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution_c(0, cells.size() - 1);
	tmp.resize(pCount_);
	for(int parNo = 0; parNo < pCount_; parNo++){
		int cellId = distribution_c(generator);
		double cx, cy, cz, sx, sy, sz;
		if(be2D){
			cells[cellId]->getCenter(cx, cy, cz);
			cells[cellId]->getDimensions(sx, sy, sz);
			std::uniform_real_distribution<double> distribution_x(cx - sx / 2.0, cx + sx / 2.0);
			std::uniform_real_distribution<double> distribution_y(cy - sy / 2.0, cy + sy / 2.0);
			std::uniform_real_distribution<double> distribution_t(0.0, 2 * M_PI);
			particleCloud.emplace_back(0.0, 0.0, distribution_t(generator), distribution_x(generator), distribution_y(generator), 0.0);
		}
		//here will go 3d distibution
	}
}

void perception_oru::particle_filter::InitializeFilter(){
	// switch(initializationType){
	//case uniform_map:
	InitializeUniformMap();
	// break;
	//  default:3
	//}
}

void perception_oru::particle_filter::UpdateAndPredict(Eigen::Affine3d tMotion, perception_oru::NDTMap ndtLocalMap_){
	Eigen::Vector3d tr = tMotion.translation();
	Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);

	if(tr[0] != 0.0 && tr[1] != 0.0 && rot[2] != 0){
		Predict2D(tr[0], tr[1], rot[2], tr[0] * 0.03 + 0.005, tr[1] * 0.03 + 0.005, rot[2] * 0.08 + 0.01);
#pragma omp parallel for
		for(int i = 0; i < particleCloud.size(); i++){
			Eigen::Affine3d T = particleCloud[i].GetAsAffine();
			std::vector<perception_oru::NDTCell*> ndts;
			ndts = ndtLocalMap_.pseudoTransformNDT(T);
			double score = 1;
			if(ndts.size() == 0) fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
			for(int n = 0; n < ndts.size(); n++){
				Eigen::Vector3d m = ndts[n]->getMean();
				perception_oru::NDTCell *cell;
				pcl::PointXYZ p;
				p.x = m[0]; p.y = m[1]; p.z = m[2];
				if(ndtMap->getCellForPoint(p, cell)){
					if(cell == NULL) continue;
					if(cell->hasGaussian_){
						Eigen::Matrix3d covCombined = cell->getCov() + ndts[n]->getCov();
						Eigen::Matrix3d icov;
						bool exists;
						double det = 0;
						covCombined.computeInverseAndDetWithCheck(icov, det, exists);
						if(!exists) continue;
						double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
						if(l * 0 != 0) continue;
						score += 0.1 + 0.9 * exp(-0.05 * l / 2.0);
					}else {
					}
				}
			}
			particleCloud[i].SetLikelihood(score);
			for(unsigned int j = 0; j < ndts.size(); j++)
				delete ndts[j];
		}
		Normalize();
		if(forceSIR)
			SIRUpdate();
		else{
			double varP = 0;
			for(int i = 0; i < particleCloud.size(); i++)
				varP += (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()))
				        * (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
			varP /= double(particleCloud.size());
			varP = sqrt(varP);
			if(varP > varLimit || sinceSIR > sirCount){
				//fprintf(stderr,"-SIR- ");
				sinceSIR = 0;
				SIRUpdate();
			}else
				sinceSIR++;
		}
	}
}
void perception_oru::particle_filter::UpdateAndPredict(Eigen::Affine3d tMotion, perception_oru::NDTMap* ndtLocalMap_){
	
	std::cout << "Update and predict" << std::endl;
	
	Eigen::Vector3d tr = tMotion.translation();
	Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);

	if(tr[0] != 0.0 && tr[1] != 0.0 && rot[2] != 0){
		Predict2D(tr[0], tr[1], rot[2], tr[0] * 0.1 + 0.005, tr[1] * 0.1 + 0.005, rot[2] * 0.1 + 0.001);
#pragma omp parallel for
		for(int i = 0; i < particleCloud.size(); i++){
			Eigen::Affine3d T = particleCloud[i].GetAsAffine();
			std::vector<perception_oru::NDTCell*> ndts;
			ndts = ndtLocalMap_->pseudoTransformNDT(T);
			double score = 1;
			if(ndts.size() == 0) fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
			for(int n = 0; n < ndts.size(); n++){
				Eigen::Vector3d m = ndts[n]->getMean();
				perception_oru::NDTCell *cell;
				//pcl::PointXYZ p;
				//p.x = m[0];p.y=m[1];p.z=m[2];
				pcl::PointXYZ p(m[0], m[1], m[2]);
				
				std::cout << "Gaussian cells " << ndtMap->getAllCellsShared().size() << std::endl;
				
				if(ndtMap->getCellForPoint(p, cell)){
					if(cell == NULL) continue;
					std::cout << "Cell : " << cell << std::endl;
					if(cell->hasGaussian_){
						
						std::cout << "GaussianS" << std::endl;
						
						Eigen::Matrix3d covCombined = cell->getCov() + ndts[n]->getCov();
						Eigen::Matrix3d icov;
						bool exists;
						double det = 0;
						covCombined.computeInverseAndDetWithCheck(icov, det, exists);
						if(!exists) continue;
						double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
						if(l * 0 != 0) continue;
						score += 0.1 + 0.9 * exp(-0.05 * l / 2.0);
					}else {
					}
				}
			}
			particleCloud[i].SetLikelihood(score);
			for(unsigned int j = 0; j < ndts.size(); j++)
				delete ndts[j];
		}
		Normalize();
		if(forceSIR)
			SIRUpdate();
		else{
			double varP = 0;
			for(int i = 0; i < particleCloud.size(); i++)
				varP += (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()))
				        * (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
			varP /= double(particleCloud.size());
			varP = sqrt(varP);
			if(varP > varLimit || sinceSIR > sirCount){
				//fprintf(stderr,"-SIR- ");
				sinceSIR = 0;
				SIRUpdate();
			}else
				sinceSIR++;
		}
	}
}

Eigen::Affine3d perception_oru::particle_filter::UpdateAndPredictEff(Eigen::Affine3d tMotion, const pcl::PointCloud< pcl::PointXYZ >& cloud, const Eigen::Affine3d& sensorpose, double subsample_level, double z_cut)
{
	
	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	pcl::transformPointCloud(cloud, transformed_cloud, sensorpose);
	UpdateAndPredictEff(tMotion, transformed_cloud, subsample_level, z_cut);

}


Eigen::Affine3d perception_oru::particle_filter::UpdateAndPredictEff(Eigen::Affine3d tMotion, const pcl::PointCloud< pcl::PointXYZ >& transformed_cloud, double subsample_level, double z_cut)
{
	
	double cellx, celly, cellz;
	bool out = getReferenceMapCellSizes(cellx, celly, cellz);
	
	std::cout << "Got size" << out << std::endl;
	std::cout << "Cells size " << cellx << " " << celly << " " << cellz << " only work with cell of same size on x and y" << std::endl;
	
	assert(cellx == celly);
	assert(celly == cellz);
	if(cellx != celly) exit(0);
	if(celly != cellz) exit(0);
	
	//Cellx/celly/cellz represent the resolution of the internal map of the localization if they are all equal.
	
	perception_oru::NDTMap *localMap = new perception_oru::NDTMap(new perception_oru::LazyGrid(cellx));
	localMap->loadPointCloud(transformed_cloud);
	localMap->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	
	UpdateAndPredictEff(tMotion, localMap, subsample_level, z_cut);
	
	delete localMap;

}



Eigen::Affine3d perception_oru::particle_filter::UpdateAndPredictEff(Eigen::Affine3d tMotion, perception_oru::NDTMap* ndtLocalMap_, double subsample_level, double z_cut){ //you may add z cut here if necessarry
	
	Eigen::Affine3d ret;
	bool ret_done = false;
	bool no_mov = true;
	
	// 	std::cout << "GO" << std::endl;
	
	if(subsample_level < 0 || subsample_level > 1) subsample_level = 1;

	Eigen::Vector3d tr = tMotion.translation();
	Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);
	
	// 	std::cout << "Motion " << tMotion.matrix() << " trans " << tr << " rot " << rot << std::endl;
	// 	std::cout << tr[0] << " != " << 0.0  << " && " << tr[1] << " != " << 0.0 << " && " << rot[2] << " != " << 0 << std::endl;
	
	
	if(tr[0] != 0.0 || tr[1] != 0.0 || rot[2] != 0){
		no_mov = false;
		int all_cells;
		int subsample_cells;
		int used_cells=0;
		Predict2D(tr[0], tr[1], rot[2], fabs(tr[0]) * _motion_model_cov_x + 0.001, fabs(tr[1]) * _motion_model_cov_y + 0.001, fabs(rot[2]) * _motion_model_cov_yaw + 0.001); //this line!!!!!!!!!!!!!!!!!!!!!!!!!
		//Predict2D(tr[0], tr[1], rot[2], fabs(tr[0]) * 0.25 + 0.001, fabs(tr[1]) * 0.25 + 0.001, fabs(rot[2]) * 0.15 + 0.001); //this line!!!!!!!!!!!!!!!!!!!!!!!!!
		std::vector<perception_oru::NDTCell*> ndts0 = ndtLocalMap_->getAllCells();
		std::vector<perception_oru::NDTCell*> ndts;
		if(subsample_level != 1){
			srand(time(NULL));
			for(int i = 0; i < ndts0.size(); ++i){
				double p = ((double)rand()) / RAND_MAX;

				if(p < subsample_level && ndts0[i]->getMean()[2]>z_cut)
					ndts.push_back(ndts0[i]);
				else
					delete ndts0[i];
			}
			//exit(0);
		} else{
 			std::cout << "No sub sample" << std::endl;
			ndts = ndts0;
		}
		
		assert(ndts0.size() > 0);
		assert(ndtMap->getAllCellsShared().size() > 0);
		// 		std::cout << "NDT CELL USED : " << ndts.size() << " gaussian in map " << ndtMap->getAllCellsShared().size() << std::endl;
		
		// 		auto cells = ndtMap->getAllCellsShared();
		//
		// 		for(auto it = cells.begin() ; it != cells.end() ; ++it){
		// 			Eigen::Vector3d mm = (*it)->getMean();
		// 			//if(m[2] < zfilt_min) continue;
		// 			pcl::PointXYZ pm;
		// 			pm.x = mm[0]; pm.y = mm[1]; pm.z = mm[2];
		// // 			std::cout << " gaussian are at point : " << pm.x << " " << pm.y << " " << pm.z << std::endl;
		// 		}
		
		//#pragma omp parallel for
		
		int all_cell_gaussians = 0;
		
		for(int i = 0; i < particleCloud.size(); i++){
			Eigen::Affine3d T = particleCloud[i].GetAsAffine();
			//			if(ret_done == false){
			ret = T;
			//				std::cout << "PARTICULE" << std::endl;
			//				ret_done = true;
			//			}
			
			// std::vector<perception_oru::NDTCell*> ndts;
			// ndts = ndtLocalMap_->pseudoTransformNDT(T);
			double score = 1;
			if(ndts.size() == 0) fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
			
			int gaussian_cells = 0;
			
			for(int n = 0; n < ndts.size(); n++){
				Eigen::Vector3d m = T * ndts[n]->getMean();
				// 				std::cout << " at point : " << T.matrix()  << " * " <<ndts[n]->getMean() << std::endl;
				//if(m[2] < zfilt_min) continue;
				perception_oru::NDTCell *cell;
				pcl::PointXYZ p;
				p.x = m[0]; p.y = m[1]; p.z = m[2];
				// 				std::cout << " at point : " << p.x << " " << p.y << " " << p.z << std::endl;
				// 				if(ndtMap->getCellAtPoint(p, cell)){
				if(ndtMap->getCellForPoint(p, cell)){
					
					// 					std::cout << "Cell is : " << cell << std::endl;
					
					if(cell != NULL){
						// 					    std::cout << "Not NULL" << std::endl;
						// 						if(cell->getClass()!=perception_oru::NDTCell::HORIZONTAL){
						
						// 							std::cout << "Cell is : " << cell->getCenter().x << " " << cell->getCenter().y << " " << cell->getCenter().z;
						// 					std::cout << "Good class" << std::endl;
						if(cell->hasGaussian_){
							gaussian_cells++;
							//		 						std::cout << "Cell has gaussian " << std::endl;
								
							Eigen::Matrix3d map_cov = cell->getCov();
							//cell->blurCov( 10 );

							//Eigen::Matrix3d covCombined = cell->getCov() + T.rotation() * ndts[n]->getCov() * T.rotation().transpose();
							Eigen::Matrix3d covCombined = map_cov + T.rotation() * ndts[n]->getCov() * T.rotation().transpose();
							Eigen::Matrix3d icov;
							bool exists;
							double det = 0;
							covCombined.computeInverseAndDetWithCheck(icov, det, exists);
							if(exists){
								double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
								// 									if(l * 0 == 0){
								// 									std::cout << " m " << m(2) << " and " << cell->getMean()(2) << std::endl;
										
								score += 0.1 + 0.9 * exp( -_scaling_factor_gaussian * l / 2.0); // l is distance between means. 0.005 is scaling down that distance simulating "bigger" Gaussians.
								// 										double score_textbook += exp( -l / 2);
								// 									}
								// 									else{
								// 										std::cout << "how is it possible ? " << l << " and " << l * 0 << std::endl;
								// 										exit(0);
								// 									}
							}
							else{
								// 									std::cout << "Doesn't exist " << std::endl;
							}
						}else{
							//		 						std::cout << "No gaussians" << std::endl;
						}
						// 						}else{
						// 							std::cout << "Cell is not horizontal: " << cell->getClass() << " while h is " << perception_oru::NDTCell::HORIZONTAL;
						// 						}
					}
					else{
						// 						std::cout << "Cell is NULL";
					}
				}
				else{
					// 					std::cout << "NO CELL" << std::endl;
				}
				// 				std::cout << "\n";
			}
			
			// 			std::cout << "Cell with gaussians " << gaussian_cells << std::endl;
			all_cell_gaussians = all_cell_gaussians + gaussian_cells;
			particleCloud[i].SetLikelihood(score);
		}
		
 		std::cout << "All cell with gaussians " << all_cell_gaussians / particleCloud.size() << std::endl;
		if(all_cell_gaussians == 0){
			exit(0);
		}
		
		
		for(unsigned int j = 0; j < ndts.size(); j++)
			delete ndts[j];

		Normalize();
		if(forceSIR)
			SIRUpdate();
		else{
			double varP = 0;
			for(int i = 0; i < particleCloud.size(); i++)
				varP += (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()))
				        * (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
			varP /= double(particleCloud.size());
			varP = sqrt(varP);
			if(varP > varLimit || sinceSIR > sirCount){
				//fprintf(stderr,"-SIR- ");
				sinceSIR = 0;
				SIRUpdate();
			}else
				sinceSIR++;
		}
		
		
		// 		exit(0);
	}
	
	// 	std::cout << " Wahat " << ret_done << " == " << false << " && "<< no_mov << " == " << false << std::endl;
	// 	if(ret_done == false && no_mov == false){
	// 		std::cout << "WHAT NO PARTICULE TESTED ?" << std::endl;
	// 		exit(0);
	// 	}
	//
	// 	std::cout << "RET \n" << ret.matrix() << std::endl;
	
	return ret;
	// 	std::cout << "END" << std::endl;
}



void perception_oru::particle_filter::UpdateAndPredictEffRe(Eigen::Affine3d tMotion, perception_oru::NDTMap* ndtLocalMap_, double subsample_level, double z_cut, double x_var, double y_var, double th_var, double r_x_var, double r_y_var, double r_th_var, int tres){
	if(subsample_level < 0 || subsample_level > 1) subsample_level = 1;
	double tre=weights[tres];
	Eigen::Vector3d tr = tMotion.translation();
	Eigen::Vector3d rot = tMotion.rotation().eulerAngles(0, 1, 2);
	if(tr[0] != 0.0 || tr[1] != 0.0 || rot[2] != 0){
		int all_cells;
		int subsample_cells;
		int used_cells=0;
		Predict2D(tr[0], tr[1], rot[2], fabs(tr[0]) * x_var, fabs(tr[1]) * y_var, fabs(rot[2]) * th_var); //this line!!!!!!!!!!!!!!!!!!!!!!!!!
		std::vector<perception_oru::NDTCell*> ndts0 = ndtLocalMap_->getAllCells();
		std::vector<perception_oru::NDTCell*> ndts;
		if(subsample_level != 1){
			srand(time(NULL));
			for(int i = 0; i < ndts0.size(); ++i){
				double p = ((double)rand()) / RAND_MAX;

				if(p < subsample_level && ndts0[i]->getMean()[2]>z_cut && ndts0[i]->getClass()!=perception_oru::NDTCell::HORIZONTAL)
					ndts.push_back(ndts0[i]);
				else
					delete ndts0[i];
			}
		} else
			ndts = ndts0;

		//#pragma omp parallel for
		for(int i = 0; i < particleCloud.size(); i++){
			Eigen::Affine3d T;
			if(particleCloud[i].GetProbability()>tre){
				T = particleCloud[i].GetAsAffine();
			}
			else{
				std::default_random_engine generator;
				std::normal_distribution<double> distribution_x(m_pose[0], r_x_var);
				std::normal_distribution<double> distribution_y(m_pose[1], r_y_var);
				std::normal_distribution<double> distribution_t(m_pose[2], r_th_var);
				particleCloud[i].Set(0.0, 0.0, distribution_t(generator), distribution_x(generator), distribution_y(generator), 0.0);

				T = particleCloud[i].GetAsAffine();
			}
			// std::vector<perception_oru::NDTCell*> ndts;
			// ndts = ndtLocalMap_->pseudoTransformNDT(T);
			double score = 1;
			if(ndts.size() == 0) fprintf(stderr, "ERROR no gaussians in measurement!!!\n");
			for(int n = 0; n < ndts.size(); n++){
				Eigen::Vector3d m = T * ndts[n]->getMean();
				//if(m[2] < zfilt_min) continue;
				perception_oru::NDTCell *cell;
				pcl::PointXYZ p;
				p.x = m[0]; p.y = m[1]; p.z = m[2];
				if(ndtMap->getCellAtPoint(p, cell)){
					if(cell == NULL) continue;
					if(cell->hasGaussian_){
						Eigen::Matrix3d covCombined = cell->getCov() + T.rotation() * ndts[n]->getCov() * T.rotation().transpose();
						Eigen::Matrix3d icov;
						bool exists;
						double det = 0;
						covCombined.computeInverseAndDetWithCheck(icov, det, exists);
						if(!exists) continue;
						double l = (cell->getMean() - m).dot(icov * (cell->getMean() - m));
						if(l * 0 != 0) continue;
						score += 0.1 + 0.9 * exp(-0.5*l / 2.0);
					}else{

					}
				}
			}
			particleCloud[i].SetLikelihood(score);
		}
		for(unsigned int j = 0; j < ndts.size(); j++)
			delete ndts[j];

		Normalize();
		if(forceSIR)
			SIRUpdate();
		else{
			double varP = 0;
			for(int i = 0; i < particleCloud.size(); i++)
				varP += (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()))
				        * (particleCloud[i].GetProbability() - 1.0 / double(particleCloud.size()));
			varP /= double(particleCloud.size());
			varP = sqrt(varP);
			if(varP > varLimit || sinceSIR > sirCount){
				//fprintf(stderr,"-SIR- ");
				sinceSIR = 0;
				SIRUpdate();
			}else
				sinceSIR++;
		}
	}
}


void perception_oru::particle_filter::Predict2D(double x, double y, double th, double sx, double sy, double sth){
	float dx = 0.0, dy = 0.0, dl = 0.0;
	float t = 0.0;
	float dxe, dye; ///<estimates
	std::default_random_engine generator;
	float eps=0.000001;
	for(int i = 0; i < particleCloud.size(); i++){
		double px, py, pz, pr, pp, pt;
		particleCloud[i].GetXYZ(px, py, pz);
		particleCloud[i].GetRPY(pr, pp, pt);

		std::normal_distribution<double> distribution_x(0, sx);
		std::normal_distribution<double> distribution_y(0, sy);
		std::normal_distribution<double> distribution_th(0, sth);
		///Generate noise from normal distribution
		dxe = x + distribution_x(generator);
		dye = y + distribution_y(generator);

		//if(fabs(x)<eps && fabs(y)<eps){
		//	dxe = x*0;
		//	dye = y*0;
		//
		//}


		dl = sqrt(dxe * dxe + dye * dye);

		//if(fabs(dxe)<eps && fabs(dye)<eps)
		//t = 0;
		//else
		t = atan2(dye, dxe);

		dx = dl * cos(pt + t);
		dy = dl * sin(pt + t);
		//ROS_INFO_STREAM("th"<<th);
		px += dx;
		py += dy;
		pt = pt + th + distribution_th(generator);
		toPI(pt);
		particleCloud[i].Set(pr, pp, pt, px, py, pz);
	}
	//    isAvgSet = false;
}
void perception_oru::particle_filter::to2PI(double &a){
	a = (double)fmod((double)(a), (double)( 2 * M_PI));
	if(a < 0) a += 2 * (double)M_PI;
}
void perception_oru::particle_filter::toPI(double &a){
	if(a > M_PI)
		while(a > M_PI) a -= 2.0 * M_PI;
	else
		if(a < -M_PI)
			while(a < -M_PI) a += 2.0 * M_PI;
}

void perception_oru::particle_filter::Normalize(){
	int i;
	double summ = 0;
	double sumX = 0, sumY = 0;
	double sumW = 0;
	double ax = 0, ay = 0;

	//isAvgSet = false;
	for(i = 0; i < particleCloud.size(); i++){
		particleCloud[i].SetProbability(particleCloud[i].GetProbability() * particleCloud[i].GetLikelihood());
		summ += particleCloud[i].GetProbability();
	}
	if(summ != 0){
		for(i = 0; i < particleCloud.size(); i++){
			particleCloud[i].SetProbability(particleCloud[i].GetProbability() / summ);
			weights[i]=particleCloud[i].GetProbability();
			double x, y, z, r, p, t;
			particleCloud[i].GetXYZ(x, y, z);
			particleCloud[i].GetRPY(r, p, t);
			sumX += particleCloud[i].GetProbability() * x;
			sumY += particleCloud[i].GetProbability() * y;
			ax += particleCloud[i].GetProbability() * cos(t);
			ay += particleCloud[i].GetProbability() * sin(t);
			sumW += particleCloud[i].GetProbability();
		}
	}
	else{
		for(i = 0; i < particleCloud.size(); i++){
			particleCloud[i].SetProbability(1.0 / particleCloud.size());
			weights[i]=particleCloud[i].GetProbability();
			double x, y, z, r, p, t;
			particleCloud[i].GetXYZ(x, y, z);
			particleCloud[i].GetRPY(r, p, t);
			sumX += particleCloud[i].GetProbability() * x;
			sumY += particleCloud[i].GetProbability() * y;
			ax += particleCloud[i].GetProbability() * cos(t);
			ay += particleCloud[i].GetProbability() * sin(t);
			sumW += particleCloud[i].GetProbability();
		}
	}
	m_pose << sumX, sumY, atan2(ay, ax);
	std::sort (weights.begin(), weights.end(),myfunction);
	//ROS_INFO_STREAM(weights.front()<<" "<< weights.back());
}
void perception_oru::particle_filter::SIRUpdate(){
	std::vector<particle, Eigen::aligned_allocator<particle> > tmp2;
	std::default_random_engine generator;
	std::uniform_real_distribution<double> dist(0, 1);
	double U = 0, Q = 0;
	int i = 0, j = 0, k = 0;
	U = dist(generator) / (double)particleCloud.size();
	//fprintf(stderr,"SIRUpdate()::U=%.6f\n",U);
	while(U < 1.0){
		if(Q > U){
			U += 1.0 / (double)particleCloud.size();
			if(k >= particleCloud.size() || i >= particleCloud.size()){
				while(i < particleCloud.size()){
					tmp[i] = particleCloud[particleCloud.size() - 1];
					tmp[i].probability = 1.0 / (double)particleCloud.size();
					i++;
				}
				//fprintf(stderr,"ERROR: SIRupdate:: Invalid index k='%d' or i='%d'\n",k,i);
				tmp2 = particleCloud;
				particleCloud = tmp;
				tmp = tmp2;
				return;
			}
			tmp[i] = particleCloud[k];
			tmp[i].probability = 1.0 / (double)particleCloud.size();
			i++;
		}else {
			j++;
			k = j;
			if(j >= particleCloud.size()){
				while(i < particleCloud.size()){
					tmp[i] = particleCloud[particleCloud.size() - 1];
					tmp[i].probability = 1.0 / (double)particleCloud.size();
					i++;
				}
				//fprintf(stderr,"ERROR: SIRupdate:: Invalid index j='%d' \n",j);
				tmp2 = particleCloud;
				particleCloud = tmp;
				tmp = tmp2;
				return;
			}
			Q += particleCloud[j].probability;
			///j++; ///WAS HERE until 30.7.2008

			if(j == particleCloud.size()){
				while(i < particleCloud.size()){
					tmp[i] = particleCloud[k - 1];
					tmp[i].probability = 1.0 / (double)particleCloud.size();
					i++;
				}
				tmp2 = particleCloud;
				particleCloud = tmp;
				tmp = tmp2;
				return;
			}
		}
	} //While
	while(i < particleCloud.size()){
		if(k >= particleCloud.size()) k = particleCloud.size() - 1;
		tmp[i] = particleCloud[k];
		tmp[i].probability = 1.0 / (double)particleCloud.size();
		i++;
	}
	//  isAvgSet = false;
	tmp2 = particleCloud;
	particleCloud = tmp;
	tmp = tmp2;
}

void perception_oru::particle_filter::GetRandomPoint(perception_oru::NDTCell* cell, double &x, double &y, double &th){
	Eigen::Vector3d mean = cell->getMean();
	Eigen::Matrix3d cov = cell->getCov();
	double mX = mean[0], mY = mean[1], mTh = mean[2];
	double varX = cov(0, 0), varY = cov(1, 1), varTh = cov(2, 2);
	std::default_random_engine generator;

	std::normal_distribution<double> distribution_x(mX, varX);
	std::normal_distribution<double> distribution_y(mY, varY);
	std::normal_distribution<double> distribution_th(mTh, varTh);
	x = distribution_x(generator);
	y = distribution_y(generator);
	th = distribution_th(generator);
}

void perception_oru::particle_filter::EigenSort( Eigen::Vector3d &eigenvalues, Eigen::Matrix3d &eigenvectors ){
	int k, j, i;
	double p;

	for(i = 0; i < 2; i++){
		p = eigenvalues(k = i);
		for(j = i + 1; j < 3; j++)
			if(fabs(eigenvalues(j)) >= fabs(p))
				p = eigenvalues(k = j);
		if(k != i){
			eigenvalues.row(k).swap(eigenvalues.row(i));
			eigenvectors.col(k).swap(eigenvectors.col(i));
		}
	}
}

Eigen::Affine3d perception_oru::particle_filter::getAsAffine(float x, float y, float yaw ){
	Eigen::Matrix3d m;

	m = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Translation3d v(x, y, 0);
	Eigen::Affine3d T = Eigen::Affine3d::Identity();
	T.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	T.translation()<<x,y,0;
	return T;
}
