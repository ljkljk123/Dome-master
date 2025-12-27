#pragma once

#include <ros/ros.h>

#include <unordered_set>
#include <cmath>
#include <functional>
#include <vector>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/kdtree/kdtree_flann.h>
#include <fstream>
#include <iomanip>
#include <bitset>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <fftw3.h>
#include <unordered_map>
#include <algorithm>
#include <numeric>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <mutex>
#include <pcl/io/io.h>
#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <string>
#include <omp.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <boost/thread/thread.hpp>
#include <vector>
#include <map>
#include <set>
#include <deque>
// FFT-based cosine similarity using FFTW
#include <complex>


// =================== struct ===================
using namespace std;
const int N = 2761;
const string seq = "05";
const int NUM_PITCH_BINS = 40;
const int NUM_YAW_BINS = 120;
const int NUM_RADIUS_BINS = 20;
const float MAX_RADIUS = 100.0f;
const int MIN_LOOP_GAP = 300;
const int MAX_BINS = 2761;

#pragma pack(push, 1)
struct AviaPoint
{
    float x;
    float y;
    float z;
    uint8_t reflectivity;
    uint8_t tag;
    uint8_t line;
    uint32_t offset_time;
};
#pragma pack(pop)

// =================== IO ===================
void loadAviaBin(const std::string &filename,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

std::vector<std::vector<int>>
getGTFromPose(const std::string &pose_path);

// =================== LM ===================
std::pair<int, int>
compute_global_lm_bin(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

// =================== Dome / Projection ===================
cv::Mat generate_dome_image(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    float phi_min,
    float phi_max);

// =================== Weighting ===================
cv::Mat convolve_with_pitch_weight(
    const cv::Mat &py_image,
    const cv::Mat &pitch_weight);

cv::Mat compute_yaw_weight(
    const cv::Mat &expanded_py,
    float epsilon = 1e-5);

cv::Mat generate_ry_max_height(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

cv::Mat compute_1d_pitch_weight(
    const cv::Mat &rp_density,
    float epsilon = 1e-5);

// =================== Feature ===================
std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, float, float>
preprocess_pointcloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    float ground_th = -1.5f);

std::vector<float>
extract_dcl_features(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    float phi_min,
    float phi_max);

// =================== Similarity ===================
void normalize_vector(std::vector<float> &v);

double compute_cosine_similarity_fft(
    const std::vector<float> &V1,
    const std::vector<float> &V2);

void compute_yaw_fft(
    const std::vector<float> &featA,
    const std::vector<float> &featB,
    double &out_max_corr,
    double &out_yaw_deg);

// =================== Ground / Filter ===================
pcl::PointCloud<pcl::PointXYZI>::Ptr
plane_clip(const pcl::PointCloud<pcl::PointXYZI>::Ptr &src_cloud,
           const Eigen::Vector4f &plane,
           bool negative);

pcl::PointCloud<pcl::PointXYZI>::Ptr
normal_filtering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    double sensor_height,
    double normal_filter_thresh);

pcl::PointCloud<pcl::PointXYZI>::Ptr
detect(pcl::PointCloud<pcl::PointXYZI> cloud);

// =================== Utils ===================
std::pair<float, float>
computeElevationRange(
    const pcl::PointCloud<pcl::PointXYZI> &cloud);
