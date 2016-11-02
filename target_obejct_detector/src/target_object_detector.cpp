#include <target_object_detector.hpp>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>
#include "libsvm/svm.h"
#include <ros/package.h>
using namespace pcl;

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle n)
  : nh_(nh),
    rate_(n.param("loop_rate", 10)),
    frame_id_(n.param<std::string>("clustering_frame_id", "base_link"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/hokuyo3d/hokuyo_cloud2"), 1, &EuclideanCluster::EuclideanCallback, this);
  fileterd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("filtered_pc_topic_name", "/filtered_pointcloud"), 1);
  euclidean_cluster_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(n.param<std::string>("box_name", "/clustering_result"), 1);

  // クラスタリングのパラメータを初期化
  n.param<double>("clusterTolerance", clusterTolerance_, 0.02);
  n.param<int>("minSize", minSize_, 100);
  n.param<int>("maxSize", maxSize_, 25000);
  // clopboxを当てはめるエリアを定義
  n.param<float>("crop_x_min", crop_min_.x, 5.5);
  n.param<float>("crop_x_max", crop_max_.x, -5.5);
  n.param<float>("crop_y_min", crop_min_.y, -5.5);
  n.param<float>("crop_y_max", crop_max_.y, 5.5);
  n.param<float>("crop_z_min", crop_min_.z, -0.1);
  n.param<float>("crop_z_max", crop_max_.z, 0.5);
  n.param<std::string>("svm_model_path", svm_model_path_,
                       ros::package::getPath("target_object_detector")
                       + "/model/train.csv.model");
  if((model_=svm_load_model(svm_model_path_.c_str()))==0)
  {
    fprintf(stderr,"can't open model file %s\n",svm_model_path_.c_str());
    exit(1);
  }
  n.param<std::string>("svm_range_path", svm_range_path_,
                       ros::package::getPath("target_object_detector")
                       + "/model/train.csv.range");
  read_scaling_parameters(svm_range_path_);
}

void EuclideanCluster::EuclideanCallback(
    const sensor_msgs::PointCloud2::ConstPtr &source_pc) {

  //点群をKinect座標系からWorld座標系に変換
  //変換されたデータはtrans_pcに格納される．
  sensor_msgs::PointCloud2 trans_pc;
  try {
    pcl_ros::transformPointCloud(frame_id_, *source_pc, trans_pc, tf_);
  } catch (tf::ExtrapolationException e) {
    ROS_ERROR("pcl_ros::transformPointCloud %s", e.what());
  }

  // sensor_msgs::PointCloud2 → pcl::PointCloud
  pcl::PointCloud<PointXYZI> pcl_source;
  pcl::fromROSMsg(trans_pc, pcl_source);
  pcl::PointCloud<PointXYZI>::Ptr pcl_source_ptr(new pcl::PointCloud<PointXYZI>(pcl_source));

  // 点群の中からnanを消す
  // std::vector<int> dummy;
  // pcl::removeNaNFromPointCloud(*pcl_source_ptr, *pcl_source_ptr, dummy);

  // Create the filtering object
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  // sor.setInputCloud (pcl_source_ptr);
  // sor.setMeanK (100);
  // sor.setStddevMulThresh (0.1);
  // sor.filter (*pcl_source_ptr);

  // 平面をしきい値で除去する→Cropboxで
  CropBox(pcl_source_ptr, crop_min_, crop_max_);

  // 処理後の点群をpublish
  // sensor_msgs::PointCloud2 filtered_pc2;
  // pcl::toROSMsg(*pcl_source_ptr, filtered_pc2);
  // filtered_pc2.header.stamp = ros::Time::now();
  // filtered_pc2.header.frame_id = "base_link";
  // fileterd_cloud_pub_.publish(filtered_pc2);

  // Creating the KdTree object for the search method of the extraction
  Clustering(pcl_source_ptr);
}

void EuclideanCluster::CropBox(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               pcl::PointXYZ min, pcl::PointXYZ max) {
  Eigen::Vector4f minPoint;

  minPoint[0] = min.x; // define minimum point x
  minPoint[1] = min.y; // define minimum point y
  minPoint[2] = min.z; // define minimum point z

  Eigen::Vector4f maxPoint;
  maxPoint[0] = max.x; // define max point x
  maxPoint[1] = max.y; // define max point y
  maxPoint[2] = max.z; // define max point z

  Eigen::Vector3f boxTranslatation;
  boxTranslatation[0] = 0;
  boxTranslatation[1] = 0;
  boxTranslatation[2] = 0;

  Eigen::Vector3f boxRotation;
  boxRotation[0] = 0; // rotation around x-axis
  boxRotation[1] = 0; // rotation around y-axis
  boxRotation[2] = 0; // in radians rotation around z-axis. this rotates your

  Eigen::Affine3f boxTransform;

  pcl::CropBox<pcl::PointXYZI> cropFilter;
  cropFilter.setInputCloud(cloud);
  cropFilter.setMin(minPoint);
  cropFilter.setMax(maxPoint);
  cropFilter.setTranslation(boxTranslatation);
  cropFilter.setRotation(boxRotation);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
  cropFilter.filter(*cloud_filtered);
  pcl::copyPointCloud<pcl::PointXYZI, pcl::PointXYZI>(*cloud_filtered, *cloud);
}

void EuclideanCluster::Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance(clusterTolerance_);
  ec.setMinClusterSize(minSize_);
  ec.setMaxClusterSize(maxSize_);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  int j = 0;
  jsk_recognition_msgs::BoundingBoxArray box_array; // clustering結果をぶち込む配列

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
      cloud_cluster->points.push_back(cloud->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    //特徴量の計算
    svm_node features[14];
    feature_calculation(cloud_cluster, features);
    ROS_INFO_STREAM("features are extracted");
    int is_target_object = static_cast<int>( svm_predict( model_, features ) );
    ROS_INFO_STREAM("is_target_object : " << is_target_object);
    if (is_target_object == 1) {
      // 一つのclusterをpushback
      jsk_recognition_msgs::BoundingBox box;
      box = MinAreaRect(cloud_cluster, j);
      box_array.boxes.push_back(box);
      j++;
    }
  }

  // int clusterLength = clusterIndices.size();
  ROS_INFO("Found %lu clusters:", cluster_indices.size());

  // publish
  box_array.header.stamp = ros::Time::now();
  box_array.header.frame_id = frame_id_;
  euclidean_cluster_pub_.publish(box_array);

  // Empty Buffer
  cluster_indices.clear();
}

jsk_recognition_msgs::BoundingBox EuclideanCluster::MinAreaRect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int cluster_cnt){
  // PCLによる点群の最大最小エリア取得
  pcl::MomentOfInertiaEstimation<pcl::PointXYZI> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.compute();

  std::vector<float> moment_of_inertia;
  std::vector<float> eccentricity;
  pcl::PointXYZI min_point_AABB;
  pcl::PointXYZI max_point_AABB;

  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 size;

  feature_extractor.getMomentOfInertia(moment_of_inertia);
  feature_extractor.getEccentricity(eccentricity);
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);

  // OpenCVで最小矩形を当てはめる
  std::vector<cv::Point2f> points;
  for(unsigned int i = 0; i < cloud->points.size(); i++){
    cv::Point2f p2d;
    p2d.x = cloud->points[i].x;
    p2d.y = cloud->points[i].y;
    points.push_back(p2d);
  }
  cv::Mat points_mat(points);
  cv::RotatedRect rrect = cv::minAreaRect(points_mat);

  // ROS_INFO("Center of mass (x, y) = (%f, %f)", rrect.center.x, rrect.center.y);
  // ROS_INFO("Height = %f Width =  %f", rrect.size.height, rrect.size.width);
  // ROS_INFO("Angle = %f [deg]", rrect.angle);

  // jsk_recognition_msgs::BoundingBoxの型に合わせて代入していく
  pose.position.x = rrect.center.x;
  pose.position.y = rrect.center.y;
  pose.position.z = (min_point_AABB.z + max_point_AABB.z) / 2.0;

  Eigen::Matrix3f AxisAngle;
  Eigen::Vector3f axis(0,0,1); //z 軸を指定
  AxisAngle = Eigen::AngleAxisf(rrect.angle*M_PI/180.0, axis); // z軸周りに90度反時計回りに回転
  Eigen::Quaternionf quat(AxisAngle); // クォータニオンに変換
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  size.x = rrect.size.width;
  size.y = rrect.size.height;
  size.z = max_point_AABB.z - min_point_AABB.z;

  // TFの名前付け
  std::stringstream ss;
  std::string object_name;
  ss << cluster_cnt;
  object_name = "object_" + ss.str();

  br_.sendTransform(tf::StampedTransform(
      tf::Transform(
          tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
          tf::Vector3(pose.position.x, pose.position.y, max_point_AABB.z)),
          ros::Time::now(), "world", object_name));

  jsk_recognition_msgs::BoundingBox box;
  box.header.frame_id = frame_id_;
  box.pose = pose;
  box.dimensions = size;
  box.label = cluster_cnt;

  return box;
}



void EuclideanCluster::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}


void EuclideanCluster::feature_calculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         svm_node *features)
  {
    int n;
    float f21,f22,f23,f24,f25,f26;
    float v1, v2, v3;
    double v11, v22, v33;
    float f31,f32,f33,f34,f35,f36,f37;
    pcl::PointXYZI min_point;
    pcl::PointXYZI max_point;
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    Eigen::Matrix3f cmat;
    Eigen::Vector4f xyz_centroid;
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();
    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point, max_point);
    pcl::compute3DCentroid (*cloud, xyz_centroid);
    pcl::computeCovarianceMatrix (*cloud, xyz_centroid, covariance_matrix);
    n = cloud->points.size();
    f21 = covariance_matrix (0,0) / (n - 1);
    f22 = covariance_matrix (0,1) / (n - 1);
    f23 = covariance_matrix (0,2) / (n - 1);
    f24 = covariance_matrix (1,1) / (n - 1);
    f25 = covariance_matrix (1,2) / (n - 1);
    f26 = covariance_matrix (2,2) / (n - 1);
    cmat (0,0) = f21;
    cmat (0,1) = f22;
    cmat (0,2) = f23;
    cmat (1,0) = f22;
    cmat (1,1) = f24;
    cmat (1,2) = f25;
    cmat (2,0) = f23;
    cmat (2,1) = f25;
    cmat (2,2) = f26;
    Eigen::EigenSolver<Eigen::MatrixXf> es(cmat, false);
    std::complex<double> lambda1 = es.eigenvalues()[0];
    std::complex<double> lambda2 = es.eigenvalues()[1];
    std::complex<double> lambda3 = es.eigenvalues()[2];
    v11 = lambda1.real();
    v22 = lambda2.real();
    v33 = lambda3.real();
    std::vector<double> v;
    v.push_back (v11);
    v.push_back (v22);
    v.push_back (v33);
    std::sort(v.begin(), v.end() );
    v3 = v[0];
    v2 = v[1];
    v1 = v[2];
    v.clear();

    double nnn = v1 * v2 * v3;
    f31 = (v1 - v2) / v1;
    f32 = (v2 - v3) / v1;
    f33 = v3 / v1;
    f34 = pow(nnn, 1.0 / 3.0);
    f35 = (v1 - v3) / v1;
    f36 = -(v1 * log(v1) + v2 * log(v2) + v3 * log(v3));
    f37 = v3 / (v1 + v2 + v3);

    features[0].value = f21;
    features[1].value = f22;
    features[2].value = f23;
    features[3].value = f24;
    features[4].value = f25;
    features[5].value = f26;
    features[6].value = f31;
    features[7].value = f32;
    features[8].value = f33;
    features[9].value = f34;
    features[10].value = f35;
    features[11].value = f36;
    features[12].value = f37;
    features[0].index = 1;
    features[1].index = 2;
    features[2].index = 3;
    features[3].index = 4;
    features[4].index = 5;
    features[5].index = 6;
    features[6].index = 7;
    features[7].index = 8;
    features[8].index = 9;
    features[9].index = 10;
    features[10].index = 11;
    features[11].index = 12;
    features[12].index = 13;
    features[13].index = -1;
    normlize_features(features);
    for (int i = 0; i < 13; ++i) {
      std::cout << features[i].value << ", ";
    }
    std::cout << "\n";

  }


void EuclideanCluster::normlize_features(svm_node *features)
{
  for (int i = 0; i < 13; ++i) {
    if (feature_max_[i] == feature_min_[i]) {
      continue;
    }
    if(features[i].value == feature_min_[i]){
      features[i].value = feature_lower_;
    }else if(features[i].value == feature_max_[i]){
      features[i].value = feature_upper_;
    }else{
      features[i].value = feature_lower_ + (feature_upper_ - feature_lower_)*
        (features[i].value - feature_min_[i])/
        (feature_max_[i] - feature_min_[i]);
    }
  }
}

//スケーリングバリューは今は直書きしてるけど余裕があれば読み込めるようにする
void EuclideanCluster::read_scaling_parameters(std::string scaling_parameter_file)
{
  scaling_parameter_file = "/home/morita/Documents/dev/ros/tc2016_ws/src/TC2016_for_thirdrobot/target_obejct_detector/model/train.csv.range";

  std::ifstream ifs(scaling_parameter_file.c_str());
  std::string str;
  if (ifs.fail())
  {
      ROS_ERROR("failed to read scaling parameter file");
      return;
  }

  // first line
  if(!getline(ifs, str))
  {
      ROS_ERROR("failed to read first line on scaling parameter file");
      return;
  }

  // second line
  if(!getline(ifs, str))
  {
      ROS_ERROR("failed to read first line on scaling parameter file");
      return;
  }

  std::vector<std::string> splited = split(str, ' ');

  feature_lower_ = atof(splited[0].c_str());
  feature_upper_ = atof(splited[1].c_str());

  // after second line
  while(getline(ifs, str))
  {
    std::vector<std::string> splited = split(str, ' ');

    feature_min_.push_back(atof(splited[1].c_str()));
    feature_max_.push_back(atof(splited[2].c_str()));
  }
}

std::vector<std::string> EuclideanCluster::split(const std::string &s, char delim)
{
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (getline(ss, item, delim)) {
    if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}
