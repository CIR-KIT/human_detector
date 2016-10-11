#include <segment_cluster_creator.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

using namespace pcl;

EuclideanCluster::EuclideanCluster(ros::NodeHandle nh, ros::NodeHandle n)
    : nh_(nh),
      rate_(n.param("loop_rate", 10)),
      accumulation_counter_(0),
      frame_id_(n.param<std::string>("clustering_frame_id", "base_link"))
{
  source_pc_sub_ = nh_.subscribe(n.param<std::string>("source_pc_topic_name", "/hokuyo3d/hokuyo_cloud2"), 1, &EuclideanCluster::EuclideanCallback, this);
  fileterd_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(n.param<std::string>("filtered_pc_topic_name", "/filtered_pointcloud"), 1);
  euclidean_cluster_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(n.param<std::string>("box_name", "/clustering_result"), 1);

  // クラスタリングのパラメータを初期化
  n.param<double>("clusterTolerance", clusterTolerance_, 0.3);
  n.param<int>("minSize", minSize_, 100);
  n.param<int>("maxSize", maxSize_, 25000);
  // clopboxを当てはめるエリアを定義
  n.param<float>("crop_x_min", crop_min_.x, -35.5);
  n.param<float>("crop_x_max", crop_max_.x, 35.5);
  n.param<float>("crop_y_min", crop_min_.y, -35.5);
  n.param<float>("crop_y_max", crop_max_.y, 35.5);
  n.param<float>("crop_z_min", crop_min_.z, 0.05);
  n.param<float>("crop_z_max", crop_max_.z, 15.5);

  boost::posix_time::ptime thistime = boost::posix_time::from_time_t(ros::Time::now().toSec());
  output_file_prefix_ = to_simple_string(thistime) + "Cluster_";
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
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (pcl_source_ptr);
  sor.setMeanK (100);
  sor.setStddevMulThresh (0.1);
  sor.filter (*pcl_source_ptr);

  // 平面をしきい値で除去する→Cropboxで
  CropBox(pcl_source_ptr, crop_min_, crop_max_);

  // 処理後の点群をpublish
  sensor_msgs::PointCloud2 filtered_pc2;
  pcl::toROSMsg(*pcl_source_ptr, filtered_pc2);
  filtered_pc2.header.stamp = ros::Time::now();
  filtered_pc2.header.frame_id = "base_link";
  fileterd_cloud_pub_.publish(filtered_pc2);

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
    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::stringstream filename;
    filename << output_file_prefix_ << std::setw(4) << std::setfill('0') <<  accumulation_counter_;
    pcl::io::savePCDFileASCII ("./"+filename.str()+".pcd" , *cloud_cluster);
    // 一つのclusterをpushback
    j++;
    accumulation_counter_++;
  }

  // int clusterLength = clusterIndices.size();
  ROS_INFO("Found %lu clusters:", cluster_indices.size());

  // Empty Buffer
  cluster_indices.clear();
}



void EuclideanCluster::run()
{
  while(nh_.ok()){
    ros::spinOnce();
    rate_.sleep();
  }
}
