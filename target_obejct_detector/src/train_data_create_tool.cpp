#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <boost/foreach.hpp>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

class Viewer
{
public:
  Viewer()
    : viewer_(new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" )),
      dir_("."),
      buffer_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
  {
    getdir(dir_, files_);
  }

  void run()
  {
    viewer_->registerKeyboardCallback( &Viewer::keyboard_callback, *this );
    viewer_->setBackgroundColor (255, 255, 255);
    string filename = files_.back();
    std::cout << filename << std::endl;
    files_.pop_back();
    std::cout << "==================================" << std::endl;
    std::cout << "Now opening file is : " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (filename, *cloud);
    //buffer_cloud_ = cloud->makeShared();
    pcl::io::loadPCDFile (filename, *buffer_cloud_);
    std::cout << "cloud points : " << cloud->points.size() << std::endl;
    viewer_->addPointCloud(cloud, "Cloud");
   
    while (!viewer_->wasStopped ())
    {
      viewer_->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }

private:

  void keyboard_callback( const pcl::visualization::KeyboardEvent& event, void* )
  {
    if( event.getKeyCode() && event.keyDown() ){

      if(event.getKeyCode() == 'y' || event.getKeyCode() == 'n'){
        std::cout << "Key : " << event.getKeyCode() << std::endl;
        //ここで特徴量の計算
        std::vector<float> features;
        feature_calculation(buffer_cloud_, features);
        if (event.getKeyCode() == 'y') {
          output_features(1, features);
        }else if(event.getKeyCode() == 'n')
        {
          output_features(0, features);
        }
        //新しいポイントクラウドの読み込み
        string filename = files_.back();
        std::cout << "==================================" << std::endl;
        std::cout << "Now opening file is : " << filename << std::endl;
        files_.pop_back();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile (filename, *cloud);
        pcl::io::loadPCDFile (filename, *buffer_cloud_);
        //buffer_cloud_ = cloud->makeShared();
        std::cout << "cloud points : " << cloud->points.size() << std::endl;
        viewer_->updatePointCloud(cloud, "Cloud");


      }
    }
  }

  void feature_calculation(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                           std::vector<float> &features)
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
    complex<double> lambda1 = es.eigenvalues()[0];
    complex<double> lambda2 = es.eigenvalues()[1];
    complex<double> lambda3 = es.eigenvalues()[2];
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

    features.push_back(f21);
    features.push_back(f22);
    features.push_back(f23);
    features.push_back(f24);
    features.push_back(f25);
    features.push_back(f26);
    features.push_back(f31);
    features.push_back(f32);
    features.push_back(f33);
    features.push_back(f34);
    features.push_back(f35);
    features.push_back(f36);
    features.push_back(f37);

  }

  void output_features(int label, std::vector<float> features)
  {
    output_file_.open("train.csv", ios::ate | ios::app);
    output_file_ << label;
    for (int i = 0; i < features.size(); ++i) {
      output_file_ << " " << i+1 << ":" << features[i];
    }
    output_file_ << "\n";
    output_file_.close();
  }
  
  int getdir (string dir, vector<string> &files)
  {
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
        files.push_back(string(dirp->d_name));
    }
    closedir(dp);
    sort( files_.begin(), files_.end() );
    for (size_t i = 0; i < files_.size(); ++i) {
      std::cout << files_[i] << std::endl;
    }

    return 0;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  string dir_;
  vector<string> files_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr buffer_cloud_;
  std::ofstream output_file_;
};

int main( int argc, char* argv[] )
{
  Viewer viewer;
  viewer.run();

  return 0;
}
