#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>

using namespace std;

class Viewer
{
public:
  Viewer()
    : viewer_(new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" )),
      dir_(".")
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (filename, *cloud);
    std::cout << "cloud points : " << cloud->points.size() << std::endl;
    viewer_->addPointCloud(cloud, "Cloud");
    // viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "Cloud");

    
    // while( !viewer_->wasStopped() ){
    //   viewer_->spinOnce();
    // }
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
      //viewer_->removeAllPointClouds();
      //viewer_->removePointCloud("Cloud");
      if(event.getKeyCode() == 'y' || event.getKeyCode() == 'n'){
        //ここで特徴量の計算
        
        string filename = files_.back();
        std::cout << filename << std::endl;
        files_.pop_back();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile (filename, *cloud);
        std::cout << "cloud points : " << cloud->points.size() << std::endl;
        viewer_->updatePointCloud(cloud, "Cloud");

        std::cout << "Key : " << event.getKeyCode() << std::endl;
      }
    }
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
    return 0;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  string dir_;
  vector<string> files_;
};

int main( int argc, char* argv[] )
{
  Viewer viewer;
  viewer.run();

  return 0;
}
