#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <boost/thread/thread.hpp>

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> 
rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void delete_plane(  std::vector<int>* inliers, 
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    int sz_inliers; sz_inliers = inliers->size();
    int sz_cloud;   sz_cloud = cloud->size();

    if(sz_inliers == 0) return;

    std::sort(inliers->begin(), inliers->end());

    int ind_cloud = 0;
    int ind_inliers=1; 
    int inlier = (*inliers)[0];

    for(int i=0; i<sz_cloud; i++){
        if(i == inlier){
            if(ind_inliers == sz_inliers) continue;
            
            inlier = (*inliers)[ind_inliers++];
            continue;
        }
        
        cloud->points[ind_cloud++] = cloud->points[i];
    }
    
    cloud->points.erase(cloud->points.begin() + (sz_cloud - sz_inliers), cloud->points.end());
}

void RANSAC_plane(std::vector<int>& inliers, 
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double threshold){
  uint8_t r(255), g(15), b(0); int k;
  inliers.clear();

  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud));

  pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac (model_p);
  ransac.setDistanceThreshold (threshold);
  ransac.computeModel();
  ransac.getInliers(inliers);

  uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  for(int i=0; i<(int)inliers.size(); i++){
    k = inliers[i];
    cloud->points[k].rgb = *reinterpret_cast<float*>(&rgb);
  } 
}

void print_inliers(std::vector<int>& inliers, 
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, FILE *F){
    int sz_in = inliers.size(); int inlier;

    for(int i=0; i<sz_in; i++){
    	inlier = inliers[i];
        fprintf(F, "%f %f\n", cloud->points[inlier].x, cloud->points[inlier].y);
    }
}

int main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);

  // populate our PointCloud with points
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  
  int Npt;
  scanf("%d", &Npt);
  cloud->width    = Npt;
  cloud->points.resize (cloud->width * cloud->height);

  float x, y, z;


  for(int i=0; i<Npt; i++){
      scanf("%f %f %f", &x, &y, &z);
      cloud->points[i].x = x;
      cloud->points[i].y = y;
      cloud->points[i].z = z; 

      uint8_t r(255-z), g(255-z), b(255-z);
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);
   }
  

  int times =1;  double threshold = 15.0;

  std::vector<int> inliers;
  std::string namef = "threshold15_it0.txt";
  
  while(times-- > 0){
      FILE *F = fopen(namef.c_str(), "w");

      RANSAC_plane(inliers, cloud, threshold);
      print_inliers(inliers, cloud, F);
      delete_plane(&inliers, cloud);
      
      
      namef[14] = (char)((int)namef[14] +1);

      fclose(F);
      if(inliers.size() < 1000) return 0;
  }
  
  RANSAC_plane(inliers, cloud, threshold);


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  viewer = rgbVis(cloud);
  
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return 0;

 }
