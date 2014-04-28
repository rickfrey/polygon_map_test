//#include "PointCloudMatcher.h"// ????

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h> // Um die Punktewolke zu visualisieren! (funktioniert noch nicht)

#include <iostream>                         // Speicherung in pcd-file
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>                  // "        "
#include <pcl/point_types.h>                // "        "

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
//Test: für Eigen::Vector3f
#include <Eigen/Core>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>
#include <eigen3/Eigen/StdVector>
typedef pcl::PointCloud<pcl::PointXYZRGB> cloud;


int main(int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    pcl::PCDWriter writer;
    reader.read ("Arena_0,05.pcd", *cloud_filtered);

    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.05);

    //Eigen::
    //seg.setAxis(Eigen::Vector3f(-0.1792,-0.082,0.9804));//für perpendicular
    seg.setEpsAngle(0.2);//für perpendicular
    //seg.setAxis();

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    //while (cloud_filtered->points.size () > 0.3 * nr_points)
    for(i=0;i<4;i++)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }
//if (inliers->indices.size()>40000)

      // Extract the inliers
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      extract.filter (*cloud_p);
      std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

      std::stringstream ss;
      ss << "Arena_plane_" << i << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

      // Create the filtering object
      extract.setNegative (true);
      extract.filter (*cloud_f);
      cloud_filtered.swap (cloud_f);
      //i++;
      std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                          << coefficients->values[1] << " "
                                          << coefficients->values[2] << " "
                                          << coefficients->values[3] << std::endl;
    }
/*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
*    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_planar (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
*
      if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("Arena_0,05.pcd", *cloud) == -1) //* load the file
      {
*        PCL_ERROR ("Couldn't read file XY.pcd \n");
        return (-1);
      }
*      std::cout << "Loaded "
                << cloud->width * cloud->height
                << " data points from test_pcd.pcd with the following fields: "
*                << std::endl;

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
*        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
*        pcl::PCDWriter writer;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
*        seg.setMaxIterations(1000);
        seg.setDistanceThreshold (0.01);//ursprünglich:0,01
        //seg.setInputCloud (cloud);
        //seg.segment (*inliers, *coefficients);

        // Inliers extrahieren:
        pcl::ExtractIndices<pcl::PointXYZRGB> extracter;
*        int i=0, nr_points= (int) cloud->points.size();
        while(cloud->points.size()>0.3*nr_points)
        {
        seg.setInputCloud(cloud);
        seg.segment(*inliers,*coefficients);
        if(inliers->indices.size()==0)
        {
*            std::cerr<<"Could not estimate a planar model for the given dataset."<<endl;
            break;
        }

        extracter.setInputCloud(cloud);
        extracter.setIndices(inliers);
        extracter.setNegative(false);//nur inliers
*        extracter.filter(*cloud_planar);
        std::stringstream ss;
        ss<<"Arena_plane_"<<i<<".pcd";
        writer.write<pcl::PointXYZRGB> (ss.str(),*cloud_planar,false);
*        extracter.setNegative(true);//alle außer inliers
        extracter.filter(*cloud_f);
*        cloud_planar.swap(cloud_f);
*        i=i+1;
*        }*/
/*
        if (inliers->indices.size () == 0)
        {
          PCL_ERROR ("Could not estimate a planar model for the given dataset.");
          return (-1);
        }

        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                                            << coefficients->values[1] << " "
                                            << coefficients->values[2] << " "
                                            << coefficients->values[3] << std::endl;
*/
/***** Auflistung aller Inliers
        std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
        for (size_t i = 0; i < inliers->indices.size (); ++i)
          std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
                                                     << cloud->points[inliers->indices[i]].y << " "
                                                     << cloud->points[inliers->indices[i]].z << std::endl;

*/
/*        // PROJECT THE MODEL INLIERS
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;// Was passiert hier????? Neues Objekt?? Was für eins?
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setIndices(inliers);
      proj.setInputCloud(cloud);
      proj.setModelCoefficients(coefficients);
      proj.filter(*cloud_projected);

      // Create a Concave/Convex Hull representation of the projected inliers

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ConcaveHull<pcl::PointXYZRGB> chull;// concave hull polygon
      //pcl::ConvexHull<pcl::PointXYZ> chull;// convex hull polygon
      chull.setInputCloud(cloud_projected);
      chull.setAlpha(0.05);// Ursprünglich: 0.1
      chull.reconstruct(*cloud_hull);
      //selbst geschrieben:
      //std::vector<pcl::Vertices> polygons; //(new std::vector<pcl::Vertices>);//dynamische Speicherallokation (warum???)
      //chull.performReconstruction(cloud_projected,polygons,true);
      //std::vector<pcl::Vertices> polygons;
      //chull.performReconstruction(cloud_hull,polygons,true);
*/
/*
      pcl::PCDWriter writer;
      writer.write("convex_hull_filled.pcd",*cloud_hull,false);
      //writer.write("concave_hull.pcd",*cloud_hull,false);
*/

      return (0);

}


