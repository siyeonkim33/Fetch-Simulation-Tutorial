#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl_ros/transforms.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <sstream>
#include <stdlib.h>

using namespace std;

class CylinderSegment
{
  ros::NodeHandle nh_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  ros::Subscriber cloud_subscriber_;

public:
  CylinderSegment()
    : cloud_subscriber_(nh_.subscribe("/head_camera/depth_registered/points", 1, &CylinderSegment::cloudCB, this))
  {
  }

  /** \brief Given the parameters of the cylinder add the cylinder to the planning scene. */

  void addCylinder(int obj_num)
  {
    stringstream ssInt;
	  ssInt << obj_num;

    // BEGIN_SUB_TUTORIAL add_cylinder
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "head_camera_rgb_optical_frame";
    collision_object.id = "box"+ ssInt.str();

    // Define a cylinder which will be added to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    ///* Setting height of cylinder. 
    primitive.dimensions[0] = bbox_params.height;
    ///* Setting radius of cylinder. 
    primitive.dimensions[1] = bbox_params.width;
    primitive.dimensions[2] = bbox_params.depth;


    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose bbox_pose;
    // Computing and setting quaternion from axis angle representation. 
    Eigen::Vector3d cylinder_z_direction(bbox_params.direction_vec[0], bbox_params.direction_vec[1],
                                         bbox_params.direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 1., 0.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    bbox_pose.orientation.x = axis.x() * sin(angle / 2);
    bbox_pose.orientation.y = axis.y() * sin(angle / 2);
    bbox_pose.orientation.z = axis.z() * sin(angle / 2);
    bbox_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    bbox_pose.position.x = bbox_params.center_pt[0];
    bbox_pose.position.y = bbox_params.center_pt[1];
    bbox_pose.position.z = bbox_params.center_pt[2];

    // Add cylinder as collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(bbox_pose);
    collision_object.operation = collision_object.ADD;
    planning_scene_interface_.applyCollisionObject(collision_object);
    // END_SUB_TUTORIAL
  }
  

    /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;  // define the "Passthrough filter"

    pass.setInputCloud(cloud);                // Set input Cloud as "cloud"

    // Assign axis and range to the passthrough filter object.
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 1.0);           // min & max values in z axis to keep
    pass.filter(*cloud);                      // apply the filter

    // Restrict the range in xy-axis (to exclude collision objects)
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.3, 0.05);           // min & max values in y axis to keep
    pass.filter(*cloud);


    // // Restrict the range in x-axis (to exclude collision objects)
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(0, 0.3);           // min & max values in z axis to keep
    // pass.filter(*cloud);
  }

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
      @param cloud - Pointcloud.
      @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }

  // ===================== Extracting outliers' normal vectors?????????
  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);    // exclude inliers and include outliers
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
  }

  // =====================

  /** \brief Given the pointcloud and indices of the plane, remove the planar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
  void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentor;
    
    segmentor.setOptimizeCoefficients(true);
    // mandatory
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);   // clustering distance threshold (determine how close a point must be to the model in order to be considered an inlier)

    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(1000);
    segmentor.setInputCloud(cloud);
    
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);

    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);    // endow index to each point

    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative(true);            // extract objects rather the plane
    extract_indices.filter(*cloud);

  }

  /** \brief Given the pointcloud containing just the cylinder,
      compute its center point and its height and store in bbox_params.
      @param cloud - point cloud containing just the cylinder. */
  void extractBbox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
  {
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloud, minPoint, maxPoint);

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    pcl::PointXYZRGB min_point_OBB;
    pcl::PointXYZRGB max_point_OBB;
    pcl::PointXYZRGB position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    cout << position_OBB << endl;
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    Eigen::Vector3f position (bbox_params.center_pt[0],bbox_params.center_pt[1],bbox_params.center_pt[2]);

    bbox_params.center_pt[0] = (minPoint.x + maxPoint.x)/2;
    bbox_params.center_pt[1] = (minPoint.y + maxPoint.y)/2;
    bbox_params.center_pt[2] = (minPoint.z + maxPoint.z)/2;

    bbox_params.height = max_point_OBB.y - min_point_OBB.y;
    bbox_params.depth = max_point_OBB.z - min_point_OBB.z;
    bbox_params.width = max_point_OBB.x - min_point_OBB.x;


    pcl::PCA<pcl::PointXYZRGB> pca(true);
    pca.setInputCloud (cloud);
    Eigen::Matrix3f eigen_vector = pca.getEigenVectors(); // returns a matrix where the columns are the axis of your bounding box    
    Eigen::Vector3f direction = eigen_vector.col(0);

    bbox_params.direction_vec[0] = direction.x();
    bbox_params.direction_vec[1] = direction.y();
    bbox_params.direction_vec[2] = direction.z();

    // // END_SUB_TUTORIAL
  }

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // BEGIN_SUB_TUTORIAL callback
 
    // 0) we convert from sensor_msgs to pcl::PointXYZ which is needed for most of the processing.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr interm(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_clouds(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    // 1) "Passthrough filter" : Use a passthrough filter to get the region of interest. - The filter removes points outside the specified range.
    passThroughFilter(cloud);

    // 2) "Normal Vector" : Compute point normals for later sample consensus step.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);     // calculate the normal vector of the planes via k neighborhood method

    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

    // 3) "Plane removal" : Detect and remove points on the (planar) surface on which the cube is resting.
    removePlaneSurface(cloud, inliers_plane);

    /* UNNECESSARY */
    // 4) "Normal vector extraction of outliers": Remove surface points from normals as well
    //extractNormals(cloud_normals, inliers_plane); 

    // ModelCoefficients will hold the parameters using which we can define a cube of infinite length.
    //pcl::ModelCoefficients::Ptr coefficients_cube(new pcl::ModelCoefficients);
    //extractCylinder(cloud, coefficients_cube, cloud_normals);

    // ******************************************************** CLUSTER ****************************************************
    // 5) Extract the objects using Euclidean cluster extraction
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGB>::Ptr>> cloud_array;
    
    // clustering(cloud, cluster_out);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> Eucluster;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    std::vector<pcl::PointIndices> cluster_indices; 

    Eucluster.setClusterTolerance (0.02);
    Eucluster.setMinClusterSize (50);
    Eucluster.setMaxClusterSize (25000);

    // set KTree object for the search method of the extraction
    tree->setInputCloud(cloud);
    Eucluster.setSearchMethod(tree);
    Eucluster.setInputCloud (cloud);
    Eucluster.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        pcl::PointXYZRGB pt = cloud->points[*pit];

        (*temp).push_back(pt); 
      }
      cloud_array.push_back(*temp);
    }

    // ******************************************************** CLUSTER ****************************************************

    pcl::io::savePCDFile("array.pcd",cloud_array[1]);

    ROS_INFO("Detected object - Adding CollisionObject to PlanningScene");


    for (int i =0; i <cloud_array.size(); i++){
      cout << i << "th object" << endl;
      *interm = cloud_array[i];

      // Compute the center point of the cylinder using standard geometry
      extractBbox(interm);  
      addCylinder(i);
    }
  }

private:
  // BEGIN_SUB_TUTORIAL param_struct
  // There are 4 fields and a total of 7 parameters used to define this.
  struct AddBboxParams
  {    
    /* Depth of the bbox. */
    double depth;
    /* Width of the bbox. */
    double width;
    /* Direction vector along the z-axis of the bbox. */
    double direction_vec[3];
    /* Center point of the bbox. */
    double center_pt[3];
    /* Height of the bbox. */
    double height;
  };
  // Declare a variable of type AddBboxParams
  AddBboxParams bbox_params;
  // END_SUB_TUTORIAL
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segment");

  // Start the segmentor
  CylinderSegment segmentor;

  // Spin
  ros::spin();
}

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);  
// pcl::io::savePCDFile("before.pcd", *cloud);
// pcl::io::savePCDFile("passthrough.pcd", *cloud_filtered);

/*
  void extractCylinder(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
                       const pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                       const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.1);
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(1000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.008);
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0.01, 0.1);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  
  */

    // cout << cloud_array.size() << endl;

    // for (int i=0; i < cloud_array.size(); i++){
    //   cout << cloud_array[i].size()<<endl;
    // }



  // // clusterize all the objects
  // void clustering(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster_out)
  // {
  //   pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> Eucluster;
  //   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  //   std::vector<pcl::PointIndices> cluster_indices; 

  //   Eucluster.setClusterTolerance (0.02);
  //   Eucluster.setMinClusterSize (50);
  //   Eucluster.setMaxClusterSize (25000);

  //   // set KTree object for the search method of the extraction
  //   tree->setInputCloud(cloud);
  //   Eucluster.setSearchMethod(tree);
  //   Eucluster.setInputCloud (cloud);
  //   Eucluster.extract(cluster_indices);


  //   for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
  //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);

  //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
  //       pcl::PointXYZRGB pt = cloud->points[*pit];

  //       (*temp).push_back(pt); 
  //     }
  //     array.push_back(*temp);
  //   }
  //   cout << array.size() << endl;
  //   cout << array[0].size()<<endl;
// 
// }

    //std::cout << "direction" << coefficients_cube->values[3] << std::endl;
    //std::cout << "direction" << coefficients_cube->values[4] << std::endl;
    //std::cout << "direction" << coefficients_cube->values[5] << std::endl;
    // /* Store direction vector of z-axis of cylinder. */
    // bbox_params.direction_vec[0] = coefficients_cylinder->values[3];
    // bbox_params.direction_vec[1] = coefficients_cylinder->values[4];
    // bbox_params.direction_vec[2] = coefficients_cylinder->values[5];