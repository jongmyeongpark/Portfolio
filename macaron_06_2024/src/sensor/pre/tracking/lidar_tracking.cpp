#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/print.h>
#include <pcl/common/common.h>

#include "macaron_06/lidar_info.h"

#include <chrono>
using namespace std::chrono;
using namespace std;

union Vector3d
{
  struct { float x; float y; float z; };
  struct { float i; float j; float k; };
  struct { float a; float b; float c; };
};

union Vector4d
{
  struct { float x; float y; float z; float intensity; };
  struct { float i; float j; float k; float w; };
  struct { float a; float b; float c; float d; };
};

struct SizeLimit
{
  Vector3d maxSize;
  Vector3d minSize;
};

enum class EMode
{
  CONE,
  PERSON,
  DELIVERY,
  PROCESSED,
  MINJAE,
  CENTROID,
};

ros::Publisher clusterPub;
ros::Publisher centriodPub;
ros::Publisher processedPub;
ros::Publisher processedPubMinjae;
bool mode[6] = { 0, };
SizeLimit objectSizeLimit = { 2.0f, 2.0f, 3.0f, 0.0f, 0.0f, 0.0f };
SizeLimit coneSizeLimit = { 0.7f, 0.7f, 1.0f, 0.0f, 0.0f, 0.0f };
SizeLimit personSizeLimit = { 0.7f, 0.7f, 2.5f, 0.07f, 0.07f, 0.07f };
SizeLimit deliverySizeLimit = { 1.0f, 1.0f, 3.0f, 0.0f, 0.0f, 0.0f };;



// utils ////////////////////////////////////

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(cloudmsg, *cloud);
  return cloud;
}

sensor_msgs::PointCloud2 cloud2cloudmsg(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, ros::Time stamp)
{
  sensor_msgs::PointCloud2 cloudmsg;
  pcl::toROSMsg(*cloud, cloudmsg);
  cloudmsg.header.frame_id = "velodyne";
  cloudmsg.header.stamp = stamp;
  return cloudmsg;
}

macaron_06::lidar_info cloud2msg(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
std::vector<pcl::PointIndices> clusterIndices, ros::Time stamp)
{
  macaron_06::lidar_info msg;

  msg.data = cloud2cloudmsg(cloud, stamp);
  msg.header.stamp = stamp;

  std::vector<int32_t> clusters;
  std::vector<uint32_t> clusterSize;
  clusterSize.reserve(clusters.size());

  for (const auto& clusterIndice : clusterIndices)
  {
    std::vector<int> cluster = clusterIndice.indices;
    clusterSize.push_back(cluster.size());
    clusters.insert(clusters.end(), cluster.begin(), cluster.end());
  }

  msg.clusters = clusters;
  msg.clusterSize = clusterSize;

  return msg;
}

////////////////////////////////////////////////

void SetROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
  pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
  const std::string& fieldName,
  const float min,
  const float max)
{
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(cloudIn);
  pass.setFilterFieldName(fieldName);
  pass.setFilterLimits(min, max);
  pass.filter(**cloudOut);
}

void DeleteCenter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
  pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
  const float frontX = 0.5f, const float backX = 2.0f,
  const float frontY = 0.7f, const float backY = 0.7f,
  const float frontZ = 1.0f, const float backZ = 0.5f)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>());

  for (pcl::PointXYZI point : cloudIn->points)
  {
    if (point.x < -backX || point.x > frontX || point.y < -backY || point.y > frontY || point.z < -backZ || point.z > frontZ)
    {
      ret->points.push_back(point);
    }
  }

  *cloudOut = ret;
}

void Voxelize(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
  pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
  const float voxelSize)
{
  pcl::VoxelGrid<pcl::PointXYZI> vg;
  vg.setInputCloud(cloudIn);
  vg.setLeafSize(voxelSize, voxelSize, voxelSize);
  vg.filter(**cloudOut);
}

bool RansacPerpendicularZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
const int maxIteration,
const float distanceThreshold)
{
  constexpr float epsAngle = 0.1;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIteration);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setEpsAngle(epsAngle);
  seg.setAxis(Eigen::Vector3f::UnitZ());

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  seg.setInputCloud(cloudIn);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    return false;
  }

  extract.setInputCloud(cloudIn);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(**cloudOut);

  return true;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Ransac(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*cloud, *cloudCopy);

  DeleteCenter(cloudCopy, &cloudCopy);
  SetROI(cloudCopy, &cloudCopy, "y", -4.0f, 4.0f);
  SetROI(cloudCopy, &cloudCopy, "x", -3.0f, 10.0f);
  
  // Voxelize(cloudCopy, &cloudCopy, 0.25f);

  RansacPerpendicularZ(cloudCopy, &cloudCopy, 2000, 0.07f);

  return cloudCopy;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr RansacMinjae(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*cloud, *cloudCopy);

  DeleteCenter(cloudCopy, &cloudCopy);
  SetROI(cloudCopy, &cloudCopy, "x", -0.5f, 2.5f);
  SetROI(cloudCopy, &cloudCopy, "y", -1.0f, 1.0f);
  SetROI(cloudCopy, &cloudCopy, "z", -0.8f, 2.0f);
  
  Voxelize(cloudCopy, &cloudCopy, 0.25f);

  RansacPerpendicularZ(cloudCopy, &cloudCopy, 2000, 0.07f);

  return cloudCopy;
}

std::vector<pcl::PointIndices> Cluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setInputCloud(cloud); 
  ec.setClusterTolerance(0.4f);
  ec.setMinClusterSize(5);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.extract(cluster_indices);

  return cluster_indices;
}

bool IsObject(float xLength, float yLength, float zLength, const SizeLimit& sizeLimit)
{
  float ratioThreshold = 5.0f;
  float lengths[] = {xLength, yLength, zLength};
  std::sort(lengths, lengths + 3, std::greater<double>());

  // if (lengths[0] >= ratioThreshold * lengths[1] && lengths[0] >= ratioThreshold * lengths[2])
  // {
  //   return false;
  // }

  if (xLength > sizeLimit.maxSize.x || xLength < sizeLimit.minSize.x)
  {
    return false;
  }
  if (yLength > sizeLimit.maxSize.y || yLength < sizeLimit.minSize.y)
  {
    return false;
  }
  if (zLength > sizeLimit.maxSize.z || zLength < sizeLimit.minSize.z)
  {
    return false;
  }

  return true;
}

std::vector<int> FindObjectWithSize(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn,
                                                  pcl::PointCloud<pcl::PointXYZI>::Ptr* cloudOut,
                                                  const std::vector<pcl::PointIndices>& clusterIndices,
                                                  const SizeLimit& sizeLimit)
{
    std::vector<int> result;
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    for (int i = 0; i < clusterIndices.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud2(new pcl::PointCloud<pcl::PointXYZ>());
        extract.setInputCloud(cloudIn);
        extract.setIndices(boost::make_shared<pcl::PointIndices>(clusterIndices[i]));
        extract.setNegative(false);
        extract.filter(*clusterCloud);

        pcl::copyPointCloud(*clusterCloud, *clusterCloud2);
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*clusterCloud2, minPoint, maxPoint);
        float xLength = maxPoint.x - minPoint.x;
        float yLength = maxPoint.y - minPoint.y;
        float zLength = maxPoint.z - minPoint.z;

        if (IsObject(xLength, yLength, zLength, sizeLimit))
        {
            result.push_back(i);
        }
    }

    for (const int idx : result)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZI>());
        extract.setInputCloud(cloudIn);
        extract.setIndices(boost::make_shared<pcl::PointIndices>(clusterIndices[idx]));
        extract.setNegative(false);
        extract.filter(*tempCloud);
        **cloudOut += *tempCloud;
    }

    return result;
}

void callbackLidar(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // ros::Time stamp = (*input).header.stamp;
     ros::Time stamp = ros::Time::now();
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(cloudmsg2cloud(*input));

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ransacResult = Ransac(cloud);
  std::vector<pcl::PointIndices> clusterResult = Cluster(ransacResult);
  std::vector<int> objectIndiceses = FindObjectWithSize(ransacResult, &cloudOut, clusterResult, objectSizeLimit);
  std::vector<pcl::PointIndices> objectIndices;
  for (const int i : objectIndiceses)
  {
    objectIndices.push_back(clusterResult[i]);
  }

  if (mode[static_cast<int>(EMode::MINJAE)])
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutMinjae(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ransacResultMinjae = RansacMinjae(cloud);
    std::vector<pcl::PointIndices> clusterResultMinjae = Cluster(ransacResultMinjae);
    FindObjectWithSize(ransacResultMinjae, &cloudOutMinjae, clusterResultMinjae, objectSizeLimit);

    sensor_msgs::PointCloud2 _m_processedPointsMinjae;
    _m_processedPointsMinjae = cloud2cloudmsg(cloudOutMinjae, stamp);
    processedPubMinjae.publish(_m_processedPointsMinjae);
  }

  if (mode[static_cast<int>(EMode::CENTROID)])
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr centroidPointCloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI temp;
    for (int i = 0; i < objectIndices.size(); ++i)
    {
      for (int j = 0; j < objectIndices[i].indices.size(); ++j)
      {
        pcl::PointXYZI point = ransacResult->points[objectIndices[i].indices[j]];
        temp.x += point.x;
        temp.y += point.y;
        temp.z += point.z;
      }
      temp.x /= objectIndices[i].indices.size();
      temp.y /= objectIndices[i].indices.size();
      temp.z /= objectIndices[i].indices.size();

      temp.intensity = 0.0f;

      centroidPointCloud->points.push_back(temp);
    }

    sensor_msgs::PointCloud2 _m_centroidPoints;
    _m_centroidPoints = cloud2cloudmsg(centroidPointCloud, stamp);
    centriodPub.publish(_m_centroidPoints);
  }

  macaron_06::lidar_info _m_cluster;
  _m_cluster = cloud2msg(ransacResult, objectIndices, stamp);
  if (mode[static_cast<int>(EMode::CONE)])
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr conesCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> coneIndices = FindObjectWithSize(ransacResult, &conesCloud, objectIndices, coneSizeLimit);
    _m_cluster.cones = coneIndices;
  }
  if (mode[static_cast<int>(EMode::PERSON)])
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr peopleCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> personIndices = FindObjectWithSize(ransacResult, &peopleCloud, objectIndices, personSizeLimit);
    _m_cluster.person = personIndices;
  }
  if (mode[static_cast<int>(EMode::DELIVERY)])
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr deliveryCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<int> deliveryIndices = FindObjectWithSize(ransacResult, &deliveryCloud, objectIndices, deliverySizeLimit);
    _m_cluster.delivery = deliveryIndices;
  }
  clusterPub.publish(_m_cluster);

  if (mode[static_cast<int>(EMode::PROCESSED)])
  {
    sensor_msgs::PointCloud2 _m_processedPoints;
    _m_processedPoints = cloud2cloudmsg(cloudOut, stamp);
    processedPub.publish(_m_processedPoints);
  }
}

int main(int argc, char** argv)
{
  freopen("/dev/null", "w", stderr);
  ros::init(argc, argv, "lidar_manager");
  ros::NodeHandle nh;

  std::string modeList[6] = { "cone", "person", "delivery", "processed", "minjae", "centroid" };

  if (argc >= 2)
  {
    std::string modes = argv[1];
    std::string buffer;
    istringstream ss(modes);
    while (getline(ss, buffer, ','))
    {
      for (int i = 0; i < sizeof(mode) / sizeof(bool); ++i)
      {
        if (buffer.compare(modeList[i]) == 0)
        {
          mode[i] = true;
          break;
        }
      }
    }
    cout << "use mode: ";
    for (int i = 0; i < sizeof(mode) / sizeof(bool); ++i)
    {
      if (mode[i])
      {
        cout << modeList[i] << " ";
      }
    }
    cout << endl;
  }
  else
  {
    cout << "Please Select Mode" << endl;

    cout << "List of Mode: ";
    for (int i = 0; i < sizeof(mode) / sizeof(bool); ++i)
    {
      cout << "[" << modeList[i] << "] ";
    }
    cout << endl;
    cout << "ex) cone,person,processed" << endl;

    return 1;
  }

  ros::Subscriber sub = nh.subscribe("/velodyne_points", 10, callbackLidar);

  clusterPub = nh.advertise<macaron_06::lidar_info>("cluster", 10);
  processedPub = nh.advertise<sensor_msgs::PointCloud2>("processed", 10);
  processedPubMinjae = nh.advertise<sensor_msgs::PointCloud2>("super_minjae_processed", 10);
  centriodPub = nh.advertise<sensor_msgs::PointCloud2>("centroid", 10);

  ros::spin();
}
