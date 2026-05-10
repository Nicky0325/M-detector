#ifndef OFFLINE_UTILS_H
#define OFFLINE_UTILS_H

#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <types.h>

namespace offline {

struct PcdEntry {
    boost::filesystem::path path;
    double stamp{0.0};
};

double parseStampFromPath(const boost::filesystem::path& path);
std::vector<PcdEntry> listPcdFilesSorted(const boost::filesystem::path& directory);
void ensureDirectory(const boost::filesystem::path& directory);

pcl::PointCloud<pcl::PointXYZI>::Ptr loadCleanXyziCloud(const boost::filesystem::path& path, double stamp);
PointCloudXYZI::Ptr toDetectorCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);
PointCloudXYZI transformDetectorCloud(const PointCloudXYZI& cloud,
                                      const Eigen::Vector3f& translation,
                                      const Eigen::Quaternionf& rotation);

}  // namespace offline

#endif
