#include <offline/offline_utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>

namespace offline {

double parseStampFromPath(const boost::filesystem::path& path)
{
    try {
        return std::stod(path.stem().string());
    } catch (const std::exception&) {
        return 0.0;
    }
}

std::vector<PcdEntry> listPcdFilesSorted(const boost::filesystem::path& directory)
{
    if (!boost::filesystem::exists(directory) || !boost::filesystem::is_directory(directory)) {
        throw std::runtime_error("Invalid PCD directory: " + directory.string());
    }

    std::vector<PcdEntry> files;
    for (boost::filesystem::directory_iterator it(directory); it != boost::filesystem::directory_iterator(); ++it) {
        if (boost::filesystem::is_regular_file(it->path()) && it->path().extension() == ".pcd") {
            files.push_back({it->path(), parseStampFromPath(it->path())});
        }
    }

    std::sort(files.begin(), files.end(), [](const PcdEntry& lhs, const PcdEntry& rhs) {
        if (std::abs(lhs.stamp - rhs.stamp) < std::numeric_limits<double>::epsilon()) {
            return lhs.path.filename().string() < rhs.path.filename().string();
        }
        return lhs.stamp < rhs.stamp;
    });

    return files;
}

void ensureDirectory(const boost::filesystem::path& directory)
{
    if (!boost::filesystem::exists(directory)) {
        if (!boost::filesystem::create_directories(directory)) {
            throw std::runtime_error("Failed to create directory: " + directory.string());
        }
    }
    if (!boost::filesystem::is_directory(directory)) {
        throw std::runtime_error("Path is not a directory: " + directory.string());
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr loadCleanXyziCloud(const boost::filesystem::path& path, double stamp)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr input(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(path.string(), *input) != 0) {
        throw std::runtime_error("Failed to load PCD: " + path.string());
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr clean(new pcl::PointCloud<pcl::PointXYZI>);
    clean->reserve(input->size());
    for (const auto& point : input->points) {
        if (pcl::isFinite(point)) {
            clean->push_back(point);
        }
    }
    clean->width = static_cast<std::uint32_t>(clean->size());
    clean->height = 1;
    clean->is_dense = true;
    clean->header.stamp = static_cast<std::uint64_t>(stamp * 1.0e9);
    return clean;
}

PointCloudXYZI::Ptr toDetectorCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    PointCloudXYZI::Ptr detector(new PointCloudXYZI);
    detector->reserve(cloud.size());
    for (const auto& point : cloud.points) {
        if (!pcl::isFinite(point)) {
            continue;
        }
        PointType out;
        out.x = point.x;
        out.y = point.y;
        out.z = point.z;
        out.intensity = point.intensity;
        out.curvature = 0.0f;
        detector->push_back(out);
    }
    detector->width = static_cast<std::uint32_t>(detector->size());
    detector->height = 1;
    detector->is_dense = true;
    detector->header = cloud.header;
    return detector;
}

PointCloudXYZI transformDetectorCloud(const PointCloudXYZI& cloud,
                                      const Eigen::Vector3f& translation,
                                      const Eigen::Quaternionf& rotation)
{
    const Eigen::Matrix3f rot = rotation.normalized().toRotationMatrix();
    PointCloudXYZI transformed;
    transformed.reserve(cloud.size());
    for (const auto& point : cloud.points) {
        PointType out = point;
        const Eigen::Vector3f xyz = rot * Eigen::Vector3f(point.x, point.y, point.z) + translation;
        out.x = xyz.x();
        out.y = xyz.y();
        out.z = xyz.z();
        transformed.push_back(out);
    }
    transformed.width = static_cast<std::uint32_t>(transformed.size());
    transformed.height = 1;
    transformed.is_dense = cloud.is_dense;
    transformed.header = cloud.header;
    return transformed;
}

}  // namespace offline
