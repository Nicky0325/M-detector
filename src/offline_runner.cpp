#include <m-detector/DynObjConfig.h>
#include <m-detector/DynObjFilter.h>
#include <offline/offline_utils.h>

#include "lidar_odometry.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>

namespace fs = boost::filesystem;

namespace {

struct CliOptions {
    fs::path data_dir{"data/5069_sequential"};
    fs::path output_dir{"output/5069_sequential"};
    fs::path config_path{"config/offline/aiv_at128.yaml"};
    std::vector<std::string> lidars{"lidar_frf_at128", "lidar_blb_at128"};
    int max_frames{-1};
};

std::vector<std::string> splitCommaList(const std::string& value)
{
    std::vector<std::string> items;
    std::stringstream ss(value);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) {
            items.push_back(item);
        }
    }
    return items;
}

CliOptions parseArgs(int argc, char** argv)
{
    CliOptions options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        auto requireValue = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + name);
            }
            return argv[++i];
        };

        if (arg == "--data") {
            options.data_dir = requireValue(arg);
        } else if (arg == "--output") {
            options.output_dir = requireValue(arg);
        } else if (arg == "--config") {
            options.config_path = requireValue(arg);
        } else if (arg == "--lidars") {
            options.lidars = splitCommaList(requireValue(arg));
        } else if (arg == "--max-frames") {
            options.max_frames = std::stoi(requireValue(arg));
        } else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: m_detector_offline [--data <sequence_dir>] [--lidars a,b] "
                << "[--config <yaml>] [--output <dir>] [--max-frames N]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    if (options.lidars.empty()) {
        throw std::runtime_error("At least one lidar name must be provided");
    }
    return options;
}

const PointCloudXYZI& dynamicCloudFor(const DynObjFilter& filter)
{
    if (filter.cluster_coupled || filter.cluster_future) {
        return *filter.laserCloudDynObj_clus;
    }
    return *filter.laserCloudDynObj_world;
}

const PointCloudXYZI& staticCloudFor(const DynObjFilter& filter)
{
    if (filter.cluster_coupled || filter.cluster_future) {
        return *filter.laserCloudSteadObj_clus;
    }
    return *filter.laserCloudSteadObj;
}

const PointCloudXYZI& dynamicLocalCloudFor(const DynObjFilter& filter)
{
    return *filter.laserCloudDynObj_local_final;
}

const PointCloudXYZI& staticLocalCloudFor(const DynObjFilter& filter)
{
    return *filter.laserCloudSteadObj_local_final;
}

void saveCloud(const fs::path& path, const PointCloudXYZI& cloud)
{
    if (cloud.empty()) {
        std::ofstream out(path.string());
        if (!out.is_open()) {
            throw std::runtime_error("Failed to write empty PCD: " + path.string());
        }
        out << "# .PCD v0.7 - Point Cloud Data file format\n"
            << "VERSION 0.7\n"
            << "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
            << "SIZE 4 4 4 4 4 4 4 4\n"
            << "TYPE F F F F F F F F\n"
            << "COUNT 1 1 1 1 1 1 1 1\n"
            << "WIDTH 0\n"
            << "HEIGHT 1\n"
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS 0\n"
            << "DATA ascii\n";
        return;
    }

    PointCloudXYZI out = cloud;
    out.width = static_cast<std::uint32_t>(out.size());
    out.height = 1;
    out.is_dense = false;
    if (pcl::io::savePCDFileBinary(path.string(), out) != 0) {
        throw std::runtime_error("Failed to write PCD: " + path.string());
    }
}

void processLidar(const CliOptions& options, const std::string& lidar_name, const DynObjConfig& config)
{
    const fs::path lidar_dir = options.data_dir / "lidar" / lidar_name;
    std::vector<offline::PcdEntry> frames = offline::listPcdFilesSorted(lidar_dir);
    if (options.max_frames > 0 && static_cast<std::size_t>(options.max_frames) < frames.size()) {
        frames.resize(static_cast<std::size_t>(options.max_frames));
    }
    if (frames.empty()) {
        throw std::runtime_error("No PCD frames found in " + lidar_dir.string());
    }

    const fs::path lidar_out = options.output_dir / lidar_name;
    const fs::path odom_out = lidar_out / "odometry";
    const fs::path dynamic_out = lidar_out / "dynamic_frames";
    const fs::path static_out = lidar_out / "static_frames";
    const fs::path dynamic_transformed_out = lidar_out / "dynamic_object_transformed";
    const fs::path static_transformed_out = lidar_out / "static_object_transformed";
    offline::ensureDirectory(odom_out);
    offline::ensureDirectory(dynamic_out);
    offline::ensureDirectory(static_out);
    offline::ensureDirectory(dynamic_transformed_out);
    offline::ensureDirectory(static_transformed_out);

    std::ofstream summary((lidar_out / "summary.csv").string());
    if (!summary.is_open()) {
        throw std::runtime_error("Failed to open summary file under " + lidar_out.string());
    }
    summary << "frame_id,file_name,timestamp,input_points,dynamic_points,static_points,"
            << "tx,ty,tz,qx,qy,qz,qw,total_time_sec\n";

    std::ofstream pose_out((odom_out / "local_pose.txt").string());
    if (!pose_out.is_open()) {
        throw std::runtime_error("Failed to open local_pose.txt under " + odom_out.string());
    }
    pose_out << "# frame_id file_name tx ty tz qx qy qz qw\n";
    pose_out << std::fixed << std::setprecision(9);

    DynObjFilter detector;
    detector.init(config);
    LidarOdometry odometry;
    PointCloudXYZI dynamic_merged;
    PointCloudXYZI dynamic_transformed_merged;
    PointCloudXYZI static_transformed_merged;

    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        std::cout << "[" << lidar_name << "] frame " << (i + 1) << "/" << frames.size()
                  << " " << frame.path.filename().string() << std::endl;

        auto clean_xyzi = offline::loadCleanXyziCloud(frame.path, frame.stamp);
        auto odom_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*clean_xyzi));
        odometry.addLidar(odom_cloud);

        const auto pose = odometry.getCurrentLocalPose();
        const Eigen::Vector3f& t = pose.first;
        const Eigen::Quaternionf& q = pose.second;
        const Eigen::Matrix3d rot = q.cast<double>().toRotationMatrix();
        const Eigen::Vector3d pos(t.x(), t.y(), t.z());

        auto detector_cloud = offline::toDetectorCloud(*clean_xyzi);
        detector.filter(detector_cloud, rot, pos, frame.stamp);

        const PointCloudXYZI& dyn = dynamicCloudFor(detector);
        const PointCloudXYZI& stat = staticCloudFor(detector);
        const PointCloudXYZI dyn_transformed = offline::transformDetectorCloud(dynamicLocalCloudFor(detector), t, q);
        const PointCloudXYZI stat_transformed = offline::transformDetectorCloud(staticLocalCloudFor(detector), t, q);
        dynamic_merged += dyn;
        dynamic_transformed_merged += dyn_transformed;
        static_transformed_merged += stat_transformed;

        const std::string stem = frame.path.stem().string();
        saveCloud(dynamic_out / (stem + ".pcd"), dyn);
        saveCloud(static_out / (stem + ".pcd"), stat);
        saveCloud(dynamic_transformed_out / (stem + ".pcd"), dyn_transformed);
        saveCloud(static_transformed_out / (stem + ".pcd"), stat_transformed);

        pose_out << i << " " << frame.path.filename().string() << " "
                 << t.x() << " " << t.y() << " " << t.z() << " "
                 << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";

        summary << i << "," << frame.path.filename().string() << ","
                << std::fixed << std::setprecision(6) << frame.stamp << ","
                << clean_xyzi->size() << "," << dyn.size() << "," << stat.size() << ","
                << std::setprecision(9)
                << t.x() << "," << t.y() << "," << t.z() << ","
                << q.x() << "," << q.y() << "," << q.z() << "," << q.w() << ","
                << detector.time_total << "\n";
    }

    saveCloud(lidar_out / "dynamic_merged.pcd", dynamic_merged);
    saveCloud(dynamic_transformed_out / "merged.pcd", dynamic_transformed_merged);
    saveCloud(static_transformed_out / "merged.pcd", static_transformed_merged);
    odometry.saveLocalMap(odom_out.string());
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        const CliOptions options = parseArgs(argc, argv);
        const DynObjConfig config = DynObjConfig::FromYamlFile(options.config_path.string());
        offline::ensureDirectory(options.output_dir);

        for (const auto& lidar : options.lidars) {
            processLidar(options, lidar, config);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "m_detector_offline: " << e.what() << std::endl;
        return 1;
    }
}
