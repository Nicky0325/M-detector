#include <offline/offline_utils.h>
#include <m-detector/DynObjConfig.h>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>

namespace fs = boost::filesystem;

int main()
{
    const fs::path root = fs::temp_directory_path() / fs::unique_path("m_detector_tests_%%%%%%");
    offline::ensureDirectory(root);

    {
        std::ofstream(root / "1777611170.300560.pcd").close();
        std::ofstream(root / "1777611162.300340.pcd").close();
        std::ofstream(root / "not_a_cloud.txt").close();
        const auto files = offline::listPcdFilesSorted(root);
        assert(files.size() == 2);
        assert(files[0].path.filename().string() == "1777611162.300340.pcd");
        assert(files[1].path.filename().string() == "1777611170.300560.pcd");
        assert(std::abs(files[0].stamp - 1777611162.300340) < 1e-6);
    }

    {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI valid;
        valid.x = 1.0f;
        valid.y = 2.0f;
        valid.z = 3.0f;
        valid.intensity = 4.0f;
        pcl::PointXYZI invalid = valid;
        invalid.x = std::numeric_limits<float>::quiet_NaN();
        cloud.push_back(valid);
        cloud.push_back(invalid);

        const auto detector = offline::toDetectorCloud(cloud);
        assert(detector->size() == 1);
        assert(detector->points[0].x == 1.0f);
        assert(detector->points[0].intensity == 4.0f);
    }

    {
        PointCloudXYZI cloud;
        PointType point;
        point.x = 1.0f;
        point.y = 0.0f;
        point.z = 0.0f;
        point.intensity = 7.0f;
        point.normal_x = 1.0f;
        point.normal_y = 0.0f;
        point.normal_z = 0.0f;
        point.curvature = 11.0f;
        cloud.push_back(point);

        const float half_pi = static_cast<float>(std::acos(-1.0) * 0.5);
        const Eigen::Quaternionf q(Eigen::AngleAxisf(half_pi, Eigen::Vector3f::UnitZ()));
        const Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
        const PointCloudXYZI transformed = offline::transformDetectorCloud(cloud, t, q);

        assert(transformed.size() == 1);
        assert(std::abs(transformed.points[0].x - 1.0f) < 1e-5f);
        assert(std::abs(transformed.points[0].y - 3.0f) < 1e-5f);
        assert(std::abs(transformed.points[0].z - 3.0f) < 1e-5f);
        assert(std::abs(transformed.points[0].normal_x - 1.0f) < 1e-5f);
        assert(std::abs(transformed.points[0].normal_y) < 1e-5f);
        assert(transformed.points[0].curvature == 11.0f);
        assert(transformed.points[0].intensity == 7.0f);
    }

    {
        const fs::path cfg_path = root / "config.yaml";
        std::ofstream cfg(cfg_path.string());
        cfg << "dyn_obj:\n";
        cfg << "  points_num_perframe: 12345\n";
        cfg << "  cluster_coupled: true\n";
        cfg << "  hor_resolution_max: 0.012\n";
        cfg << "  ver_resolution_max: 0.034\n";
        cfg.close();

        const DynObjConfig parsed = DynObjConfig::FromYamlFile(cfg_path.string());
        assert(parsed.points_num_perframe == 12345);
        assert(parsed.cluster_coupled);
        assert(std::abs(parsed.hor_resolution_max - 0.012f) < 1e-6f);
        assert(std::abs(parsed.ver_resolution_max - 0.034f) < 1e-6f);
    }

    fs::remove_all(root);
    std::cout << "offline utility tests passed\n";
    return 0;
}
