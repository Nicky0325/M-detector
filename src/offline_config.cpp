#include <m-detector/DynObjConfig.h>

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace {

template <typename T>
void readParam(const YAML::Node& node, const char* name, T& value)
{
    if (node[name]) {
        value = node[name].as<T>();
    }
}

}  // namespace

DynObjConfig DynObjConfig::FromYamlFile(const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node node = root["dyn_obj"] ? root["dyn_obj"] : root;
    if (!node || !node.IsMap()) {
        throw std::runtime_error("Config file must contain a dyn_obj map: " + path);
    }

    DynObjConfig cfg;

    readParam(node, "frame_id", cfg.frame_id);
    readParam(node, "time_file", cfg.time_file);
    readParam(node, "time_breakdown_file", cfg.time_breakdown_file);
    readParam(node, "cluster_out_file", cfg.cluster_out_file);
    readParam(node, "dataset", cfg.dataset);
    readParam(node, "buffer_delay", cfg.buffer_delay);
    readParam(node, "buffer_size", cfg.buffer_size);
    readParam(node, "points_num_perframe", cfg.points_num_perframe);
    readParam(node, "depth_map_dur", cfg.depth_map_dur);
    readParam(node, "max_depth_map_num", cfg.max_depth_map_num);
    readParam(node, "max_pixel_points", cfg.max_pixel_points);
    readParam(node, "frame_dur", cfg.frame_dur);
    readParam(node, "self_x_f", cfg.self_x_f);
    readParam(node, "self_x_b", cfg.self_x_b);
    readParam(node, "self_y_l", cfg.self_y_l);
    readParam(node, "self_y_r", cfg.self_y_r);
    readParam(node, "blind_dis", cfg.blind_dis);
    readParam(node, "fov_up", cfg.fov_up);
    readParam(node, "fov_down", cfg.fov_down);
    readParam(node, "fov_cut", cfg.fov_cut);
    readParam(node, "fov_left", cfg.fov_left);
    readParam(node, "fov_right", cfg.fov_right);
    readParam(node, "checkneighbor_range", cfg.checkneighbor_range);
    readParam(node, "stop_object_detect", cfg.stop_object_detect);
    readParam(node, "depth_thr1", cfg.depth_thr1);
    readParam(node, "enter_min_thr1", cfg.enter_min_thr1);
    readParam(node, "enter_max_thr1", cfg.enter_max_thr1);
    readParam(node, "map_cons_depth_thr1", cfg.map_cons_depth_thr1);
    readParam(node, "map_cons_hor_thr1", cfg.map_cons_hor_thr1);
    readParam(node, "map_cons_ver_thr1", cfg.map_cons_ver_thr1);
    readParam(node, "map_cons_hor_dis1", cfg.map_cons_hor_dis1);
    readParam(node, "map_cons_ver_dis1", cfg.map_cons_ver_dis1);
    readParam(node, "depth_cons_depth_thr1", cfg.depth_cons_depth_thr1);
    readParam(node, "depth_cons_depth_max_thr1", cfg.depth_cons_depth_max_thr1);
    readParam(node, "depth_cons_hor_thr1", cfg.depth_cons_hor_thr1);
    readParam(node, "depth_cons_ver_thr1", cfg.depth_cons_ver_thr1);
    readParam(node, "enlarge_z_thr1", cfg.enlarge_z_thr1);
    readParam(node, "enlarge_angle", cfg.enlarge_angle);
    readParam(node, "enlarge_depth", cfg.enlarge_depth);
    readParam(node, "occluded_map_thr1", cfg.occluded_map_thr1);
    readParam(node, "case1_interp_en", cfg.case1_interp_en);
    readParam(node, "k_depth_min_thr1", cfg.k_depth_min_thr1);
    readParam(node, "d_depth_min_thr1", cfg.d_depth_min_thr1);
    readParam(node, "k_depth_max_thr1", cfg.k_depth_max_thr1);
    readParam(node, "d_depth_max_thr1", cfg.d_depth_max_thr1);
    readParam(node, "v_min_thr2", cfg.v_min_thr2);
    readParam(node, "acc_thr2", cfg.acc_thr2);
    readParam(node, "map_cons_depth_thr2", cfg.map_cons_depth_thr2);
    readParam(node, "map_cons_hor_thr2", cfg.map_cons_hor_thr2);
    readParam(node, "map_cons_ver_thr2", cfg.map_cons_ver_thr2);
    readParam(node, "occ_depth_thr2", cfg.occ_depth_thr2);
    readParam(node, "occ_hor_thr2", cfg.occ_hor_thr2);
    readParam(node, "occ_ver_thr2", cfg.occ_ver_thr2);
    readParam(node, "depth_cons_depth_thr2", cfg.depth_cons_depth_thr2);
    readParam(node, "depth_cons_depth_max_thr2", cfg.depth_cons_depth_max_thr2);
    readParam(node, "depth_cons_hor_thr2", cfg.depth_cons_hor_thr2);
    readParam(node, "depth_cons_ver_thr2", cfg.depth_cons_ver_thr2);
    readParam(node, "k_depth2", cfg.k_depth2);
    readParam(node, "occluded_times_thr2", cfg.occluded_times_thr2);
    readParam(node, "case2_interp_en", cfg.case2_interp_en);
    readParam(node, "k_depth_max_thr2", cfg.k_depth_max_thr2);
    readParam(node, "d_depth_max_thr2", cfg.d_depth_max_thr2);
    readParam(node, "v_min_thr3", cfg.v_min_thr3);
    readParam(node, "acc_thr3", cfg.acc_thr3);
    readParam(node, "map_cons_depth_thr3", cfg.map_cons_depth_thr3);
    readParam(node, "map_cons_hor_thr3", cfg.map_cons_hor_thr3);
    readParam(node, "map_cons_ver_thr3", cfg.map_cons_ver_thr3);
    readParam(node, "occ_depth_thr3", cfg.occ_depth_thr3);
    readParam(node, "occ_hor_thr3", cfg.occ_hor_thr3);
    readParam(node, "occ_ver_thr3", cfg.occ_ver_thr3);
    readParam(node, "depth_cons_depth_thr3", cfg.depth_cons_depth_thr3);
    readParam(node, "depth_cons_depth_max_thr3", cfg.depth_cons_depth_max_thr3);
    readParam(node, "depth_cons_hor_thr3", cfg.depth_cons_hor_thr3);
    readParam(node, "depth_cons_ver_thr3", cfg.depth_cons_ver_thr3);
    readParam(node, "k_depth3", cfg.k_depth3);
    readParam(node, "occluding_times_thr3", cfg.occluding_times_thr3);
    readParam(node, "occluded_times_thr3", cfg.occluding_times_thr3);
    readParam(node, "case3_interp_en", cfg.case3_interp_en);
    readParam(node, "k_depth_max_thr3", cfg.k_depth_max_thr3);
    readParam(node, "d_depth_max_thr3", cfg.d_depth_max_thr3);
    readParam(node, "interp_hor_thr", cfg.interp_hor_thr);
    readParam(node, "interp_ver_thr", cfg.interp_ver_thr);
    readParam(node, "interp_thr1", cfg.interp_thr1);
    readParam(node, "interp_static_max", cfg.interp_static_max);
    readParam(node, "interp_start_depth1", cfg.interp_start_depth1);
    readParam(node, "interp_kp1", cfg.interp_kp1);
    readParam(node, "interp_kd1", cfg.interp_kd1);
    readParam(node, "interp_thr2", cfg.interp_thr2);
    readParam(node, "interp_thr3", cfg.interp_thr3);
    readParam(node, "dyn_filter_en", cfg.dyn_filter_en);
    readParam(node, "debug_publish", cfg.debug_publish);
    readParam(node, "laserCloudSteadObj_accu_limit", cfg.laserCloudSteadObj_accu_limit);
    readParam(node, "voxel_filter_size", cfg.voxel_filter_size);
    readParam(node, "cluster_coupled", cfg.cluster_coupled);
    readParam(node, "cluster_future", cfg.cluster_future);
    readParam(node, "cluster_extend_pixel", cfg.cluster_extend_pixel);
    readParam(node, "cluster_min_pixel_number", cfg.cluster_min_pixel_number);
    readParam(node, "cluster_thrustable_thresold", cfg.cluster_thrustable_thresold);
    readParam(node, "cluster_Voxel_revolusion", cfg.cluster_Voxel_revolusion);
    readParam(node, "cluster_debug_en", cfg.cluster_debug_en);
    readParam(node, "ver_resolution_max", cfg.ver_resolution_max);
    readParam(node, "hor_resolution_max", cfg.hor_resolution_max);
    readParam(node, "buffer_dur", cfg.buffer_dur);
    readParam(node, "point_index", cfg.point_index);

    return cfg;
}
