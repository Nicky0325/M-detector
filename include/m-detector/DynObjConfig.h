#ifndef DYN_OBJ_CONFIG_H
#define DYN_OBJ_CONFIG_H

#include <string>

struct DynObjConfig {
    std::string frame_id{"camera_init"};
    std::string time_file;
    std::string time_breakdown_file;
    std::string cluster_out_file;

    int dataset{3};
    double buffer_delay{0.1};
    int buffer_size{300000};
    int points_num_perframe{200000};
    double depth_map_dur{0.2};
    int max_depth_map_num{5};
    int max_pixel_points{50};
    double frame_dur{0.1};
    float self_x_f{0.15f};
    float self_x_b{0.15f};
    float self_y_l{0.15f};
    float self_y_r{0.5f};
    float blind_dis{0.15f};
    float fov_up{0.15f};
    float fov_down{0.15f};
    float fov_cut{0.15f};
    float fov_left{180.0f};
    float fov_right{-180.0f};
    int checkneighbor_range{1};
    bool stop_object_detect{false};

    float depth_thr1{0.15f};
    float enter_min_thr1{0.15f};
    float enter_max_thr1{0.15f};
    float map_cons_depth_thr1{0.5f};
    float map_cons_hor_thr1{0.01f};
    float map_cons_ver_thr1{0.01f};
    float map_cons_hor_dis1{0.2f};
    float map_cons_ver_dis1{0.1f};
    float depth_cons_depth_thr1{0.5f};
    float depth_cons_depth_max_thr1{0.5f};
    float depth_cons_hor_thr1{0.02f};
    float depth_cons_ver_thr1{0.01f};
    float enlarge_z_thr1{0.05f};
    float enlarge_angle{2.0f};
    float enlarge_depth{3.0f};
    int occluded_map_thr1{3};
    bool case1_interp_en{false};
    float k_depth_min_thr1{0.0f};
    float d_depth_min_thr1{0.15f};
    float k_depth_max_thr1{0.0f};
    float d_depth_max_thr1{0.15f};

    float v_min_thr2{0.5f};
    float acc_thr2{1.0f};
    float map_cons_depth_thr2{0.15f};
    float map_cons_hor_thr2{0.02f};
    float map_cons_ver_thr2{0.01f};
    float occ_depth_thr2{0.15f};
    float occ_hor_thr2{0.02f};
    float occ_ver_thr2{0.01f};
    float depth_cons_depth_thr2{0.5f};
    float depth_cons_depth_max_thr2{0.5f};
    float depth_cons_hor_thr2{0.02f};
    float depth_cons_ver_thr2{0.01f};
    float k_depth2{0.005f};
    int occluded_times_thr2{3};
    bool case2_interp_en{false};
    float k_depth_max_thr2{0.0f};
    float d_depth_max_thr2{0.15f};

    float v_min_thr3{0.5f};
    float acc_thr3{1.0f};
    float map_cons_depth_thr3{0.15f};
    float map_cons_hor_thr3{0.02f};
    float map_cons_ver_thr3{0.01f};
    float occ_depth_thr3{0.15f};
    float occ_hor_thr3{0.02f};
    float occ_ver_thr3{0.01f};
    float depth_cons_depth_thr3{0.5f};
    float depth_cons_depth_max_thr3{0.5f};
    float depth_cons_hor_thr3{0.02f};
    float depth_cons_ver_thr3{0.01f};
    float k_depth3{0.005f};
    int occluding_times_thr3{3};
    bool case3_interp_en{false};
    float k_depth_max_thr3{0.0f};
    float d_depth_max_thr3{0.15f};

    float interp_hor_thr{0.01f};
    float interp_ver_thr{0.01f};
    float interp_thr1{1.0f};
    float interp_static_max{10.0f};
    float interp_start_depth1{20.0f};
    float interp_kp1{0.1f};
    float interp_kd1{1.0f};
    float interp_thr2{0.15f};
    float interp_thr3{0.15f};

    bool dyn_filter_en{true};
    bool debug_publish{true};
    int laserCloudSteadObj_accu_limit{5};
    float voxel_filter_size{0.1f};
    bool cluster_coupled{false};
    bool cluster_future{false};
    int cluster_extend_pixel{2};
    int cluster_min_pixel_number{4};
    float cluster_thrustable_thresold{0.3f};
    float cluster_Voxel_revolusion{0.3f};
    bool cluster_debug_en{false};

    float ver_resolution_max{0.0025f};
    float hor_resolution_max{0.0025f};
    float buffer_dur{0.1f};
    int point_index{0};

    static DynObjConfig FromYamlFile(const std::string& path);
};

#endif
