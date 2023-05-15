#include <Windows.h>
#include <MEComputeLib.h>
#include <iostream>

bool run_analysis = true;
bool use_bson_format = false;
std::string job_config_path = "F:/programming_projects/Motion Engine/Assets/Calibration tests/Tracking Test/calib03.json";
std::string job_cache_path = "F:/programming_projects/Motion Engine/Assets/Calibration tests/Tracking Test/params03.json";
std::string scene_out_path = "F:/programming_projects/Motion Engine/Assets/Calibration tests/Tracking Test/scene.json";
std::string repose_pose_targets = "1ee7a6a24d05a7be76b88253ad27261eed53eb02;4c8aa77bf9b3d88ddc72a690de3c75c06f5cc7f9";
std::string repose_calibration_targets = "a6a9a71387f03f7cb534ae2b6de91a8a1bf54253;f9f82087e93e3b9c9f6ba58c7b2fd0fcfeb9019d";
std::string triangulation_targets = "1ee7a6a24d05a7be76b88253ad27261eed53eb02;4c8aa77bf9b3d88ddc72a690de3c75c06f5cc7f9";
std::string ground_plane_target = "1ee7a6a24d05a7be76b88253ad27261eed53eb02";
float marker_length = 0.0508f;
double board_threshold = 2.0;
double marker_threshold = 2.0;
double scene_fps = 60.0;


int main() {
    if(run_analysis){
        analyze_frames((char*)job_config_path.c_str(), (char*)job_cache_path.c_str(), use_bson_format);
    }
    //repose_with_target((char*)job_cache_path.c_str(), (char*)repose_pose_targets.c_str(), (char*)repose_calibration_targets.c_str());
    triangulate_points((char*)job_cache_path.c_str(), (char*)triangulation_targets.c_str(), (char*)ground_plane_target.c_str(), marker_length, (char*)scene_out_path.c_str(), use_bson_format, board_threshold, marker_threshold, scene_fps);
}
