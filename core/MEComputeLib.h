#pragma once
#include <cstdint>

#ifdef MECOMPUTELIBRARY_EXPORTS

#define MECOMPUTELIBRARY_API __declspec(dllexport)

#else

#define MECOMPUTELIBRARY_API __declspec(dllimport)

#endif

/// <summary>
/// 
/// </summary>
/// <param name="input_file"></param>
/// <returns>true if the operation completed without error</returns>
extern "C" MECOMPUTELIBRARY_API bool hash_file(char* input_file, char* output_hash);

/// <summary>
/// Runs a frame-by-frame analysis to detect points as defined in the provided configuration file. 
/// Existing data groups not targeted by this operation will remain unaffected.
/// </summary>
/// <param name="job_config_file">Path to the desired job configuration file. Must be in json format.</param>
/// <param name="data_cache_file">Path to the desired output cache file. Can be in either bson or json format.</param>
/// <param name="use_bson_format">Whether or not to write cache in bson format</param>
/// <returns>true if the operation completed without error</returns>
extern "C" MECOMPUTELIBRARY_API bool analyze_frames(char* job_config_file, char* data_cache_file, bool use_bson_format);

/// <summary>
/// 
/// </summary>
/// <param name="data_cache_paths"></param>
/// <param name="target_data_groups"></param>
/// <param name="ground_plane_target"></param>
/// <param name="marker_length"></param>
/// <param name="output_path"></param>
/// <param name="use_bson_format"></param>
/// <param name="board_threshold"></param>
/// <param name="marker_threshold"></param>
/// <param name="scene_fps"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API bool triangulate_points(char* data_cache_paths, char* target_data_groups, char* ground_plane_target, float marker_length, char* output_path, bool use_bson_format, double board_threshold, double marker_threshold, double scene_fps);

/// <summary>
/// 
/// </summary>
/// <param name="data_cache_paths"></param>
/// <param name="pose_groups"></param>
/// <param name="calibration_groups"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API bool repose_with_target(char* data_cache_paths, char* pose_groups, char* calibration_groups);

/// <summary>
/// 
/// </summary>
/// <param name="schema_dir"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API void set_schema_directory(char* schema_dir);

/// <summary>
/// 
/// </summary>
/// <param name="window_name"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API void extern_create_window(char* window_name);

/// <summary>
/// 
/// </summary>
/// <param name="window_name"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API void extern_destroy_window(char* window_name);

/// <summary>
/// 
/// </summary>
/// <param name="window_name"></param>
/// <param name="img_mat_r"></param>
/// <param name="img_mat_g"></param>
/// <param name="img_mat_b"></param>
/// <param name="img_mat_a"></param>
/// <param name="alpha"></param>
/// <param name="rows"></param>
/// <param name="cols"></param>
/// <param name="flip_x"></param>
/// <param name="flip_y"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API void extern_show_image(char* window_name, uint8_t * img_mat_r, uint8_t * img_mat_g, uint8_t * img_mat_b, uint8_t * img_mat_a, bool alpha, int rows, int cols, bool flip_x, bool flip_y);

/// <summary>
/// 
/// </summary>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API void extern_destroy_windows();

/// <summary>
/// 
/// </summary>
/// <param name="out_file"></param>
/// <param name="dictionary"></param>
/// <param name="marker_id"></param>
/// <param name="size_ratio"></param>
/// <param name="size"></param>
/// <param name="delay_interval"></param>
/// <param name="start_delays"></param>
/// <param name="end_delays"></param>
/// <returns></returns>
extern "C" MECOMPUTELIBRARY_API void generate_synchronization_video(char* out_file, int dictionary, int marker_id, float size_ratio, int size, float delay_interval, int start_delays, int end_delays);

