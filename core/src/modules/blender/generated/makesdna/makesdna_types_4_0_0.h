/*
Copyright (C) 2024 Blender Foundation. All rights reserved.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

---------------------------------------------------------------------

Various structs from Blender v4.0 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_4_0_0_H
#define MAKESDNA_TYPES_4_0_0_H

#include "makesdna_types_3_3_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_5_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_6_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_1_0.h"

struct bPoseChannel_BBoneSegmentBoundary4_0_0;
struct bPoseChannel_Runtime4_0_0;
struct Bone_Runtime4_0_0;
struct bArmature_Runtime4_0_0;
struct bArmature4_0_0;
struct BoneCollection4_0_0;
struct BoneCollectionMember4_0_0;
struct BoneCollectionReference4_0_0;
struct BrushGpencilSettings4_0_0;
struct Camera4_0_0;
struct CollectionLightLinking4_0_0;
struct CollectionObject4_0_0;
struct CollectionChild4_0_0;
struct Collection4_0_0;
struct CurveMap4_0_0;
struct CustomData4_0_0;
struct OpacityGpencilModifierData4_0_0;
struct GreasePencilDrawingBase4_0_0;
struct GreasePencilDrawingReference4_0_0;
struct GreasePencilLayerFramesMapStorage4_0_0;
struct GreasePencilLayerMask4_0_0;
struct GreasePencilOnionSkinningSettings4_0_0;
struct IDOverrideLibraryPropertyOperation4_0_0;
struct IpoCurve4_0_0;
struct Ipo4_0_0;
struct LayerCollection4_0_0;
struct LightProbe4_0_0;
struct LightProbeConnectivityData4_0_0;
struct NodesModifierBake4_0_0;
struct MeshToVolumeModifierData4_0_0;
struct bNodeTreeInterfaceItem4_0_0;
struct bNodePreview4_0_0;
struct bNodeSocketValueRotation4_0_0;
struct GeometryNodeAssetTraits4_0_0;
struct NodeKuwaharaData4_0_0;
struct NodeTexVoronoi4_0_0;
struct NodeShaderHairPrincipled4_0_0;
struct NodeGeometryRaycast4_0_0;
struct NodeGeometryRepeatInput4_0_0;
struct LightLinkingRuntime4_0_0;
struct LightLinking4_0_0;
struct ToolSettings4_0_0;
struct RaytraceEEVEE4_0_0;
struct SceneEEVEE4_0_0;
struct SceneHydra4_0_0;
struct Scene4_0_0;
struct Panel4_0_0;
struct SeqRetimingKey4_0_0;
struct Sequence4_0_0;
struct EQCurveMappingData4_0_0;
struct SoundEqualizerModifierData4_0_0;
struct FileAssetSelectParams4_0_0;
struct SpaceNodeOverlay4_0_0;
struct SpaceNode4_0_0;
struct ThemeUI4_0_0;
struct ThemeAssetShelf4_0_0;
struct ThemeSpace4_0_0;
struct bAddon4_0_0;
struct bUserAssetLibrary4_0_0;
struct bUserExtensionRepo4_0_0;
struct UserDef_Experimental4_0_0;
struct vec4f4_0_0;
struct View2D4_0_0;
struct RegionView3D4_0_0;
struct ViewerPathElem4_0_0;
struct GroupNodeViewerPathElem4_0_0;
struct SimulationZoneViewerPathElem4_0_0;
struct RepeatZoneViewerPathElem4_0_0;
struct ViewerNodeViewerPathElem4_0_0;
struct ReportTimerInfo4_0_0;
struct wmWindowManager4_0_0;
struct wmWindow4_0_0;
struct SpaceAction4_0_0;
struct CurveMapping4_0_0;
struct RenderData4_0_0;
struct ARegion4_0_0;
struct CurvesModifierData4_0_0;
struct HueCorrectModifierData4_0_0;
struct SpaceOutliner4_0_0;
struct SpaceGraph4_0_0;
struct SpaceNla4_0_0;
struct SpaceSeq4_0_0;
struct bTheme4_0_0;
struct IDViewerPathElem4_0_0;
struct ModifierViewerPathElem4_0_0;

struct bPoseChannel_BBoneSegmentBoundary4_0_0 {
    float point[3];
    float plane_normal[3];
    float plane_offset;
    float depth_scale;
};

struct bArmature_Runtime4_0_0 {
    int active_collection_index;
    uint8_t _pad0[4];
    BoneCollection4_0_0 *active_collection;
};

struct BoneCollectionMember4_0_0 {
    BoneCollectionMember4_0_0 *next, *prev;
    Bone3_0_0 *bone;
};

struct BoneCollectionReference4_0_0 {
    BoneCollectionReference4_0_0 *next, *prev;
    BoneCollection4_0_0 *bcoll;
};

struct BrushGpencilSettings4_0_0 {
    float draw_smoothfac;
    float fill_factor;
    float draw_strength;
    float draw_jitter;
    float draw_angle;
    float draw_angle_factor;
    float draw_random_press;
    float draw_random_strength;
    short draw_smoothlvl;
    short draw_subdivide;
    short fill_layer_mode;
    short fill_direction;
    float fill_threshold;
    char _pad2[2];
    int8_t caps_type;
    char _pad[5];
    int flag2;
    int fill_simplylvl;
    int fill_draw_mode;
    int fill_extend_mode;
    int icon_id;
    int input_samples;
    float uv_random;
    int brush_type;
    int eraser_mode;
    float active_smooth;
    float era_strength_f;
    float era_thickness_f;
    int flag;
    float hardness;
    float aspect_ratio[2];
    float simplify_f;
    float vertex_factor;
    int vertex_mode;
    int sculpt_flag;
    int sculpt_mode_flag;
    short preset_type;
    short brush_draw_mode;
    float random_hue;
    float random_saturation;
    float random_value;
    float fill_extend_fac;
    int dilate_pixels;
    CurveMapping4_0_0 *curve_sensitivity;
    CurveMapping4_0_0 *curve_strength;
    CurveMapping4_0_0 *curve_jitter;
    CurveMapping4_0_0 *curve_rand_pressure;
    CurveMapping4_0_0 *curve_rand_strength;
    CurveMapping4_0_0 *curve_rand_uv;
    CurveMapping4_0_0 *curve_rand_hue;
    CurveMapping4_0_0 *curve_rand_saturation;
    CurveMapping4_0_0 *curve_rand_value;
    float outline_fac;
    char _pad1[4];
    Material2_93_0 *material;
    Material2_93_0 *material_alt;
};

struct CollectionLightLinking4_0_0 {
    uint8_t link_state;
    uint8_t _pad[3];
};

struct CurveMap4_0_0 {
    short totpoint;
    short flag;
    float range;
    float mintable, maxtable;
    float ext_in[2], ext_out[2];
    CurveMapPoint2_93_0 *curve;
    CurveMapPoint2_93_0 *table;
    CurveMapPoint2_93_0 *premultable;
    float premul_ext_in[2];
    float premul_ext_out[2];
    short default_handle_type;
    char _pad[6];
};

struct CustomData4_0_0 {
    CustomDataLayer3_6_0 *layers;
    int typemap[53];
    int totlayer, maxlayer;
    int totsize;
    void *pool;
    CustomDataExternal3_2_0 *external;
};

struct GreasePencilDrawingBase4_0_0 {
    int8_t type;
    char _pad[3];
    uint32_t flag;
};

struct GreasePencilLayerFramesMapStorage4_0_0 {
    int *keys;
    void *values;
    int num;
    int flag;
};

struct GreasePencilLayerMask4_0_0 {
    GreasePencilLayerMask4_0_0 *next, *prev;
    char *layer_name;
    uint16_t flag;
    char _pad[6];
};

struct GreasePencilOnionSkinningSettings4_0_0 {
    float opacity;
    int8_t mode;
    uint8_t filter;
    char _pad[2];
    int16_t num_frames_before;
    int16_t num_frames_after;
    float color_before[3];
    float color_after[3];
    char _pad2[4];
};

struct IDOverrideLibraryPropertyOperation4_0_0 {
    IDOverrideLibraryPropertyOperation4_0_0 *next, *prev;
    short operation;
    short flag;
    short tag;
    char _pad0[2];
    char *subitem_reference_name;
    char *subitem_local_name;
    int subitem_reference_index;
    int subitem_local_index;
    ID3_4_0 *subitem_reference_id;
    ID3_4_0 *subitem_local_id;
};

struct LightProbeConnectivityData4_0_0 {
    uint8_t *validity;
};

struct NodesModifierBake4_0_0 {
    int id;
    uint32_t flag;
    char *directory;
    int frame_start;
    int frame_end;
};

struct bNodeTreeInterfaceItem4_0_0 {
    char item_type;
    char _pad[7];
};

struct bNodeSocketValueRotation4_0_0 {
    float value_euler[3];
};

struct GeometryNodeAssetTraits4_0_0 {
    int flag;
};

struct NodeKuwaharaData4_0_0 {
    short size;
    short variation;
    int uniformity;
    float sharpness;
    float eccentricity;
};

struct NodeShaderHairPrincipled4_0_0 {
    short model;
    short parametrization;
    char _pad[4];
};

struct NodeGeometryRaycast4_0_0 {
    uint8_t mapping;
    int8_t data_type;
};

struct NodeGeometryRepeatInput4_0_0 {
    int32_t output_node_id;
};

struct LightLinkingRuntime4_0_0 {
    uint64_t light_set_membership;
    uint64_t shadow_set_membership;
    uint8_t receiver_light_set;
    uint8_t blocker_shadow_set;
    uint8_t _pad[6];
};

struct RaytraceEEVEE4_0_0 {
    float screen_trace_quality;
    float screen_trace_thickness;
    int resolution_scale;
    float sample_clamp;
    int flag;
    int denoise_stages;
};

struct SceneHydra4_0_0 {
    int export_method;
    int _pad0;
};

struct SeqRetimingKey4_0_0 {
    int strip_frame_index;
    int flag;
    int _pad0;
    float retiming_factor;
    int original_strip_frame_index;
    float original_retiming_factor;
};

struct SpaceNodeOverlay4_0_0 {
    int flag;
    int preview_shape;
};

struct ThemeAssetShelf4_0_0 {
    unsigned char header_back[4];
    unsigned char back[4];
};

struct bAddon4_0_0 {
    bAddon4_0_0 *next, *prev;
    char module[128];
    IDProperty3_5_0 *prop;
};

struct bUserAssetLibrary4_0_0 {
    bUserAssetLibrary4_0_0 *next, *prev;
    char name[64];
    char dirpath[1024];
    short import_method;
    short flag;
    char _pad0[4];
};

struct bUserExtensionRepo4_0_0 {
    bUserExtensionRepo4_0_0 *next, *prev;
    char name[64];
    char module[48];
    char dirpath[1024];
    char remote_path[1024];
    int flag;
    char _pad0[4];
};

struct UserDef_Experimental4_0_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
    char use_eevee_debug;
    char show_asset_debug_info;
    char no_asset_indexing;
    char use_viewport_debug;
    char use_all_linked_data_direct;
    char SANITIZE_AFTER_HERE;
    char use_new_curves_tools;
    char use_new_point_cloud_type;
    char use_full_frame_compositor;
    char use_sculpt_tools_tilt;
    char use_extended_asset_browser;
    char use_override_templates;
    char enable_eevee_next;
    char use_sculpt_texture_paint;
    char use_grease_pencil_version3;
    char enable_overlay_next;
    char use_new_volume_nodes;
    char use_shader_node_previews;
    char use_extension_repos;
    char _pad[2];
};

struct vec4f4_0_0 {
    float x, y, z, w;
};

struct RegionView3D4_0_0 {
    float winmat[4][4];
    float viewmat[4][4];
    float viewinv[4][4];
    float persmat[4][4];
    float persinv[4][4];
    float viewcamtexcofac[4];
    float viewmatob[4][4];
    float persmatob[4][4];
    float clip[6][4];
    float clip_local[6][4];
    BoundBox2_93_0 *clipbb;
    RegionView3D4_0_0 *localvd;
    void *view_render;
    void *sms;
    void *smooth_timer;
    float twmat[4][4];
    float tw_axis_min[3], tw_axis_max[3];
    float tw_axis_matrix[3][3];
    float gridview;
    float viewquat[4];
    float dist;
    float camdx, camdy;
    float pixsize;
    float ofs[3];
    float camzoom;
    char is_persp;
    char persp;
    char view;
    char view_axis_roll;
    char viewlock;
    char runtime_viewlock;
    char viewlock_quad;
    char _pad[1];
    float ofs_lock[2];
    short twdrawflag;
    short rflag;
    float lviewquat[4];
    char lpersp;
    char lview;
    char lview_axis_roll;
    char _pad8[1];
    float rot_angle;
    float rot_axis[3];
};

struct ViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 *next, *prev;
    int type;
    char _pad[4];
    char *ui_name;
};

struct ReportTimerInfo4_0_0 {
    float widthfac;
    float flash_progress;
};

struct CollectionObject4_0_0 {
    CollectionObject4_0_0 *next, *prev;
    Object3_0_0 *ob;
    CollectionLightLinking4_0_0 light_linking;
    int _pad;
};

struct CollectionChild4_0_0 {
    CollectionChild4_0_0 *next, *prev;
    Collection4_0_0 *collection;
    CollectionLightLinking4_0_0 light_linking;
    int _pad;
};

struct GreasePencilDrawingReference4_0_0 {
    GreasePencilDrawingBase4_0_0 base;
    void *id_reference;
};

struct LightLinking4_0_0 {
    Collection4_0_0 *receiver_collection;
    Collection4_0_0 *blocker_collection;
    LightLinkingRuntime4_0_0 runtime;
};

struct SceneEEVEE4_0_0 {
    int flag;
    int gi_diffuse_bounces;
    int gi_cubemap_resolution;
    int gi_visibility_resolution;
    float gi_irradiance_smoothing;
    float gi_glossy_clamp;
    float gi_filter_quality;
    int gi_irradiance_pool_size;
    float gi_cubemap_draw_size;
    float gi_irradiance_draw_size;
    int taa_samples;
    int taa_render_samples;
    int sss_samples;
    float sss_jitter_threshold;
    float ssr_quality;
    float ssr_max_roughness;
    float ssr_thickness;
    float ssr_border_fade;
    float ssr_firefly_fac;
    float volumetric_start;
    float volumetric_end;
    int volumetric_tile_size;
    int volumetric_samples;
    float volumetric_sample_distribution;
    float volumetric_light_clamp;
    int volumetric_shadow_samples;
    float gtao_distance;
    float gtao_factor;
    float gtao_quality;
    float bokeh_overblur;
    float bokeh_max_size;
    float bokeh_threshold;
    float bokeh_neighbor_max;
    float bokeh_denoise_fac;
    float bloom_color[3];
    float bloom_threshold;
    float bloom_knee;
    float bloom_intensity;
    float bloom_radius;
    float bloom_clamp;
    int motion_blur_samples;
    int motion_blur_max;
    int motion_blur_steps;
    int motion_blur_position;
    float motion_blur_shutter;
    float motion_blur_depth_scale;
    int shadow_method;
    int shadow_cube_size;
    int shadow_cascade_size;
    int shadow_pool_size;
    int shadow_ray_count;
    int shadow_step_count;
    float shadow_normal_bias;
    char _pad[4];
    int ray_split_settings;
    int ray_tracing_method;
    RaytraceEEVEE4_0_0 reflection_options;
    RaytraceEEVEE4_0_0 refraction_options;
    RaytraceEEVEE4_0_0 diffuse_options;
    LightCache2_93_0 *light_cache;
    LightCache2_93_0 *light_cache_data;
    char light_cache_info[128];
    float overscan;
    float light_threshold;
};

struct GroupNodeViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 base;
    int32_t node_id;
    char _pad1[4];
};

struct SimulationZoneViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 base;
    int32_t sim_output_node_id;
    char _pad1[4];
};

struct RepeatZoneViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 base;
    int repeat_output_node_id;
    int iteration;
};

struct ViewerNodeViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 base;
    int32_t node_id;
    char _pad1[4];
};

struct IDViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 base;
    ID3_4_0 *id;
};

struct ModifierViewerPathElem4_0_0 {
    ViewerPathElem4_0_0 base;
    char *modifier_name;
};

struct bPoseChannel_Runtime4_0_0 {
    SessionUUID2_93_0 session_uuid;
    DualQuat2_93_0 deform_dual_quat;
    int bbone_segments;
    float bbone_arc_length_reciprocal;
    char _pad1[4];
    void *bbone_rest_mats;
    void *bbone_pose_mats;
    void *bbone_deform_mats;
    DualQuat2_93_0 *bbone_dual_quats;
    bPoseChannel_BBoneSegmentBoundary4_0_0 *bbone_segment_boundaries;
    void *_pad;
};

struct Bone_Runtime4_0_0 {
    ListBase2_93_0 collections;
};

struct bArmature4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 bonebase;
    void *bonehash;
    void *_pad1;
    ListBase2_93_0 *edbo;
    Bone3_0_0 *act_bone;
    void *act_edbone;
    char needs_flush_to_id;
    char _pad0[3];
    int flag;
    int drawtype;
    short deformflag;
    short pathflag;
    ListBase2_93_0 collections;
    char active_collection_name[64];
    unsigned int layer_used;
    unsigned int layer , layer_protected;
    float axes_position;
    bArmature_Runtime4_0_0 runtime;
};

struct BoneCollection4_0_0 {
    BoneCollection4_0_0 *next, *prev;
    char name[64];
    ListBase2_93_0 bones;
    uint8_t flags;
    uint8_t _pad0[7];
    IDProperty3_5_0 *prop;
};

struct Camera4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    char type;
    char dtx;
    short flag;
    float passepartalpha;
    float clip_start, clip_end;
    float lens, ortho_scale, drawsize;
    float sensor_x, sensor_y;
    float shiftx, shifty;
    float dof_distance;
    char sensor_fit;
    char panorama_type;
    char _pad[2];
    float fisheye_fov;
    float fisheye_lens;
    float latitude_min, latitude_max;
    float longitude_min, longitude_max;
    float fisheye_polynomial_k0;
    float fisheye_polynomial_k1;
    float fisheye_polynomial_k2;
    float fisheye_polynomial_k3;
    float fisheye_polynomial_k4;
    Ipo4_0_0 *ipo;
    Object3_0_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings3_3_0 dof;
    ListBase2_93_0 bg_images;
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct Collection4_0_0 {
    ID3_4_0 id;
    ListBase2_93_0 gobject;
    ListBase2_93_0 children;
    PreviewImage2_93_0 *preview;
    unsigned int layer;
    float instance_offset[3];
    uint8_t flag;
    int8_t color_tag;
    char _pad0[2];
    uint8_t lineart_usage;
    uint8_t lineart_flags;
    uint8_t lineart_intersection_mask;
    uint8_t lineart_intersection_priority;
    void *_pad1;
    ViewLayer3_2_0 *view_layer;
    Collection_Runtime3_6_0 runtime;
};

struct OpacityGpencilModifierData4_0_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float factor;
    char modify_color;
    char _pad[3];
    int layer_pass;
    float hardness;
    CurveMapping4_0_0 *curve_intensity;
};

struct IpoCurve4_0_0 {
    IpoCurve4_0_0 *next, *prev;
    BPoint2_93_0 *bp;
    BezTriple2_93_0 *bezt;
    rctf2_93_0 maxrct, totrct;
    short blocktype;
    short adrcode;
    short vartype;
    short totvert;
    short ipo, extrap;
    short flag;
    char _pad0[2];
    float ymin, ymax;
    unsigned int bitmask;
    float slide_min, slide_max;
    float curval;
    IpoDriver2_93_0 *driver;
};

struct Ipo4_0_0 {
    ID3_4_0 id;
    ListBase2_93_0 curve;
    rctf2_93_0 cur;
    short blocktype;
    short showkey;
    short muteipo;
    char _pad[2];
};

struct LayerCollection4_0_0 {
    LayerCollection4_0_0 *next, *prev;
    Collection4_0_0 *collection;
    void *_pad1;
    short flag;
    short runtime_flag;
    char _pad[4];
    ListBase2_93_0 layer_collections;
    unsigned short local_collections_bits;
    short _pad2[3];
};

struct LightProbe4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    char type;
    char flag;
    char attenuation_type;
    char parallax_type;
    char grid_flag;
    char _pad0[3];
    float distinf;
    float distpar;
    float falloff;
    float clipsta, clipend;
    float vis_bias, vis_bleedbias;
    float vis_blur;
    float intensity;
    int grid_resolution_x;
    int grid_resolution_y;
    int grid_resolution_z;
    int grid_bake_samples;
    float grid_surface_bias;
    float grid_escape_bias;
    float grid_normal_bias;
    float grid_view_bias;
    float grid_facing_bias;
    float grid_validity_threshold;
    float grid_dilation_threshold;
    float grid_dilation_radius;
    char _pad1[4];
    float grid_clamp_direct;
    float grid_clamp_indirect;
    float surfel_density;
    int resolution;
    Collection4_0_0 *visibility_grp;
};

struct MeshToVolumeModifierData4_0_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    float interior_band_width;
    float density;
    char _pad2[4];
    void *_pad3;
};

struct bNodePreview4_0_0 {
    bNodeInstanceHashEntry2_93_0 hash_entry;
    void *ibuf;
};

struct NodeTexVoronoi4_0_0 {
    NodeTexBase2_93_0 base;
    int dimensions;
    int feature;
    int distance;
    int normalize;
    int coloring;
    char _pad[4];
};

struct ToolSettings4_0_0 {
    VPaint2_93_0 *vpaint;
    VPaint2_93_0 *wpaint;
    Sculpt3_4_0 *sculpt;
    UvSculpt2_93_0 *uvsculpt;
    GpPaint2_93_0 *gp_paint;
    GpVertexPaint2_93_0 *gp_vertexpaint;
    GpSculptPaint2_93_0 *gp_sculptpaint;
    GpWeightPaint2_93_0 *gp_weightpaint;
    CurvesSculpt3_2_0 *curves_sculpt;
    float vgroup_weight;
    float doublimit;
    char automerge;
    char object_flag;
    char selectmode;
    char unwrapper;
    char uvcalc_flag;
    char uv_flag;
    char uv_selectmode;
    char uv_sticky;
    float uvcalc_margin;
    short autoik_chainlen;
    char gpencil_flags;
    char gpencil_v3d_align;
    char gpencil_v2d_align;
    char _pad0[2];
    char annotate_v3d_align;
    short annotate_thickness;
    char gpencil_selectmode_edit;
    char gpencil_selectmode_sculpt;
    GP_Sculpt_Settings2_93_0 gp_sculpt;
    GP_Interpolate_Settings2_93_0 gp_interpolate;
    ImagePaintSettings2_93_0 imapaint;
    PaintModeSettings3_2_0 paint_mode;
    ParticleEditSettings3_0_0 particle;
    float proportional_size;
    float select_thresh;
    short autokey_flag;
    char autokey_mode;
    char keyframe_type;
    char multires_subdiv_type;
    char edge_mode;
    char edge_mode_live_unwrap;
    char transform_pivot_point;
    char transform_flag;
    char snap_node_mode;
    short snap_mode;
    short snap_uv_mode;
    short snap_anim_mode;
    short snap_flag;
    short snap_flag_node;
    short snap_flag_seq;
    short snap_flag_anim;
    short snap_uv_flag;
    char _pad[4];
    char snap_target;
    char snap_transform_mode_flag;
    short snap_face_nearest_steps;
    char proportional_edit, prop_mode;
    char proportional_objects;
    char proportional_mask;
    char proportional_action;
    char proportional_fcurve;
    char lock_markers;
    char auto_normalize;
    char wpaint_lock_relative;
    char multipaint;
    char weightuser;
    char vgroupsubset;
    char gpencil_selectmode_vertex;
    char uv_sculpt_settings;
    char uv_relax_method;
    char workspace_tool_type;
    short sculpt_paint_settings;
    int sculpt_paint_unified_size;
    float sculpt_paint_unified_unprojected_radius;
    float sculpt_paint_unified_alpha;
    UnifiedPaintSettings3_2_0 unified_paint_settings;
    CurvePaintSettings2_93_0 curve_paint_settings;
    MeshStatVis2_93_0 statvis;
    float normal_vector[3];
    char _pad6[4];
    CurveProfile2_93_0 *custom_bevel_profile_preset;
    SequencerToolSettings3_0_0 *sequencer_tool_settings;
    short snap_mode_tools;
    char plane_axis;
    char plane_depth;
    char plane_orient;
    char use_plane_axis_auto;
    char _pad7[2];
};

struct CurveMapping4_0_0 {
    int flag, cur;
    int preset;
    int changed_timestamp;
    rctf2_93_0 curr, clipr;
    CurveMap4_0_0 cm[4];
    float black[3], white[3];
    float bwmul[3];
    float sample[3];
    short tone;
    char _pad[6];
};

struct RenderData4_0_0 {
    ImageFormatData3_2_0 im_format;
    AviCodecData2_93_0* avicodecdata;
    FFMpegCodecData3_1_0 ffcodecdata;
    int cfra, sfra, efra;
    float subframe;
    int psfra, pefra;
    int images, framapto;
    short flag, threads;
    float framelen, blurfac;
    int frame_step;
    char _pad10[2];
    short dimensionspreset;
    short size;
    char _pad6[2];
    int xsch;
    int ysch;
    int tilex;
    int tiley;
    short planes;
    short imtype;
    short subimtype;
    short quality;
    char use_lock_interface;
    char _pad7[3];
    int scemode;
    int mode;
    short frs_sec;
    char alphamode;
    char _pad0[1];
    rctf2_93_0 border;
    ListBase2_93_0 layers;
    short actlay;
    char _pad1[2];
    float xasp, yasp;
    float frs_sec_base;
    float gauss;
    int color_mgt_flag;
    float dither_intensity;
    short bake_mode, bake_flag;
    short bake_margin, bake_samples;
    short bake_margin_type;
    char _pad9[6];
    float bake_biasdist, bake_user_scale;
    char pic[1024];
    int stamp;
    short stamp_font_id;
    char _pad3[2];
    char stamp_udata[768];
    float fg_stamp[4];
    float bg_stamp[4];
    char seq_prev_type;
    char seq_rend_type;
    char seq_flag;
    char _pad5[3];
    short simplify_subsurf;
    short simplify_subsurf_render;
    short simplify_gpencil;
    float simplify_particles;
    float simplify_particles_render;
    float simplify_volumes;
    float simplify_shadows;
    float simplify_shadows_render;
    int line_thickness_mode;
    float unit_line_thickness;
    char engine[32];
    char _pad2[2];
    short perf_flag;
    BakeData3_4_0 bake;
    int _pad8;
    short preview_pixel_size;
    short _pad4;
    ListBase2_93_0 views;
    short actview;
    short views_format;
    short hair_type, hair_subdiv;
    CurveMapping4_0_0 mblur_shutter_curve;
};

struct Scene4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object3_0_0 *camera;
    World2_93_0 *world;
    Scene4_0_0 *set;
    ListBase2_93_0 base;
    Base3_4_0 *basact;
    void *_pad1;
    View3DCursor2_93_0 cursor;
    unsigned int lay;
    int layact;
    char _pad2[4];
    short flag;
    char use_nodes;
    char _pad3[1];
    void *nodetree;
    Editing3_2_0 *ed;
    ToolSettings4_0_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData4_0_0 r;
    AudioData2_93_0 audio;
    ListBase2_93_0 markers;
    ListBase2_93_0 transform_spaces;
    TransformOrientationSlot2_93_0 orientation_slots[4];
    void *sound_scene;
    void *playback_handle;
    void *sound_scrub_handle;
    void *speaker_handles;
    void *fps_info;
    void *depsgraph_hash;
    char _pad7[4];
    int active_keyingset;
    ListBase2_93_0 keyingsets;
    UnitSettings2_93_0 unit;
    bGPdata3_0_0 *gpd;
    MovieClip3_5_0 *clip;
    PhysicsSettings3_0_0 physics_settings;
    void *_pad8;
    CustomData_MeshMasks2_93_0 customdata_mask;
    CustomData_MeshMasks2_93_0 customdata_mask_modal;
    ColorManagedViewSettings2_93_0 view_settings;
    ColorManagedDisplaySettings2_93_0 display_settings;
    ColorManagedColorspaceSettings2_93_0 sequencer_colorspace_settings;
    RigidBodyWorld2_93_0 *rigidbody_world;
    PreviewImage2_93_0 *preview;
    ListBase2_93_0 view_layers;
    Collection4_0_0 *master_collection;
    IDProperty3_5_0 *layer_properties;
    int simulation_frame_start;
    int simulation_frame_end;
    SceneDisplay2_93_0 display;
    SceneEEVEE4_0_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
    SceneHydra4_0_0 hydra;
};

struct Panel4_0_0 {
    Panel4_0_0 *next, *prev;
    void *type;
    void *layout;
    char panelname[64];
    char *drawname;
    int ofsx, ofsy;
    int sizex, sizey;
    int blocksizex, blocksizey;
    short labelofs;
    short flag, runtime_flag;
    char _pad[6];
    int sortorder;
    void *activedata;
    ListBase2_93_0 children;
    Panel_Runtime3_0_0 runtime;
};

struct Sequence4_0_0 {
    Sequence4_0_0 *next, *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag, type;
    int len;
    float start;
    float startofs, endofs;
    float startstill, endstill;
    int machine;
    int _pad;
    int startdisp, enddisp;
    float sat;
    float mul;
    float _pad1;
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    Strip3_6_0 *strip;
    Ipo4_0_0 *ipo;
    Scene4_0_0 *scene;
    Object3_0_0 *scene_camera;
    MovieClip3_5_0 *clip;
    Mask3_6_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence4_0_0 *seq1, *seq2, *seq3;
    ListBase2_93_0 seqbase;
    ListBase2_93_0 channels;
    bSound3_3_0 *sound;
    void *scene_sound;
    float volume;
    float pitch , pan;
    float strobe;
    void *effectdata;
    int anim_startofs;
    int anim_endofs;
    int blend_mode;
    float blend_opacity;
    int8_t color_tag;
    char alpha_mode;
    char _pad2[2];
    int cache_flag;
    int sfra;
    char views_format;
    char _pad3[3];
    Stereo3dFormat2_93_0 *stereo3d_format;
    IDProperty3_5_0 *prop;
    ListBase2_93_0 modifiers;
    float media_playback_rate;
    float speed_factor;
    SeqRetimingKey4_0_0 *retiming_keys;
    void *_pad5;
    int retiming_keys_num;
    char _pad6[4];
    SequenceRuntime2_93_0 runtime;
};

struct EQCurveMappingData4_0_0 {
    EQCurveMappingData4_0_0 *next, *prev;
    CurveMapping4_0_0 curve_mapping;
};

struct SoundEqualizerModifierData4_0_0 {
    SequenceModifierData2_93_0 modifier;
    ListBase2_93_0 graphics;
};

struct FileAssetSelectParams4_0_0 {
    FileSelectParams3_0_0 base_params;
    AssetLibraryReference3_0_0 asset_library_ref;
    short asset_catalog_visibility;
    char _pad[6];
    bUUID3_0_0 catalog_id;
    short import_method;
    char _pad2[6];
};

struct View2D4_0_0 {
    rctf2_93_0 tot, cur;
    rcti2_93_0 vert, hor;
    rcti2_93_0 mask;
    float min[2], max[2];
    float minzoom, maxzoom;
    short scroll;
    short scroll_ui;
    short keeptot;
    short keepzoom;
    short keepofs;
    short flag;
    short align;
    short winx, winy;
    short oldwinx, oldwiny;
    short around;
    char alpha_vert, alpha_hor;
    char _pad[2];
    float page_size_y;
    void* sms;
    void* smooth_timer;
};

struct SpaceNode4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    ID3_4_0 *id, *from;
    short flag;
    char insert_ofs_dir;
    char _pad1;
    float xof, yof;
    float zoom;
    ListBase2_93_0 treepath;
    void *edittree;
    void *nodetree;
    char tree_idname[64];
    int treetype;
    short texfrom;
    char shaderfrom;
    char geometry_nodes_type;
    void *geometry_nodes_tool_tree;
    bGPdata3_0_0 *gpd;
    SpaceNodeOverlay4_0_0 overlay;
    void *runtime;
};

struct ThemeUI4_0_0 {
    uiWidgetColors2_93_0 wcol_regular, wcol_tool, wcol_toolbar_item, wcol_text;
    uiWidgetColors2_93_0 wcol_radio, wcol_option, wcol_toggle;
    uiWidgetColors2_93_0 wcol_num, wcol_numslider, wcol_tab;
    uiWidgetColors2_93_0 wcol_menu, wcol_pulldown, wcol_menu_back, wcol_menu_item, wcol_tooltip;
    uiWidgetColors2_93_0 wcol_box, wcol_scroll, wcol_progress, wcol_list_item, wcol_pie_menu;
    uiWidgetStateColors2_93_0 wcol_state;
    unsigned char widget_emboss[4];
    float menu_shadow_fac;
    short menu_shadow_width;
    unsigned char editor_outline[4];
    unsigned char transparent_checker_primary[4], transparent_checker_secondary[4];
    unsigned char transparent_checker_size;
    char _pad1[1];
    float icon_alpha;
    float icon_saturation;
    unsigned char widget_text_cursor[4];
    unsigned char xaxis[4], yaxis[4], zaxis[4];
    unsigned char gizmo_hi[4];
    unsigned char gizmo_primary[4];
    unsigned char gizmo_secondary[4];
    unsigned char gizmo_view_align[4];
    unsigned char gizmo_a[4];
    unsigned char gizmo_b[4];
    unsigned char icon_scene[4];
    unsigned char icon_collection[4];
    unsigned char icon_object[4];
    unsigned char icon_object_data[4];
    unsigned char icon_modifier[4];
    unsigned char icon_shading[4];
    unsigned char icon_folder[4];
    float icon_border_intensity;
    float panel_roundness;
    char _pad2[4];
};

struct ThemeSpace4_0_0 {
    unsigned char back[4];
    unsigned char back_grad[4];
    char background_type;
    char _pad0[3];
    unsigned char title[4];
    unsigned char text[4];
    unsigned char text_hi[4];
    unsigned char header[4];
    unsigned char header_title[4];
    unsigned char header_text[4];
    unsigned char header_text_hi[4];
    unsigned char tab_active[4];
    unsigned char tab_inactive[4];
    unsigned char tab_back[4];
    unsigned char tab_outline[4];
    unsigned char button[4];
    unsigned char button_title[4];
    unsigned char button_text[4];
    unsigned char button_text_hi[4];
    unsigned char list[4];
    unsigned char list_title[4];
    unsigned char list_text[4];
    unsigned char list_text_hi[4];
    unsigned char navigation_bar[4];
    unsigned char execution_buts[4];
    uiPanelColors2_93_0 panelcolors;
    ThemeAssetShelf4_0_0 asset_shelf;
    unsigned char shade1[4];
    unsigned char shade2[4];
    unsigned char hilite[4];
    unsigned char grid[4];
    unsigned char view_overlay[4];
    unsigned char wire[4], wire_edit[4], select[4];
    unsigned char lamp[4], speaker[4], empty[4], camera[4];
    unsigned char active[4], group[4], group_active[4], transform[4];
    unsigned char vertex[4], vertex_select[4], vertex_active[4], vertex_bevel[4],      vertex_unreferenced[4];
    unsigned char edge[4], edge_select[4];
    unsigned char edge_seam[4], edge_sharp[4], edge_facesel[4], edge_crease[4], edge_bevel[4];
    unsigned char face[4], face_select[4], face_retopology[4], face_back[4], face_front[4];
    unsigned char face_dot[4];
    unsigned char extra_edge_len[4], extra_edge_angle[4], extra_face_angle[4], extra_face_area[4];
    unsigned char normal[4];
    unsigned char vertex_normal[4];
    unsigned char loop_normal[4];
    unsigned char bone_solid[4], bone_pose[4], bone_pose_active[4], bone_locked_weight[4];
    unsigned char strip[4], strip_select[4];
    unsigned char cframe[4];
    unsigned char time_keyframe[4], time_gp_keyframe[4];
    unsigned char freestyle_edge_mark[4], freestyle_face_mark[4];
    unsigned char time_scrub_background[4];
    unsigned char time_marker_line[4], time_marker_line_selected[4];
    unsigned char nurb_uline[4], nurb_vline[4];
    unsigned char act_spline[4], nurb_sel_uline[4], nurb_sel_vline[4], lastsel_point[4];
    unsigned char handle_free[4], handle_auto[4], handle_vect[4], handle_align[4],      handle_auto_clamped[4];
    unsigned char handle_sel_free[4], handle_sel_auto[4], handle_sel_vect[4], handle_sel_align[4],      handle_sel_auto_clamped[4];
    unsigned char ds_channel[4], ds_subchannel[4], ds_ipoline[4];
    unsigned char keytype_keyframe[4], keytype_extreme[4], keytype_breakdown[4], keytype_jitter[4],      keytype_movehold[4];
    unsigned char keytype_keyframe_select[4], keytype_extreme_select[4], keytype_breakdown_select[4],      keytype_jitter_select[4], keytype_movehold_select[4];
    unsigned char keyborder[4], keyborder_select[4];
    char _pad4[3];
    unsigned char console_output[4], console_input[4], console_info[4], console_error[4];
    unsigned char console_cursor[4], console_select[4];
    unsigned char vertex_size, edge_width, outline_width, obcenter_dia, facedot_size;
    unsigned char noodle_curving;
    unsigned char grid_levels;
    char _pad5[2];
    float dash_alpha;
    unsigned char syntaxl[4], syntaxs[4];
    unsigned char syntaxb[4], syntaxn[4];
    unsigned char syntaxv[4], syntaxc[4];
    unsigned char syntaxd[4], syntaxr[4];
    unsigned char line_numbers[4];
    char _pad6[3];
    unsigned char nodeclass_output[4], nodeclass_filter[4];
    unsigned char nodeclass_vector[4], nodeclass_texture[4];
    unsigned char nodeclass_shader[4], nodeclass_script[4];
    unsigned char nodeclass_pattern[4], nodeclass_layout[4];
    unsigned char nodeclass_geometry[4], nodeclass_attribute[4];
    unsigned char node_zone_simulation[4];
    unsigned char node_zone_repeat[4];
    unsigned char _pad9[4];
    unsigned char simulated_frames[4];
    unsigned char movie[4], movieclip[4], mask[4], image[4], scene[4], audio[4];
    unsigned char effect[4], transition[4], meta[4], text_strip[4], color_strip[4];
    unsigned char active_strip[4], selected_strip[4];
    char _pad7[1];
    float keyframe_scale_fac;
    unsigned char editmesh_active[4];
    unsigned char handle_vertex[4];
    unsigned char handle_vertex_select[4];
    unsigned char handle_vertex_size;
    unsigned char clipping_border_3d[4];
    unsigned char marker_outline[4], marker[4], act_marker[4], sel_marker[4], dis_marker[4],      lock_marker[4];
    unsigned char bundle_solid[4];
    unsigned char path_before[4], path_after[4];
    unsigned char path_keyframe_before[4], path_keyframe_after[4];
    unsigned char camera_path[4];
    unsigned char camera_passepartout[4];
    unsigned char _pad1[6];
    unsigned char gp_vertex_size;
    unsigned char gp_vertex[4], gp_vertex_select[4];
    unsigned char preview_back[4];
    unsigned char preview_stitch_face[4];
    unsigned char preview_stitch_edge[4];
    unsigned char preview_stitch_vert[4];
    unsigned char preview_stitch_stitchable[4];
    unsigned char preview_stitch_unstitchable[4];
    unsigned char preview_stitch_active[4];
    unsigned char uv_shadow[4];
    unsigned char match[4];
    unsigned char selected_highlight[4];
    unsigned char selected_object[4];
    unsigned char active_object[4];
    unsigned char edited_object[4];
    unsigned char row_alternate[4];
    unsigned char skin_root[4];
    unsigned char anim_active[4];
    unsigned char anim_non_active[4];
    unsigned char anim_preview_range[4];
    unsigned char nla_tweaking[4];
    unsigned char nla_tweakdupli[4];
    unsigned char nla_track[4];
    unsigned char nla_transition[4], nla_transition_sel[4];
    unsigned char nla_meta[4], nla_meta_sel[4];
    unsigned char nla_sound[4], nla_sound_sel[4];
    unsigned char info_selected[4], info_selected_text[4];
    unsigned char info_error[4], info_error_text[4];
    unsigned char info_warning[4], info_warning_text[4];
    unsigned char info_info[4], info_info_text[4];
    unsigned char info_debug[4], info_debug_text[4];
    unsigned char info_property[4], info_property_text[4];
    unsigned char info_operator[4], info_operator_text[4];
    unsigned char paint_curve_pivot[4];
    unsigned char paint_curve_handle[4];
    unsigned char metadatabg[4];
    unsigned char metadatatext[4];
};

struct wmWindowManager4_0_0 {
    ID3_4_0 id;
    wmWindow4_0_0 *windrawable;
    wmWindow4_0_0 *winactive;
    ListBase2_93_0 windows;
    uint8_t init_flag;
    char _pad0[1];
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    ListBase2_93_0 operators;
    ListBase2_93_0 notifier_queue;
    void *notifier_queue_set;
    ReportList2_93_0 reports;
    ListBase2_93_0 jobs;
    ListBase2_93_0 paintcursors;
    ListBase2_93_0 drags;
    ListBase2_93_0 keyconfigs;
    wmKeyConfig2_93_0 *defaultconf;
    wmKeyConfig2_93_0 *addonconf;
    wmKeyConfig2_93_0 *userconf;
    ListBase2_93_0 timers;
    void *autosavetimer;
    void *undo_stack;
    char is_interface_locked;
    char _pad[7];
    void *message_bus;
    wmXrData2_93_0 xr;
};

struct wmWindow4_0_0 {
    wmWindow4_0_0 *next, *prev;
    void *ghostwin;
    void *gpuctx;
    wmWindow4_0_0 *parent;
    Scene4_0_0 *scene;
    Scene4_0_0 *new_scene;
    char view_layer_name[64];
    Scene4_0_0 *unpinned_scene;
    WorkSpaceInstanceHook2_93_0 *workspace_hook;
    ScrAreaMap2_93_0 global_areas;
    bScreen2_93_0 *screen;
    int winid;
    short posx, posy;
    short sizex, sizey;
    char windowstate;
    char active;
    short cursor;
    short lastcursor;
    short modalcursor;
    short grabcursor;
    short pie_event_type_lock;
    short pie_event_type_last;
    char addmousemove;
    char tag_cursor_refresh;
    char event_queue_check_click;
    char event_queue_check_drag;
    char event_queue_check_drag_handled;
    char event_queue_consecutive_gesture_type;
    int event_queue_consecutive_gesture_xy[2];
    void *event_queue_consecutive_gesture_data;
    void *eventstate;
    void *event_last_handled;
    void *ime_data;
    ListBase2_93_0 event_queue;
    ListBase2_93_0 handlers;
    ListBase2_93_0 modalhandlers;
    ListBase2_93_0 gesture;
    Stereo3dFormat2_93_0 *stereo3d_format;
    ListBase2_93_0 drawcalls;
    void *cursor_keymap_status;
};

struct SpaceAction4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    bAction3_1_0 *action;
    bDopeSheet2_93_0 ads;
    float timeslide;
    short flag;
    char mode;
    char mode_prev;
    char autosnap;
    char cache_display;
    char _pad1[6];
    SpaceAction_Runtime2_93_0 runtime;
};

struct ARegion4_0_0 {
    ARegion4_0_0 *next, *prev;
    View2D4_0_0 v2d;
    rcti2_93_0 winrct;
    rcti2_93_0 drawrct;
    short winx, winy;
    int category_scroll;
    char _pad0[4];
    short visible;
    short regiontype;
    short alignment;
    short flag;
    short sizex, sizey;
    short do_draw;
    short do_draw_paintcursor;
    short overlap;
    short flagfullscreen;
    void *type;
    ListBase2_93_0 uiblocks;
    ListBase2_93_0 panels;
    ListBase2_93_0 panels_category_active;
    ListBase2_93_0 ui_lists;
    ListBase2_93_0 ui_previews;
    ListBase2_93_0 handlers;
    ListBase2_93_0 panels_category;
    void *gizmo_map;
    void *regiontimer;
    void *draw_buffer;
    char *headerstr;
    void *regiondata;
    ARegion_Runtime3_1_0 runtime;
};

struct CurvesModifierData4_0_0 {
    SequenceModifierData2_93_0 modifier;
    CurveMapping4_0_0 curve_mapping;
};

struct HueCorrectModifierData4_0_0 {
    SequenceModifierData2_93_0 modifier;
    CurveMapping4_0_0 curve_mapping;
};

struct SpaceOutliner4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    ListBase2_93_0 tree;
    void *treestore;
    char search_string[64];
    short flag;
    short outlinevis;
    short lib_override_view_mode;
    short storeflag;
    char search_flags;
    char _pad[6];
    char sync_select_dirty;
    int filter;
    char filter_state;
    char show_restrict_flags;
    short filter_id_type;
    void *runtime;
};

struct SpaceGraph4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    bDopeSheet2_93_0 *ads;
    short mode;
    short autosnap;
    int flag;
    float cursorTime;
    float cursorVal;
    int around;
    char _pad[4];
    SpaceGraph_Runtime2_93_0 runtime;
};

struct SpaceNla4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    short autosnap;
    short flag;
    char _pad[4];
    bDopeSheet2_93_0 *ads;
    View2D4_0_0 v2d;
};

struct SpaceSeq4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    float xof , yof;
    short mainb;
    short render_size;
    short chanshown;
    short zebra;
    int flag;
    float zoom;
    char view;
    char overlay_frame_type;
    char draw_flag;
    char gizmo_flag;
    char _pad[4];
    float cursor[2];
    bGPdata3_0_0 *gpd;
    SequencerScopes2_93_0 scopes;
    SequencerPreviewOverlay3_0_0 preview_overlay;
    SequencerTimelineOverlay3_0_0 timeline_overlay;
    char multiview_eye;
    char _pad2[7];
    SpaceSeqRuntime3_2_0 runtime;
};

struct bTheme4_0_0 {
    bTheme4_0_0 *next, *prev;
    char name[32];
    ThemeUI4_0_0 tui;
    ThemeSpace4_0_0 space_properties;
    ThemeSpace4_0_0 space_view3d;
    ThemeSpace4_0_0 space_file;
    ThemeSpace4_0_0 space_graph;
    ThemeSpace4_0_0 space_info;
    ThemeSpace4_0_0 space_action;
    ThemeSpace4_0_0 space_nla;
    ThemeSpace4_0_0 space_sequencer;
    ThemeSpace4_0_0 space_image;
    ThemeSpace4_0_0 space_text;
    ThemeSpace4_0_0 space_outliner;
    ThemeSpace4_0_0 space_node;
    ThemeSpace4_0_0 space_preferences;
    ThemeSpace4_0_0 space_console;
    ThemeSpace4_0_0 space_clip;
    ThemeSpace4_0_0 space_topbar;
    ThemeSpace4_0_0 space_statusbar;
    ThemeSpace4_0_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

#endif
