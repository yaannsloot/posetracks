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

#include <stdint.h>
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_1_0.h"
#include "makesdna_types_3_5_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_6_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_3_0.h"

struct bPoseChannel_BBoneSegmentBoundary4_0_0;
struct bPoseChannel_Runtime4_0_0;
struct bPoseChannel4_0_0;
struct BoneColor4_0_0;
struct Bone_Runtime4_0_0;
struct Bone4_0_0;
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
struct CurvesGeometry4_0_0;
struct CustomData4_0_0;
struct bGPDstroke4_0_0;
struct OpacityGpencilModifierData4_0_0;
struct GreasePencilDrawingBase4_0_0;
struct GreasePencilDrawing4_0_0;
struct GreasePencilDrawingReference4_0_0;
struct GreasePencilFrame4_0_0;
struct GreasePencilLayerFramesMapStorage4_0_0;
struct GreasePencilLayerMask4_0_0;
struct GreasePencilLayerTreeNode4_0_0;
struct GreasePencilLayer4_0_0;
struct GreasePencilLayerTreeGroup4_0_0;
struct GreasePencilOnionSkinningSettings4_0_0;
struct GreasePencil4_0_0;
struct IDOverrideLibraryPropertyOperation4_0_0;
struct IpoCurve4_0_0;
struct Ipo4_0_0;
struct LayerCollection4_0_0;
struct LightProbe4_0_0;
struct LightProbeConnectivityData4_0_0;
struct Light4_0_0;
struct Mesh4_0_0;
struct NodesModifierBake4_0_0;
struct NodesModifierData4_0_0;
struct MeshToVolumeModifierData4_0_0;
struct bNodeTreeInterfaceItem4_0_0;
struct bNodeTreeInterfaceSocket4_0_0;
struct bNodeTreeInterfacePanel4_0_0;
struct bNodeTreeInterface4_0_0;
struct bNodeSocket4_0_0;
struct bNodePanelState4_0_0;
struct bNode4_0_0;
struct bNodePreview4_0_0;
struct bNestedNodePath4_0_0;
struct bNestedNodeRef4_0_0;
struct bNodeTree4_0_0;
struct bNodeSocketValueRotation4_0_0;
struct GeometryNodeAssetTraits4_0_0;
struct NodeKuwaharaData4_0_0;
struct NodeTexVoronoi4_0_0;
struct NodeShaderHairPrincipled4_0_0;
struct NodeGeometryRaycast4_0_0;
struct NodeRepeatItem4_0_0;
struct NodeGeometryRepeatInput4_0_0;
struct NodeGeometryRepeatOutput4_0_0;
struct LightLinkingRuntime4_0_0;
struct LightLinking4_0_0;
struct Object4_0_0;
struct ToolSettings4_0_0;
struct RaytraceEEVEE4_0_0;
struct SceneEEVEE4_0_0;
struct SceneHydra4_0_0;
struct Scene4_0_0;
struct Panel4_0_0;
struct AssetShelfSettings4_0_0;
struct AssetShelf4_0_0;
struct RegionAssetShelf4_0_0;
struct SeqRetimingKey4_0_0;
struct Sequence4_0_0;
struct EQCurveMappingData4_0_0;
struct SoundEqualizerModifierData4_0_0;
struct FileAssetSelectParams4_0_0;
struct SpaceNodeOverlay4_0_0;
struct SpaceNode4_0_0;
struct MTex4_0_0;
struct Tex4_0_0;
struct ThemeUI4_0_0;
struct ThemeAssetShelf4_0_0;
struct ThemeSpace4_0_0;
struct bAddon4_0_0;
struct bUserAssetLibrary4_0_0;
struct bUserExtensionRepo4_0_0;
struct UserDef_Experimental4_0_0;
struct UserDef4_0_0;
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
struct World4_0_0;
struct SpaceAction4_0_0;
struct Brush4_0_0;
struct CurveMapping4_0_0;
struct Curves4_0_0;
struct PointCloud4_0_0;
struct RenderData4_0_0;
struct ARegion4_0_0;
struct CurvesModifierData4_0_0;
struct HueCorrectModifierData4_0_0;
struct SpaceProperties4_0_0;
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
    Bone4_0_0 *bone;
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
    Material3_5_0 *material;
    Material3_5_0 *material_alt;
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

struct GreasePencilFrame4_0_0 {
    int drawing_index;
    uint32_t flag;
    int8_t type;
    char _pad[3];
};

struct GreasePencilLayerFramesMapStorage4_0_0 {
    int *keys;
    GreasePencilFrame4_0_0 *values;
    int num;
    int flag;
};

struct GreasePencilLayerMask4_0_0 {
    GreasePencilLayerMask4_0_0 *next, *prev;
    char *layer_name;
    uint16_t flag;
    char _pad[6];
};

struct GreasePencilLayerTreeNode4_0_0 {
    GreasePencilLayerTreeNode4_0_0 *next, *prev;
    GreasePencilLayerTreeGroup4_0_0 *parent;
    char *name;
    int8_t type;
    uint8_t color[3];
    uint32_t flag;
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

struct bNodePanelState4_0_0 {
    int identifier;
    char flag;
    char _pad[3];
};

struct bNestedNodePath4_0_0 {
    int32_t node_id;
    int32_t id_in_node;
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

struct NodeRepeatItem4_0_0 {
    char *name;
    short socket_type;
    char _pad[2];
    int identifier;
};

struct NodeGeometryRepeatInput4_0_0 {
    int32_t output_node_id;
};

struct NodeGeometryRepeatOutput4_0_0 {
    NodeRepeatItem4_0_0 *items;
    int items_num;
    int active_index;
    int next_identifier;
    int inspection_index;
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

struct MTex4_0_0 {
    short texco, mapto, blendtype;
    char _pad2[2];
    Object4_0_0 *object;
    Tex4_0_0 *tex;
    char uvname[68];
    char projx, projy, projz, mapping;
    char brush_map_mode, brush_angle_mode;
    short which_output;
    float ofs[3], size[3], rot, random_angle;
    float r, g, b, k;
    float def_var;
    float colfac;
    float alphafac;
    float timefac, lengthfac, clumpfac, dampfac;
    float kinkfac, kinkampfac, roughfac, padensfac, gravityfac;
    float lifefac, sizefac, ivelfac, fieldfac;
    float twistfac;
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
    Object4_0_0 *ob;
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
    GreasePencil4_0_0 *id_reference;
};

struct bNodeTreeInterfaceSocket4_0_0 {
    bNodeTreeInterfaceItem4_0_0 item;
    char *name;
    char *description;
    char *socket_type;
    int flag;
    int attribute_domain;
    char *default_attribute_name;
    char *identifier;
    void *socket_data;
    IDProperty3_5_0 *properties;
};

struct bNodeTreeInterfacePanel4_0_0 {
    bNodeTreeInterfaceItem4_0_0 item;
    char *name;
    char *description;
    int flag;
    char _pad[4];
    bNodeTreeInterfaceItem4_0_0 **items_array;
    int items_num;
    int identifier;
};

struct bNestedNodeRef4_0_0 {
    int32_t id;
    char _pad[4];
    bNestedNodePath4_0_0 path;
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

struct BoneColor4_0_0 {
    int8_t palette_index;
    uint8_t _pad0[7];
    ThemeWireColor2_93_0 custom;
};

struct bPoseChannel4_0_0 {
    bPoseChannel4_0_0 *next, *prev;
    IDProperty3_5_0 *prop;
    ListBase2_93_0 constraints;
    char name[64];
    short flag;
    short ikflag;
    short protectflag;
    short agrp_index;
    char constflag;
    char selectflag;
    char drawflag;
    char bboneflag;
    char _pad0[4];
    Bone4_0_0 *bone;
    bPoseChannel4_0_0 *parent;
    bPoseChannel4_0_0 *child;
    ListBase2_93_0 iktree;
    ListBase2_93_0 siktree;
    bMotionPath2_93_0 *mpath;
    Object4_0_0 *custom;
    bPoseChannel4_0_0 *custom_tx;
    float custom_scale;
    float custom_scale_xyz[3];
    float custom_translation[3];
    float custom_rotation_euler[3];
    float loc[3];
    float size[3];
    float eul[3];
    float quat[4];
    float rotAxis[3], rotAngle;
    short rotmode;
    char _pad[2];
    float chan_mat[4][4];
    float pose_mat[4][4];
    float disp_mat[4][4];
    float disp_tail_mat[4][4];
    float constinv[4][4];
    float pose_head[3];
    float pose_tail[3];
    float limitmin[3], limitmax[3];
    float stiffness[3];
    float ikstretch;
    float ikrotweight;
    float iklinweight;
    float roll1, roll2;
    float curve_in_x, curve_in_z;
    float curve_out_x, curve_out_z;
    float ease1, ease2;
    float scale_in_x , scale_in_z;
    float scale_out_x , scale_out_z;
    float scale_in[3], scale_out[3];
    bPoseChannel4_0_0 *bbone_prev;
    bPoseChannel4_0_0 *bbone_next;
    void *temp;
    bPoseChannelDrawData2_93_0 *draw_data;
    bPoseChannel4_0_0 *orig_pchan;
    BoneColor4_0_0 color;
    bPoseChannel_Runtime4_0_0 runtime;
};

struct Bone_Runtime4_0_0 {
    ListBase2_93_0 collections;
};

struct Bone4_0_0 {
    Bone4_0_0 *next, *prev;
    IDProperty3_5_0 *prop;
    Bone4_0_0 *parent;
    ListBase2_93_0 childbase;
    char name[64];
    float roll;
    float head[3];
    float tail[3];
    float bone_mat[3][3];
    int flag;
    char _pad1[4];
    BoneColor4_0_0 color;
    char inherit_scale_mode;
    char _pad[3];
    float arm_head[3];
    float arm_tail[3];
    float arm_mat[4][4];
    float arm_roll;
    float dist, weight;
    float xwidth, length, zwidth;
    float rad_head, rad_tail;
    float roll1, roll2;
    float curve_in_x, curve_in_z;
    float curve_out_x, curve_out_z;
    float ease1, ease2;
    float scale_in_x , scale_in_z;
    float scale_out_x , scale_out_z;
    float scale_in[3], scale_out[3];
    float size[3];
    int layer;
    short segments;
    char bbone_mapping_mode;
    char _pad2[7];
    char bbone_prev_type;
    char bbone_next_type;
    int bbone_flag;
    short bbone_prev_flag;
    short bbone_next_flag;
    Bone4_0_0 *bbone_prev;
    Bone4_0_0 *bbone_next;
    Bone_Runtime4_0_0 runtime;
};

struct bArmature4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 bonebase;
    void *bonehash;
    void *_pad1;
    ListBase2_93_0 *edbo;
    Bone4_0_0 *act_bone;
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
    Object4_0_0 *dof_ob;
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

struct CurvesGeometry4_0_0 {
    int *curve_offsets;
    CustomData4_0_0 point_data;
    CustomData4_0_0 curve_data;
    int point_num;
    int curve_num;
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad[4];
    void *runtime;
};

struct bGPDstroke4_0_0 {
    bGPDstroke4_0_0 *next, *prev;
    bGPDspoint2_93_0 *points;
    bGPDtriangle2_93_0 *triangles;
    int totpoints;
    int tot_triangles;
    short thickness;
    short flag, _pad[2];
    double inittime;
    char colorname[128];
    int mat_nr;
    short caps[2];
    float hardness;
    float aspect_ratio[2];
    float fill_opacity_fac;
    float boundbox_min[3];
    float boundbox_max[3];
    float uv_rotation;
    float uv_translation[2];
    float uv_scale;
    int select_index;
    char _pad4[4];
    MDeformVert2_93_0 *dvert;
    void *_pad3;
    float vert_color_fill[4];
    bGPDcurve2_93_0 *editcurve;
    bGPDstroke_Runtime3_4_0 runtime;
    void *_pad5;
};

struct OpacityGpencilModifierData4_0_0 {
    GpencilModifierData2_93_0 modifier;
    Material3_5_0 *material;
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

struct GreasePencilDrawing4_0_0 {
    GreasePencilDrawingBase4_0_0 base;
    CurvesGeometry4_0_0 geometry;
    void *runtime;
};

struct GreasePencilLayer4_0_0 {
    GreasePencilLayerTreeNode4_0_0 base;
    GreasePencilLayerFramesMapStorage4_0_0 frames_storage;
    int8_t blend_mode;
    char _pad[3];
    float opacity;
    ListBase2_93_0 masks;
    void *runtime;
};

struct GreasePencilLayerTreeGroup4_0_0 {
    GreasePencilLayerTreeNode4_0_0 base;
    ListBase2_93_0 children;
    void *runtime;
};

struct GreasePencil4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    GreasePencilDrawingBase4_0_0 **drawing_array;
    int drawing_array_num;
    char _pad[4];
    GreasePencilLayerTreeGroup4_0_0 *root_group_ptr;
    GreasePencilLayer4_0_0 *active_layer;
    Material3_5_0 **material_array;
    short material_array_num;
    char _pad2[2];
    uint32_t flag;
    GreasePencilOnionSkinningSettings4_0_0 onion_skinning_settings;
    void *runtime;
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

struct Light4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    short type, flag;
    int mode;
    float r, g, b;
    float energy;
    float radius;
    float spotsize;
    float spotblend;
    short area_shape;
    short _pad1;
    float area_size;
    float area_sizey;
    float area_sizez;
    float area_spread;
    float sun_angle;
    float shdwr, shdwg, shdwb;
    short pr_texture, use_nodes;
    float bias;
    float clipsta;
    float clipend;
    float cascade_max_dist;
    float cascade_exponent;
    float cascade_fade;
    int cascade_count;
    float contact_dist;
    float contact_bias;
    float contact_thickness;
    float diff_fac, volume_fac;
    float spec_fac, att_dist;
    float shadow_softness_factor;
    float shadow_trace_distance;
    float _pad3;
    PreviewImage2_93_0 *preview;
    bNodeTree4_0_0 *nodetree;
    Ipo4_0_0 *ipo;
    float energy_deprecated;
    float _pad2;
};

struct Mesh4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    Ipo4_0_0 *ipo;
    Key2_93_0 *key;
    Material3_5_0 **mat;
    int totvert;
    int totedge;
    int faces_num;
    int totloop;
    int *face_offset_indices;
    CustomData4_0_0 vert_data;
    CustomData4_0_0 edge_data;
    CustomData4_0_0 face_data;
    CustomData4_0_0 loop_data;
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    void *edit_mesh;
    MSelect2_93_0 *mselect;
    int totselect;
    int act_face;
    Mesh4_0_0 *texcomesh;
    float texspace_location[3];
    float texspace_size[3];
    char texspace_flag;
    char editflag;
    uint16_t flag;
    float smoothresh;
    float remesh_voxel_size;
    float remesh_voxel_adaptivity;
    int face_sets_color_seed;
    int face_sets_color_default;
    char *active_color_attribute;
    char *default_color_attribute;
    char symmetry;
    char remesh_mode;
    short totcol;
    char cd_flag;
    char subdiv;
    char subdivr;
    char subsurftype;
    MPoly3_6_0 *mpoly;
    MLoop2_93_0 *mloop;
    MVert3_5_0 *mvert;
    MEdge3_6_0 *medge;
    MDeformVert2_93_0 *dvert;
    MTFace2_93_0 *mtface;
    TFace2_93_0 *tface;
    MCol2_93_0 *mcol;
    MFace2_93_0 *mface;
    CustomData4_0_0 fdata_legacy;
    int totface_legacy;
    char _pad1[4];
    void *runtime;
};

struct NodesModifierData4_0_0 {
    ModifierData3_5_0 modifier;
    bNodeTree4_0_0 *node_group;
    NodesModifierSettings2_93_0 settings;
    char *simulation_bake_directory;
    int8_t flag;
    char _pad[3];
    int bakes_num;
    NodesModifierBake4_0_0 *bakes;
    void *_pad2;
    void *runtime;
};

struct MeshToVolumeModifierData4_0_0 {
    ModifierData3_5_0 modifier;
    Object4_0_0 *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    float interior_band_width;
    float density;
    char _pad2[4];
    void *_pad3;
};

struct bNodeTreeInterface4_0_0 {
    bNodeTreeInterfacePanel4_0_0 root_panel;
    int active_index;
    int next_uid;
    void *runtime;
};

struct bNodeSocket4_0_0 {
    bNodeSocket4_0_0 *next, *prev;
    IDProperty3_5_0 *prop;
    char identifier[64];
    char name[64];
    void *storage;
    short type;
    short flag;
    short limit;
    short in_out;
    void *typeinfo;
    char idname[64];
    void *default_value;
    short stack_index;
    char display_shape;
    char attribute_domain;
    char _pad[4];
    char label[64];
    char short_label[64];
    char description[64];
    char *default_attribute_name;
    int own_index;
    int to_index;
    bNodeLink2_93_0 *link;
    bNodeStack2_93_0 ns;
    void *runtime;
};

struct bNode4_0_0 {
    bNode4_0_0 *next, *prev;
    ListBase2_93_0 inputs, outputs;
    char name[64];
    int32_t identifier;
    int flag;
    char idname[64];
    void *typeinfo;
    int16_t type;
    char _pad1[2];
    int16_t custom1, custom2;
    float custom3, custom4;
    ID3_4_0 *id;
    void *storage;
    IDProperty3_5_0 *prop;
    bNode4_0_0 *parent;
    float locx, locy;
    float width, height;
    float offsetx, offsety;
    char label[64];
    float color[3];
    int num_panel_states;
    bNodePanelState4_0_0 *panel_states_array;
    void *runtime;
};

struct bNodePreview4_0_0 {
    bNodeInstanceHashEntry2_93_0 hash_entry;
    void *ibuf;
};

struct bNodeTree4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    ID3_4_0 *owner_id;
    void *typeinfo;
    char idname[64];
    bGPdata3_0_0 *gpd;
    float view_center[2];
    ListBase2_93_0 nodes, links;
    int type;
    int cur_index;
    int flag;
    short edit_quality;
    short render_quality;
    int chunksize;
    int execution_mode;
    rctf2_93_0 viewer_border;
    ListBase2_93_0 inputs_legacy , outputs_legacy;
    bNodeTreeInterface4_0_0 tree_interface;
    void *previews;
    bNodeInstanceKey2_93_0 active_viewer_key;
    int nested_node_refs_num;
    bNestedNodeRef4_0_0 *nested_node_refs;
    GeometryNodeAssetTraits4_0_0 *geometry_node_asset_traits;
    PreviewImage2_93_0 *preview;
    void *runtime;
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

struct Object4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    void *sculpt;
    short type;
    short partype;
    int par1, par2, par3;
    char parsubstr[64];
    Object4_0_0 *parent, *track;
    Object4_0_0 *proxy;
    Object4_0_0 *proxy_group;
    Object4_0_0 *proxy_from;
    Ipo4_0_0 *ipo;
    bAction3_1_0 *action;
    bAction3_1_0 *poselib;
    bPose3_2_0 *pose;
    void *data;
    bGPdata3_0_0 *gpd;
    bAnimVizSettings3_2_0 avs;
    bMotionPath2_93_0 *mpath;
    void *_pad0;
    ListBase2_93_0 constraintChannels;
    ListBase2_93_0 effect;
    ListBase2_93_0 defbase;
    ListBase2_93_0 fmaps;
    ListBase2_93_0 modifiers;
    ListBase2_93_0 greasepencil_modifiers;
    ListBase2_93_0 shader_fx;
    int mode;
    int restore_mode;
    Material3_5_0 **mat;
    char *matbits;
    int totcol;
    int actcol;
    float loc[3], dloc[3];
    float scale[3];
    float dsize[3];
    float dscale[3];
    float rot[3], drot[3];
    float quat[4], dquat[4];
    float rotAxis[3], drotAxis[3];
    float rotAngle, drotAngle;
    float object_to_world[4][4];
    float world_to_object[4][4];
    float parentinv[4][4];
    float constinv[4][4];
    unsigned int lay;
    short flag;
    short colbits;
    short transflag, protectflag;
    short trackflag, upflag;
    short nlaflag;
    char _pad1;
    char duplicator_visibility_flag;
    short base_flag;
    unsigned short base_local_view_bits;
    unsigned short col_group, col_mask;
    short rotmode;
    char boundtype;
    char collision_boundtype;
    short dtx;
    char dt;
    char empty_drawtype;
    float empty_drawsize;
    float instance_faces_scale;
    short index;
    unsigned short actdef;
    char _pad2[4];
    float color[4];
    short softflag;
    short visibility_flag;
    short shapenr;
    char shapeflag;
    char _pad3[1];
    ListBase2_93_0 constraints;
    ListBase2_93_0 nlastrips;
    ListBase2_93_0 hooks;
    ListBase2_93_0 particlesystem;
    PartDeflect2_93_0 *pd;
    SoftBody3_0_0 *soft;
    Collection4_0_0 *instance_collection;
    FluidsimSettings2_93_0 *fluidsimSettings;
    ListBase2_93_0 pc_ids;
    RigidBodyOb2_93_0 *rigidbody_object;
    RigidBodyCon2_93_0 *rigidbody_constraint;
    float ima_ofs[2];
    ImageUser3_0_0 *iuser;
    char empty_image_visibility_flag;
    char empty_image_depth;
    char empty_image_flag;
    uint8_t modifier_flag;
    char _pad8[4];
    PreviewImage2_93_0 *preview;
    ObjectLineArt3_3_0 lineart;
    LightgroupMembership3_2_0 *lightgroup;
    LightLinking4_0_0 *light_linking;
    LightProbeObjectCache3_6_0 *lightprobe_cache;
    Object_Runtime3_0_0 runtime;
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
    Object4_0_0 *camera;
    World4_0_0 *world;
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
    bNodeTree4_0_0 *nodetree;
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

struct AssetShelfSettings4_0_0 {
    AssetShelfSettings4_0_0 *next, *prev;
    AssetLibraryReference3_0_0 asset_library_reference;
    ListBase2_93_0 enabled_catalog_paths;
    const char *active_catalog_path;
    char search_string[64];
    short preview_size;
    short display_flag;
    char _pad1[4];
};

struct AssetShelf4_0_0 {
    AssetShelf4_0_0 *next, *prev;
    char idname[64];
    void *type;
    AssetShelfSettings4_0_0 settings;
    short preferred_row_count;
    char _pad[6];
};

struct RegionAssetShelf4_0_0 {
    ListBase2_93_0 shelves;
    AssetShelf4_0_0 *active_shelf;
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
    Object4_0_0 *scene_camera;
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
    bNodeTree4_0_0 *edittree;
    bNodeTree4_0_0 *nodetree;
    char tree_idname[64];
    int treetype;
    short texfrom;
    char shaderfrom;
    char geometry_nodes_type;
    bNodeTree4_0_0 *geometry_nodes_tool_tree;
    bGPdata3_0_0 *gpd;
    SpaceNodeOverlay4_0_0 overlay;
    void *runtime;
};

struct Tex4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    float noisesize, turbul;
    float bright, contrast, saturation, rfac, gfac, bfac;
    float filtersize;
    char _pad2[4];
    float mg_H, mg_lacunarity, mg_octaves, mg_offset, mg_gain;
    float dist_amount, ns_outscale;
    float vn_w1;
    float vn_w2;
    float vn_w3;
    float vn_w4;
    float vn_mexp;
    short vn_distm, vn_coltype;
    short noisedepth, noisetype;
    short noisebasis, noisebasis2;
    short imaflag, flag;
    short type, stype;
    float cropxmin, cropymin, cropxmax, cropymax;
    int texfilter;
    int afmax;
    short xrepeat, yrepeat;
    short extend;
    short _pad0;
    int len;
    int frames;
    int offset;
    int sfra;
    float checkerdist, nabla;
    char _pad1[4];
    ImageUser3_0_0 iuser;
    bNodeTree4_0_0 *nodetree;
    Ipo4_0_0 *ipo;
    Image3_6_0 *ima;
    ColorBand2_93_0 *coba;
    PreviewImage2_93_0 *preview;
    char use_nodes;
    char _pad[7];
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

struct UserDef4_0_0 {
    int versionfile, subversionfile;
    int flag;
    unsigned int dupflag;
    char pref_flag;
    char savetime;
    char mouse_emulate_3_button_modifier;
    char _pad4[1];
    char tempdir[768];
    char fontdir[768];
    char renderdir[1024];
    char render_cachedir[768];
    char textudir[768];
    char pythondir_legacy[768];
    char sounddir[768];
    char i18ndir[768];
    char image_editor[1024];
    char text_editor[1024];
    char text_editor_args[256];
    char anim_player[1024];
    int anim_player_preset;
    short v2d_min_gridsize;
    short timecode_style;
    short versions;
    short dbl_click_time;
    char _pad0[3];
    char mini_axis_type;
    int uiflag;
    char uiflag2;
    char gpu_flag;
    char _pad8[6];
    char app_flag;
    char viewzoom;
    short language;
    int mixbufsize;
    int audiodevice;
    int audiorate;
    int audioformat;
    int audiochannels;
    float ui_scale;
    int ui_line_width;
    int dpi;
    float scale_factor;
    float inv_scale_factor;
    float pixelsize;
    int virtual_pixel;
    int scrollback;
    char node_margin;
    char node_preview_res;
    short transopts;
    short menuthreshold1, menuthreshold2;
    char app_template[64];
    ListBase2_93_0 themes;
    ListBase2_93_0 uifonts;
    ListBase2_93_0 uistyles;
    ListBase2_93_0 user_keymaps;
    ListBase2_93_0 user_keyconfig_prefs;
    ListBase2_93_0 addons;
    ListBase2_93_0 autoexec_paths;
    ListBase2_93_0 script_directories;
    ListBase2_93_0 user_menus;
    ListBase2_93_0 asset_libraries;
    ListBase2_93_0 extension_repos;
    char keyconfigstr[64];
    short active_asset_library;
    short active_extension_repo;
    char _pad14[6];
    short undosteps;
    int undomemory;
    float gpu_viewport_quality;
    short gp_manhattandist, gp_euclideandist, gp_eraser;
    short gp_settings;
    char _pad13[4];
    SolidLight3_4_0 light_param[4];
    float light_ambient[3];
    char gizmo_flag;
    char gizmo_size;
    char gizmo_size_navigate_v3d;
    char _pad3[5];
    short edit_studio_light;
    short lookdev_sphere_size;
    short vbotimeout, vbocollectrate;
    short textimeout, texcollectrate;
    int memcachelimit;
    int prefetchframes;
    float pad_rot_angle;
    char _pad12[4];
    short rvisize;
    short rvibright;
    short recent_files;
    short smooth_viewtx;
    short glreslimit;
    short color_picker_type;
    char auto_smoothing_new;
    char ipo_new;
    char keyhandles_new;
    char _pad11[4];
    char view_frame_type;
    int view_frame_keyframes;
    float view_frame_seconds;
    short gpu_backend;
    short playback_fps_samples;
    char _pad7[2];
    short widget_unit;
    short anisotropic_filter;
    short tablet_api;
    float pressure_threshold_max;
    float pressure_softness;
    float ndof_sensitivity;
    float ndof_orbit_sensitivity;
    float ndof_deadzone;
    int ndof_flag;
    short ogl_multisamples;
    short image_draw_method;
    float glalphaclip;
    short autokey_mode;
    short autokey_flag;
    short animation_flag;
    char text_render;
    char navigation_mode;
    float view_rotate_sensitivity_turntable;
    float view_rotate_sensitivity_trackball;
    ColorBand2_93_0 coba_weight;
    float sculpt_paint_overlay_col[3];
    float gpencil_new_layer_col[4];
    char drag_threshold_mouse;
    char drag_threshold_tablet;
    char drag_threshold;
    char move_threshold;
    char font_path_ui[1024];
    char font_path_ui_mono[1024];
    int compute_device_type;
    float fcu_inactive_alpha;
    short pie_tap_timeout;
    short pie_initial_timeout;
    short pie_animation_timeout;
    short pie_menu_confirm;
    short pie_menu_radius;
    short pie_menu_threshold;
    short _pad6[2];
    char factor_display_type;
    char viewport_aa;
    char render_display_type;
    char filebrowser_display_type;
    char sequencer_disk_cache_dir[1024];
    int sequencer_disk_cache_compression;
    int sequencer_disk_cache_size_limit;
    short sequencer_disk_cache_flag;
    short sequencer_proxy_setup;
    float collection_instance_empty_size;
    char text_flag;
    char _pad10[1];
    char file_preview_type;
    char statusbar_flag;
    WalkNavigation2_93_0 walk_navigation;
    UserDef_SpaceData2_93_0 space_data;
    UserDef_FileSpaceData2_93_0 file_space_data;
    UserDef_Experimental4_0_0 experimental;
    UserDef_Runtime2_93_0 runtime;
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

struct World4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    char _pad0[4];
    short texact, mistype;
    float horr, horg, horb;
    float exposure, exp, range;
    short mode;
    char _pad2[6];
    float misi, miststa, mistdist, misthi;
    float aodist, aoenergy;
    short flag;
    char _pad3[2];
    int probe_resolution;
    Ipo4_0_0 *ipo;
    short pr_texture, use_nodes;
    char _pad[4];
    PreviewImage2_93_0 *preview;
    bNodeTree4_0_0 *nodetree;
    LightgroupMembership3_2_0 *lightgroup;
    ListBase2_93_0 gpumaterial;
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

struct Brush4_0_0 {
    ID3_4_0 id;
    BrushClone2_93_0 clone;
    CurveMapping4_0_0 *curve;
    MTex4_0_0 mtex;
    MTex4_0_0 mask_mtex;
    Brush4_0_0 *toggle_brush;
    void *icon_imbuf;
    PreviewImage2_93_0 *preview;
    ColorBand2_93_0 *gradient;
    PaintCurve2_93_0 *paint_curve;
    char icon_filepath[1024];
    float normal_weight;
    float rake_factor;
    short blend;
    short ob_mode;
    float weight;
    int size;
    int flag;
    int flag2;
    int sampling_flag;
    int mask_pressure;
    float jitter;
    int jitter_absolute;
    int overlay_flags;
    int spacing;
    int smooth_stroke_radius;
    float smooth_stroke_factor;
    float rate;
    float rgb[3];
    float alpha;
    float hardness;
    float flow;
    float wet_mix;
    float wet_persistence;
    float density;
    int paint_flags;
    float tip_roundness;
    float tip_scale_x;
    float secondary_rgb[3];
    float dash_ratio;
    int dash_samples;
    int sculpt_plane;
    float plane_offset;
    int gradient_spacing;
    char gradient_stroke_mode;
    char gradient_fill_mode;
    char _pad0[5];
    char falloff_shape;
    float falloff_angle;
    char sculpt_tool;
    char uv_sculpt_tool;
    char vertexpaint_tool;
    char weightpaint_tool;
    char imagepaint_tool;
    char mask_tool;
    char gpencil_tool;
    char gpencil_vertex_tool;
    char gpencil_sculpt_tool;
    char gpencil_weight_tool;
    char curves_sculpt_tool;
    char _pad1[5];
    float autosmooth_factor;
    float tilt_strength_factor;
    float topology_rake_factor;
    float crease_pinch_factor;
    float normal_radius_factor;
    float area_radius_factor;
    float wet_paint_radius_factor;
    float plane_trim;
    float height;
    float texture_sample_bias;
    int curve_preset;
    float disconnected_distance_max;
    int deform_target;
    int automasking_flags;
    int automasking_boundary_edges_propagation_steps;
    int elastic_deform_type;
    float elastic_deform_volume_preservation;
    int snake_hook_deform_type;
    int pose_deform_type;
    float pose_offset;
    int pose_smooth_iterations;
    int pose_ik_segments;
    int pose_origin_type;
    int boundary_deform_type;
    int boundary_falloff_type;
    float boundary_offset;
    int cloth_deform_type;
    int cloth_force_falloff_type;
    int cloth_simulation_area_type;
    float cloth_mass;
    float cloth_damping;
    float cloth_sim_limit;
    float cloth_sim_falloff;
    float cloth_constraint_softbody_strength;
    int smooth_deform_type;
    float surface_smooth_shape_preservation;
    float surface_smooth_current_vertex;
    int surface_smooth_iterations;
    float multiplane_scrape_angle;
    int smear_deform_type;
    int slide_deform_type;
    int texture_overlay_alpha;
    int mask_overlay_alpha;
    int cursor_overlay_alpha;
    float unprojected_radius;
    float sharp_threshold;
    int blur_kernel_radius;
    int blur_mode;
    float fill_threshold;
    float add_col[4];
    float sub_col[4];
    float stencil_pos[2];
    float stencil_dimension[2];
    float mask_stencil_pos[2];
    float mask_stencil_dimension[2];
    BrushGpencilSettings4_0_0 *gpencil_settings;
    BrushCurvesSculptSettings3_5_0 *curves_sculpt_settings;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    CurveMapping4_0_0 *automasking_cavity_curve;
};

struct Curves4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    CurvesGeometry4_0_0 geometry;
    int flag;
    int attributes_active_index;
    Material3_5_0 **mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    Object4_0_0 *surface;
    char *surface_uv_map;
    void *batch_cache;
};

struct PointCloud4_0_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    int flag;
    int totpoint;
    CustomData4_0_0 pdata;
    int attributes_active_index;
    int _pad4;
    Material3_5_0 **mat;
    short totcol;
    short _pad3[3];
    void *runtime;
    void *batch_cache;
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

struct SpaceProperties4_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    short space_subtype;
    short mainb, mainbo, mainbuser;
    short preview;
    char _pad[4];
    char flag;
    char outliner_sync;
    void *path;
    int pathflag, dataicon;
    ID3_4_0 *pinid;
    void *texuser;
    void *runtime;
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
