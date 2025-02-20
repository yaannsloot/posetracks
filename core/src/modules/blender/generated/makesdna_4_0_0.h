/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef MAKESDNA_4_0_0_H
#define MAKESDNA_4_0_0_H

#include "makesdna_3_6_0.h"

struct ARegion4_0_0;
struct AssetShelf4_0_0;
struct AssetShelfSettings4_0_0;
struct BakeData4_0_0;
struct Base4_0_0;
struct Bone4_0_0;
struct BoneCollection4_0_0;
struct BoneCollectionMember4_0_0;
struct BoneCollectionReference4_0_0;
struct BoneColor4_0_0;
struct Bone_Runtime4_0_0;
struct BrightContrastModifierData4_0_0;
struct Brush4_0_0;
struct BrushGpencilSettings4_0_0;
struct Camera4_0_0;
struct Collection4_0_0;
struct CollectionChild4_0_0;
struct CollectionLightLinking4_0_0;
struct CollectionObject4_0_0;
struct ColorBalanceModifierData4_0_0;
struct CurveMap4_0_0;
struct Curves4_0_0;
struct CurvesGeometry4_0_0;
struct CustomData4_0_0;
struct Editing4_0_0;
struct FileAssetSelectParams4_0_0;
struct GP_Sculpt_Guide4_0_0;
struct GP_Sculpt_Settings4_0_0;
struct GeometryNodeAssetTraits4_0_0;
struct GreasePencil4_0_0;
struct GreasePencilDrawing4_0_0;
struct GreasePencilDrawingBase4_0_0;
struct GreasePencilDrawingReference4_0_0;
struct GreasePencilFrame4_0_0;
struct GreasePencilLayer4_0_0;
struct GreasePencilLayerFramesMapStorage4_0_0;
struct GreasePencilLayerMask4_0_0;
struct GreasePencilLayerTreeGroup4_0_0;
struct GreasePencilLayerTreeNode4_0_0;
struct GreasePencilOnionSkinningSettings4_0_0;
struct GroupNodeViewerPathElem4_0_0;
struct IDOverrideLibraryPropertyOperation4_0_0;
struct IDViewerPathElem4_0_0;
struct LayerCollection4_0_0;
struct Light4_0_0;
struct LightLinking4_0_0;
struct LightLinkingRuntime4_0_0;
struct LightProbe4_0_0;
struct LightProbeBakingData4_0_0;
struct LightProbeConnectivityData4_0_0;
struct LightProbeGridCacheFrame4_0_0;
struct LightProbeObjectCache4_0_0;
struct MTex4_0_0;
struct Mesh4_0_0;
struct MeshToVolumeModifierData4_0_0;
struct MetaStack4_0_0;
struct ModifierViewerPathElem4_0_0;
struct NodeGeometryRaycast4_0_0;
struct NodeGeometryRepeatInput4_0_0;
struct NodeGeometryRepeatOutput4_0_0;
struct NodeKuwaharaData4_0_0;
struct NodeRepeatItem4_0_0;
struct NodeShaderHairPrincipled4_0_0;
struct NodesModifierBake4_0_0;
struct NodesModifierData4_0_0;
struct ObHook4_0_0;
struct Object4_0_0;
struct OpacityGpencilModifierData4_0_0;
struct Panel4_0_0;
struct PointCloud4_0_0;
struct PointDensity4_0_0;
struct RaytraceEEVEE4_0_0;
struct RegionAssetShelf4_0_0;
struct RegionView3D4_0_0;
struct RepeatZoneViewerPathElem4_0_0;
struct ReportTimerInfo4_0_0;
struct SceneEEVEE4_0_0;
struct SceneHydra4_0_0;
struct ScrArea4_0_0;
struct Sculpt4_0_0;
struct SeqRetimingKey4_0_0;
struct Sequence4_0_0;
struct SequenceModifierData4_0_0;
struct SequencerMaskModifierData4_0_0;
struct SequencerTonemapModifierData4_0_0;
struct SimulationZoneViewerPathElem4_0_0;
struct SoundEqualizerModifierData4_0_0;
struct SpaceAction4_0_0;
struct SpaceFile4_0_0;
struct SpaceGraph4_0_0;
struct SpaceNla4_0_0;
struct SpaceNode4_0_0;
struct SpaceNodeOverlay4_0_0;
struct SpaceOutliner4_0_0;
struct SpaceProperties4_0_0;
struct SpaceSeq4_0_0;
struct TexMapping4_0_0;
struct ThemeAssetShelf4_0_0;
struct ThemeSpace4_0_0;
struct ThemeUI4_0_0;
struct TimeMarker4_0_0;
struct UserDef_Experimental4_0_0;
struct View2D4_0_0;
struct ViewLayer4_0_0;
struct ViewerNodeViewerPathElem4_0_0;
struct ViewerPathElem4_0_0;
struct WhiteBalanceModifierData4_0_0;
struct World4_0_0;
struct XrSessionSettings4_0_0;
struct bAddon4_0_0;
struct bArmature4_0_0;
struct bArmature_Runtime4_0_0;
struct bGPDstroke4_0_0;
struct bGPDstroke_Runtime4_0_0;
struct bGPdata4_0_0;
struct bGPdata_Runtime4_0_0;
struct bNestedNodePath4_0_0;
struct bNestedNodeRef4_0_0;
struct bNode4_0_0;
struct bNodeLink4_0_0;
struct bNodePanelState4_0_0;
struct bNodePreview4_0_0;
struct bNodeSocket4_0_0;
struct bNodeSocketValueObject4_0_0;
struct bNodeSocketValueRotation4_0_0;
struct bNodeTree4_0_0;
struct bNodeTreeInterface4_0_0;
struct bNodeTreeInterfaceItem4_0_0;
struct bNodeTreeInterfacePanel4_0_0;
struct bNodeTreeInterfaceSocket4_0_0;
struct bNodeTreePath4_0_0;
struct bPose4_0_0;
struct bPoseChannel4_0_0;
struct bPoseChannel_BBoneSegmentBoundary4_0_0;
struct bPoseChannel_Runtime4_0_0;
struct bScreen4_0_0;
struct bUserAssetLibrary4_0_0;
struct bUserExtensionRepo4_0_0;
struct vec4f4_0_0;
struct wmWindow4_0_0;
struct wmWindowManager4_0_0;
struct wmXrData4_0_0;

struct vec4f4_0_0 {
    float x;
    float y;
    float z;
    float w;
};

struct bArmature_Runtime4_0_0 {
    int active_collection_index;
    unsigned char _pad0[4];
    struct BoneCollection4_0_0 *active_collection;
};

struct CustomData4_0_0 {
    struct CustomDataLayer3_6_0 *layers;
    int typemap[53];
    int totlayer;
    int maxlayer;
    int totsize;
    void *pool;
    struct CustomDataExternal3_6_0 *external;
};

struct Base4_0_0 {
    struct Base4_0_0 *next;
    struct Base4_0_0 *prev;
    struct Object4_0_0 *object;
    struct Base4_0_0 *base_orig;
    unsigned int lay;
    short flag;
    short flag_from_collection;
    short flag_legacy;
    unsigned short local_view_bits;
    unsigned short local_collections_bits;
    char _pad1[2];
};

struct MetaStack4_0_0 {
    struct MetaStack4_0_0 *next;
    struct MetaStack4_0_0 *prev;
    struct ListBase3_6_0 *oldbasep;
    struct ListBase3_6_0 *old_channels;
    struct Sequence4_0_0 *parseq;
    int disp_range[2];
};

struct MTex4_0_0 {
    short texco;
    short mapto;
    short blendtype;
    char _pad2[2];
    struct Object4_0_0 *object;
    struct Tex3_6_0 *tex;
    char uvname[68];
    char projx;
    char projy;
    char projz;
    char mapping;
    char brush_map_mode;
    char brush_angle_mode;
    short which_output;
    float ofs[3];
    float size[3];
    float rot;
    float random_angle;
    float r;
    float g;
    float b;
    float k;
    float def_var;
    float colfac;
    float alphafac;
    float timefac;
    float lengthfac;
    float clumpfac;
    float dampfac;
    float kinkfac;
    float kinkampfac;
    float roughfac;
    float padensfac;
    float gravityfac;
    float lifefac;
    float sizefac;
    float ivelfac;
    float fieldfac;
    float twistfac;
};

struct ThemeAssetShelf4_0_0 {
    unsigned char header_back[4];
    unsigned char back[4];
};

struct ObHook4_0_0 {
    struct ObHook4_0_0 *next;
    struct ObHook4_0_0 *prev;
    struct Object4_0_0 *parent;
    float parentinv[4][4];
    float mat[4][4];
    float cent[3];
    float falloff;
    char name[64];
    int *indexar;
    int totindex;
    int curindex;
    short type;
    short active;
    float force;
};

struct TimeMarker4_0_0 {
    struct TimeMarker4_0_0 *next;
    struct TimeMarker4_0_0 *prev;
    int frame;
    char name[64];
    unsigned int flag;
    struct Object4_0_0 *camera;
    struct IDProperty3_6_0 *prop;
};

struct CurveMap4_0_0 {
    short totpoint;
    short flag;
    float range;
    float mintable;
    float maxtable;
    float ext_in[2];
    float ext_out[2];
    struct CurveMapPoint3_6_0 *curve;
    struct CurveMapPoint3_6_0 *table;
    struct CurveMapPoint3_6_0 *premultable;
    float premul_ext_in[2];
    float premul_ext_out[2];
    short default_handle_type;
    char _pad[6];
};

struct bNodeLink4_0_0 {
    struct bNodeLink4_0_0 *next;
    struct bNodeLink4_0_0 *prev;
    struct bNode4_0_0 *fromnode;
    struct bNode4_0_0 *tonode;
    struct bNodeSocket4_0_0 *fromsock;
    struct bNodeSocket4_0_0 *tosock;
    int flag;
    int multi_input_socket_index;
};

struct SpaceNodeOverlay4_0_0 {
    int flag;
    int preview_shape;
};

struct TexMapping4_0_0 {
    float loc[3];
    float rot[3];
    float size[3];
    int flag;
    char projx;
    char projy;
    char projz;
    char mapping;
    int type;
    float mat[4][4];
    float min[3];
    float max[3];
    struct Object4_0_0 *ob;
};

struct bGPDstroke_Runtime4_0_0 {
    char tmp_layerinfo[128];
    float multi_frame_falloff;
    int stroke_start;
    int fill_start;
    int vertex_start;
    int curve_start;
    int _pad0;
    struct bGPDstroke4_0_0 *gps_orig;
    void *_pad2;
};

struct bGPdata_Runtime4_0_0 {
    void *sbuffer;
    void *sbuffer_position_buf;
    void *sbuffer_color_buf;
    void *sbuffer_batch;
    struct bGPDstroke4_0_0 *sbuffer_gps;
    short playing;
    short matid;
    short sbuffer_sflag;
    char _pad1[2];
    int sbuffer_used;
    int sbuffer_size;
    float vert_color_fill[4];
    float arrow_start[8];
    float arrow_end[8];
    int arrow_start_style;
    int arrow_end_style;
    int tot_cp_points;
    char _pad2[4];
    struct bGPDcontrolpoint3_6_0 *cp_points;
    struct Brush4_0_0 *sbuffer_brush;
    void *gpencil_cache;
    void *lineart_cache;
    void *update_cache;
};

struct PointDensity4_0_0 {
    short flag;
    short falloff_type;
    float falloff_softness;
    float radius;
    short source;
    char _pad0[2];
    short color_source;
    short ob_color_source;
    int totpoints;
    struct Object4_0_0 *object;
    int psys;
    short psys_cache_space;
    short ob_cache_space;
    char vertex_attribute_name[68];
    char _pad1[4];
    void *point_tree;
    float *point_data;
    float noise_size;
    short noise_depth;
    short noise_influence;
    short noise_basis;
    char _pad2[6];
    float noise_fac;
    float speed_scale;
    float falloff_speed_scale;
    char _pad3[4];
    void *coba;
    void *falloff_curve;
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
    float clip[4][6];
    float clip_local[4][6];
    struct BoundBox3_6_0 *clipbb;
    struct RegionView3D4_0_0 *localvd;
    void *view_render;
    void *sms;
    void *smooth_timer;
    float twmat[4][4];
    float tw_axis_min[3];
    float tw_axis_max[3];
    float tw_axis_matrix[3][3];
    float gridview;
    float viewquat[4];
    float dist;
    float camdx;
    float camdy;
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

struct bAddon4_0_0 {
    struct bAddon4_0_0 *next;
    struct bAddon4_0_0 *prev;
    char module[128];
    struct IDProperty3_6_0 *prop;
};

struct ReportTimerInfo4_0_0 {
    float widthfac;
    float flash_progress;
};

struct SequenceModifierData4_0_0 {
    struct SequenceModifierData4_0_0 *next;
    struct SequenceModifierData4_0_0 *prev;
    int type;
    int flag;
    char name[64];
    int mask_input_type;
    int mask_time;
    struct Sequence4_0_0 *mask_sequence;
    void *mask_id;
};

struct IDOverrideLibraryPropertyOperation4_0_0 {
    struct IDOverrideLibraryPropertyOperation4_0_0 *next;
    struct IDOverrideLibraryPropertyOperation4_0_0 *prev;
    short operation;
    short flag;
    short tag;
    char _pad0[2];
    char *subitem_reference_name;
    char *subitem_local_name;
    int subitem_reference_index;
    int subitem_local_index;
    struct ID3_6_0 *subitem_reference_id;
    struct ID3_6_0 *subitem_local_id;
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
    signed char caps_type;
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
    void *curve_sensitivity;
    void *curve_strength;
    void *curve_jitter;
    void *curve_rand_pressure;
    void *curve_rand_strength;
    void *curve_rand_uv;
    void *curve_rand_hue;
    void *curve_rand_saturation;
    void *curve_rand_value;
    float outline_fac;
    char _pad1[4];
    void *material;
    void *material_alt;
};

struct CollectionLightLinking4_0_0 {
    unsigned char link_state;
    unsigned char _pad[3];
};

struct GP_Sculpt_Guide4_0_0 {
    char use_guide;
    char use_snapping;
    char reference_point;
    char type;
    char _pad2[4];
    float angle;
    float angle_snap;
    float spacing;
    float location[3];
    struct Object4_0_0 *reference_object;
};

struct RaytraceEEVEE4_0_0 {
    float screen_trace_quality;
    float screen_trace_thickness;
    int resolution_scale;
    float sample_clamp;
    int flag;
    int denoise_stages;
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

struct bNodeSocketValueObject4_0_0 {
    struct Object4_0_0 *value;
};

struct bUserAssetLibrary4_0_0 {
    struct bUserAssetLibrary4_0_0 *next;
    struct bUserAssetLibrary4_0_0 *prev;
    char name[64];
    char dirpath[1024];
    short import_method;
    short flag;
    char _pad0[4];
};

struct NodeGeometryRaycast4_0_0 {
    unsigned char mapping;
    signed char data_type;
};

struct ViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 *next;
    struct ViewerPathElem4_0_0 *prev;
    int type;
    char _pad[4];
    char *ui_name;
};

struct LightProbeBakingData4_0_0 {
    float *(L0[4]);
    float *(L1_a[4]);
    float *(L1_b[4]);
    float *(L1_c[4]);
    float *validity;
    float *(virtual_offset[4]);
};

struct LightProbeConnectivityData4_0_0 {
    unsigned char *validity;
};

struct LightProbeObjectCache4_0_0 {
    int cache_type;
    char shared;
    char dirty;
    char _pad0[2];
    struct LightProbeGridCacheFrame4_0_0 *grid_static_cache;
};

struct bUserExtensionRepo4_0_0 {
    struct bUserExtensionRepo4_0_0 *next;
    struct bUserExtensionRepo4_0_0 *prev;
    char name[64];
    char module[48];
    char dirpath[1024];
    char remote_path[1024];
    int flag;
    char _pad0[4];
};

struct BoneCollectionMember4_0_0 {
    struct BoneCollectionMember4_0_0 *next;
    struct BoneCollectionMember4_0_0 *prev;
    struct Bone4_0_0 *bone;
};

struct BoneCollectionReference4_0_0 {
    struct BoneCollectionReference4_0_0 *next;
    struct BoneCollectionReference4_0_0 *prev;
    struct BoneCollection4_0_0 *bcoll;
};

struct bPoseChannel_BBoneSegmentBoundary4_0_0 {
    float point[3];
    float plane_normal[3];
    float plane_offset;
    float depth_scale;
};

struct LightLinkingRuntime4_0_0 {
    unsigned long long light_set_membership;
    unsigned long long shadow_set_membership;
    unsigned char receiver_light_set;
    unsigned char blocker_shadow_set;
    unsigned char _pad[6];
};

struct GreasePencilDrawingBase4_0_0 {
    signed char type;
    char _pad[3];
    unsigned int flag;
};

struct GreasePencilFrame4_0_0 {
    int drawing_index;
    unsigned int flag;
    signed char type;
    char _pad[3];
};

struct GreasePencilLayerFramesMapStorage4_0_0 {
    int *keys;
    struct GreasePencilFrame4_0_0 *values;
    int num;
    int flag;
};

struct GreasePencilLayerMask4_0_0 {
    struct GreasePencilLayerMask4_0_0 *next;
    struct GreasePencilLayerMask4_0_0 *prev;
    char *layer_name;
    unsigned short flag;
    char _pad[6];
};

struct GreasePencilLayerTreeNode4_0_0 {
    struct GreasePencilLayerTreeNode4_0_0 *next;
    struct GreasePencilLayerTreeNode4_0_0 *prev;
    struct GreasePencilLayerTreeGroup4_0_0 *parent;
    char *name;
    signed char type;
    unsigned char color[3];
    unsigned int flag;
};

struct GreasePencilOnionSkinningSettings4_0_0 {
    float opacity;
    signed char mode;
    unsigned char filter;
    char _pad[2];
    short num_frames_before;
    short num_frames_after;
    float color_before[3];
    float color_after[3];
    char _pad2[4];
};

struct NodesModifierBake4_0_0 {
    int id;
    unsigned int flag;
    char *directory;
    int frame_start;
    int frame_end;
};

struct bNodeTreeInterfaceItem4_0_0 {
    char item_type;
    char _pad[7];
};

struct SceneHydra4_0_0 {
    int export_method;
    int _pad0;
};

struct bNodePanelState4_0_0 {
    int identifier;
    char flag;
    char _pad[3];
};

struct bNestedNodePath4_0_0 {
    int node_id;
    int id_in_node;
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

struct NodeRepeatItem4_0_0 {
    char *name;
    short socket_type;
    char _pad[2];
    int identifier;
};

struct NodeGeometryRepeatInput4_0_0 {
    int output_node_id;
};

struct NodeGeometryRepeatOutput4_0_0 {
    struct NodeRepeatItem4_0_0 *items;
    int items_num;
    int active_index;
    int next_identifier;
    int inspection_index;
};

struct SeqRetimingKey4_0_0 {
    int strip_frame_index;
    int flag;
    int _pad0;
    float retiming_factor;
    int original_strip_frame_index;
    float original_retiming_factor;
};

struct View2D4_0_0 {
    struct rctf3_6_0 tot;
    struct rctf3_6_0 cur;
    struct rcti3_6_0 vert;
    struct rcti3_6_0 hor;
    struct rcti3_6_0 mask;
    float min[2];
    float max[2];
    float minzoom;
    float maxzoom;
    short scroll;
    short scroll_ui;
    short keeptot;
    short keepzoom;
    short keepofs;
    short flag;
    short align;
    short winx;
    short winy;
    short oldwinx;
    short oldwiny;
    short around;
    char alpha_vert;
    char alpha_hor;
    char _pad[2];
    float page_size_y;
    void *sms;
    void *smooth_timer;
};

struct bPoseChannel_Runtime4_0_0 {
    struct SessionUUID3_6_0 session_uuid;
    struct DualQuat3_6_0 deform_dual_quat;
    int bbone_segments;
    float bbone_arc_length_reciprocal;
    char _pad1[4];
    void *bbone_rest_mats;
    void *bbone_pose_mats;
    void *bbone_deform_mats;
    struct DualQuat3_6_0 *bbone_dual_quats;
    struct bPoseChannel_BBoneSegmentBoundary4_0_0 *bbone_segment_boundaries;
    void *_pad;
};

struct BoneColor4_0_0 {
    signed char palette_index;
    unsigned char _pad0[7];
    struct ThemeWireColor3_6_0 custom;
};

struct bPose4_0_0 {
    struct ListBase3_6_0 chanbase;
    void *chanhash;
    struct bPoseChannel4_0_0 **chan_array;
    short flag;
    char _pad[2];
    float ctime;
    float stride_offset[3];
    float cyclic_offset[3];
    struct ListBase3_6_0 agroups;
    int active_group;
    int iksolver;
    void *ikdata;
    void *ikparam;
    struct bAnimVizSettings3_6_0 avs;
};

struct Bone_Runtime4_0_0 {
    struct ListBase3_6_0 collections;
};

struct ScrArea4_0_0 {
    struct ScrArea4_0_0 *next;
    struct ScrArea4_0_0 *prev;
    struct ScrVert3_6_0 *v1;
    struct ScrVert3_6_0 *v2;
    struct ScrVert3_6_0 *v3;
    struct ScrVert3_6_0 *v4;
    struct bScreen4_0_0 *full;
    struct rcti3_6_0 totrct;
    char spacetype;
    char butspacetype;
    short butspacetype_subtype;
    short winx;
    short winy;
    char headertype;
    char do_refresh;
    short flag;
    short region_active_win;
    char _pad[2];
    void *type;
    struct ScrGlobalAreaData3_6_0 *global;
    struct ListBase3_6_0 spacedata;
    struct ListBase3_6_0 regionbase;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 actionzones;
    struct ScrArea_Runtime3_6_0 runtime;
};

struct Editing4_0_0 {
    struct ListBase3_6_0 *seqbasep;
    struct ListBase3_6_0 *displayed_channels;
    void *_pad0;
    struct ListBase3_6_0 seqbase;
    struct ListBase3_6_0 metastack;
    struct ListBase3_6_0 channels;
    struct Sequence4_0_0 *act_seq;
    char act_imagedir[1024];
    char act_sounddir[1024];
    char proxy_dir[1024];
    int proxy_storage;
    int overlay_frame_ofs;
    int overlay_frame_abs;
    int overlay_frame_flag;
    struct rctf3_6_0 overlay_frame_rect;
    void *cache;
    float recycle_max_cost;
    int cache_flag;
    void *prefetch_job;
    long long disk_cache_timestamp;
    struct EditingRuntime3_6_0 runtime;
    void *_pad1;
};

struct Panel4_0_0 {
    struct Panel4_0_0 *next;
    struct Panel4_0_0 *prev;
    void *type;
    void *layout;
    char panelname[64];
    char *drawname;
    int ofsx;
    int ofsy;
    int sizex;
    int sizey;
    int blocksizex;
    int blocksizey;
    short labelofs;
    short flag;
    short runtime_flag;
    char _pad[6];
    int sortorder;
    void *activedata;
    struct ListBase3_6_0 children;
    struct Panel_Runtime3_6_0 runtime;
};

struct ThemeUI4_0_0 {
    struct uiWidgetColors3_6_0 wcol_regular;
    struct uiWidgetColors3_6_0 wcol_tool;
    struct uiWidgetColors3_6_0 wcol_toolbar_item;
    struct uiWidgetColors3_6_0 wcol_text;
    struct uiWidgetColors3_6_0 wcol_radio;
    struct uiWidgetColors3_6_0 wcol_option;
    struct uiWidgetColors3_6_0 wcol_toggle;
    struct uiWidgetColors3_6_0 wcol_num;
    struct uiWidgetColors3_6_0 wcol_numslider;
    struct uiWidgetColors3_6_0 wcol_tab;
    struct uiWidgetColors3_6_0 wcol_menu;
    struct uiWidgetColors3_6_0 wcol_pulldown;
    struct uiWidgetColors3_6_0 wcol_menu_back;
    struct uiWidgetColors3_6_0 wcol_menu_item;
    struct uiWidgetColors3_6_0 wcol_tooltip;
    struct uiWidgetColors3_6_0 wcol_box;
    struct uiWidgetColors3_6_0 wcol_scroll;
    struct uiWidgetColors3_6_0 wcol_progress;
    struct uiWidgetColors3_6_0 wcol_list_item;
    struct uiWidgetColors3_6_0 wcol_pie_menu;
    struct uiWidgetStateColors3_6_0 wcol_state;
    unsigned char widget_emboss[4];
    float menu_shadow_fac;
    short menu_shadow_width;
    unsigned char editor_outline[4];
    unsigned char transparent_checker_primary[4];
    unsigned char transparent_checker_secondary[4];
    unsigned char transparent_checker_size;
    char _pad1[1];
    float icon_alpha;
    float icon_saturation;
    unsigned char widget_text_cursor[4];
    unsigned char xaxis[4];
    unsigned char yaxis[4];
    unsigned char zaxis[4];
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
    struct uiPanelColors3_6_0 panelcolors;
    struct ThemeAssetShelf4_0_0 asset_shelf;
    unsigned char shade1[4];
    unsigned char shade2[4];
    unsigned char hilite[4];
    unsigned char grid[4];
    unsigned char view_overlay[4];
    unsigned char wire[4];
    unsigned char wire_edit[4];
    unsigned char select[4];
    unsigned char lamp[4];
    unsigned char speaker[4];
    unsigned char empty[4];
    unsigned char camera[4];
    unsigned char active[4];
    unsigned char group[4];
    unsigned char group_active[4];
    unsigned char transform[4];
    unsigned char vertex[4];
    unsigned char vertex_select[4];
    unsigned char vertex_active[4];
    unsigned char vertex_bevel[4];
    unsigned char vertex_unreferenced[4];
    unsigned char edge[4];
    unsigned char edge_select[4];
    unsigned char edge_seam[4];
    unsigned char edge_sharp[4];
    unsigned char edge_facesel[4];
    unsigned char edge_crease[4];
    unsigned char edge_bevel[4];
    unsigned char face[4];
    unsigned char face_select[4];
    unsigned char face_retopology[4];
    unsigned char face_back[4];
    unsigned char face_front[4];
    unsigned char face_dot[4];
    unsigned char extra_edge_len[4];
    unsigned char extra_edge_angle[4];
    unsigned char extra_face_angle[4];
    unsigned char extra_face_area[4];
    unsigned char normal[4];
    unsigned char vertex_normal[4];
    unsigned char loop_normal[4];
    unsigned char bone_solid[4];
    unsigned char bone_pose[4];
    unsigned char bone_pose_active[4];
    unsigned char bone_locked_weight[4];
    unsigned char strip[4];
    unsigned char strip_select[4];
    unsigned char cframe[4];
    unsigned char time_keyframe[4];
    unsigned char time_gp_keyframe[4];
    unsigned char freestyle_edge_mark[4];
    unsigned char freestyle_face_mark[4];
    unsigned char time_scrub_background[4];
    unsigned char time_marker_line[4];
    unsigned char time_marker_line_selected[4];
    unsigned char nurb_uline[4];
    unsigned char nurb_vline[4];
    unsigned char act_spline[4];
    unsigned char nurb_sel_uline[4];
    unsigned char nurb_sel_vline[4];
    unsigned char lastsel_point[4];
    unsigned char handle_free[4];
    unsigned char handle_auto[4];
    unsigned char handle_vect[4];
    unsigned char handle_align[4];
    unsigned char handle_auto_clamped[4];
    unsigned char handle_sel_free[4];
    unsigned char handle_sel_auto[4];
    unsigned char handle_sel_vect[4];
    unsigned char handle_sel_align[4];
    unsigned char handle_sel_auto_clamped[4];
    unsigned char ds_channel[4];
    unsigned char ds_subchannel[4];
    unsigned char ds_ipoline[4];
    unsigned char keytype_keyframe[4];
    unsigned char keytype_extreme[4];
    unsigned char keytype_breakdown[4];
    unsigned char keytype_jitter[4];
    unsigned char keytype_movehold[4];
    unsigned char keytype_keyframe_select[4];
    unsigned char keytype_extreme_select[4];
    unsigned char keytype_breakdown_select[4];
    unsigned char keytype_jitter_select[4];
    unsigned char keytype_movehold_select[4];
    unsigned char keyborder[4];
    unsigned char keyborder_select[4];
    char _pad4[3];
    unsigned char console_output[4];
    unsigned char console_input[4];
    unsigned char console_info[4];
    unsigned char console_error[4];
    unsigned char console_cursor[4];
    unsigned char console_select[4];
    unsigned char vertex_size;
    unsigned char edge_width;
    unsigned char outline_width;
    unsigned char obcenter_dia;
    unsigned char facedot_size;
    unsigned char noodle_curving;
    unsigned char grid_levels;
    char _pad5[2];
    float dash_alpha;
    unsigned char syntaxl[4];
    unsigned char syntaxs[4];
    unsigned char syntaxb[4];
    unsigned char syntaxn[4];
    unsigned char syntaxv[4];
    unsigned char syntaxc[4];
    unsigned char syntaxd[4];
    unsigned char syntaxr[4];
    unsigned char line_numbers[4];
    char _pad6[3];
    unsigned char nodeclass_output[4];
    unsigned char nodeclass_filter[4];
    unsigned char nodeclass_vector[4];
    unsigned char nodeclass_texture[4];
    unsigned char nodeclass_shader[4];
    unsigned char nodeclass_script[4];
    unsigned char nodeclass_pattern[4];
    unsigned char nodeclass_layout[4];
    unsigned char nodeclass_geometry[4];
    unsigned char nodeclass_attribute[4];
    unsigned char node_zone_simulation[4];
    unsigned char node_zone_repeat[4];
    unsigned char _pad9[4];
    unsigned char simulated_frames[4];
    unsigned char movie[4];
    unsigned char movieclip[4];
    unsigned char mask[4];
    unsigned char image[4];
    unsigned char scene[4];
    unsigned char audio[4];
    unsigned char effect[4];
    unsigned char transition[4];
    unsigned char meta[4];
    unsigned char text_strip[4];
    unsigned char color_strip[4];
    unsigned char active_strip[4];
    unsigned char selected_strip[4];
    char _pad7[1];
    float keyframe_scale_fac;
    unsigned char editmesh_active[4];
    unsigned char handle_vertex[4];
    unsigned char handle_vertex_select[4];
    unsigned char handle_vertex_size;
    unsigned char clipping_border_3d[4];
    unsigned char marker_outline[4];
    unsigned char marker[4];
    unsigned char act_marker[4];
    unsigned char sel_marker[4];
    unsigned char dis_marker[4];
    unsigned char lock_marker[4];
    unsigned char bundle_solid[4];
    unsigned char path_before[4];
    unsigned char path_after[4];
    unsigned char path_keyframe_before[4];
    unsigned char path_keyframe_after[4];
    unsigned char camera_path[4];
    unsigned char camera_passepartout[4];
    unsigned char _pad1[6];
    unsigned char gp_vertex_size;
    unsigned char gp_vertex[4];
    unsigned char gp_vertex_select[4];
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
    unsigned char nla_transition[4];
    unsigned char nla_transition_sel[4];
    unsigned char nla_meta[4];
    unsigned char nla_meta_sel[4];
    unsigned char nla_sound[4];
    unsigned char nla_sound_sel[4];
    unsigned char info_selected[4];
    unsigned char info_selected_text[4];
    unsigned char info_error[4];
    unsigned char info_error_text[4];
    unsigned char info_warning[4];
    unsigned char info_warning_text[4];
    unsigned char info_info[4];
    unsigned char info_info_text[4];
    unsigned char info_debug[4];
    unsigned char info_debug_text[4];
    unsigned char info_property[4];
    unsigned char info_property_text[4];
    unsigned char info_operator[4];
    unsigned char info_operator_text[4];
    unsigned char paint_curve_pivot[4];
    unsigned char paint_curve_handle[4];
    unsigned char metadatabg[4];
    unsigned char metadatatext[4];
};

struct bNodeSocket4_0_0 {
    struct bNodeSocket4_0_0 *next;
    struct bNodeSocket4_0_0 *prev;
    struct IDProperty3_6_0 *prop;
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
    struct bNodeLink4_0_0 *link;
    struct bNodeStack3_6_0 ns;
    void *runtime;
};

struct bNode4_0_0 {
    struct bNode4_0_0 *next;
    struct bNode4_0_0 *prev;
    struct ListBase3_6_0 inputs;
    struct ListBase3_6_0 outputs;
    char name[64];
    int identifier;
    int flag;
    char idname[64];
    void *typeinfo;
    short type;
    char _pad1[2];
    short custom1;
    short custom2;
    float custom3;
    float custom4;
    struct ID3_6_0 *id;
    void *storage;
    struct IDProperty3_6_0 *prop;
    struct bNode4_0_0 *parent;
    float locx;
    float locy;
    float width;
    float height;
    float offsetx;
    float offsety;
    char label[64];
    float color[3];
    int num_panel_states;
    struct bNodePanelState4_0_0 *panel_states_array;
    void *runtime;
};

struct bGPDstroke4_0_0 {
    struct bGPDstroke4_0_0 *next;
    struct bGPDstroke4_0_0 *prev;
    struct bGPDspoint3_6_0 *points;
    struct bGPDtriangle3_6_0 *triangles;
    int totpoints;
    int tot_triangles;
    short thickness;
    short flag;
    short _pad[2];
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
    void *dvert;
    void *_pad3;
    float vert_color_fill[4];
    struct bGPDcurve3_6_0 *editcurve;
    struct bGPDstroke_Runtime4_0_0 runtime;
    void *_pad5;
};

struct SpaceFile4_0_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char browse_mode;
    char _pad1[1];
    short tags;
    int scroll_offset;
    struct FileSelectParams3_6_0 *params;
    struct FileAssetSelectParams4_0_0 *asset_params;
    void *_pad2;
    void *files;
    struct ListBase3_6_0 *folders_prev;
    struct ListBase3_6_0 *folders_next;
    struct ListBase3_6_0 folder_histories;
    void *op;
    void *smoothscroll_timer;
    void *previews_timer;
    void *layout;
    short recentnr;
    short bookmarknr;
    short systemnr;
    short system_bookmarknr;
    void *runtime;
};

struct ColorBalanceModifierData4_0_0 {
    struct SequenceModifierData4_0_0 modifier;
    struct StripColorBalance3_6_0 color_balance;
    float color_multiply;
};

struct BrightContrastModifierData4_0_0 {
    struct SequenceModifierData4_0_0 modifier;
    float bright;
    float contrast;
};

struct bNodeTreePath4_0_0 {
    struct bNodeTreePath4_0_0 *next;
    struct bNodeTreePath4_0_0 *prev;
    struct bNodeTree4_0_0 *nodetree;
    struct bNodeInstanceKey3_6_0 parent_key;
    char _pad[4];
    float view_center[2];
    char node_name[64];
    char display_name[64];
};

struct SequencerMaskModifierData4_0_0 {
    struct SequenceModifierData4_0_0 modifier;
};

struct WhiteBalanceModifierData4_0_0 {
    struct SequenceModifierData4_0_0 modifier;
    float white_value[3];
    char _pad[4];
};

struct SequencerTonemapModifierData4_0_0 {
    struct SequenceModifierData4_0_0 modifier;
    float key;
    float offset;
    float gamma;
    float intensity;
    float contrast;
    float adaptation;
    float correction;
    int type;
};

struct CollectionObject4_0_0 {
    struct CollectionObject4_0_0 *next;
    struct CollectionObject4_0_0 *prev;
    void *ob;
    struct CollectionLightLinking4_0_0 light_linking;
    int _pad;
};

struct CollectionChild4_0_0 {
    struct CollectionChild4_0_0 *next;
    struct CollectionChild4_0_0 *prev;
    struct Collection4_0_0 *collection;
    struct CollectionLightLinking4_0_0 light_linking;
    int _pad;
};

struct OpacityGpencilModifierData4_0_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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
    void *curve_intensity;
};

struct LayerCollection4_0_0 {
    struct LayerCollection4_0_0 *next;
    struct LayerCollection4_0_0 *prev;
    void *collection;
    void *_pad1;
    short flag;
    short runtime_flag;
    char _pad[4];
    struct ListBase3_6_0 layer_collections;
    unsigned short local_collections_bits;
    short _pad2[3];
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
    struct RaytraceEEVEE4_0_0 reflection_options;
    struct RaytraceEEVEE4_0_0 refraction_options;
    struct RaytraceEEVEE4_0_0 diffuse_options;
    void *light_cache;
    void *light_cache_data;
    char light_cache_info[128];
    float overscan;
    float light_threshold;
};

struct GP_Sculpt_Settings4_0_0 {
    void *paintcursor;
    int flag;
    int lock_axis;
    float isect_threshold;
    char _pad[4];
    void *cur_falloff;
    void *cur_primitive;
    struct GP_Sculpt_Guide4_0_0 guide;
};

struct XrSessionSettings4_0_0 {
    struct View3DShading3_6_0 shading;
    float base_scale;
    char _pad[3];
    char base_pose_type;
    struct Object4_0_0 *base_pose_object;
    float base_pose_location[3];
    float base_pose_angle;
    char draw_flags;
    char controller_draw_style;
    char _pad2[2];
    float clip_start;
    float clip_end;
    int flag;
    int object_type_exclude_viewport;
    int object_type_exclude_select;
};

struct FileAssetSelectParams4_0_0 {
    struct FileSelectParams3_6_0 base_params;
    struct AssetLibraryReference3_6_0 asset_library_ref;
    short asset_catalog_visibility;
    char _pad[6];
    struct bUUID3_6_0 catalog_id;
    short import_method;
    char _pad2[6];
};

struct CurvesGeometry4_0_0 {
    int *curve_offsets;
    struct CustomData4_0_0 point_data;
    struct CustomData4_0_0 curve_data;
    int point_num;
    int curve_num;
    struct ListBase3_6_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad[4];
    void *runtime;
};

struct IDViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 base;
    struct ID3_6_0 *id;
};

struct ModifierViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 base;
    char *modifier_name;
};

struct LightProbeGridCacheFrame4_0_0 {
    int size[3];
    int data_layout;
    int block_len;
    int block_size;
    struct LightProbeBlockData3_6_0 *block_infos;
    struct LightProbeBakingData4_0_0 baking;
    struct LightProbeIrradianceData3_6_0 irradiance;
    struct LightProbeVisibilityData3_6_0 visibility;
    struct LightProbeConnectivityData4_0_0 connectivity;
    char _pad[4];
    int surfels_len;
    void *surfels;
};

struct BoneCollection4_0_0 {
    struct BoneCollection4_0_0 *next;
    struct BoneCollection4_0_0 *prev;
    char name[64];
    struct ListBase3_6_0 bones;
    unsigned char flags;
    unsigned char _pad0[7];
    struct IDProperty3_6_0 *prop;
};

struct LightLinking4_0_0 {
    void *receiver_collection;
    void *blocker_collection;
    struct LightLinkingRuntime4_0_0 runtime;
};

struct GreasePencilDrawingReference4_0_0 {
    struct GreasePencilDrawingBase4_0_0 base;
    struct GreasePencil4_0_0 *id_reference;
};

struct GreasePencilLayer4_0_0 {
    struct GreasePencilLayerTreeNode4_0_0 base;
    struct GreasePencilLayerFramesMapStorage4_0_0 frames_storage;
    signed char blend_mode;
    char _pad[3];
    float opacity;
    struct ListBase3_6_0 masks;
    void *runtime;
};

struct GreasePencilLayerTreeGroup4_0_0 {
    struct GreasePencilLayerTreeNode4_0_0 base;
    struct ListBase3_6_0 children;
    void *runtime;
};

struct bNodeTreeInterfaceSocket4_0_0 {
    struct bNodeTreeInterfaceItem4_0_0 item;
    char *name;
    char *description;
    char *socket_type;
    int flag;
    int attribute_domain;
    char *default_attribute_name;
    char *identifier;
    void *socket_data;
    struct IDProperty3_6_0 *properties;
};

struct bNodeTreeInterfacePanel4_0_0 {
    struct bNodeTreeInterfaceItem4_0_0 item;
    char *name;
    char *description;
    int flag;
    char _pad[4];
    struct bNodeTreeInterfaceItem4_0_0 **items_array;
    int items_num;
    int identifier;
};

struct GroupNodeViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 base;
    int node_id;
    char _pad1[4];
};

struct SimulationZoneViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 base;
    int sim_output_node_id;
    char _pad1[4];
};

struct RepeatZoneViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 base;
    int repeat_output_node_id;
    int iteration;
};

struct ViewerNodeViewerPathElem4_0_0 {
    struct ViewerPathElem4_0_0 base;
    int node_id;
    char _pad1[4];
};

struct bNestedNodeRef4_0_0 {
    int id;
    char _pad[4];
    struct bNestedNodePath4_0_0 path;
};

struct AssetShelfSettings4_0_0 {
    struct AssetShelfSettings4_0_0 *next;
    struct AssetShelfSettings4_0_0 *prev;
    struct AssetLibraryReference3_6_0 asset_library_reference;
    struct ListBase3_6_0 enabled_catalog_paths;
    const  char *active_catalog_path;
    char search_string[64];
    short preview_size;
    short display_flag;
    char _pad1[4];
};

struct RegionAssetShelf4_0_0 {
    struct ListBase3_6_0 shelves;
    struct AssetShelf4_0_0 *active_shelf;
};

struct SoundEqualizerModifierData4_0_0 {
    struct SequenceModifierData4_0_0 modifier;
    struct ListBase3_6_0 graphics;
};

struct bPoseChannel4_0_0 {
    struct bPoseChannel4_0_0 *next;
    struct bPoseChannel4_0_0 *prev;
    struct IDProperty3_6_0 *prop;
    struct ListBase3_6_0 constraints;
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
    struct Bone4_0_0 *bone;
    struct bPoseChannel4_0_0 *parent;
    struct bPoseChannel4_0_0 *child;
    struct ListBase3_6_0 iktree;
    struct ListBase3_6_0 siktree;
    struct bMotionPath3_6_0 *mpath;
    struct Object4_0_0 *custom;
    struct bPoseChannel4_0_0 *custom_tx;
    float custom_scale;
    float custom_scale_xyz[3];
    float custom_translation[3];
    float custom_rotation_euler[3];
    float loc[3];
    float size[3];
    float eul[3];
    float quat[4];
    float rotAxis[3];
    float rotAngle;
    short rotmode;
    char _pad[2];
    float chan_mat[4][4];
    float pose_mat[4][4];
    float disp_mat[4][4];
    float disp_tail_mat[4][4];
    float constinv[4][4];
    float pose_head[3];
    float pose_tail[3];
    float limitmin[3];
    float limitmax[3];
    float stiffness[3];
    float ikstretch;
    float ikrotweight;
    float iklinweight;
    float roll1;
    float roll2;
    float curve_in_x;
    float curve_in_z;
    float curve_out_x;
    float curve_out_z;
    float ease1;
    float ease2;
    float scale_in_x;
    float scale_in_z;
    float scale_out_x;
    float scale_out_z;
    float scale_in[3];
    float scale_out[3];
    struct bPoseChannel4_0_0 *bbone_prev;
    struct bPoseChannel4_0_0 *bbone_next;
    void *temp;
    struct bPoseChannelDrawData3_6_0 *draw_data;
    struct bPoseChannel4_0_0 *orig_pchan;
    struct BoneColor4_0_0 color;
    struct bPoseChannel_Runtime4_0_0 runtime;
};

struct SpaceAction4_0_0 {
    void *next;
    void *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct bAction3_6_0 *action;
    struct bDopeSheet3_6_0 ads;
    float timeslide;
    short flag;
    char mode;
    char mode_prev;
    char autosnap;
    char cache_display;
    char _pad1[6];
    struct SpaceAction_Runtime3_6_0 runtime;
};

struct Bone4_0_0 {
    struct Bone4_0_0 *next;
    struct Bone4_0_0 *prev;
    struct IDProperty3_6_0 *prop;
    struct Bone4_0_0 *parent;
    struct ListBase3_6_0 childbase;
    char name[64];
    float roll;
    float head[3];
    float tail[3];
    float bone_mat[3][3];
    int flag;
    char _pad1[4];
    struct BoneColor4_0_0 color;
    char inherit_scale_mode;
    char _pad[3];
    float arm_head[3];
    float arm_tail[3];
    float arm_mat[4][4];
    float arm_roll;
    float dist;
    float weight;
    float xwidth;
    float length;
    float zwidth;
    float rad_head;
    float rad_tail;
    float roll1;
    float roll2;
    float curve_in_x;
    float curve_in_z;
    float curve_out_x;
    float curve_out_z;
    float ease1;
    float ease2;
    float scale_in_x;
    float scale_in_z;
    float scale_out_x;
    float scale_out_z;
    float scale_in[3];
    float scale_out[3];
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
    struct Bone4_0_0 *bbone_prev;
    struct Bone4_0_0 *bbone_next;
    struct Bone_Runtime4_0_0 runtime;
};

struct Sequence4_0_0 {
    struct Sequence4_0_0 *next;
    struct Sequence4_0_0 *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag;
    int type;
    int len;
    float start;
    float startofs;
    float endofs;
    float startstill;
    float endstill;
    int machine;
    int _pad;
    int startdisp;
    int enddisp;
    float sat;
    float mul;
    float _pad1;
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    struct Strip3_6_0 *strip;
    void *ipo;
    void *scene;
    struct Object4_0_0 *scene_camera;
    struct MovieClip3_6_0 *clip;
    void *mask;
    struct ListBase3_6_0 anims;
    float effect_fader;
    float speed_fader;
    struct Sequence4_0_0 *seq1;
    struct Sequence4_0_0 *seq2;
    struct Sequence4_0_0 *seq3;
    struct ListBase3_6_0 seqbase;
    struct ListBase3_6_0 channels;
    void *sound;
    void *scene_sound;
    float volume;
    float pitch;
    float pan;
    float strobe;
    void *effectdata;
    int anim_startofs;
    int anim_endofs;
    int blend_mode;
    float blend_opacity;
    signed char color_tag;
    char alpha_mode;
    char _pad2[2];
    int cache_flag;
    int sfra;
    char views_format;
    char _pad3[3];
    struct Stereo3dFormat3_6_0 *stereo3d_format;
    struct IDProperty3_6_0 *prop;
    struct ListBase3_6_0 modifiers;
    float media_playback_rate;
    float speed_factor;
    struct SeqRetimingKey4_0_0 *retiming_keys;
    void *_pad5;
    int retiming_keys_num;
    char _pad6[4];
    struct SequenceRuntime3_6_0 runtime;
};

struct SpaceSeq4_0_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    float xof;
    float yof;
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
    void *gpd;
    struct SequencerScopes3_6_0 scopes;
    struct SequencerPreviewOverlay3_6_0 preview_overlay;
    struct SequencerTimelineOverlay3_6_0 timeline_overlay;
    char multiview_eye;
    char _pad2[7];
    struct SpaceSeqRuntime3_6_0 runtime;
};

struct SpaceNla4_0_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    short autosnap;
    short flag;
    char _pad[4];
    struct bDopeSheet3_6_0 *ads;
    struct View2D4_0_0 v2d;
};

struct bNodePreview4_0_0 {
    struct bNodeInstanceHashEntry3_6_0 hash_entry;
    void *ibuf;
};

struct SpaceNode4_0_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct ID3_6_0 *id;
    struct ID3_6_0 *from;
    short flag;
    char insert_ofs_dir;
    char _pad1;
    float xof;
    float yof;
    float zoom;
    struct ListBase3_6_0 treepath;
    struct bNodeTree4_0_0 *edittree;
    struct bNodeTree4_0_0 *nodetree;
    char tree_idname[64];
    int treetype;
    short texfrom;
    char shaderfrom;
    char geometry_nodes_type;
    struct bNodeTree4_0_0 *geometry_nodes_tool_tree;
    void *gpd;
    struct SpaceNodeOverlay4_0_0 overlay;
    void *runtime;
};

struct Sculpt4_0_0 {
    struct Paint3_6_0 paint;
    int flags;
    int transform_mode;
    int automasking_flags;
    int radial_symm[3];
    float detail_size;
    int symmetrize_direction;
    float gravity_factor;
    float constant_detail;
    float detail_percent;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    char _pad[4];
    float automasking_start_normal_limit;
    float automasking_start_normal_falloff;
    float automasking_view_normal_limit;
    float automasking_view_normal_falloff;
    void *automasking_cavity_curve;
    void *automasking_cavity_curve_op;
    struct Object4_0_0 *gravity_object;
};

struct ARegion4_0_0 {
    struct ARegion4_0_0 *next;
    struct ARegion4_0_0 *prev;
    struct View2D4_0_0 v2d;
    struct rcti3_6_0 winrct;
    struct rcti3_6_0 drawrct;
    short winx;
    short winy;
    int category_scroll;
    char _pad0[4];
    short visible;
    short regiontype;
    short alignment;
    short flag;
    short sizex;
    short sizey;
    short do_draw;
    short do_draw_paintcursor;
    short overlap;
    short flagfullscreen;
    void *type;
    struct ListBase3_6_0 uiblocks;
    struct ListBase3_6_0 panels;
    struct ListBase3_6_0 panels_category_active;
    struct ListBase3_6_0 ui_lists;
    struct ListBase3_6_0 ui_previews;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 panels_category;
    void *gizmo_map;
    void *regiontimer;
    void *draw_buffer;
    char *headerstr;
    void *regiondata;
    struct ARegion_Runtime3_6_0 runtime;
};

struct wmXrData4_0_0 {
    void *runtime;
    struct XrSessionSettings4_0_0 session_settings;
};

struct wmWindow4_0_0 {
    struct wmWindow4_0_0 *next;
    struct wmWindow4_0_0 *prev;
    void *ghostwin;
    void *gpuctx;
    struct wmWindow4_0_0 *parent;
    void *scene;
    void *new_scene;
    char view_layer_name[64];
    void *unpinned_scene;
    void *workspace_hook;
    struct ScrAreaMap3_6_0 global_areas;
    struct bScreen4_0_0 *screen;
    int winid;
    short posx;
    short posy;
    short sizex;
    short sizey;
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
    struct ListBase3_6_0 event_queue;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 modalhandlers;
    struct ListBase3_6_0 gesture;
    void *stereo3d_format;
    struct ListBase3_6_0 drawcalls;
    void *cursor_keymap_status;
};

struct BakeData4_0_0 {
    struct ImageFormatData3_6_0 im_format;
    char filepath[1024];
    short width;
    short height;
    short margin;
    short flag;
    float cage_extrusion;
    float max_ray_distance;
    int pass_filter;
    char normal_swizzle[3];
    char normal_space;
    char target;
    char save_mode;
    char margin_type;
    char view_from;
    char _pad[4];
    struct Object4_0_0 *cage_object;
};

struct ViewLayer4_0_0 {
    struct ViewLayer4_0_0 *next;
    struct ViewLayer4_0_0 *prev;
    char name[64];
    short flag;
    char _pad[6];
    struct ListBase3_6_0 object_bases;
    void *stats;
    struct Base4_0_0 *basact;
    struct ListBase3_6_0 layer_collections;
    struct LayerCollection4_0_0 *active_collection;
    int layflag;
    int passflag;
    float pass_alpha_threshold;
    short cryptomatte_flag;
    short cryptomatte_levels;
    char _pad1[4];
    int samples;
    void *mat_override;
    struct IDProperty3_6_0 *id_properties;
    struct FreestyleConfig3_6_0 freestyle_config;
    struct ViewLayerEEVEE3_6_0 eevee;
    struct ListBase3_6_0 aovs;
    struct ViewLayerAOV3_6_0 *active_aov;
    struct ListBase3_6_0 lightgroups;
    struct ViewLayerLightgroup3_6_0 *active_lightgroup;
    struct ListBase3_6_0 drawdata;
    struct Base4_0_0 **object_bases_array;
    void *object_bases_hash;
};

struct SpaceProperties4_0_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    short space_subtype;
    short mainb;
    short mainbo;
    short mainbuser;
    short preview;
    char _pad[4];
    char flag;
    char outliner_sync;
    void *path;
    int pathflag;
    int dataicon;
    struct ID3_6_0 *pinid;
    void *texuser;
    void *runtime;
};

struct SpaceOutliner4_0_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct ListBase3_6_0 tree;
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
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct bDopeSheet3_6_0 *ads;
    short mode;
    short autosnap;
    int flag;
    float cursorTime;
    float cursorVal;
    int around;
    char _pad[4];
    struct SpaceGraph_Runtime3_6_0 runtime;
};

struct MeshToVolumeModifierData4_0_0 {
    struct ModifierData3_6_0 modifier;
    void *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    float interior_band_width;
    float density;
    char _pad2[4];
    void *_pad3;
};

struct NodesModifierData4_0_0 {
    struct ModifierData3_6_0 modifier;
    void *node_group;
    struct NodesModifierSettings3_6_0 settings;
    char *simulation_bake_directory;
    signed char flag;
    char _pad[3];
    int bakes_num;
    struct NodesModifierBake4_0_0 *bakes;
    void *_pad2;
    void *runtime;
};

struct bNodeTreeInterface4_0_0 {
    struct bNodeTreeInterfacePanel4_0_0 root_panel;
    int active_index;
    int next_uid;
    void *runtime;
};

struct GreasePencilDrawing4_0_0 {
    struct GreasePencilDrawingBase4_0_0 base;
    struct CurvesGeometry4_0_0 geometry;
    void *runtime;
};

struct AssetShelf4_0_0 {
    struct AssetShelf4_0_0 *next;
    struct AssetShelf4_0_0 *prev;
    char idname[64];
    void *type;
    struct AssetShelfSettings4_0_0 settings;
    short preferred_row_count;
    char _pad[6];
};

struct bArmature4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct ListBase3_6_0 bonebase;
    void *bonehash;
    void *_pad1;
    struct ListBase3_6_0 *edbo;
    struct Bone4_0_0 *act_bone;
    void *act_edbone;
    char needs_flush_to_id;
    char _pad0[3];
    int flag;
    int drawtype;
    short deformflag;
    short pathflag;
    struct ListBase3_6_0 collections;
    char active_collection_name[64];
    unsigned int layer_used;
    unsigned int layer;
    unsigned int layer_protected;
    float axes_position;
    struct bArmature_Runtime4_0_0 runtime;
};

struct Camera4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    char type;
    char dtx;
    short flag;
    float passepartalpha;
    float clip_start;
    float clip_end;
    float lens;
    float ortho_scale;
    float drawsize;
    float sensor_x;
    float sensor_y;
    float shiftx;
    float shifty;
    float dof_distance;
    char sensor_fit;
    char panorama_type;
    char _pad[2];
    float fisheye_fov;
    float fisheye_lens;
    float latitude_min;
    float latitude_max;
    float longitude_min;
    float longitude_max;
    float fisheye_polynomial_k0;
    float fisheye_polynomial_k1;
    float fisheye_polynomial_k2;
    float fisheye_polynomial_k3;
    float fisheye_polynomial_k4;
    void *ipo;
    void *dof_ob;
    struct GPUDOFSettings3_6_0 gpu_dof;
    struct CameraDOFSettings3_6_0 dof;
    struct ListBase3_6_0 bg_images;
    struct CameraStereoSettings3_6_0 stereo;
    struct Camera_Runtime3_6_0 runtime;
};

struct Object4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    void *sculpt;
    short type;
    short partype;
    int par1;
    int par2;
    int par3;
    char parsubstr[64];
    struct Object4_0_0 *parent;
    struct Object4_0_0 *track;
    struct Object4_0_0 *proxy;
    struct Object4_0_0 *proxy_group;
    struct Object4_0_0 *proxy_from;
    void *ipo;
    struct bAction3_6_0 *action;
    struct bAction3_6_0 *poselib;
    struct bPose4_0_0 *pose;
    void *data;
    void *gpd;
    struct bAnimVizSettings3_6_0 avs;
    struct bMotionPath3_6_0 *mpath;
    void *_pad0;
    struct ListBase3_6_0 constraintChannels;
    struct ListBase3_6_0 effect;
    struct ListBase3_6_0 defbase;
    struct ListBase3_6_0 fmaps;
    struct ListBase3_6_0 modifiers;
    struct ListBase3_6_0 greasepencil_modifiers;
    struct ListBase3_6_0 shader_fx;
    int mode;
    int restore_mode;
    void *mat;
    char *matbits;
    int totcol;
    int actcol;
    float loc[3];
    float dloc[3];
    float scale[3];
    float dsize[3];
    float dscale[3];
    float rot[3];
    float drot[3];
    float quat[4];
    float dquat[4];
    float rotAxis[3];
    float drotAxis[3];
    float rotAngle;
    float drotAngle;
    float object_to_world[4][4];
    float world_to_object[4][4];
    float parentinv[4][4];
    float constinv[4][4];
    unsigned int lay;
    short flag;
    short colbits;
    short transflag;
    short protectflag;
    short trackflag;
    short upflag;
    short nlaflag;
    char _pad1;
    char duplicator_visibility_flag;
    short base_flag;
    unsigned short base_local_view_bits;
    unsigned short col_group;
    unsigned short col_mask;
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
    struct ListBase3_6_0 constraints;
    struct ListBase3_6_0 nlastrips;
    struct ListBase3_6_0 hooks;
    struct ListBase3_6_0 particlesystem;
    void *pd;
    void *soft;
    void *instance_collection;
    void *fluidsimSettings;
    struct ListBase3_6_0 pc_ids;
    void *rigidbody_object;
    void *rigidbody_constraint;
    float ima_ofs[2];
    struct ImageUser3_6_0 *iuser;
    char empty_image_visibility_flag;
    char empty_image_depth;
    char empty_image_flag;
    unsigned char modifier_flag;
    char _pad8[4];
    struct PreviewImage3_6_0 *preview;
    struct ObjectLineArt3_6_0 lineart;
    void *lightgroup;
    struct LightLinking4_0_0 *light_linking;
    void *lightprobe_cache;
    struct Object_Runtime3_6_0 runtime;
};

struct Mesh4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    void *ipo;
    void *key;
    void *mat;
    int totvert;
    int totedge;
    int faces_num;
    int totloop;
    int *face_offset_indices;
    struct CustomData4_0_0 vert_data;
    struct CustomData4_0_0 edge_data;
    struct CustomData4_0_0 face_data;
    struct CustomData4_0_0 loop_data;
    struct ListBase3_6_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    void *edit_mesh;
    struct MSelect3_6_0 *mselect;
    int totselect;
    int act_face;
    struct Mesh4_0_0 *texcomesh;
    float texspace_location[3];
    float texspace_size[3];
    char texspace_flag;
    char editflag;
    unsigned short flag;
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
    void *mpoly;
    void *mloop;
    void *mvert;
    void *medge;
    struct MDeformVert3_6_0 *dvert;
    struct MTFace3_6_0 *mtface;
    void *tface;
    struct MCol3_6_0 *mcol;
    struct MFace3_6_0 *mface;
    struct CustomData4_0_0 fdata_legacy;
    int totface_legacy;
    char _pad1[4];
    void *runtime;
};

struct bScreen4_0_0 {
    struct ID3_6_0 id;
    struct ListBase3_6_0 vertbase;
    struct ListBase3_6_0 edgebase;
    struct ListBase3_6_0 areabase;
    struct ListBase3_6_0 regionbase;
    void *scene;
    short flag;
    short winid;
    short redraws_flag;
    char temp;
    char state;
    char do_draw;
    char do_refresh;
    char do_draw_gesture;
    char do_draw_paintcursor;
    char do_draw_drag;
    char skip_handling;
    char scrubbing;
    char _pad[1];
    struct ARegion4_0_0 *active_region;
    void *animtimer;
    void *context;
    void *tool_tip;
    struct PreviewImage3_6_0 *preview;
};

struct World4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    char _pad0[4];
    short texact;
    short mistype;
    float horr;
    float horg;
    float horb;
    float exposure;
    float exp;
    float range;
    short mode;
    char _pad2[6];
    float misi;
    float miststa;
    float mistdist;
    float misthi;
    float aodist;
    float aoenergy;
    short flag;
    char _pad3[2];
    int probe_resolution;
    void *ipo;
    short pr_texture;
    short use_nodes;
    char _pad[4];
    struct PreviewImage3_6_0 *preview;
    void *nodetree;
    void *lightgroup;
    struct ListBase3_6_0 gpumaterial;
};

struct Brush4_0_0 {
    struct ID3_6_0 id;
    struct BrushClone3_6_0 clone;
    void *curve;
    struct MTex4_0_0 mtex;
    struct MTex4_0_0 mask_mtex;
    struct Brush4_0_0 *toggle_brush;
    void *icon_imbuf;
    struct PreviewImage3_6_0 *preview;
    void *gradient;
    struct PaintCurve3_6_0 *paint_curve;
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
    struct BrushGpencilSettings4_0_0 *gpencil_settings;
    struct BrushCurvesSculptSettings3_6_0 *curves_sculpt_settings;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    void *automasking_cavity_curve;
};

struct bGPdata4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct ListBase3_6_0 layers;
    int flag;
    int curve_edit_resolution;
    float curve_edit_threshold;
    float curve_edit_corner_angle;
    struct ListBase3_6_0 palettes;
    struct ListBase3_6_0 vertex_group_names;
    float pixfactor;
    float line_color[4];
    float onion_factor;
    int onion_mode;
    int onion_flag;
    short gstep;
    short gstep_next;
    float gcolor_prev[3];
    float gcolor_next[3];
    float zdepth_offset;
    void *mat;
    short totcol;
    short totlayer;
    short totframe;
    char _pad2[6];
    int totstroke;
    int totpoint;
    short draw_mode;
    short onion_keytype;
    int select_last_index;
    int vertex_group_active_index;
    struct bGPgrid3_6_0 grid;
    struct bGPdata_Runtime4_0_0 runtime;
};

struct wmWindowManager4_0_0 {
    struct ID3_6_0 id;
    struct wmWindow4_0_0 *windrawable;
    struct wmWindow4_0_0 *winactive;
    struct ListBase3_6_0 windows;
    unsigned char init_flag;
    char _pad0[1];
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    struct ListBase3_6_0 operators;
    struct ListBase3_6_0 notifier_queue;
    void *notifier_queue_set;
    struct ReportList3_6_0 reports;
    struct ListBase3_6_0 jobs;
    struct ListBase3_6_0 paintcursors;
    struct ListBase3_6_0 drags;
    struct ListBase3_6_0 keyconfigs;
    struct wmKeyConfig3_6_0 *defaultconf;
    struct wmKeyConfig3_6_0 *addonconf;
    struct wmKeyConfig3_6_0 *userconf;
    struct ListBase3_6_0 timers;
    void *autosavetimer;
    void *undo_stack;
    char is_interface_locked;
    char _pad[7];
    void *message_bus;
    struct wmXrData4_0_0 xr;
};

struct Collection4_0_0 {
    struct ID3_6_0 id;
    struct ListBase3_6_0 gobject;
    struct ListBase3_6_0 children;
    struct PreviewImage3_6_0 *preview;
    unsigned int layer;
    float instance_offset[3];
    unsigned char flag;
    signed char color_tag;
    char _pad0[2];
    unsigned char lineart_usage;
    unsigned char lineart_flags;
    unsigned char lineart_intersection_mask;
    unsigned char lineart_intersection_priority;
    void *_pad1;
    void *view_layer;
    struct Collection_Runtime3_6_0 runtime;
};

struct LightProbe4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    char type;
    char flag;
    char attenuation_type;
    char parallax_type;
    char grid_flag;
    char _pad0[3];
    float distinf;
    float distpar;
    float falloff;
    float clipsta;
    float clipend;
    float vis_bias;
    float vis_bleedbias;
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
    void *visibility_grp;
};

struct Light4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    short type;
    short flag;
    int mode;
    float r;
    float g;
    float b;
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
    float shdwr;
    float shdwg;
    float shdwb;
    short pr_texture;
    short use_nodes;
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
    float diff_fac;
    float volume_fac;
    float spec_fac;
    float att_dist;
    float shadow_softness_factor;
    float shadow_trace_distance;
    float _pad3;
    struct PreviewImage3_6_0 *preview;
    void *nodetree;
    void *ipo;
    float energy_deprecated;
    float _pad2;
};

struct PointCloud4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    int flag;
    int totpoint;
    struct CustomData4_0_0 pdata;
    int attributes_active_index;
    int _pad4;
    void *mat;
    short totcol;
    short _pad3[3];
    void *runtime;
    void *batch_cache;
};

struct Curves4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct CurvesGeometry4_0_0 geometry;
    int flag;
    int attributes_active_index;
    void *mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    struct Object4_0_0 *surface;
    char *surface_uv_map;
    void *batch_cache;
};

struct bNodeTree4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct ID3_6_0 *owner_id;
    void *typeinfo;
    char idname[64];
    void *gpd;
    float view_center[2];
    struct ListBase3_6_0 nodes;
    struct ListBase3_6_0 links;
    int type;
    int cur_index;
    int flag;
    short edit_quality;
    short render_quality;
    int chunksize;
    int execution_mode;
    struct rctf3_6_0 viewer_border;
    struct ListBase3_6_0 inputs_legacy;
    struct ListBase3_6_0 outputs_legacy;
    struct bNodeTreeInterface4_0_0 tree_interface;
    void *previews;
    struct bNodeInstanceKey3_6_0 active_viewer_key;
    int nested_node_refs_num;
    struct bNestedNodeRef4_0_0 *nested_node_refs;
    struct GeometryNodeAssetTraits4_0_0 *geometry_node_asset_traits;
    struct PreviewImage3_6_0 *preview;
    void *runtime;
};

struct GreasePencil4_0_0 {
    struct ID3_6_0 id;
    void *adt;
    struct GreasePencilDrawingBase4_0_0 **drawing_array;
    int drawing_array_num;
    char _pad[4];
    struct GreasePencilLayerTreeGroup4_0_0 *root_group_ptr;
    struct GreasePencilLayer4_0_0 *active_layer;
    void *material_array;
    short material_array_num;
    char _pad2[2];
    unsigned int flag;
    struct GreasePencilOnionSkinningSettings4_0_0 onion_skinning_settings;
    void *runtime;
};

#endif