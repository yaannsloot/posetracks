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

Various structs from Blender v3.5 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_5_0_H
#define MAKESDNA_TYPES_3_5_0_H

#include "makesdna_types_3_3_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_1_0.h"

struct KeyingSet3_5_0;
struct BrushCurvesSculptSettings3_5_0;
struct Collection_Runtime3_5_0;
struct Collection3_5_0;
struct BevList3_5_0;
struct CustomDataLayer3_5_0;
struct DynamicPaintSurface3_5_0;
struct FluidFlowSettings3_5_0;
struct BuildGpencilModifierData3_5_0;
struct OffsetGpencilModifierData3_5_0;
struct IDPropertyUIDataBool3_5_0;
struct IDPropertyUIDataID3_5_0;
struct IDProperty3_5_0;
struct Image3_5_0;
struct MVert3_5_0;
struct MetaBall3_5_0;
struct ModifierData3_5_0;
struct MappingInfoModifierData3_5_0;
struct DisplaceModifierData3_5_0;
struct UVProjectModifierData3_5_0;
struct WaveModifierData3_5_0;
struct ParticleInstanceModifierData3_5_0;
struct ExplodeModifierData3_5_0;
struct OceanModifierData3_5_0;
struct WarpModifierData3_5_0;
struct WeightVGEditModifierData3_5_0;
struct WeightVGMixModifierData3_5_0;
struct WeightVGProximityModifierData3_5_0;
struct UVWarpModifierData3_5_0;
struct MovieClip3_5_0;
struct NodeDBlurData3_5_0;
struct NodeShaderTexPointDensity3_5_0;
struct NodeGeometryImageTexture3_5_0;
struct RenderData3_5_0;
struct SceneEEVEE3_5_0;
struct SpaceClip3_5_0;
struct MovieTrackingCamera3_5_0;
struct MovieTrackingObject3_5_0;
struct MovieTracking3_5_0;
struct ThemeSpace3_5_0;
struct bUserAssetLibrary3_5_0;
struct UserDef_Experimental3_5_0;
struct View3DShading3_5_0;
struct View3DOverlay3_5_0;
struct NodeViewerPathElem3_5_0;
struct wmWindow3_5_0;
struct SubsurfModifierData3_5_0;
struct LatticeModifierData3_5_0;
struct CurveModifierData3_5_0;
struct BuildModifierData3_5_0;
struct MaskModifierData3_5_0;
struct ArrayModifierData3_5_0;
struct MirrorModifierData3_5_0;
struct EdgeSplitModifierData3_5_0;
struct BevelModifierData3_5_0;
struct FluidModifierData3_5_0;
struct DecimateModifierData3_5_0;
struct SmoothModifierData3_5_0;
struct CastModifierData3_5_0;
struct HookModifierData3_5_0;
struct SoftbodyModifierData3_5_0;
struct ClothModifierData3_5_0;
struct BooleanModifierData3_5_0;
struct ParticleSystemModifierData3_5_0;
struct FluidsimModifierData3_5_0;
struct SmokeModifierData3_5_0;
struct ShrinkwrapModifierData3_5_0;
struct SimpleDeformModifierData3_5_0;
struct ShapeKeyModifierData3_5_0;
struct SolidifyModifierData3_5_0;
struct ScrewModifierData3_5_0;
struct DynamicPaintModifierData3_5_0;
struct RemeshModifierData3_5_0;
struct SkinModifierData3_5_0;
struct TriangulateModifierData3_5_0;
struct LaplacianSmoothModifierData3_5_0;
struct MeshCacheModifierData3_5_0;
struct LaplacianDeformModifierData3_5_0;
struct WireframeModifierData3_5_0;
struct WeldModifierData3_5_0;
struct DataTransferModifierData3_5_0;
struct NormalEditModifierData3_5_0;
struct MeshSeqCacheModifierData3_5_0;
struct SurfaceDeformModifierData3_5_0;
struct WeightedNormalModifierData3_5_0;
struct NodesModifierData3_5_0;
struct MeshToVolumeModifierData3_5_0;
struct VolumeDisplaceModifierData3_5_0;
struct VolumeToMeshModifierData3_5_0;
struct SceneDisplay3_5_0;
struct Scene3_5_0;
struct bTheme3_5_0;
struct XrSessionSettings3_5_0;
struct wmXrData3_5_0;
struct wmWindowManager3_5_0;

struct BrushCurvesSculptSettings3_5_0 {
    int add_amount;
    int points_per_curve;
    uint32_t flag;
    float minimum_length;
    float curve_length;
    float minimum_distance;
    int density_add_attempts;
    uint8_t density_mode;
    char _pad[3];
    CurveMapping2_93_0 *curve_parameter_falloff;
};

struct BevList3_5_0 {
    BevList3_5_0 *next, *prev;
    int nr, dupe_nr;
    int poly;
    int hole;
    int charidx;
    int *segbevcount;
    float *seglen;
    BevPoint2_93_0 *bevpoints;
};

struct CustomDataLayer3_5_0 {
    int type;
    int offset;
    int flag;
    int active;
    int active_rnd;
    int active_clone;
    int active_mask;
    int uid;
    char name[68];
    char _pad1[4];
    void *data;
    const void *anonymous_id;
};

struct FluidFlowSettings3_5_0 {
    FluidModifierData3_5_0 *fmd;
    Mesh3_1_0 *mesh;
    void *psys;
    Tex2_93_0 *noise_texture;
    float *verts_old;
    int numverts;
    float vel_multi;
    float vel_normal;
    float vel_random;
    float vel_coord[3];
    char _pad1[4];
    float density;
    float color[3];
    float fuel_amount;
    float temperature;
    float volume_density;
    float surface_distance;
    float particle_size;
    int subframes;
    float texture_size;
    float texture_offset;
    char _pad2[4];
    char uvlayer_name[68];
    char _pad3[4];
    short vgroup_density;
    short type;
    short behavior;
    short source;
    short texture_type;
    short _pad4[3];
    int flags;
};

struct MVert3_5_0 {
    float co_legacy[3];
    char flag_legacy;
    char bweight_legacy;
    char _pad[2];
};

struct NodeDBlurData3_5_0 {
    float center_x, center_y, distance, angle, spin, zoom;
    short iter;
    char _pad[2];
};

struct NodeGeometryImageTexture3_5_0 {
    int8_t interpolation;
    int8_t extension;
};

struct SceneEEVEE3_5_0 {
    int flag;
    int gi_diffuse_bounces;
    int gi_cubemap_resolution;
    int gi_visibility_resolution;
    float gi_irradiance_smoothing;
    float gi_glossy_clamp;
    float gi_filter_quality;
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
    char _pad[4];
    LightCache2_93_0 *light_cache;
    LightCache2_93_0 *light_cache_data;
    char light_cache_info[64];
    float overscan;
    float light_threshold;
};

struct MovieTrackingCamera3_5_0 {
    void *intrinsics;
    short distortion_model;
    char _pad[2];
    float sensor_width;
    float pixel_aspect;
    float focal;
    short units;
    char _pad1[2];
    float principal_point[2];
    float principal_legacy[2];
    float k1, k2, k3;
    float division_k1, division_k2;
    float nuke_k1, nuke_k2;
    float brown_k1, brown_k2, brown_k3, brown_k4;
    float brown_p1, brown_p2;
};

struct bUserAssetLibrary3_5_0 {
    bUserAssetLibrary3_5_0 *next, *prev;
    char name[64];
    char path[1024];
    short import_method;
    char _pad0[6];
};

struct UserDef_Experimental3_5_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
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
    char enable_workbench_next;
    char use_new_volume_nodes;
    char _pad[6];
};

struct View3DShading3_5_0 {
    char type;
    char prev_type;
    char prev_type_wire;
    char color_type;
    short flag;
    char light;
    char background_type;
    char cavity_type;
    char wire_color_type;
    char use_compositor;
    char _pad;
    char studio_light[256];
    char lookdev_light[256];
    char matcap[256];
    float shadow_intensity;
    float single_color[3];
    float studiolight_rot_z;
    float studiolight_background;
    float studiolight_intensity;
    float studiolight_blur;
    float object_outline_color[3];
    float xray_alpha;
    float xray_alpha_wire;
    float cavity_valley_factor;
    float cavity_ridge_factor;
    float background_color[3];
    float curvature_ridge_factor;
    float curvature_valley_factor;
    int render_pass;
    char aov_name[64];
    IDProperty3_5_0 *prop;
    void *_pad2;
};

struct View3DOverlay3_5_0 {
    int flag;
    int edit_flag;
    float normals_length;
    float normals_constant_screen_size;
    float backwire_opacity;
    int paint_flag;
    int wpaint_flag;
    float texture_paint_mode_opacity;
    float vertex_paint_mode_opacity;
    float weight_paint_mode_opacity;
    float sculpt_mode_mask_opacity;
    float sculpt_mode_face_sets_opacity;
    float viewer_attribute_opacity;
    float xray_alpha_bone;
    float bone_wire_alpha;
    char _pad1[4];
    float fade_alpha;
    float wireframe_threshold;
    float wireframe_opacity;
    float gpencil_paper_opacity;
    float gpencil_grid_opacity;
    float gpencil_fade_layer;
    float gpencil_vertex_paint_opacity;
    int handle_display;
    float sculpt_curves_cage_opacity;
    char _pad[4];
};

struct SceneDisplay3_5_0 {
    float light_direction[3];
    float shadow_shift, shadow_focus;
    float matcap_ssao_distance;
    float matcap_ssao_attenuation;
    int matcap_ssao_samples;
    char viewport_aa;
    char render_aa;
    char _pad[6];
    View3DShading3_5_0 shading;
};

struct XrSessionSettings3_5_0 {
    View3DShading3_5_0 shading;
    float base_scale;
    char _pad[3];
    char base_pose_type;
    Object3_0_0 *base_pose_object;
    float base_pose_location[3];
    float base_pose_angle;
    char draw_flags;
    char controller_draw_style;
    char _pad2[2];
    float clip_start, clip_end;
    int flag;
    int object_type_exclude_viewport;
    int object_type_exclude_select;
};

struct KeyingSet3_5_0 {
    KeyingSet3_5_0 *next, *prev;
    ListBase2_93_0 paths;
    char idname[64];
    char name[64];
    char description[1024];
    char typeinfo[64];
    int active_path;
    short flag;
    short keyingflag;
    short keyingoverride;
    char _pad[6];
};

struct Collection_Runtime3_5_0 {
    ID3_4_0 *owner_id;
    ListBase2_93_0 object_cache;
    ListBase2_93_0 object_cache_instanced;
    ListBase2_93_0 parents;
    uint8_t tag;
    char _pad0[7];
};

struct Collection3_5_0 {
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
    SceneCollection2_93_0 *collection;
    ViewLayer3_2_0 *view_layer;
    Collection_Runtime3_5_0 runtime;
};

struct DynamicPaintSurface3_5_0 {
    DynamicPaintSurface3_5_0 *next, *prev;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    void *data;
    Collection3_5_0 *brush_group;
    EffectorWeights3_0_0 *effector_weights;
    void *pointcache;
    ListBase2_93_0 ptcaches;
    int current_frame;
    char name[64];
    short format, type;
    short disp_type, image_fileformat;
    short effect_ui;
    short init_color_type;
    int flags, effect;
    int image_resolution, substeps;
    int start_frame, end_frame;
    float init_color[4];
    Tex2_93_0 *init_texture;
    char init_layername[68];
    int dry_speed, diss_speed;
    float color_dry_threshold;
    float depth_clamp, disp_factor;
    float spread_speed, color_spread_speed, shrink_speed;
    float drip_vel, drip_acc;
    float influence_scale, radius_scale;
    float wave_damping, wave_speed, wave_timescale, wave_spring, wave_smoothness;
    char _pad2[4];
    char uvlayer_name[68];
    char image_output_path[1024];
    char output_name[68];
    char output_name2[68];
};

struct BuildGpencilModifierData3_5_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    int pass_index;
    char materialname[64];
    int layer_pass;
    float start_frame;
    float end_frame;
    float start_delay;
    float length;
    short flag;
    short mode;
    short transition;
    short time_alignment;
    float speed_fac;
    float speed_maxgap;
    short time_mode;
    char _pad[6];
    Object3_0_0 *object;
    float percentage_fac;
    float fade_fac;
    char target_vgname[64];
    float fade_opacity_strength;
    float fade_thickness_strength;
};

struct OffsetGpencilModifierData3_5_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float loc[3];
    float rot[3];
    float scale[3];
    float rnd_offset[3];
    float rnd_rot[3];
    float rnd_scale[3];
    int seed;
    int mode;
    int stroke_step;
    int stroke_start_offset;
    int layer_pass;
    char _pad[4];
};

struct IDPropertyUIDataBool3_5_0 {
    IDPropertyUIData3_0_0 base;
    int8_t *default_array;
    int default_array_len;
    char _pad[3];
    int8_t default_value;
};

struct IDPropertyUIDataID3_5_0 {
    IDPropertyUIData3_0_0 base;
    short id_type;
    char _pad[6];
};

struct IDProperty3_5_0 {
    IDProperty3_5_0 *next, *prev;
    char type;
    char subtype;
    short flag;
    char name[64];
    char _pad0[4];
    IDPropertyData2_93_0 data;
    int len;
    int totallen;
    IDPropertyUIData3_0_0 *ui_data;
};

struct Image3_5_0 {
    ID3_4_0 id;
    char filepath[1024];
    void *cache;
    void *gputexture[3][2];
    ListBase2_93_0 anims;
    void *rr;
    ListBase2_93_0 renderslots;
    short render_slot, last_render_slot;
    int flag;
    short source, type;
    int lastframe;
    int gpuframenr;
    short gpuflag;
    short gpu_pass;
    short gpu_layer;
    short gpu_view;
    short seam_margin;
    char _pad2[2];
    PackedFile2_93_0 *packedfile;
    ListBase2_93_0 packedfiles;
    PreviewImage2_93_0 *preview;
    int lastused;
    int gen_x , gen_y;
    char gen_type , gen_flag;
    short gen_depth;
    float gen_color[4];
    float aspx, aspy;
    ColorManagedColorspaceSettings2_93_0 colorspace_settings;
    char alpha_mode;
    char _pad;
    char eye;
    char views_format;
    int active_tile_index;
    ListBase2_93_0 tiles;
    ListBase2_93_0 views;
    Stereo3dFormat2_93_0 *stereo3d_format;
    Image_Runtime3_1_0 runtime;
};

struct MetaBall3_5_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 *editelems;
    Ipo2_93_0 *ipo;
    Material2_93_0 **mat;
    char flag, flag2;
    short totcol;
    char texspace_flag;
    char _pad[2];
    char needs_flush_to_id;
    float texspace_location[3];
    float texspace_size[3];
    float wiresize, rendersize;
    float thresh;
    char _pad0[4];
    MetaElem2_93_0 *lastelem;
};

struct ModifierData3_5_0 {
    ModifierData3_5_0 *next, *prev;
    int type, mode;
    float execution_time;
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
    SessionUUID2_93_0 session_uuid;
    void *runtime;
};

struct MappingInfoModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
};

struct DisplaceModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    float strength;
    int direction;
    char defgrp_name[64];
    float midlevel;
    int space;
    short flag;
    char _pad2[6];
};

struct UVProjectModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *projectors[10];
    char _pad2[4];
    int projectors_num;
    float aspectx, aspecty;
    float scalex, scaley;
    char uvlayer_name[68];
    int uvlayer_tmp;
};

struct WaveModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    Object3_0_0 *objectcenter;
    char defgrp_name[64];
    short flag;
    char _pad2[2];
    float startx, starty, height, width;
    float narrow, speed, damp, falloff;
    float timeoffs, lifetime;
    char _pad3[4];
    void *_pad4;
};

struct ParticleInstanceModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *ob;
    short psys, flag, axis, space;
    float position, random_position;
    float rotation, random_rotation;
    float particle_amount, particle_offset;
    char index_layer_name[68];
    char value_layer_name[68];
    void *_pad1;
};

struct ExplodeModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    int *facepa;
    short flag, vgroup;
    float protect;
    char uvname[68];
    char _pad1[4];
    void *_pad2;
};

struct OceanModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    void *ocean;
    void *oceancache;
    int resolution;
    int viewport_resolution;
    int spatial_size;
    float wind_velocity;
    float damp;
    float smallest_wave;
    float depth;
    float wave_alignment;
    float wave_direction;
    float wave_scale;
    float chop_amount;
    float foam_coverage;
    float time;
    int spectrum;
    float fetch_jonswap;
    float sharpen_peak_jonswap;
    int bakestart;
    int bakeend;
    char cachepath[1024];
    char foamlayername[68];
    char spraylayername[68];
    char cached;
    char geometry_mode;
    char flag;
    char _pad2;
    short repeat_x;
    short repeat_y;
    int seed;
    float size;
    float foam_fade;
    char _pad[4];
};

struct WarpModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    Object3_0_0 *object_from;
    Object3_0_0 *object_to;
    char bone_from[64];
    char bone_to[64];
    CurveMapping2_93_0 *curfalloff;
    char defgrp_name[64];
    float strength;
    float falloff_radius;
    char flag;
    char falloff_type;
    char _pad2[6];
    void *_pad3;
};

struct WeightVGEditModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name[64];
    short edit_flags;
    short falloff_type;
    float default_weight;
    CurveMapping2_93_0 *cmap_curve;
    float add_threshold, rem_threshold;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex2_93_0 *mask_texture;
    Object3_0_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    void *_pad1;
};

struct WeightVGMixModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name_a[64];
    char defgrp_name_b[64];
    float default_weight_a;
    float default_weight_b;
    char mix_mode;
    char mix_set;
    char _pad0[6];
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex2_93_0 *mask_texture;
    Object3_0_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    char _pad1[4];
    char flag;
    char _pad2[3];
};

struct WeightVGProximityModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name[64];
    CurveMapping2_93_0 *cmap_curve;
    int proximity_mode;
    int proximity_flags;
    Object3_0_0 *proximity_ob_target;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex2_93_0 *mask_texture;
    Object3_0_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    char _pad1[4];
    float min_dist, max_dist;
    short falloff_type;
    char _pad0[2];
};

struct UVWarpModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char axis_u, axis_v;
    short flag;
    float center[2];
    float offset[2];
    float scale[2];
    float rotation;
    Object3_0_0 *object_src;
    char bone_src[64];
    Object3_0_0 *object_dst;
    char bone_dst[64];
    char vgroup_name[64];
    char uvlayer_name[68];
    char _pad[4];
};

struct MovieTracking3_5_0 {
    MovieTrackingSettings2_93_0 settings;
    MovieTrackingCamera3_5_0 camera;
    ListBase2_93_0 tracks_legacy;
    ListBase2_93_0 plane_tracks_legacy;
    MovieTrackingReconstruction2_93_0 reconstruction_legacy;
    MovieTrackingStabilization2_93_0 stabilization;
    MovieTrackingTrack2_93_0* act_track_legacy;
    MovieTrackingPlaneTrack2_93_0* act_plane_track_legacy;
    ListBase2_93_0 objects;
    int objectnr, tot_object;
    MovieTrackingStats2_93_0* stats;
    MovieTrackingDopesheet2_93_0 dopesheet;
};

struct MovieClip3_5_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    char filepath[1024];
    int source;
    int _pad;
    int lastsize[2];
    float aspx, aspy;
    void *anim;
    void *cache;
    bGPdata3_0_0 *gpd;
    MovieTracking3_5_0 tracking;
    void *tracking_context;
    MovieClipProxy2_93_0 proxy;
    int flag;
    int len;
    int start_frame;
    int frame_offset;
    ColorManagedColorspaceSettings2_93_0 colorspace_settings;
    MovieClip_Runtime2_93_0 runtime;
};

struct NodeShaderTexPointDensity3_5_0 {
    NodeTexBase2_93_0 base;
    short point_source;
    char _pad[2];
    int particle_system;
    float radius;
    int resolution;
    short space;
    short interpolation;
    short color_source;
    short ob_color_source;
    PointDensity2_93_0 pd;
    int cached_resolution;
    char vertex_attribute_name[68];
};

struct RenderData3_5_0 {
    ImageFormatData3_2_0 im_format;
    AviCodecData2_93_0 *avicodecdata;
    FFMpegCodecData3_1_0 ffcodecdata;
    int cfra, sfra, efra;
    float subframe;
    int psfra, pefra;
    int images, framapto;
    short flag, threads;
    float framelen, blurfac;
    int frame_step;
    short stereomode;
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
    CurveMapping2_93_0 mblur_shutter_curve;
};

struct SpaceClip3_5_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char gizmo_flag;
    char _pad1[3];
    float xof, yof;
    float xlockof, ylockof;
    float zoom;
    MovieClipUser2_93_0 user;
    MovieClip3_5_0 *clip;
    MovieClipScopes2_93_0 scopes;
    int flag;
    short mode;
    short view;
    int path_length;
    float loc[2], scale, angle;
    char _pad[4];
    float stabmat[4][4], unistabmat[4][4];
    int postproc_flag;
    short gpencil_src;
    char _pad2[2];
    int around;
    char _pad4[4];
    float cursor[2];
    MaskSpaceInfo3_3_0 mask_info;
};

struct MovieTrackingObject3_5_0 {
    MovieTrackingObject3_5_0 *next, *prev;
    char name[64];
    int flag;
    float scale;
    ListBase2_93_0 tracks;
    ListBase2_93_0 plane_tracks;
    MovieTrackingTrack2_93_0 *active_track;
    MovieTrackingPlaneTrack2_93_0 *active_plane_track;
    MovieTrackingReconstruction2_93_0 reconstruction;
    int keyframe1, keyframe2;
};

struct ThemeSpace3_5_0 {
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
    unsigned char face[4], face_select[4], face_back[4], face_front[4];
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
    unsigned char vertex_size, outline_width, obcenter_dia, facedot_size;
    unsigned char noodle_curving;
    unsigned char grid_levels;
    char _pad5[3];
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
    unsigned char _pad1[2];
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

struct NodeViewerPathElem3_5_0 {
    ViewerPathElem3_4_0 base;
    int32_t node_id;
    char _pad1[4];
    char *node_name;
};

struct wmWindow3_5_0 {
    wmWindow3_5_0 *next, *prev;
    void *ghostwin;
    void *gpuctx;
    wmWindow3_5_0 *parent;
    Scene3_5_0 *scene;
    Scene3_5_0 *new_scene;
    char view_layer_name[64];
    Scene3_5_0 *unpinned_scene;
    WorkSpaceInstanceHook2_93_0 *workspace_hook;
    ScrAreaMap2_93_0 global_areas;
    bScreen2_93_0 *screen;
    int winid;
    short posx, posy, sizex, sizey;
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

struct SubsurfModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    short subdivType, levels, renderLevels, flags;
    short uv_smooth;
    short quality;
    short boundary_smooth;
    char _pad[2];
    void *emCache, *mCache;
};

struct LatticeModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct CurveModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
    void *_pad1;
};

struct BuildModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float start, length;
    short flag;
    short randomize;
    int seed;
};

struct MaskModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
    void *_pad1;
};

struct ArrayModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *start_cap;
    Object3_0_0 *end_cap;
    Object3_0_0 *curve_ob;
    Object3_0_0 *offset_ob;
    float offset[3];
    float scale[3];
    float length;
    float merge_dist;
    int fit_type;
    int offset_type;
    int flags;
    int count;
    float uv_offset[2];
};

struct MirrorModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    short axis;
    short flag;
    float tolerance;
    float bisect_threshold;
    uint8_t use_correct_order_on_merge;
    char _pad[3];
    float uv_offset[2];
    float uv_offset_copy[2];
    Object3_0_0 *mirror_ob;
    void *_pad1;
};

struct EdgeSplitModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float split_angle;
    int flags;
};

struct BevelModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float value;
    int res;
    short flags;
    short val_flags;
    short profile_type;
    short lim_flags;
    short e_flags;
    short mat;
    short edge_flags;
    short face_str_mode;
    short miter_inner;
    short miter_outer;
    short vmesh_method;
    char affect_type;
    char _pad;
    float profile;
    float bevel_angle;
    float spread;
    char defgrp_name[64];
    char _pad1[4];
    CurveProfile2_93_0 *custom_profile;
    void *_pad2;
};

struct FluidModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    FluidDomainSettings3_2_0 *domain;
    FluidFlowSettings3_5_0 *flow;
    FluidEffectorSettings2_93_0 *effector;
    float time;
    int type;
    void *_pad1;
};

struct DecimateModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float percent;
    short iter;
    char delimit;
    char symmetry_axis;
    float angle;
    char defgrp_name[64];
    float defgrp_factor;
    short flag, mode;
    int face_count;
};

struct SmoothModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float fac;
    char defgrp_name[64];
    short flag, repeat;
};

struct CastModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag;
    short type;
    void *_pad1;
};

struct HookModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    char subtarget[64];
    char flag;
    char falloff_type;
    char _pad[6];
    float parentinv[4][4];
    float cent[3];
    float falloff;
    CurveMapping2_93_0 *curfalloff;
    int *indexar;
    int indexar_num;
    float force;
    char name[64];
    void *_pad1;
};

struct SoftbodyModifierData3_5_0 {
    ModifierData3_5_0 modifier;
};

struct ClothModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    void *clothObject;
    ClothSimSettings2_93_0 *sim_parms;
    ClothCollSettings2_93_0 *coll_parms;
    void *point_cache;
    ListBase2_93_0 ptcaches;
    void *hairdata;
    float hair_grid_min[3];
    float hair_grid_max[3];
    int hair_grid_res[3];
    float hair_grid_cellsize;
    void *solver_result;
};

struct BooleanModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    Collection3_5_0 *collection;
    float double_threshold;
    char operation;
    char solver;
    char material_mode;
    char flag;
    char bm_flag;
    char _pad[7];
};

struct ParticleSystemModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    void *psys;
    Mesh3_1_0 *mesh_final;
    Mesh3_1_0 *mesh_original;
    int totdmvert, totdmedge, totdmface;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct FluidsimModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    FluidsimSettings2_93_0 *fss;
    void *_pad1;
};

struct SmokeModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    int type;
    int _pad;
};

struct ShrinkwrapModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *target;
    Object3_0_0 *auxTarget;
    char vgroup_name[64];
    float keepDist;
    short shrinkType;
    char shrinkOpts;
    char shrinkMode;
    float projLimit;
    char projAxis;
    char subsurfLevels;
    char _pad[2];
};

struct SimpleDeformModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *origin;
    char vgroup_name[64];
    float factor;
    float limit[2];
    char mode;
    char axis;
    char deform_axis;
    char flag;
    void *_pad1;
};

struct ShapeKeyModifierData3_5_0 {
    ModifierData3_5_0 modifier;
};

struct SolidifyModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name[64];
    char shell_defgrp_name[64];
    char rim_defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float offset_clamp;
    char mode;
    char nonmanifold_offset_mode;
    char nonmanifold_boundary_mode;
    char _pad;
    float crease_inner;
    float crease_outer;
    float crease_rim;
    int flag;
    short mat_ofs;
    short mat_ofs_rim;
    float merge_tolerance;
    float bevel_convex;
};

struct ScrewModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *ob_axis;
    unsigned int steps;
    unsigned int render_steps;
    unsigned int iter;
    float screw_ofs;
    float angle;
    float merge_dist;
    short flag;
    char axis;
    char _pad[5];
    void *_pad1;
};

struct DynamicPaintModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    DynamicPaintBrushSettings2_93_0 *brush;
    int type;
    char _pad[4];
};

struct RemeshModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float threshold;
    float scale;
    float hermite_num;
    char depth;
    char flag;
    char mode;
    char _pad;
    float voxel_size;
    float adaptivity;
};

struct SkinModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct TriangulateModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float lambda, lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag, repeat;
};

struct MeshCacheModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char flag;
    char type;
    char time_mode;
    char play_mode;
    char forward_axis;
    char up_axis;
    char flip_axis;
    char interp;
    float factor;
    char deform_mode;
    char defgrp_name[64];
    char _pad[7];
    float frame_start;
    float frame_scale;
    float eval_frame;
    float eval_time;
    float eval_factor;
    char filepath[1024];
};

struct LaplacianDeformModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char anchor_grp_name[64];
    int verts_num, repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag, mat_ofs;
    char _pad[4];
};

struct WeldModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct DataTransferModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *ob_source;
    int data_types;
    int vmap_mode;
    int emap_mode;
    int lmap_mode;
    int pmap_mode;
    float map_max_distance;
    float map_ray_radius;
    float islands_precision;
    char _pad1[4];
    int layers_select_src[5];
    int layers_select_dst[5];
    int mix_mode;
    float mix_factor;
    char defgrp_name[64];
    int flags;
    void *_pad2;
};

struct NormalEditModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name[64];
    Object3_0_0 *target;
    short mode;
    short flag;
    short mix_mode;
    char _pad[2];
    float mix_factor;
    float mix_limit;
    float offset[3];
    char _pad0[4];
    void *_pad1;
};

struct MeshSeqCacheModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    CacheFile3_1_0 *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
};

struct SurfaceDeformModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    void *depsgraph;
    Object3_0_0 *target;
    SDefVert3_2_0 *verts;
    void *_pad1;
    float falloff;
    unsigned int mesh_verts_num;
    unsigned int bind_verts_num;
    unsigned int target_verts_num, target_polys_num;
    int flags;
    float mat[4][4];
    float strength;
    char defgrp_name[64];
    int _pad2;
};

struct WeightedNormalModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    char defgrp_name[64];
    char mode, flag;
    short weight;
    float thresh;
};

struct NodesModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    void *node_group;
    NodesModifierSettings2_93_0 settings;
    void *runtime_eval_log;
    void *_pad1;
};

struct MeshToVolumeModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char fill_volume;
    char _pad1[3];
    float interior_band_width;
    float exterior_band_width;
    float density;
    char _pad2[4];
    void *_pad3;
};

struct VolumeDisplaceModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

struct VolumeToMeshModifierData3_5_0 {
    ModifierData3_5_0 modifier;
    Object3_0_0 *object;
    float threshold;
    float adaptivity;
    uint32_t flag;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char grid_name[64];
    void *_pad1;
};

struct Scene3_5_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object3_0_0 *camera;
    World2_93_0 *world;
    Scene3_5_0 *set;
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
    ToolSettings3_3_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData3_5_0 r;
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
    Collection3_5_0 *master_collection;
    SceneCollection2_93_0 *collection;
    IDProperty3_5_0 *layer_properties;
    void *_pad9;
    SceneDisplay3_5_0 display;
    SceneEEVEE3_5_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
};

struct bTheme3_5_0 {
    bTheme3_5_0 *next, *prev;
    char name[32];
    ThemeUI3_3_0 tui;
    ThemeSpace3_5_0 space_properties;
    ThemeSpace3_5_0 space_view3d;
    ThemeSpace3_5_0 space_file;
    ThemeSpace3_5_0 space_graph;
    ThemeSpace3_5_0 space_info;
    ThemeSpace3_5_0 space_action;
    ThemeSpace3_5_0 space_nla;
    ThemeSpace3_5_0 space_sequencer;
    ThemeSpace3_5_0 space_image;
    ThemeSpace3_5_0 space_text;
    ThemeSpace3_5_0 space_outliner;
    ThemeSpace3_5_0 space_node;
    ThemeSpace3_5_0 space_preferences;
    ThemeSpace3_5_0 space_console;
    ThemeSpace3_5_0 space_clip;
    ThemeSpace3_5_0 space_topbar;
    ThemeSpace3_5_0 space_statusbar;
    ThemeSpace3_5_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

struct wmXrData3_5_0 {
    void *runtime;
    XrSessionSettings3_5_0 session_settings;
};

struct wmWindowManager3_5_0 {
    ID3_4_0 id;
    wmWindow3_5_0 *windrawable;
    wmWindow3_5_0 *winactive;
    ListBase2_93_0 windows;
    short initialized;
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
    wmXrData3_5_0 xr;
};

#endif
