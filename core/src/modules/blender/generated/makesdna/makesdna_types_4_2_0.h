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

Various structs from Blender v4.2 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_4_2_0_H
#define MAKESDNA_TYPES_4_2_0_H

#include "makesdna_types_3_3_0.h"
#include "makesdna_types_4_0_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_4_1_0.h"
#include "makesdna_types_3_5_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_1_0.h"

struct bMotionPath4_2_0;
struct AnimData4_2_0;
struct AssetCatalogPathLink4_2_0;
struct BrushCurvesSculptSettings4_2_0;
struct CollectionExport4_2_0;
struct Collection4_2_0;
struct GreasePencilOnionSkinningSettings4_2_0;
struct ID_Runtime4_2_0;
struct Library4_2_0;
struct Image_Runtime4_2_0;
struct Image4_2_0;
struct ViewLayer4_2_0;
struct LightProbe4_2_0;
struct GreasePencilLatticeModifierData4_2_0;
struct GreasePencilDashModifierSegment4_2_0;
struct GreasePencilMultiModifierData4_2_0;
struct GreasePencilLengthModifierData4_2_0;
struct GreasePencilWeightAngleModifierData4_2_0;
struct GreasePencilArrayModifierData4_2_0;
struct GreasePencilWeightProximityModifierData4_2_0;
struct GreasePencilHookModifierData4_2_0;
struct GreasePencilLineartModifierData4_2_0;
struct GreasePencilArmatureModifierData4_2_0;
struct GreasePencilTimeModifierSegment4_2_0;
struct GreasePencilEnvelopeModifierData4_2_0;
struct GreasePencilOutlineModifierData4_2_0;
struct GreasePencilShrinkwrapModifierData4_2_0;
struct GreasePencilBuildModifierData4_2_0;
struct GreasePencilSimplifyModifierData4_2_0;
struct GreasePencilTextureModifierData4_2_0;
struct NodeShaderAttribute4_2_0;
struct NodeTranslateData4_2_0;
struct NodeInputRotation4_2_0;
struct NodeGeometryAttributeCaptureItem4_2_0;
struct NodeGeometryAttributeCapture4_2_0;
struct PartDeflect4_2_0;
struct SceneRenderLayer4_2_0;
struct RenderData4_2_0;
struct UvSculpt4_2_0;
struct ToolSettings4_2_0;
struct RaytraceEEVEE4_2_0;
struct SceneEEVEE4_2_0;
struct Scene4_2_0;
struct ARegion_Runtime4_2_0;
struct EditingRuntime4_2_0;
struct Editing4_2_0;
struct TextVars4_2_0;
struct SequencerCacheOverlay4_2_0;
struct SpaceSeq4_2_0;
struct SpaceImage4_2_0;
struct ThemeSpace4_2_0;
struct bTheme4_2_0;
struct bUserExtensionRepo4_2_0;
struct UserDef_Experimental4_2_0;
struct bUserAssetShelfSettings4_2_0;
struct mat4x4f4_2_0;
struct wmWindowManager4_2_0;
struct WorkSpace4_2_0;
struct ID4_2_0;
struct Ipo4_2_0;
struct Key4_2_0;
struct Mask4_2_0;
struct MetaBall4_2_0;
struct MovieClip4_2_0;
struct ParticleSettings4_2_0;
struct bScreen4_2_0;
struct ARegion4_2_0;
struct bSound4_2_0;
struct Script4_2_0;
struct Speaker4_2_0;
struct Text4_2_0;
struct VFont4_2_0;
struct Volume4_2_0;
struct IdAdtTemplate4_2_0;
struct Palette4_2_0;
struct PaintCurve4_2_0;
struct CacheFile4_2_0;
struct Camera4_2_0;
struct Curves4_2_0;

struct bMotionPath4_2_0 {
    bMotionPathVert2_93_0 *points;
    int length;
    int start_frame;
    int end_frame;
    float color[3];
    float color_post[3];
    int line_thickness;
    int flag;
    char _pad2[4];
    void *points_vbo;
    void *batch_line;
    void *batch_points;
    void *_pad;
};

struct AssetCatalogPathLink4_2_0 {
    AssetCatalogPathLink4_2_0 *next, *prev;
    char *path;
};

struct BrushCurvesSculptSettings4_2_0 {
    int add_amount;
    int points_per_curve;
    uint32_t flag;
    float minimum_length;
    float curve_length;
    float minimum_distance;
    float curve_radius;
    int density_add_attempts;
    uint8_t density_mode;
    char _pad[7];
    CurveMapping2_93_0 *curve_parameter_falloff;
};

struct CollectionExport4_2_0 {
    CollectionExport4_2_0 *next, *prev;
    char fh_idname[64];
    IDProperty3_5_0 *export_properties;
    uint32_t flag;
    uint32_t _pad0;
};

struct GreasePencilOnionSkinningSettings4_2_0 {
    float opacity;
    int8_t mode;
    uint8_t flag;
    uint8_t filter;
    char _pad[1];
    int16_t num_frames_before;
    int16_t num_frames_after;
    float color_before[3];
    float color_after[3];
    char _pad2[4];
};

struct Image_Runtime4_2_0 {
    void *cache_mutex;
    void *partial_update_register;
    void *partial_update_user;
    float backdrop_offset[2];
};

struct GreasePencilDashModifierSegment4_2_0 {
    char name[64];
    int dash;
    int gap;
    float radius;
    float opacity;
    int mat_nr;
    int flag;
};

struct GreasePencilTimeModifierSegment4_2_0 {
    char name[64];
    int segment_start;
    int segment_end;
    int segment_mode;
    int segment_repeat;
};

struct NodeShaderAttribute4_2_0 {
    char name[256];
    int type;
    char _pad[4];
};

struct NodeTranslateData4_2_0 {
    char wrap_axis;
    char relative;
    short interpolation;
};

struct NodeInputRotation4_2_0 {
    float rotation_euler[3];
};

struct NodeGeometryAttributeCaptureItem4_2_0 {
    int8_t data_type;
    char _pad[3];
    int identifier;
    char *name;
};

struct NodeGeometryAttributeCapture4_2_0 {
    int8_t data_type_legacy;
    int8_t domain;
    char _pad[2];
    int next_identifier;
    NodeGeometryAttributeCaptureItem4_2_0 *capture_items;
    int capture_items_num;
    int active_index;
};

struct PartDeflect4_2_0 {
    int flag;
    short deflect;
    short forcefield;
    short falloff;
    short shape;
    short tex_mode;
    short kink, kink_axis;
    short zdir;
    float f_strength;
    float f_damp;
    float f_flow;
    float f_wind_factor;
    char _pad0[4];
    float f_size;
    float f_power;
    float maxdist;
    float mindist;
    float f_power_r;
    float maxrad;
    float minrad;
    float pdef_damp;
    float pdef_rdamp;
    float pdef_perm;
    float pdef_frict;
    float pdef_rfrict;
    float pdef_stickness;
    float absorption;
    float pdef_sbdamp;
    float pdef_sbift;
    float pdef_sboft;
    float clump_fac, clump_pow;
    float kink_freq, kink_shape, kink_amp, free_end;
    float tex_nabla;
    Tex2_93_0 *tex;
    float f_noise;
    int seed;
    float drawvec1[4];
    float drawvec2[4];
    float drawvec_falloff_min[3];
    char _pad1[4];
    float drawvec_falloff_max[3];
    char _pad2[4];
    Object3_0_0 *f_source;
    float pdef_cfrict;
    char _pad[4];
};

struct UvSculpt4_2_0 {
    CurveMapping2_93_0 *strength_curve;
    int size;
    float strength;
    int8_t curve_preset;
    char _pad[7];
};

struct RaytraceEEVEE4_2_0 {
    float screen_trace_quality;
    float screen_trace_thickness;
    float trace_max_roughness;
    int resolution_scale;
    int flag;
    int denoise_stages;
};

struct EditingRuntime4_2_0 {
    void *sequence_lookup;
    void *media_presence;
};

struct TextVars4_2_0 {
    char text[512];
    VFont4_2_0 *text_font;
    int text_blf_id;
    float text_size;
    float color[4], shadow_color[4], box_color[4], outline_color[4];
    float loc[2];
    float wrap_width;
    float box_margin;
    float shadow_angle;
    float shadow_offset;
    float shadow_blur;
    float outline_width;
    char flag;
    char align, align_y;
    char _pad[5];
};

struct SequencerCacheOverlay4_2_0 {
    int flag;
    char _pad0[4];
};

struct bUserExtensionRepo4_2_0 {
    bUserExtensionRepo4_2_0 *next, *prev;
    char name[64];
    char module[48];
    char *access_token;
    char custom_dirpath[1024];
    char remote_url[1024];
    uint8_t flag;
    uint8_t source;
    char _pad0[6];
};

struct UserDef_Experimental4_2_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
    char use_eevee_debug;
    char show_asset_debug_info;
    char no_asset_indexing;
    char use_viewport_debug;
    char use_all_linked_data_direct;
    char use_extensions_debug;
    char use_recompute_usercount_on_save_debug;
    char SANITIZE_AFTER_HERE;
    char use_new_curves_tools;
    char use_new_point_cloud_type;
    char use_sculpt_tools_tilt;
    char use_extended_asset_browser;
    char use_sculpt_texture_paint;
    char use_grease_pencil_version3;
    char enable_overlay_next;
    char use_new_volume_nodes;
    char use_shader_node_previews;
    char use_grease_pencil_version3_convert_on_load;
    char use_animation_baklava;
    char _pad[2];
};

struct mat4x4f4_2_0 {
    float value[4][4];
};

struct SceneEEVEE4_2_0 {
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
    int volumetric_ray_depth;
    float gtao_distance;
    float gtao_factor;
    float gtao_quality;
    float gtao_thickness;
    float gtao_focus;
    int gtao_resolution;
    int fast_gi_step_count;
    int fast_gi_ray_count;
    float fast_gi_distance;
    float fast_gi_thickness_near;
    float fast_gi_thickness_far;
    char fast_gi_method;
    char _pad0[3];
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
    int motion_blur_position_deprecated;
    float motion_blur_shutter_deprecated;
    float motion_blur_depth_scale;
    int shadow_method;
    int shadow_cube_size;
    int shadow_cascade_size;
    int shadow_pool_size;
    int shadow_ray_count;
    int shadow_step_count;
    float shadow_resolution_scale;
    float clamp_surface_direct;
    float clamp_surface_indirect;
    float clamp_volume_direct;
    float clamp_volume_indirect;
    int ray_tracing_method;
    RaytraceEEVEE4_2_0 ray_tracing_options;
    float overscan;
    float light_threshold;
};

struct AnimData4_2_0 {
    bAction3_1_0 *action;
    int32_t binding_handle;
    char binding_name[66];
    uint8_t _pad0[2];
    bAction3_1_0 *tmpact;
    ListBase2_93_0 nla_tracks;
    NlaTrack2_93_0 *act_track;
    NlaStrip2_93_0 *actstrip;
    ListBase2_93_0 drivers;
    ListBase2_93_0 overrides;
    FCurve2_93_0 **driver_array;
    int flag;
    short act_blendmode;
    short act_extendmode;
    float act_influence;
    uint8_t _pad1[4];
};

struct ID_Runtime4_2_0 {
    ID_Runtime_Remap3_2_0 remap;
    void* depsgraph;
    void* _pad;
};

struct ID4_2_0 {
    void* next, * prev;
    ID4_2_0* newid;
    Library4_2_0* lib;
    AssetMetaData3_0_0* asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    unsigned int recalc;
    unsigned int recalc_up_to_undo_push;
    unsigned int recalc_after_undo_push;
    unsigned int session_uid;
    IDProperty3_5_0* properties;
    IDOverrideLibrary3_2_0* override_library;
    ID4_2_0* orig_id;
    void* py_instance;
    LibraryWeakReference3_0_0* library_weak_reference;
    ID_Runtime4_2_0 runtime;
};

struct Collection4_2_0 {
    ID4_2_0 id;
    ID4_2_0 *owner_id;
    ListBase2_93_0 gobject;
    ListBase2_93_0 children;
    char _pad0[4];
    int active_exporter_index;
    ListBase2_93_0 exporters;
    PreviewImage2_93_0 *preview;
    unsigned int layer;
    float instance_offset[3];
    uint8_t flag;
    int8_t color_tag;
    char _pad1[2];
    uint8_t lineart_usage;
    uint8_t lineart_flags;
    uint8_t lineart_intersection_mask;
    uint8_t lineart_intersection_priority;
    ViewLayer4_2_0 *view_layer;
    Collection_Runtime4_1_0 runtime;
};

struct Library4_2_0 {
    ID4_2_0 id;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    Library_Runtime3_3_0 runtime;
};

struct Image4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    DrawDataList2_93_0 drawdata;
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
    Image_Runtime4_2_0 runtime;
};

struct ViewLayer4_2_0 {
    ViewLayer4_2_0 *next, *prev;
    char name[64];
    short flag;
    char _pad[6];
    ListBase2_93_0 object_bases;
    void *stats;
    Base3_4_0 *basact;
    ListBase2_93_0 layer_collections;
    LayerCollection4_0_0 *active_collection;
    int layflag;
    int passflag;
    float pass_alpha_threshold;
    short cryptomatte_flag;
    short cryptomatte_levels;
    char _pad1[4];
    int samples;
    Material2_93_0 *mat_override;
    World2_93_0 *world_override;
    IDProperty3_5_0 *id_properties;
    FreestyleConfig2_93_0 freestyle_config;
    ViewLayerEEVEE2_93_0 eevee;
    ListBase2_93_0 aovs;
    ViewLayerAOV2_93_0 *active_aov;
    ListBase2_93_0 lightgroups;
    ViewLayerLightgroup3_2_0 *active_lightgroup;
    ListBase2_93_0 drawdata;
    Base3_4_0 **object_bases_array;
    void *object_bases_hash;
};

struct LightProbe4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
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
    float grid_clamp_direct;
    float grid_clamp_indirect;
    int grid_surfel_density;
    Collection4_2_0 *visibility_grp;
    float data_display_size;
    char _pad1[4];
};

struct GreasePencilLatticeModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object3_0_0 *object;
    float strength;
    char _pad[4];
};

struct GreasePencilMultiModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    int duplications;
    float distance;
    float offset;
    float fading_center;
    float fading_thickness;
    float fading_opacity;
    int _pad0;
    void *_pad;
};

struct GreasePencilLengthModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float start_fac, end_fac;
    float rand_start_fac, rand_end_fac, rand_offset;
    float overshoot_fac;
    int seed;
    int step;
    int mode;
    char _pad[4];
    float point_density;
    float segment_influence;
    float max_angle;
    void *_pad1;
};

struct GreasePencilWeightAngleModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float min_weight;
    int16_t axis;
    int16_t space;
    float angle;
    char target_vgname[64];
    void *_pad;
};

struct GreasePencilArrayModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object3_0_0 *object;
    int count;
    int flag;
    float offset[3];
    float shift[3];
    float rnd_offset[3];
    float rnd_rot[3];
    float rnd_scale[3];
    char _pad[4];
    int seed;
    int mat_rpl;
};

struct GreasePencilWeightProximityModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    char target_vgname[64];
    float min_weight;
    float dist_start;
    float dist_end;
    Object3_0_0 *object;
};

struct GreasePencilHookModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object3_0_0 *object;
    char subtarget[64];
    char _pad[4];
    int flag;
    char falloff_type;
    char _pad1[3];
    float parentinv[4][4];
    float cent[3];
    float falloff;
    float force;
};

struct GreasePencilLineartModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    uint16_t edge_types;
    char source_type;
    char use_multiple_levels;
    short level_start;
    short level_end;
    Object3_0_0 *source_camera;
    Object3_0_0 *light_contour_object;
    Object3_0_0 *source_object;
    Collection4_2_0 *source_collection;
    Material2_93_0 *target_material;
    char target_layer[64];
    char source_vertex_group[64];
    char vgname[64];
    float overscan;
    float shadow_camera_fov;
    float shadow_camera_size;
    float shadow_camera_near;
    float shadow_camera_far;
    float opacity;
    short thickness;
    unsigned char mask_switches;
    unsigned char material_mask_bits;
    unsigned char intersection_mask;
    unsigned char shadow_selection;
    unsigned char silhouette_selection;
    char _pad[1];
    float crease_threshold;
    float angle_splitting_threshold;
    float chain_smooth_tolerance;
    float chaining_image_threshold;
    int calculation_flags;
    int flags;
    float stroke_depth_offset;
    char level_start_override;
    char level_end_override;
    short edge_types_override;
    char shadow_selection_override;
    char shadow_use_silhouette_override;
    char _pad2[6];
    void *shared_cache;
    void *cache;
    void *la_data_ptr;
};

struct GreasePencilArmatureModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object3_0_0 *object;
    short deformflag;
    char _pad[6];
};

struct GreasePencilEnvelopeModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int mode;
    int mat_nr;
    float thickness;
    float strength;
    int skip;
    int spread;
};

struct GreasePencilOutlineModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object3_0_0 *object;
    int flag;
    int thickness;
    float sample_length;
    int subdiv;
    Material2_93_0 *outline_material;
};

struct GreasePencilShrinkwrapModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object3_0_0 *target;
    Object3_0_0 *aux_target;
    float keep_dist;
    short shrink_type;
    char shrink_opts;
    char shrink_mode;
    float proj_limit;
    char proj_axis;
    char subsurf_levels;
    char _pad[2];
    float smooth_factor;
    int smooth_step;
    void *cache_data;
};

struct GreasePencilBuildModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
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

struct GreasePencilSimplifyModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    short mode;
    char _pad[4];
    short step;
    float factor;
    float length;
    float sharp_threshold;
    float distance;
};

struct GreasePencilTextureModifierData4_2_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    float uv_offset;
    float uv_scale;
    float fill_rotation;
    float fill_offset[2];
    float fill_scale;
    int layer_pass;
    short fit_method;
    short mode;
    float alignment_rotation;
    char _pad[4];
};

struct SceneRenderLayer4_2_0 {
    SceneRenderLayer4_2_0 *next, *prev;
    char name[64];
    Material2_93_0 *mat_override;
    World2_93_0 *world_override;
    unsigned int lay;
    unsigned int lay_zmask;
    unsigned int lay_exclude;
    int layflag;
    int passflag;
    int pass_xor;
    int samples;
    float pass_alpha_threshold;
    IDProperty3_5_0 *prop;
    FreestyleConfig2_93_0 freestyleConfig;
};

struct RenderData4_2_0 {
    ImageFormatData3_2_0 im_format;
    void *_pad;
    FFMpegCodecData3_1_0 ffcodecdata;
    int cfra, sfra, efra;
    float subframe;
    int psfra, pefra;
    int images, framapto;
    short flag, threads;
    float framelen;
    int frame_step;
    short dimensionspreset;
    short size;
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
    float motion_blur_shutter;
    int motion_blur_position;
    CurveMapping2_93_0 mblur_shutter_curve;
    int compositor_device;
    int compositor_precision;
};

struct ToolSettings4_2_0 {
    VPaint2_93_0 *vpaint;
    VPaint2_93_0 *wpaint;
    Sculpt4_1_0 *sculpt;
    UvSculpt4_2_0 uvsculpt;
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
    char annotate_v3d_align;
    short annotate_thickness;
    float gpencil_surface_offset;
    char gpencil_selectmode_edit;
    char gpencil_selectmode_sculpt;
    char _pad0[6];
    GP_Sculpt_Settings2_93_0 gp_sculpt;
    GP_Interpolate_Settings2_93_0 gp_interpolate;
    ImagePaintSettings2_93_0 imapaint;
    PaintModeSettings3_2_0 paint_mode;
    ParticleEditSettings3_0_0 particle;
    float proportional_size;
    float select_thresh;
    short keying_flag;
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
    char workspace_tool_type;
    char _pad5[1];
    short sculpt_paint_settings;
    int sculpt_paint_unified_size;
    float sculpt_paint_unified_unprojected_radius;
    float sculpt_paint_unified_alpha;
    UnifiedPaintSettings4_1_0 unified_paint_settings;
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
    float snap_angle_increment_2d;
    float snap_angle_increment_2d_precision;
    float snap_angle_increment_3d;
    float snap_angle_increment_3d_precision;
};

struct Scene4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object3_0_0 *camera;
    World2_93_0 *world;
    Scene4_2_0 *set;
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
    Editing4_2_0 *ed;
    ToolSettings4_2_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData4_2_0 r;
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
    MovieClip4_2_0 *clip;
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
    Collection4_2_0 *master_collection;
    IDProperty3_5_0 *layer_properties;
    int simulation_frame_start;
    int simulation_frame_end;
    SceneDisplay2_93_0 display;
    SceneEEVEE4_2_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
    SceneHydra4_0_0 hydra;
    void *runtime;
    void *_pad9;
};

struct ARegion_Runtime4_2_0 {
    const char *category;
    rcti2_93_0 visible_rect;
    int offset_x, offset_y;
    void *block_name_map;
    Panel4_1_0 *popup_block_panel;
};

struct Editing4_2_0 {
    ListBase2_93_0 *seqbasep;
    ListBase2_93_0 *displayed_channels;
    void *_pad0;
    ListBase2_93_0 seqbase;
    ListBase2_93_0 metastack;
    ListBase2_93_0 channels;
    Sequence4_0_0 *act_seq;
    char act_imagedir[1024];
    char act_sounddir[1024];
    char proxy_dir[1024];
    int proxy_storage;
    int overlay_frame_ofs, overlay_frame_abs;
    int overlay_frame_flag;
    rctf2_93_0 overlay_frame_rect;
    int show_missing_media_flag;
    int _pad1;
    void *cache;
    float recycle_max_cost;
    int cache_flag;
    void *prefetch_job;
    int64_t disk_cache_timestamp;
    EditingRuntime4_2_0 runtime;
};

struct SpaceSeq4_2_0 {
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
    SequencerPreviewOverlay3_0_0 preview_overlay;
    SequencerTimelineOverlay3_0_0 timeline_overlay;
    SequencerCacheOverlay4_2_0 cache_overlay;
    char multiview_eye;
    char _pad2[7];
    void *runtime;
};

struct SpaceImage4_2_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Image4_2_0 *image;
    ImageUser3_0_0 iuser;
    Scopes4_1_0 scopes;
    Histogram2_93_0 sample_line_hist;
    bGPdata3_0_0 *gpd;
    float cursor[2];
    float xof, yof;
    float zoom;
    float centx, centy;
    char mode;
    char mode_prev;
    char pin;
    char pixel_round_mode;
    char lock;
    char dt_uv;
    char dt_uvstretch;
    char around;
    char gizmo_flag;
    char grid_shape_source;
    char _pad1[6];
    int flag;
    float uv_opacity;
    float stretch_opacity;
    int tile_grid_shape[2];
    int custom_grid_subdiv[2];
    MaskSpaceInfo3_3_0 mask_info;
    SpaceImageOverlay2_93_0 overlay;
};

struct ThemeSpace4_2_0 {
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
    unsigned char edge[4], edge_select[4], edge_mode_select[4];
    unsigned char edge_seam[4], edge_sharp[4], edge_facesel[4], edge_crease[4], edge_bevel[4];
    unsigned char face[4], face_select[4], face_mode_select[4], face_retopology[4];
    unsigned char face_back[4], face_front[4];
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
    unsigned char keytype_keyframe[4], keytype_extreme[4], keytype_breakdown[4], keytype_jitter[4],      keytype_movehold[4], keytype_generated[4];
    unsigned char keytype_keyframe_select[4], keytype_extreme_select[4], keytype_breakdown_select[4],      keytype_jitter_select[4], keytype_movehold_select[4], keytype_generated_select[4];
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

struct bTheme4_2_0 {
    bTheme4_2_0 *next, *prev;
    char name[64];
    char filepath[1024];
    ThemeUI4_0_0 tui;
    ThemeSpace4_2_0 space_properties;
    ThemeSpace4_2_0 space_view3d;
    ThemeSpace4_2_0 space_file;
    ThemeSpace4_2_0 space_graph;
    ThemeSpace4_2_0 space_info;
    ThemeSpace4_2_0 space_action;
    ThemeSpace4_2_0 space_nla;
    ThemeSpace4_2_0 space_sequencer;
    ThemeSpace4_2_0 space_image;
    ThemeSpace4_2_0 space_text;
    ThemeSpace4_2_0 space_outliner;
    ThemeSpace4_2_0 space_node;
    ThemeSpace4_2_0 space_preferences;
    ThemeSpace4_2_0 space_console;
    ThemeSpace4_2_0 space_clip;
    ThemeSpace4_2_0 space_topbar;
    ThemeSpace4_2_0 space_statusbar;
    ThemeSpace4_2_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

struct bUserAssetShelfSettings4_2_0 {
    bUserAssetShelfSettings4_2_0 *next, *prev;
    char shelf_idname[64];
    ListBase2_93_0 enabled_catalog_paths;
};

struct wmWindowManager4_2_0 {
    ID4_2_0 id;
    wmWindow4_1_0 *windrawable;
    wmWindow4_1_0 *winactive;
    ListBase2_93_0 windows;
    uint8_t init_flag;
    char _pad0[1];
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    ListBase2_93_0 operators;
    ListBase2_93_0 notifier_queue;
    void *notifier_queue_set;
    void *_pad1;
    int extensions_updates;
    int _pad3;
    ListBase2_93_0 jobs;
    ListBase2_93_0 paintcursors;
    ListBase2_93_0 drags;
    ListBase2_93_0 keyconfigs;
    wmKeyConfig2_93_0 *defaultconf;
    wmKeyConfig2_93_0 *addonconf;
    wmKeyConfig2_93_0 *userconf;
    ListBase2_93_0 timers;
    void *autosavetimer;
    char autosave_scheduled;
    char _pad2[7];
    void *undo_stack;
    void *message_bus;
    wmXrData2_93_0 xr;
    void *runtime;
};

struct WorkSpace4_2_0 {
    ID4_2_0 id;
    ListBase2_93_0 layouts;
    ListBase2_93_0 hook_layout_relations;
    ListBase2_93_0 owner_ids;
    ListBase2_93_0 tools;
    Scene4_2_0 *pin_scene;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    void *runtime;
    AssetLibraryReference3_0_0 asset_library_ref;
    ViewerPath3_4_0 viewer_path;
};

struct Ipo4_2_0 {
    ID4_2_0 id;
    ListBase2_93_0 curve;
    rctf2_93_0 cur;
    short blocktype;
    short showkey;
    short muteipo;
    char _pad[2];
};

struct Key4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    KeyBlock2_93_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    ListBase2_93_0 block;
    Ipo4_2_0 *ipo;
    ID4_2_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct Mask4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    DrawDataList2_93_0 drawdata;
    ListBase2_93_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra, efra;
    int flag;
    char _pad[4];
};

struct MetaBall4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 *editelems;
    Ipo4_2_0 *ipo;
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

struct MovieClip4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    DrawDataList2_93_0 drawdata;
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

struct ParticleSettings4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    BoidSettings2_93_0 *boids;
    SPHFluidSettings2_93_0 *fluid;
    EffectorWeights3_0_0 *effector_weights;
    Collection4_2_0 *collision_group;
    int flag;
    char _pad1[4];
    short type, from, distr, texact;
    short phystype, rotmode, avemode, reactevent;
    int draw;
    float draw_size;
    short draw_as, childtype;
    char _pad2[4];
    short ren_as, subframes, draw_col;
    short draw_step, ren_step;
    short hair_step, keys_step;
    short adapt_angle, adapt_pix;
    short disp, omat, interpolation, integrator;
    short rotfrom;
    short kink, kink_axis;
    short bb_align, bb_uv_split, bb_anim, bb_split_offset;
    float bb_tilt, bb_rand_tilt, bb_offset[2], bb_size[2], bb_vel_head, bb_vel_tail;
    float color_vec_max;
    float sta, end, lifetime, randlife;
    float timetweak, courant_target;
    float jitfac, eff_hair, grid_rand, ps_offset[1];
    int totpart, userjit, grid_res, effector_amount;
    short time_flag;
    char _pad0[6];
    float normfac, obfac, randfac, partfac, tanfac, tanphase, reactfac;
    float ob_vel[3];
    float avefac, phasefac, randrotfac, randphasefac;
    float mass, size, randsize;
    float acc[3], dragfac, brownfac, dampfac;
    float randlength;
    int child_flag;
    char _pad3[4];
    int child_percent, child_render_percent;
    float parents, childsize, childrandsize;
    float childrad, childflat;
    float clumpfac, clumppow;
    float kink_amp, kink_freq, kink_shape, kink_flat;
    float kink_amp_clump;
    int kink_extra_steps;
    char _pad4[4];
    float kink_axis_random, kink_amp_random;
    float rough1, rough1_size;
    float rough2, rough2_size, rough2_thres;
    float rough_end, rough_end_shape;
    float clength, clength_thres;
    float parting_fac;
    float parting_min, parting_max;
    float branch_thres;
    float draw_line[2];
    float path_start, path_end;
    int trail_count;
    int keyed_loops;
    CurveMapping2_93_0 *clumpcurve;
    CurveMapping2_93_0 *roughcurve;
    float clump_noise_size;
    float bending_random;
    MTex3_0_0 *mtex[18];
    Collection4_2_0 *instance_collection;
    ListBase2_93_0 instance_weights;
    Collection4_2_0 *force_group;
    Object3_0_0 *instance_object;
    Object3_0_0 *bb_ob;
    Ipo4_2_0 *ipo;
    PartDeflect4_2_0 *pd;
    PartDeflect4_2_0 *pd2;
    short use_modifier_stack;
    char _pad5[2];
    short shape_flag;
    char _pad6[2];
    float twist;
    char _pad8[4];
    float shape;
    float rad_root, rad_tip, rad_scale;
    CurveMapping2_93_0 *twistcurve;
    void *_pad7;
};

struct bScreen4_2_0 {
    ID4_2_0 id;
    ListBase2_93_0 vertbase;
    ListBase2_93_0 edgebase;
    ListBase2_93_0 areabase;
    ListBase2_93_0 regionbase;
    Scene4_2_0 *scene;
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
    ARegion4_2_0 *active_region;
    void *animtimer;
    void  *context;
    void *tool_tip;
    PreviewImage2_93_0 *preview;
};

struct ARegion4_2_0 {
    ARegion4_2_0 *next, *prev;
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
    ARegion_Runtime4_2_0 runtime;
};

struct bSound4_2_0 {
    ID4_2_0 id;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    void *handle;
    PackedFile2_93_0 *newpackedfile;
    Ipo4_2_0 *ipo;
    float volume;
    float attenuation;
    float pitch;
    float min_gain;
    float max_gain;
    float distance;
    short flags;
    short tags;
    char _pad[4];
    double offset_time;
    void *cache;
    void *waveform;
    void *playback_handle;
    void *spinlock;
    int audio_channels;
    int samplerate;
};

struct Script4_2_0 {
    ID4_2_0 id;
    void *py_draw;
    void *py_event;
    void *py_button;
    void *py_browsercallback;
    void *py_globaldict;
    int flags, lastspace;
    char scriptname[1024];
    char scriptarg[256];
};

struct Speaker4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    bSound4_2_0 *sound;
    float volume_max;
    float volume_min;
    float distance_max;
    float distance_reference;
    float attenuation;
    float cone_angle_outer;
    float cone_angle_inner;
    float cone_volume_outer;
    float volume;
    float pitch;
    short flag;
    char _pad1[6];
};

struct Text4_2_0 {
    ID4_2_0 id;
    char *filepath;
    void *compiled;
    int flags;
    char _pad0[4];
    ListBase2_93_0 lines;
    TextLine2_93_0 *curl, *sell;
    int curc, selc;
    double mtime;
};

struct VFont4_2_0 {
    ID4_2_0 id;
    char filepath[1024];
    void *data;
    PackedFile2_93_0 *packedfile;
    PackedFile2_93_0 *temp_pf;
};

struct Volume4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    char is_sequence;
    char sequence_mode;
    char _pad1[2];
    int frame_start;
    int frame_duration;
    int frame_offset;
    int flag;
    int active_grid;
    Material2_93_0 **mat;
    short totcol;
    short _pad2[3];
    VolumeRender2_93_0 render;
    VolumeDisplay2_93_0 display;
    char velocity_grid[64];
    char _pad3[3];
    char velocity_unit;
    float velocity_scale;
    void *batch_cache;
    void *runtime;
};

struct IdAdtTemplate4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
};

struct Palette4_2_0 {
    ID4_2_0 id;
    ListBase2_93_0 colors;
    int active_color;
    char _pad[4];
};

struct PaintCurve4_2_0 {
    ID4_2_0 id;
    PaintCurvePoint2_93_0 *points;
    int tot_points;
    int add_index;
};

struct CacheFile4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    ListBase2_93_0 object_paths;
    ListBase2_93_0 layers;
    char filepath[1024];
    char is_sequence;
    char forward_axis;
    char up_axis;
    char override_frame;
    float scale;
    float frame;
    float frame_offset;
    char _pad[4];
    short flag;
    char type;
    char use_render_procedural;
    char _pad1[3];
    char use_prefetch;
    int prefetch_cache_size;
    int active_layer;
    char _pad2[3];
    char velocity_unit;
    char velocity_name[64];
    void *handle;
    char handle_filepath[1024];
    void *handle_readers;
};

struct Camera4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
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
    Ipo4_2_0 *ipo;
    Object3_0_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings3_3_0 dof;
    ListBase2_93_0 bg_images;
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct Curves4_2_0 {
    ID4_2_0 id;
    AnimData4_2_0 *adt;
    CurvesGeometry3_3_0 geometry;
    int flag;
    int attributes_active_index;
    Material2_93_0 **mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    Object3_0_0 *surface;
    char *surface_uv_map;
    void *batch_cache;
};

#endif
