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

Various structs from Blender v4.1 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_4_1_0_H
#define MAKESDNA_TYPES_4_1_0_H

#include <stdint.h>
#include "makesdna_types_4_0_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_5_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_3_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_6_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_1_0.h"

struct bPoseChannel_Runtime4_1_0;
struct DriverTarget4_1_0;
struct bArmature4_1_0;
struct BoneCollection4_1_0;
struct Brush4_1_0;
struct Collection_Runtime4_1_0;
struct Collection4_1_0;
struct Scopes4_1_0;
struct GreasePencil4_1_0;
struct IDPropertyUIDataEnumItem4_1_0;
struct IDPropertyUIDataInt4_1_0;
struct ID4_1_0;
struct Image4_1_0;
struct LightProbe4_1_0;
struct Material4_1_0;
struct Mesh4_1_0;
struct ModifierData4_1_0;
struct NodesModifierDataBlock4_1_0;
struct NodesModifierBake4_1_0;
struct NodesModifierPanel4_1_0;
struct NodesModifierData4_1_0;
struct GreasePencilModifierInfluenceData4_1_0;
struct GreasePencilOpacityModifierData4_1_0;
struct GreasePencilSubdivModifierData4_1_0;
struct GreasePencilColorModifierData4_1_0;
struct GreasePencilTintModifierData4_1_0;
struct GreasePencilSmoothModifierData4_1_0;
struct GreasePencilOffsetModifierData4_1_0;
struct GreasePencilNoiseModifierData4_1_0;
struct GreasePencilMirrorModifierData4_1_0;
struct GreasePencilThickModifierData4_1_0;
struct MovieClip4_1_0;
struct bNodeTreeInterfaceSocket4_1_0;
struct bNode4_1_0;
struct bNodeTree4_1_0;
struct bNodeSocketValueMenu4_1_0;
struct NodeKuwaharaData4_1_0;
struct NodeKeyingScreenData4_1_0;
struct NodeEnumItem4_1_0;
struct NodeEnumDefinition4_1_0;
struct NodeMenuSwitch4_1_0;
struct IndexSwitchItem4_1_0;
struct NodeIndexSwitch4_1_0;
struct NodeGeometryBakeItem4_1_0;
struct NodeGeometryBake4_1_0;
struct BoundBox4_1_0;
struct Object4_1_0;
struct Paint4_1_0;
struct Sculpt4_1_0;
struct UnifiedPaintSettings4_1_0;
struct ToolSettings4_1_0;
struct RaytraceEEVEE4_1_0;
struct SceneEEVEE4_1_0;
struct LayoutPanelState4_1_0;
struct Panel4_1_0;
struct FileHandler4_1_0;
struct SeqRetimingKey4_1_0;
struct SequenceRuntime4_1_0;
struct SessionUID4_1_0;
struct SpaceSeq4_1_0;
struct SpaceText4_1_0;
struct SpaceConsole4_1_0;
struct SpaceSpreadsheet4_1_0;
struct uiFontStyle4_1_0;
struct ThemeSpace4_1_0;
struct bUserExtensionRepo4_1_0;
struct UserDef_Experimental4_1_0;
struct UserDef4_1_0;
struct vec3i4_1_0;
struct View3DOverlay4_1_0;
struct View3D4_1_0;
struct Volume4_1_0;
struct ReportList4_1_0;
struct wmWindowManager4_1_0;
struct wmWindow4_1_0;
struct bPoseChannel4_1_0;
struct bAction4_1_0;
struct DriverVar4_1_0;
struct IdAdtTemplate4_1_0;
struct Palette4_1_0;
struct PaintCurve4_1_0;
struct CacheFile4_1_0;
struct Camera4_1_0;
struct Curves4_1_0;
struct Curve4_1_0;
struct bGPdata4_1_0;
struct Ipo4_1_0;
struct Key4_1_0;
struct Lattice4_1_0;
struct Light4_1_0;
struct FreestyleLineStyle4_1_0;
struct Mask4_1_0;
struct MetaBall4_1_0;
struct MappingInfoModifierData4_1_0;
struct SubsurfModifierData4_1_0;
struct LatticeModifierData4_1_0;
struct CurveModifierData4_1_0;
struct BuildModifierData4_1_0;
struct MaskModifierData4_1_0;
struct ArrayModifierData4_1_0;
struct MirrorModifierData4_1_0;
struct EdgeSplitModifierData4_1_0;
struct BevelModifierData4_1_0;
struct FluidModifierData4_1_0;
struct DisplaceModifierData4_1_0;
struct UVProjectModifierData4_1_0;
struct DecimateModifierData4_1_0;
struct SmoothModifierData4_1_0;
struct CastModifierData4_1_0;
struct WaveModifierData4_1_0;
struct HookModifierData4_1_0;
struct SoftbodyModifierData4_1_0;
struct ClothModifierData4_1_0;
struct BooleanModifierData4_1_0;
struct ParticleSystemModifierData4_1_0;
struct ParticleInstanceModifierData4_1_0;
struct ExplodeModifierData4_1_0;
struct MultiresModifierData4_1_0;
struct FluidsimModifierData4_1_0;
struct SmokeModifierData4_1_0;
struct ShrinkwrapModifierData4_1_0;
struct SimpleDeformModifierData4_1_0;
struct ShapeKeyModifierData4_1_0;
struct SolidifyModifierData4_1_0;
struct ScrewModifierData4_1_0;
struct OceanModifierData4_1_0;
struct WarpModifierData4_1_0;
struct WeightVGEditModifierData4_1_0;
struct WeightVGMixModifierData4_1_0;
struct WeightVGProximityModifierData4_1_0;
struct DynamicPaintModifierData4_1_0;
struct RemeshModifierData4_1_0;
struct SkinModifierData4_1_0;
struct TriangulateModifierData4_1_0;
struct LaplacianSmoothModifierData4_1_0;
struct UVWarpModifierData4_1_0;
struct MeshCacheModifierData4_1_0;
struct LaplacianDeformModifierData4_1_0;
struct WireframeModifierData4_1_0;
struct WeldModifierData4_1_0;
struct DataTransferModifierData4_1_0;
struct NormalEditModifierData4_1_0;
struct MeshSeqCacheModifierData4_1_0;
struct SurfaceDeformModifierData4_1_0;
struct WeightedNormalModifierData4_1_0;
struct MeshToVolumeModifierData4_1_0;
struct VolumeDisplaceModifierData4_1_0;
struct VolumeToMeshModifierData4_1_0;
struct ParticleSettings4_1_0;
struct PointCloud4_1_0;
struct ImagePaintSettings4_1_0;
struct CurvesSculpt4_1_0;
struct UvSculpt4_1_0;
struct GpPaint4_1_0;
struct GpVertexPaint4_1_0;
struct GpSculptPaint4_1_0;
struct GpWeightPaint4_1_0;
struct VPaint4_1_0;
struct Scene4_1_0;
struct bScreen4_1_0;
struct Sequence4_1_0;
struct bSound4_1_0;
struct SpaceImage4_1_0;
struct Script4_1_0;
struct Speaker4_1_0;
struct Tex4_1_0;
struct Text4_1_0;
struct uiStyle4_1_0;
struct bTheme4_1_0;
struct VFont4_1_0;
struct WorkSpace4_1_0;
struct World4_1_0;

struct DriverTarget4_1_0 {
    ID4_1_0 *id;
    char *rna_path;
    char pchan_name[64];
    short transChan;
    char rotation_mode;
    char _pad[5];
    short flag;
    short options;
    int idtype;
    int context_property;
    float fallback_value;
};

struct IDPropertyUIDataEnumItem4_1_0 {
    char *identifier;
    char *name;
    char *description;
    int value;
    int icon;
};

struct ModifierData4_1_0 {
    ModifierData4_1_0 *next, *prev;
    int type, mode;
    float execution_time;
    short flag;
    short ui_expand_flag;
    uint16_t layout_panel_open_flag;
    char _pad[2];
    int persistent_uid;
    char name[64];
    char *error;
    void *runtime;
};

struct NodesModifierDataBlock4_1_0 {
    char *id_name;
    char *lib_name;
    ID4_1_0 *id;
    int id_type;
    char _pad[4];
};

struct NodesModifierBake4_1_0 {
    int id;
    uint32_t flag;
    uint8_t bake_mode;
    char _pad[7];
    char *directory;
    int frame_start;
    int frame_end;
    int data_blocks_num;
    int active_data_block;
    NodesModifierDataBlock4_1_0 *data_blocks;
};

struct NodesModifierPanel4_1_0 {
    int id;
    uint32_t flag;
};

struct GreasePencilModifierInfluenceData4_1_0 {
    int flag;
    char _pad1[4];
    char layer_name[64];
    Material4_1_0 *material;
    int layer_pass;
    int material_pass;
    char vertex_group_name[64];
    CurveMapping2_93_0 *custom_curve;
    void *_pad2;
};

struct bNodeSocketValueMenu4_1_0 {
    int value;
    int runtime_flag;
    const void *enum_items;
};

struct NodeKuwaharaData4_1_0 {
    short size;
    short variation;
    int uniformity;
    float sharpness;
    float eccentricity;
    char high_precision;
    char _pad[3];
};

struct NodeKeyingScreenData4_1_0 {
    char tracking_object[64];
    float smoothness;
};

struct NodeEnumItem4_1_0 {
    char *name;
    char *description;
    int32_t identifier;
    char _pad[4];
};

struct NodeEnumDefinition4_1_0 {
    NodeEnumItem4_1_0 *items_array;
    int16_t items_num;
    int16_t active_index;
    uint32_t next_identifier;
};

struct IndexSwitchItem4_1_0 {
    int identifier;
};

struct NodeIndexSwitch4_1_0 {
    IndexSwitchItem4_1_0 *items;
    int items_num;
    int data_type;
    int next_identifier;
    char _pad[4];
};

struct NodeGeometryBakeItem4_1_0 {
    char *name;
    int16_t socket_type;
    int16_t attribute_domain;
    int identifier;
    int32_t flag;
    char _pad[4];
};

struct NodeGeometryBake4_1_0 {
    NodeGeometryBakeItem4_1_0 *items;
    int items_num;
    int next_identifier;
    int active_index;
    char _pad[4];
};

struct BoundBox4_1_0 {
    float vec[8][3];
};

struct UnifiedPaintSettings4_1_0 {
    int size;
    float unprojected_radius;
    float alpha;
    float weight;
    float rgb[3];
    float secondary_rgb[3];
    int input_samples;
    int flag;
    char _pad[4];
    float last_rake[2];
    float last_rake_angle;
    int last_stroke_valid;
    float average_stroke_accum[3];
    int average_stroke_counter;
    float brush_rotation;
    float brush_rotation_sec;
    int anchored_size;
    float overlap_factor;
    char draw_inverted;
    char stroke_active;
    char draw_anchored;
    char do_linear_conversion;
    float last_location[3];
    int last_hit;
    float anchored_initial_mouse[2];
    float pixel_radius;
    float initial_pixel_radius;
    float start_pixel_radius;
    float size_pressure_value;
    float tex_mouse[2];
    float mask_tex_mouse[2];
    void *colorspace;
};

struct RaytraceEEVEE4_1_0 {
    float screen_trace_quality;
    float screen_trace_thickness;
    float screen_trace_max_roughness;
    int resolution_scale;
    float sample_clamp;
    int flag;
    int denoise_stages;
    char _pad0[4];
};

struct LayoutPanelState4_1_0 {
    LayoutPanelState4_1_0 *next, *prev;
    char *idname;
    uint8_t flag;
    char _pad[7];
};

struct FileHandler4_1_0 {
    void *type;
};

struct SeqRetimingKey4_1_0 {
    double strip_frame_index;
    int flag;
    int _pad0;
    float retiming_factor;
    char _pad1[4];
    double original_strip_frame_index;
    float original_retiming_factor;
    char _pad2[4];
};

struct SessionUID4_1_0 {
    uint64_t uid_;
};

struct uiFontStyle4_1_0 {
    short uifont_id;
    char _pad1[2];
    float points;
    short italic, bold;
    short shadow;
    short shadx, shady;
    char _pad0[2];
    float shadowalpha;
    float shadowcolor;
    int character_weight;
};

struct bUserExtensionRepo4_1_0 {
    bUserExtensionRepo4_1_0 *next, *prev;
    char name[64];
    char module[48];
    char custom_dirpath[1024];
    char remote_path[1024];
    int flag;
    char _pad0[4];
};

struct UserDef_Experimental4_1_0 {
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
    char use_sculpt_texture_paint;
    char use_grease_pencil_version3;
    char enable_overlay_next;
    char use_new_volume_nodes;
    char use_shader_node_previews;
    char use_extension_repos;
    char _pad[4];
};

struct vec3i4_1_0 {
    int x, y, z;
};

struct View3DOverlay4_1_0 {
    int flag;
    int edit_flag;
    float normals_length;
    float normals_constant_screen_size;
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
    float fade_alpha;
    float wireframe_threshold;
    float wireframe_opacity;
    float retopology_offset;
    float gpencil_paper_opacity;
    float gpencil_grid_opacity;
    float gpencil_fade_layer;
    float gpencil_vertex_paint_opacity;
    int handle_display;
    float sculpt_curves_cage_opacity;
};

struct GreasePencilOpacityModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    char color_mode;
    char _pad1[3];
    float color_factor;
    float hardness_factor;
    void *_pad2;
};

struct GreasePencilSubdivModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int type;
    int level;
    char _pad[8];
    void *_pad1;
};

struct GreasePencilColorModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    char color_mode;
    char _pad1[3];
    float hsv[3];
    void *_pad2;
};

struct GreasePencilTintModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    short flag;
    char color_mode;
    char tint_mode;
    float factor;
    float radius;
    float color[3];
    Object4_1_0 *object;
    ColorBand2_93_0 *color_ramp;
    void *_pad;
};

struct GreasePencilSmoothModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float factor;
    int step;
    char _pad[4];
    void *_pad1;
};

struct GreasePencilOffsetModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    int offset_mode;
    float loc[3];
    float rot[3];
    float scale[3];
    float stroke_loc[3];
    float stroke_rot[3];
    float stroke_scale[3];
    int seed;
    int stroke_step;
    int stroke_start_offset;
    char _pad1[4];
    void *_pad2;
};

struct GreasePencilNoiseModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float factor;
    float factor_strength;
    float factor_thickness;
    float factor_uvs;
    float noise_scale;
    float noise_offset;
    short noise_mode;
    char _pad[2];
    int step;
    int seed;
    void *_pad1;
};

struct GreasePencilMirrorModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    Object4_1_0 *object;
    int flag;
    char _pad[4];
};

struct GreasePencilThickModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float thickness_fac;
    float thickness;
    char _pad[4];
    void *_pad1;
};

struct NodeMenuSwitch4_1_0 {
    NodeEnumDefinition4_1_0 enum_definition;
    uint8_t data_type;
    char _pad[7];
};

struct SceneEEVEE4_1_0 {
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
    int ray_tracing_method;
    RaytraceEEVEE4_1_0 ray_tracing_options;
    LightCache2_93_0 *light_cache;
    LightCache2_93_0 *light_cache_data;
    char light_cache_info[128];
    float overscan;
    float light_threshold;
};

struct SequenceRuntime4_1_0 {
    SessionUID4_1_0 session_uid;
};

struct DriverVar4_1_0 {
    DriverVar4_1_0 *next, *prev;
    char name[64];
    DriverTarget4_1_0 targets[8];
    char num_targets;
    char type;
    short flag;
    float curval;
};

struct MappingInfoModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Tex4_1_0 *texture;
    Object4_1_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
};

struct SubsurfModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    short subdivType, levels, renderLevels, flags;
    short uv_smooth;
    short quality;
    short boundary_smooth;
    char _pad[2];
    void *emCache, *mCache;
};

struct LatticeModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct CurveModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
    void *_pad1;
};

struct BuildModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    float start, length;
    short flag;
    short randomize;
    int seed;
};

struct MaskModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
    void *_pad1;
};

struct ArrayModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *start_cap;
    Object4_1_0 *end_cap;
    Object4_1_0 *curve_ob;
    Object4_1_0 *offset_ob;
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

struct MirrorModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    short axis;
    short flag;
    float tolerance;
    float bisect_threshold;
    uint8_t use_correct_order_on_merge;
    char _pad[3];
    float uv_offset[2];
    float uv_offset_copy[2];
    Object4_1_0 *mirror_ob;
    void *_pad1;
};

struct EdgeSplitModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    float split_angle;
    int flags;
};

struct BevelModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct FluidModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    FluidDomainSettings3_2_0 *domain;
    FluidFlowSettings3_5_0 *flow;
    FluidEffectorSettings2_93_0 *effector;
    float time;
    int type;
    void *_pad1;
};

struct DisplaceModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Tex4_1_0 *texture;
    Object4_1_0 *map_object;
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

struct UVProjectModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *projectors[10];
    char _pad2[4];
    int projectors_num;
    float aspectx, aspecty;
    float scalex, scaley;
    char uvlayer_name[68];
    int uvlayer_tmp;
};

struct DecimateModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct SmoothModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    float fac;
    char defgrp_name[64];
    short flag, repeat;
};

struct CastModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag;
    short type;
    void *_pad1;
};

struct WaveModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Tex4_1_0 *texture;
    Object4_1_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    Object4_1_0 *objectcenter;
    char defgrp_name[64];
    short flag;
    char _pad2[2];
    float startx, starty, height, width;
    float narrow, speed, damp, falloff;
    float timeoffs, lifetime;
    char _pad3[4];
    void *_pad4;
};

struct HookModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
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

struct SoftbodyModifierData4_1_0 {
    ModifierData4_1_0 modifier;
};

struct BooleanModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
    Collection4_1_0 *collection;
    float double_threshold;
    char operation;
    char solver;
    char material_mode;
    char flag;
    char bm_flag;
    char _pad[7];
};

struct ParticleSystemModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    void *psys;
    Mesh4_1_0 *mesh_final;
    Mesh4_1_0 *mesh_original;
    int totdmvert, totdmedge, totdmface;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct ParticleInstanceModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *ob;
    short psys, flag, axis, space;
    float position, random_position;
    float rotation, random_rotation;
    float particle_amount, particle_offset;
    char index_layer_name[68];
    char value_layer_name[68];
    void *_pad1;
};

struct ExplodeModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    int *facepa;
    short flag, vgroup;
    float protect;
    char uvname[68];
    char _pad1[4];
    void *_pad2;
};

struct MultiresModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char lvl, sculptlvl, renderlvl, totlvl;
    char simple;
    char flags, _pad[2];
    short quality;
    short uv_smooth;
    short boundary_smooth;
    char _pad2[2];
};

struct FluidsimModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    FluidsimSettings2_93_0 *fss;
    void *_pad1;
};

struct SmokeModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    int type;
    int _pad;
};

struct ShrinkwrapModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *target;
    Object4_1_0 *auxTarget;
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

struct SimpleDeformModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *origin;
    char vgroup_name[64];
    float factor;
    float limit[2];
    char mode;
    char axis;
    char deform_axis;
    char flag;
    void *_pad1;
};

struct ShapeKeyModifierData4_1_0 {
    ModifierData4_1_0 modifier;
};

struct SolidifyModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct ScrewModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *ob_axis;
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

struct OceanModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct WarpModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Tex4_1_0 *texture;
    Object4_1_0 *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    Object4_1_0 *object_from;
    Object4_1_0 *object_to;
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

struct WeightVGEditModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char defgrp_name[64];
    short edit_flags;
    short falloff_type;
    float default_weight;
    CurveMapping2_93_0 *cmap_curve;
    float add_threshold, rem_threshold;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex4_1_0 *mask_texture;
    Object4_1_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    void *_pad1;
};

struct WeightVGMixModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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
    Tex4_1_0 *mask_texture;
    Object4_1_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    char _pad1[4];
    char flag;
    char _pad2[3];
};

struct WeightVGProximityModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char defgrp_name[64];
    CurveMapping2_93_0 *cmap_curve;
    int proximity_mode;
    int proximity_flags;
    Object4_1_0 *proximity_ob_target;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex4_1_0 *mask_texture;
    Object4_1_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    char _pad1[4];
    float min_dist, max_dist;
    short falloff_type;
    char _pad0[2];
};

struct DynamicPaintModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    DynamicPaintBrushSettings2_93_0 *brush;
    int type;
    char _pad[4];
};

struct RemeshModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct SkinModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct TriangulateModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    float lambda, lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag, repeat;
};

struct UVWarpModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char axis_u, axis_v;
    short flag;
    float center[2];
    float offset[2];
    float scale[2];
    float rotation;
    Object4_1_0 *object_src;
    char bone_src[64];
    Object4_1_0 *object_dst;
    char bone_dst[64];
    char vgroup_name[64];
    char uvlayer_name[68];
    char _pad[4];
};

struct MeshCacheModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct LaplacianDeformModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char anchor_grp_name[64];
    int verts_num, repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag, mat_ofs;
    char _pad[4];
};

struct WeldModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct DataTransferModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *ob_source;
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

struct NormalEditModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char defgrp_name[64];
    Object4_1_0 *target;
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

struct MeshSeqCacheModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    CacheFile4_1_0 *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
};

struct SurfaceDeformModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    void *depsgraph;
    Object4_1_0 *target;
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

struct WeightedNormalModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    char defgrp_name[64];
    char mode, flag;
    short weight;
    float thresh;
};

struct MeshToVolumeModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    float interior_band_width;
    float density;
    char _pad2[4];
    void *_pad3;
};

struct VolumeDisplaceModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Tex4_1_0 *texture;
    Object4_1_0 *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

struct VolumeToMeshModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    Object4_1_0 *object;
    float threshold;
    float adaptivity;
    uint32_t flag;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char grid_name[64];
    void *_pad1;
};

struct uiStyle4_1_0 {
    uiStyle4_1_0 *next, *prev;
    char name[64];
    uiFontStyle4_1_0 paneltitle;
    uiFontStyle4_1_0 grouplabel;
    uiFontStyle4_1_0 widgetlabel;
    uiFontStyle4_1_0 widget;
    float panelzoom;
    short minlabelchars;
    short minwidgetchars;
    short columnspace;
    short templatespace;
    short boxspace;
    short buttonspacex;
    short buttonspacey;
    short panelspace;
    short panelouter;
    char _pad0[2];
};

struct bPoseChannel_Runtime4_1_0 {
    SessionUID4_1_0 session_uid;
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

struct ID4_1_0 {
    void* next, * prev;
    ID4_1_0* newid;
    Library2_93_0* lib;
    AssetMetaData3_5_0* asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    unsigned int recalc;
    unsigned int recalc_up_to_undo_push;
    unsigned int recalc_after_undo_push;
    unsigned int session_uuid;
    IDProperty3_5_0* properties;
    IDOverrideLibrary3_2_0* override_library;
    ID4_1_0* orig_id;
    void* py_instance;
    LibraryWeakReference3_0_0* library_weak_reference;
    ID_Runtime3_2_0 runtime;
};

struct bArmature4_1_0 {
    ID4_1_0 id;
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
    ListBase2_93_0 collections_legacy;
    BoneCollection4_1_0 **collection_array;
    int collection_array_num;
    int collection_root_count;
    char active_collection_name[64];
    unsigned int layer_used;
    unsigned int layer , layer_protected;
    float axes_position;
    bArmature_Runtime4_0_0 runtime;
};

struct BoneCollection4_1_0 {
    BoneCollection4_1_0 *next, *prev;
    char name[64];
    ListBase2_93_0 bones;
    uint8_t flags;
    uint8_t _pad0[7];
    int child_index;
    int child_count;
    IDProperty3_5_0 *prop;
};

struct Brush4_1_0 {
    ID4_1_0 id;
    BrushClone2_93_0 clone;
    CurveMapping2_93_0 *curve;
    MTex4_0_0 mtex;
    MTex4_0_0 mask_mtex;
    Brush4_1_0 *toggle_brush;
    void *icon_imbuf;
    PreviewImage2_93_0 *preview;
    ColorBand2_93_0 *gradient;
    PaintCurve4_1_0 *paint_curve;
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
    int input_samples;
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
    char _pad0;
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
    float automasking_start_normal_limit;
    float automasking_start_normal_falloff;
    float automasking_view_normal_limit;
    float automasking_view_normal_falloff;
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
    CurveMapping2_93_0 *automasking_cavity_curve;
};

struct Collection_Runtime4_1_0 {
    ListBase2_93_0 object_cache;
    ListBase2_93_0 object_cache_instanced;
    ListBase2_93_0 parents;
    void *gobject_hash;
    uint8_t tag;
    char _pad0[7];
};

struct Collection4_1_0 {
    ID4_1_0 id;
    ID4_1_0 *owner_id;
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
    ViewLayer3_2_0 *view_layer;
    Collection_Runtime4_1_0 runtime;
};

struct Scopes4_1_0 {
    int ok;
    int sample_full;
    int sample_lines;
    int wavefrm_mode;
    int vecscope_mode;
    int wavefrm_height;
    int vecscope_height;
    int waveform_tot;
    float accuracy;
    float wavefrm_alpha;
    float wavefrm_yfac;
    float vecscope_alpha;
    float minmax[3][2];
    Histogram2_93_0 hist;
    float *waveform_1;
    float *waveform_2;
    float *waveform_3;
    float *vecscope;
    float *vecscope_rgb;
};

struct GreasePencil4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    GreasePencilDrawingBase4_0_0 **drawing_array;
    int drawing_array_num;
    char _pad[4];
    GreasePencilLayerTreeGroup4_0_0 *root_group_ptr;
    CustomData4_0_0 layers_data;
    int attributes_active_index;
    char _pad2[4];
    GreasePencilLayer4_0_0 *active_layer;
    Material4_1_0 **material_array;
    short material_array_num;
    char _pad3[2];
    uint32_t flag;
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad4[4];
    GreasePencilOnionSkinningSettings4_0_0 onion_skinning_settings;
    void *runtime;
};

struct IDPropertyUIDataInt4_1_0 {
    IDPropertyUIData3_0_0 base;
    int *default_array;
    int default_array_len;
    int min;
    int max;
    int soft_min;
    int soft_max;
    int step;
    int default_value;
    int enum_items_num;
    IDPropertyUIDataEnumItem4_1_0 *enum_items;
};

struct Image4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
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
    int offset_x, offset_y;
    int active_tile_index;
    ListBase2_93_0 tiles;
    ListBase2_93_0 views;
    Stereo3dFormat2_93_0 *stereo3d_format;
    Image_Runtime3_1_0 runtime;
};

struct LightProbe4_1_0 {
    ID4_1_0 id;
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
    float grid_clamp_direct;
    float grid_clamp_indirect;
    float surfel_density;
    Collection4_1_0 *visibility_grp;
    float data_display_size;
    char _pad1[4];
};

struct Material4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    short flag;
    char surface_render_method;
    char _pad1[1];
    float r, g, b, a;
    float specr, specg, specb;
    float alpha;
    float ray_mirror;
    float spec;
    float gloss_mir;
    float roughness;
    float metallic;
    char use_nodes;
    char pr_type;
    short pr_texture;
    short pr_flag;
    short index;
    bNodeTree4_1_0 *nodetree;
    Ipo4_1_0 *ipo;
    PreviewImage2_93_0 *preview;
    float line_col[4];
    short line_priority;
    short vcol_alpha;
    short paint_active_slot;
    short paint_clone_slot;
    short tot_slots;
    char displacement_method;
    char _pad2[1];
    float alpha_threshold;
    float refract_depth;
    char blend_method;
    char blend_shadow;
    char blend_flag;
    char volume_intersection_method;
    float inflate_bounds;
    char _pad3[4];
    TexPaintSlot3_2_0 *texpaintslot;
    ListBase2_93_0 gpumaterial;
    MaterialGPencilStyle2_93_0 *gp_style;
    MaterialLineArt3_3_0 lineart;
};

struct Mesh4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    Ipo4_1_0 *ipo;
    Key4_1_0 *key;
    Material4_1_0 **mat;
    int verts_num;
    int edges_num;
    int faces_num;
    int corners_num;
    int *face_offset_indices;
    CustomData4_0_0 vert_data;
    CustomData4_0_0 edge_data;
    CustomData4_0_0 face_data;
    CustomData4_0_0 corner_data;
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    void *edit_mesh;
    MSelect2_93_0 *mselect;
    int totselect;
    int act_face;
    Mesh4_1_0 *texcomesh;
    float texspace_location[3];
    float texspace_size[3];
    char texspace_flag;
    char editflag;
    uint16_t flag;
    float smoothresh_legacy;
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

struct NodesModifierData4_1_0 {
    ModifierData4_1_0 modifier;
    bNodeTree4_1_0 *node_group;
    NodesModifierSettings2_93_0 settings;
    char *bake_directory;
    int8_t flag;
    char _pad[3];
    int bakes_num;
    NodesModifierBake4_1_0 *bakes;
    char _pad2[4];
    int panels_num;
    NodesModifierPanel4_1_0 *panels;
    void *runtime;
};

struct MovieClip4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    char filepath[1024];
    int source;
    int _pad;
    int lastsize[2];
    float aspx, aspy;
    void *anim;
    void *cache;
    bGPdata4_1_0 *gpd;
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

struct bNodeTreeInterfaceSocket4_1_0 {
    bNodeTreeInterfaceItem4_0_0 item;
    char *name;
    char *description;
    char *socket_type;
    int flag;
    int16_t attribute_domain;
    int16_t default_input;
    char *default_attribute_name;
    char *identifier;
    void *socket_data;
    IDProperty3_5_0 *properties;
};

struct bNode4_1_0 {
    bNode4_1_0 *next, *prev;
    ListBase2_93_0 inputs, outputs;
    char name[64];
    int32_t identifier;
    int flag;
    char idname[64];
    void *typeinfo;
    int16_t type;
    int16_t ui_order;
    int16_t custom1, custom2;
    float custom3, custom4;
    ID4_1_0 *id;
    void *storage;
    IDProperty3_5_0 *prop;
    bNode4_1_0 *parent;
    float locx, locy;
    float width, height;
    float offsetx, offsety;
    char label[64];
    float color[3];
    int num_panel_states;
    bNodePanelState4_0_0 *panel_states_array;
    void *runtime;
};

struct bNodeTree4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    ID4_1_0 *owner_id;
    void *typeinfo;
    char idname[64];
    bGPdata4_1_0 *gpd;
    float view_center[2];
    ListBase2_93_0 nodes, links;
    int type;
    int cur_index;
    int flag;
    short edit_quality;
    short render_quality;
    int chunksize;
    int execution_mode;
    int precision;
    char _pad[4];
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

struct Object4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    void *sculpt;
    short type;
    short partype;
    int par1, par2, par3;
    char parsubstr[64];
    Object4_1_0 *parent, *track;
    Object4_1_0 *proxy;
    Object4_1_0 *proxy_group;
    Object4_1_0 *proxy_from;
    Ipo4_1_0 *ipo;
    bAction4_1_0 *action;
    bAction4_1_0 *poselib;
    bPose3_2_0 *pose;
    void *data;
    bGPdata4_1_0 *gpd;
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
    Material4_1_0 **mat;
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
    Collection4_1_0 *instance_collection;
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
    void *runtime;
};

struct Paint4_1_0 {
    Brush4_1_0 *brush;
    PaintToolSlot2_93_0 *tool_slots;
    int tool_slots_len;
    char _pad1[4];
    Palette4_1_0 *palette;
    CurveMapping2_93_0 *cavity_curve;
    void *paint_cursor;
    unsigned char paint_cursor_col[4];
    int flags;
    int num_input_samples_deprecated;
    int symmetry_flags;
    float tile_offset[3];
    char _pad2[4];
    Paint_Runtime2_93_0 runtime;
};

struct Sculpt4_1_0 {
    Paint4_1_0 paint;
    int flags;
    int transform_mode;
    int automasking_flags;
    int radial_symm[3];
    float detail_size;
    int symmetrize_direction;
    float gravity_factor;
    float constant_detail;
    float detail_percent;
    int automasking_boundary_edges_propagation_steps;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    float automasking_start_normal_limit, automasking_start_normal_falloff;
    float automasking_view_normal_limit, automasking_view_normal_falloff;
    CurveMapping2_93_0 *automasking_cavity_curve;
    CurveMapping2_93_0 *automasking_cavity_curve_op;
    Object4_1_0 *gravity_object;
};

struct ImagePaintSettings4_1_0 {
    Paint4_1_0 paint;
    short flag, missing_data;
    short seam_bleed, normal_angle;
    short screen_grab_size[2];
    int mode;
    Image4_1_0* stencil;
    Image4_1_0* clone;
    Image4_1_0* canvas;
    float stencil_col[3];
    float dither;
    int interp;
    char _pad[4];
};

struct ToolSettings4_1_0 {
    VPaint4_1_0 *vpaint;
    VPaint4_1_0 *wpaint;
    Sculpt4_1_0 *sculpt;
    UvSculpt4_1_0 *uvsculpt;
    GpPaint4_1_0 *gp_paint;
    GpVertexPaint4_1_0 *gp_vertexpaint;
    GpSculptPaint4_1_0 *gp_sculptpaint;
    GpWeightPaint4_1_0 *gp_weightpaint;
    CurvesSculpt4_1_0 *curves_sculpt;
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
    ImagePaintSettings4_1_0 imapaint;
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
    char uv_relax_method;
    char workspace_tool_type;
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
};

struct Panel4_1_0 {
    Panel4_1_0 *next, *prev;
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
    ListBase2_93_0 layout_panel_states;
    Panel_Runtime3_0_0 *runtime;
};

struct SpaceSeq4_1_0 {
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
    bGPdata4_1_0 *gpd;
    SequencerPreviewOverlay3_0_0 preview_overlay;
    SequencerTimelineOverlay3_0_0 timeline_overlay;
    char multiview_eye;
    char _pad2[7];
    void *runtime;
};

struct SpaceText4_1_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Text4_1_0 *text;
    int top;
    int left;
    char _pad1[4];
    short flags;
    short lheight;
    int tabnumber;
    char wordwrap;
    char doplugins;
    char showlinenrs;
    char showsyntax;
    char line_hlight;
    char overwrite;
    char live_edit;
    char _pad2[1];
    char findstr[256];
    char replacestr[256];
    short margin_column;
    char _pad3[2];
    SpaceText_Runtime2_93_0 *runtime;
};

struct SpaceConsole4_1_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    ListBase2_93_0 scrollback;
    ListBase2_93_0 history;
    char prompt[256];
    char language[32];
    int lheight;
    int history_index;
    int sel_start;
    int sel_end;
};

struct SpaceSpreadsheet4_1_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    ListBase2_93_0 columns;
    ListBase2_93_0 row_filters;
    ViewerPath3_4_0 viewer_path;
    uint8_t filter_flag;
    uint8_t geometry_component_type;
    uint8_t attribute_domain;
    uint8_t object_eval_state;
    int active_layer_index;
    uint32_t flag;
    char _pad1[4];
    void *runtime;
};

struct ThemeSpace4_1_0 {
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

struct UserDef4_1_0 {
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
    short keying_flag;
    short key_insert_channels;
    char _pad15[6];
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
    UserDef_Experimental4_1_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct View3D4_1_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    float viewquat[4];
    float dist;
    float bundle_size;
    char bundle_drawtype;
    char drawtype;
    char _pad3[1];
    char multiview_eye;
    int object_type_exclude_viewport;
    int object_type_exclude_select;
    short persp;
    short view;
    Object4_1_0 *camera, *ob_center;
    rctf2_93_0 render_border;
    View3D4_1_0 *localvd;
    char ob_center_bone[64];
    unsigned short local_view_uid;
    char _pad6[2];
    int layact;
    unsigned short local_collections_uid;
    short _pad7[2];
    short debug_flag;
    short ob_center_cursor;
    short scenelock;
    short gp_flag;
    short flag;
    int flag2;
    float lens, grid;
    float clip_start, clip_end;
    float ofs[3];
    char _pad[1];
    char gizmo_flag;
    char gizmo_show_object;
    char gizmo_show_armature;
    char gizmo_show_empty;
    char gizmo_show_light;
    char gizmo_show_camera;
    char gridflag;
    short gridlines;
    short gridsubdiv;
    float vertex_opacity;
    bGPdata4_1_0 *gpd;
    short stereo3d_flag;
    char stereo3d_camera;
    char _pad4;
    float stereo3d_convergence_factor;
    float stereo3d_volume_alpha;
    float stereo3d_convergence_alpha;
    View3DShading3_5_0 shading;
    View3DOverlay4_1_0 overlay;
    ViewerPath3_4_0 viewer_path;
    View3D_Runtime3_0_0 runtime;
};

struct Volume4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
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
    Material4_1_0 **mat;
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

struct ReportList4_1_0 {
    ListBase2_93_0 list;
    int printlevel;
    int storelevel;
    int flag;
    char _pad[4];
    void *reporttimer;
    void *lock;
};

struct wmWindowManager4_1_0 {
    ID4_1_0 id;
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
    ReportList4_1_0 reports;
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

struct wmWindow4_1_0 {
    wmWindow4_1_0 *next, *prev;
    void *ghostwin;
    void *gpuctx;
    wmWindow4_1_0 *parent;
    Scene4_1_0 *scene;
    Scene4_1_0 *new_scene;
    char view_layer_name[64];
    Scene4_1_0 *unpinned_scene;
    WorkSpaceInstanceHook2_93_0 *workspace_hook;
    ScrAreaMap2_93_0 global_areas;
    bScreen4_1_0 *screen;
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
    const void *ime_data;
    char ime_data_is_composing;
    char _pad1[7];
    ListBase2_93_0 event_queue;
    ListBase2_93_0 handlers;
    ListBase2_93_0 modalhandlers;
    ListBase2_93_0 gesture;
    Stereo3dFormat2_93_0 *stereo3d_format;
    ListBase2_93_0 drawcalls;
    void *cursor_keymap_status;
    uint64_t eventstate_prev_press_time_ms;
};

struct bPoseChannel4_1_0 {
    bPoseChannel4_1_0 *next, *prev;
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
    bPoseChannel4_1_0 *parent;
    bPoseChannel4_1_0 *child;
    ListBase2_93_0 iktree;
    ListBase2_93_0 siktree;
    bMotionPath2_93_0 *mpath;
    Object4_1_0 *custom;
    bPoseChannel4_1_0 *custom_tx;
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
    bPoseChannel4_1_0 *bbone_prev;
    bPoseChannel4_1_0 *bbone_next;
    void *temp;
    bPoseChannelDrawData2_93_0 *draw_data;
    bPoseChannel4_1_0 *orig_pchan;
    BoneColor4_0_0 color;
    bPoseChannel_Runtime4_1_0 runtime;
};

struct bAction4_1_0 {
    ID4_1_0 id;
    ListBase2_93_0 curves;
    ListBase2_93_0 chanbase;
    ListBase2_93_0 groups;
    ListBase2_93_0 markers;
    int flag;
    int active_marker;
    int idroot;
    char _pad[4];
    float frame_start, frame_end;
    PreviewImage2_93_0 *preview;
};

struct IdAdtTemplate4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
};

struct Palette4_1_0 {
    ID4_1_0 id;
    ListBase2_93_0 colors;
    int active_color;
    char _pad[4];
};

struct PaintCurve4_1_0 {
    ID4_1_0 id;
    PaintCurvePoint2_93_0 *points;
    int tot_points;
    int add_index;
};

struct CacheFile4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
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

struct Camera4_1_0 {
    ID4_1_0 id;
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
    Ipo4_1_0 *ipo;
    Object4_1_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings3_3_0 dof;
    ListBase2_93_0 bg_images;
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct Curves4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    CurvesGeometry4_0_0 geometry;
    int flag;
    int attributes_active_index;
    Material4_1_0 **mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    Object4_1_0 *surface;
    char *surface_uv_map;
    void *batch_cache;
};

struct Curve4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 nurb;
    EditNurb2_93_0 *editnurb;
    Object4_1_0 *bevobj, *taperobj, *textoncurve;
    Ipo4_1_0 *ipo;
    Key4_1_0 *key;
    Material4_1_0 **mat;
    CurveProfile2_93_0 *bevel_profile;
    float texspace_location[3];
    float texspace_size[3];
    short type;
    char texspace_flag;
    char _pad0[7];
    short twist_mode;
    float twist_smooth, smallcaps_scale;
    int pathlen;
    short bevresol, totcol;
    int flag;
    float offset, extrude, bevel_radius;
    short resolu, resolv;
    short resolu_ren, resolv_ren;
    int actnu;
    int actvert;
    char overflow;
    char spacemode, align_y;
    char bevel_mode;
    char taper_radius_mode;
    char _pad;
    short lines;
    float spacing, linedist, shear, fsize, wordspace, ulpos, ulheight;
    float xof, yof;
    float linewidth;
    int pos;
    int selstart, selend;
    int len_char32;
    int len;
    char *str;
    void *editfont;
    char family[64];
    VFont4_1_0 *vfont;
    VFont4_1_0 *vfontb;
    VFont4_1_0 *vfonti;
    VFont4_1_0 *vfontbi;
    TextBox2_93_0 *tb;
    int totbox, actbox;
    CharInfo3_6_0 *strinfo;
    CharInfo3_6_0 curinfo;
    float ctime;
    float bevfac1, bevfac2;
    char bevfac1_mapping, bevfac2_mapping;
    char _pad2[6];
    float fsize_realtime;
    const Curves4_1_0 *curve_eval;
    char edit_data_from_original;
    char _pad3[7];
    void *batch_cache;
};

struct bGPdata4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 layers;
    int flag;
    int curve_edit_resolution;
    float curve_edit_threshold;
    float curve_edit_corner_angle;
    ListBase2_93_0 palettes;
    ListBase2_93_0 vertex_group_names;
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
    Material4_1_0 **mat;
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
    bGPgrid2_93_0 grid;
    bGPdata_Runtime3_4_0 runtime;
};

struct Ipo4_1_0 {
    ID4_1_0 id;
    ListBase2_93_0 curve;
    rctf2_93_0 cur;
    short blocktype;
    short showkey;
    short muteipo;
    char _pad[2];
};

struct Key4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    KeyBlock2_93_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    ListBase2_93_0 block;
    Ipo4_1_0 *ipo;
    ID4_1_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct Lattice4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    short pntsu, pntsv, pntsw, flag;
    short opntsu, opntsv, opntsw;
    char _pad2[3];
    char typeu, typev, typew;
    int actbp;
    float fu, fv, fw, du, dv, dw;
    BPoint2_93_0 *def;
    Ipo4_1_0 *ipo;
    Key4_1_0 *key;
    MDeformVert2_93_0 *dvert;
    char vgroup[64];
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad0[4];
    EditLatt2_93_0 *editlatt;
    void *batch_cache;
};

struct Light4_1_0 {
    ID4_1_0 id;
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
    bNodeTree4_1_0 *nodetree;
    Ipo4_1_0 *ipo;
    float energy_deprecated;
    float _pad2;
};

struct FreestyleLineStyle4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    float r, g, b, alpha;
    float thickness;
    int thickness_position;
    float thickness_ratio;
    int flag, caps;
    int chaining;
    unsigned int rounds;
    float split_length;
    float min_angle, max_angle;
    float min_length, max_length;
    unsigned int chain_count;
    unsigned short split_dash1, split_gap1;
    unsigned short split_dash2, split_gap2;
    unsigned short split_dash3, split_gap3;
    int sort_key, integration_type;
    float texstep;
    short texact, pr_texture;
    short use_nodes;
    char _pad[6];
    unsigned short dash1, gap1, dash2, gap2, dash3, gap3;
    int panel;
    MTex4_0_0 *mtex[18];
    bNodeTree4_1_0 *nodetree;
    ListBase2_93_0 color_modifiers;
    ListBase2_93_0 alpha_modifiers;
    ListBase2_93_0 thickness_modifiers;
    ListBase2_93_0 geometry_modifiers;
};

struct Mask4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    ListBase2_93_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra, efra;
    int flag;
    char _pad[4];
};

struct MetaBall4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 *editelems;
    Ipo4_1_0 *ipo;
    Material4_1_0 **mat;
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

struct ClothModifierData4_1_0 {
    ModifierData4_1_0 modifier;
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

struct ParticleSettings4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    BoidSettings2_93_0 *boids;
    SPHFluidSettings2_93_0 *fluid;
    EffectorWeights3_0_0 *effector_weights;
    Collection4_1_0 *collision_group;
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
    MTex4_0_0 *mtex[18];
    Collection4_1_0 *instance_collection;
    ListBase2_93_0 instance_weights;
    Collection4_1_0 *force_group;
    Object4_1_0 *instance_object;
    Object4_1_0 *bb_ob;
    Ipo4_1_0 *ipo;
    PartDeflect2_93_0 *pd;
    PartDeflect2_93_0 *pd2;
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

struct PointCloud4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    int flag;
    int totpoint;
    CustomData4_0_0 pdata;
    int attributes_active_index;
    int _pad4;
    Material4_1_0 **mat;
    short totcol;
    short _pad3[3];
    void *runtime;
    void *batch_cache;
};

struct CurvesSculpt4_1_0 {
    Paint4_1_0 paint;
};

struct UvSculpt4_1_0 {
    Paint4_1_0 paint;
};

struct GpPaint4_1_0 {
    Paint4_1_0 paint;
    int flag;
    int mode;
};

struct GpVertexPaint4_1_0 {
    Paint4_1_0 paint;
    int flag;
    char _pad[4];
};

struct GpSculptPaint4_1_0 {
    Paint4_1_0 paint;
    int flag;
    char _pad[4];
};

struct GpWeightPaint4_1_0 {
    Paint4_1_0 paint;
    int flag;
    char _pad[4];
};

struct VPaint4_1_0 {
    Paint4_1_0 paint;
    char flag;
    char _pad[3];
    int radial_symm[3];
};

struct Scene4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object4_1_0 *camera;
    World4_1_0 *world;
    Scene4_1_0 *set;
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
    bNodeTree4_1_0 *nodetree;
    Editing3_2_0 *ed;
    ToolSettings4_1_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData3_6_0 r;
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
    bGPdata4_1_0 *gpd;
    MovieClip4_1_0 *clip;
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
    Collection4_1_0 *master_collection;
    IDProperty3_5_0 *layer_properties;
    int simulation_frame_start;
    int simulation_frame_end;
    SceneDisplay2_93_0 display;
    SceneEEVEE4_1_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
    SceneHydra4_0_0 hydra;
};

struct bScreen4_1_0 {
    ID4_1_0 id;
    ListBase2_93_0 vertbase;
    ListBase2_93_0 edgebase;
    ListBase2_93_0 areabase;
    ListBase2_93_0 regionbase;
    Scene4_1_0 *scene;
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
    ARegion3_6_0 *active_region;
    void *animtimer;
    void  *context;
    void *tool_tip;
    PreviewImage2_93_0 *preview;
};

struct Sequence4_1_0 {
    Sequence4_1_0 *next, *prev;
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
    Ipo4_1_0 *ipo;
    Scene4_1_0 *scene;
    Object4_1_0 *scene_camera;
    MovieClip4_1_0 *clip;
    Mask4_1_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence4_1_0 *seq1, *seq2, *seq3;
    ListBase2_93_0 seqbase;
    ListBase2_93_0 channels;
    bSound4_1_0 *sound;
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
    SeqRetimingKey4_1_0 *retiming_keys;
    void *_pad5;
    int retiming_keys_num;
    char _pad6[4];
    SequenceRuntime4_1_0 runtime;
};

struct bSound4_1_0 {
    ID4_1_0 id;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    void *handle;
    PackedFile2_93_0 *newpackedfile;
    Ipo4_1_0 *ipo;
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

struct SpaceImage4_1_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Image4_1_0 *image;
    ImageUser3_0_0 iuser;
    Scopes4_1_0 scopes;
    Histogram2_93_0 sample_line_hist;
    bGPdata4_1_0 *gpd;
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
    char _pad1[2];
    int flag;
    float uv_opacity;
    int tile_grid_shape[2];
    int custom_grid_subdiv[2];
    MaskSpaceInfo3_3_0 mask_info;
    SpaceImageOverlay2_93_0 overlay;
};

struct Script4_1_0 {
    ID4_1_0 id;
    void *py_draw;
    void *py_event;
    void *py_button;
    void *py_browsercallback;
    void *py_globaldict;
    int flags, lastspace;
    char scriptname[1024];
    char scriptarg[256];
};

struct Speaker4_1_0 {
    ID4_1_0 id;
    AnimData2_93_0 *adt;
    bSound4_1_0 *sound;
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

struct Tex4_1_0 {
    ID4_1_0 id;
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
    bNodeTree4_1_0 *nodetree;
    Ipo4_1_0 *ipo;
    Image4_1_0 *ima;
    ColorBand2_93_0 *coba;
    PreviewImage2_93_0 *preview;
    char use_nodes;
    char _pad[7];
};

struct Text4_1_0 {
    ID4_1_0 id;
    char *filepath;
    void *compiled;
    int flags;
    char _pad0[4];
    ListBase2_93_0 lines;
    TextLine2_93_0 *curl, *sell;
    int curc, selc;
    double mtime;
};

struct bTheme4_1_0 {
    bTheme4_1_0 *next, *prev;
    char name[32];
    ThemeUI4_0_0 tui;
    ThemeSpace4_1_0 space_properties;
    ThemeSpace4_1_0 space_view3d;
    ThemeSpace4_1_0 space_file;
    ThemeSpace4_1_0 space_graph;
    ThemeSpace4_1_0 space_info;
    ThemeSpace4_1_0 space_action;
    ThemeSpace4_1_0 space_nla;
    ThemeSpace4_1_0 space_sequencer;
    ThemeSpace4_1_0 space_image;
    ThemeSpace4_1_0 space_text;
    ThemeSpace4_1_0 space_outliner;
    ThemeSpace4_1_0 space_node;
    ThemeSpace4_1_0 space_preferences;
    ThemeSpace4_1_0 space_console;
    ThemeSpace4_1_0 space_clip;
    ThemeSpace4_1_0 space_topbar;
    ThemeSpace4_1_0 space_statusbar;
    ThemeSpace4_1_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

struct VFont4_1_0 {
    ID4_1_0 id;
    char filepath[1024];
    void *data;
    PackedFile2_93_0 *packedfile;
    PackedFile2_93_0 *temp_pf;
};

struct WorkSpace4_1_0 {
    ID4_1_0 id;
    ListBase2_93_0 layouts;
    ListBase2_93_0 hook_layout_relations;
    ListBase2_93_0 owner_ids;
    ListBase2_93_0 tools;
    Scene4_1_0 *pin_scene;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    char *status_text;
    AssetLibraryReference3_0_0 asset_library_ref;
    ViewerPath3_4_0 viewer_path;
};

struct World4_1_0 {
    ID4_1_0 id;
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
    Ipo4_1_0 *ipo;
    short pr_texture, use_nodes;
    char _pad[4];
    PreviewImage2_93_0 *preview;
    bNodeTree4_1_0 *nodetree;
    LightgroupMembership3_2_0 *lightgroup;
    ListBase2_93_0 gpumaterial;
};

#endif
