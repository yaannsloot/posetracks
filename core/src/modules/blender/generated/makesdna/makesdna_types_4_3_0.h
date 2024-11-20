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

Various structs from Blender v4.3 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_4_3_0_H
#define MAKESDNA_TYPES_4_3_0_H

#include <stdint.h>
#include "makesdna_types_3_6_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_4_1_0.h"
#include "makesdna_types_4_2_0.h"
#include "makesdna_types_3_3_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_1_0.h"
#include "makesdna_types_3_5_0.h"
#include "makesdna_types_4_0_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_0_0.h"

struct bActionGroup4_3_0;
struct bAction4_3_0;
struct SpaceAction4_3_0;
struct ActionSlot4_3_0;
struct ActionStrip4_3_0;
struct ActionStripKeyframeData4_3_0;
struct ActionChannelBag4_3_0;
struct NlaStrip4_3_0;
struct AnimData4_3_0;
struct BrushGpencilSettings4_3_0;
struct Brush4_3_0;
struct Camera4_3_0;
struct CollectionExport4_3_0;
struct ColorManagedViewSettings4_3_0;
struct bActionConstraint4_3_0;
struct CurvesGeometry4_3_0;
struct Curves4_3_0;
struct CustomDataLayer4_3_0;
struct GreasePencilLayerTreeGroup4_3_0;
struct Light4_3_0;
struct BevelModifierData4_3_0;
struct NodesModifierPackedBake4_3_0;
struct NodesModifierBake4_3_0;
struct NodesModifierData4_3_0;
struct GreasePencilLineartModifierData4_3_0;
struct bNode4_3_0;
struct bNodeTree4_3_0;
struct NodeReroute4_3_0;
struct NodeImageMultiFile4_3_0;
struct NodeColorBalance4_3_0;
struct NodeTexGabor4_3_0;
struct NodeGeometryMergeLayers4_3_0;
struct NodeGeometryForeachGeometryElementInput4_3_0;
struct NodeForeachGeometryElementInputItem4_3_0;
struct NodeForeachGeometryElementMainItem4_3_0;
struct NodeForeachGeometryElementGenerationItem4_3_0;
struct NodeForeachGeometryElementInputItems4_3_0;
struct NodeForeachGeometryElementMainItems4_3_0;
struct NodeForeachGeometryElementGenerationItems4_3_0;
struct NodeGeometryForeachGeometryElementOutput4_3_0;
struct NodeGeometryLinearGizmo4_3_0;
struct NodeGeometryDialGizmo4_3_0;
struct NodeGeometryTransformGizmo4_3_0;
struct PackedFile4_3_0;
struct RenderData4_3_0;
struct Paint_Runtime4_3_0;
struct NamedBrushAssetReference4_3_0;
struct ToolSystemBrushBindings4_3_0;
struct Paint4_3_0;
struct ToolSettings4_3_0;
struct SceneEEVEE4_3_0;
struct AssetShelf4_3_0;
struct SDNA_StructMember4_3_0;
struct SDNA_Struct4_3_0;
struct Sequence4_3_0;
struct SeqConnection4_3_0;
struct EditingRuntime4_3_0;
struct SpreadsheetInstanceID4_3_0;
struct SpaceSpreadsheet4_3_0;
struct uiStyle4_3_0;
struct ThemeUI4_3_0;
struct ThemeSpace4_3_0;
struct UserDef_Experimental4_3_0;
struct UserDef4_3_0;
struct View3DOverlay4_3_0;
struct ForeachGeometryElementZoneViewerPathElem4_3_0;
struct bToolRef_Runtime4_3_0;
struct GreasePencilDrawing4_3_0;
struct ImageFormatData4_3_0;
struct BakeData4_3_0;
struct ImagePaintSettings4_3_0;
struct Sculpt4_3_0;
struct CurvesSculpt4_3_0;
struct GpPaint4_3_0;
struct GpVertexPaint4_3_0;
struct GpSculptPaint4_3_0;
struct GpWeightPaint4_3_0;
struct VPaint4_3_0;
struct Scene4_3_0;
struct Editing4_3_0;
struct bTheme4_3_0;
struct View3D4_3_0;
struct NodeImageFile4_3_0;
struct NodeImageMultiFileSocket4_3_0;

struct ActionSlot4_3_0 {
    char name[66];
    uint8_t _pad0[2];
    int idtype;
    int32_t handle;
    int8_t slot_flags;
    uint8_t _pad1[3];
    void *runtime;
};

struct ActionStrip4_3_0 {
    int8_t strip_type;
    uint8_t _pad0[3];
    int data_index;
    float frame_start;
    float frame_end;
    float frame_offset;
    uint8_t _pad1[4];
};

struct ActionStripKeyframeData4_3_0 {
    ActionChannelBag4_3_0 **channelbag_array;
    int channelbag_array_num;
    uint8_t _pad[4];
};

struct ActionChannelBag4_3_0 {
    int32_t slot_handle;
    int group_array_num;
    bActionGroup4_3_0 **group_array;
    uint8_t _pad[4];
    int fcurve_array_num;
    FCurve2_93_0 **fcurve_array;
};

struct BrushGpencilSettings4_3_0 {
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
    char _pad[1];
    int flag2;
    int fill_simplylvl;
    int fill_draw_mode;
    int fill_extend_mode;
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
    CurveMapping2_93_0 *curve_sensitivity;
    CurveMapping2_93_0 *curve_strength;
    CurveMapping2_93_0 *curve_jitter;
    CurveMapping2_93_0 *curve_rand_pressure;
    CurveMapping2_93_0 *curve_rand_strength;
    CurveMapping2_93_0 *curve_rand_uv;
    CurveMapping2_93_0 *curve_rand_hue;
    CurveMapping2_93_0 *curve_rand_saturation;
    CurveMapping2_93_0 *curve_rand_value;
    float outline_fac;
    float simplify_px;
    Material4_2_0 *material;
    Material4_2_0 *material_alt;
};

struct CollectionExport4_3_0 {
    CollectionExport4_3_0 *next, *prev;
    char fh_idname[64];
    char name[64];
    IDProperty3_5_0 *export_properties;
    uint32_t flag;
    uint32_t _pad0;
};

struct ColorManagedViewSettings4_3_0 {
    int flag;
    char _pad[4];
    char look[64];
    char view_transform[64];
    float exposure;
    float gamma;
    float temperature;
    float tint;
    CurveMapping2_93_0 *curve_mapping;
    void *_pad2;
};

struct bActionConstraint4_3_0 {
    Object4_2_0 *tar;
    short type;
    short local;
    int start;
    int end;
    float min;
    float max;
    int flag;
    char mix_mode;
    char _pad[3];
    float eval_time;
    bAction4_3_0 *act;
    int32_t action_slot_handle;
    char action_slot_name[66];
    char _pad1[2];
    char subtarget[64];
};

struct CustomDataLayer4_3_0 {
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
    const void *sharing_info;
};

struct NodesModifierPackedBake4_3_0 {
    int meta_files_num;
    int blob_files_num;
    void *meta_files;
    void *blob_files;
};

struct NodesModifierBake4_3_0 {
    int id;
    uint32_t flag;
    uint8_t bake_mode;
    int8_t bake_target;
    char _pad[6];
    char *directory;
    int frame_start;
    int frame_end;
    int data_blocks_num;
    int active_data_block;
    NodesModifierDataBlock4_1_0 *data_blocks;
    NodesModifierPackedBake4_3_0 *packed;
    void *_pad2;
    int64_t bake_size;
};

struct NodeReroute4_3_0 {
    char type_idname[64];
};

struct NodeColorBalance4_3_0 {
    float slope[3];
    float offset[3];
    float power[3];
    float offset_basis;
    char _pad[4];
    float lift[3];
    float gamma[3];
    float gain[3];
    float input_temperature;
    float input_tint;
    float output_temperature;
    float output_tint;
};

struct NodeGeometryMergeLayers4_3_0 {
    int8_t mode;
};

struct NodeGeometryForeachGeometryElementInput4_3_0 {
    int32_t output_node_id;
};

struct NodeForeachGeometryElementInputItem4_3_0 {
    char *name;
    short socket_type;
    char _pad[2];
    int identifier;
};

struct NodeForeachGeometryElementMainItem4_3_0 {
    char *name;
    short socket_type;
    char _pad[2];
    int identifier;
};

struct NodeForeachGeometryElementGenerationItem4_3_0 {
    char *name;
    short socket_type;
    uint8_t domain;
    char _pad[1];
    int identifier;
};

struct NodeForeachGeometryElementInputItems4_3_0 {
    NodeForeachGeometryElementInputItem4_3_0 *items;
    int items_num;
    int active_index;
    int next_identifier;
    char _pad[4];
};

struct NodeForeachGeometryElementMainItems4_3_0 {
    NodeForeachGeometryElementMainItem4_3_0 *items;
    int items_num;
    int active_index;
    int next_identifier;
    char _pad[4];
};

struct NodeForeachGeometryElementGenerationItems4_3_0 {
    NodeForeachGeometryElementGenerationItem4_3_0 *items;
    int items_num;
    int active_index;
    int next_identifier;
    char _pad[4];
};

struct NodeGeometryLinearGizmo4_3_0 {
    int color_id;
    int draw_style;
};

struct NodeGeometryDialGizmo4_3_0 {
    int color_id;
};

struct NodeGeometryTransformGizmo4_3_0 {
    uint32_t flag;
};

struct PackedFile4_3_0 {
    int size;
    int seek;
    const void *data;
    const void *sharing_info;
};

struct Paint_Runtime4_3_0 {
    unsigned int initialized;
    unsigned short ob_mode;
    char _pad[2];
};

struct NamedBrushAssetReference4_3_0 {
    NamedBrushAssetReference4_3_0 *next, *prev;
    const char *name;
    AssetWeakReference3_6_0 *brush_asset_reference;
};

struct SDNA_StructMember4_3_0 {
    short type_index;
    short member_index;
};

struct SeqConnection4_3_0 {
    SeqConnection4_3_0 *next, *prev;
    Sequence4_3_0 *seq_ref;
};

struct EditingRuntime4_3_0 {
    void *sequence_lookup;
    void *media_presence;
    void *thumbnail_cache;
    void *_pad;
};

struct SpreadsheetInstanceID4_3_0 {
    int reference_index;
};

struct UserDef_Experimental4_3_0 {
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
    char enable_overlay_next;
    char use_new_volume_nodes;
    char use_new_file_import_nodes;
    char use_shader_node_previews;
    char use_animation_baklava;
    char enable_new_cpu_compositor;
    char _pad[2];
};

struct View3DOverlay4_3_0 {
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
    float gpencil_grid_color[3];
    float gpencil_grid_scale[2];
    float gpencil_grid_offset[2];
    int gpencil_grid_subdivisions;
    float gpencil_vertex_paint_opacity;
    int handle_display;
    float sculpt_curves_cage_opacity;
};

struct bToolRef_Runtime4_3_0 {
    int cursor;
    char keymap[64];
    char gizmo_group[64];
    char data_block[64];
    int brush_type;
    char keymap_fallback[64];
    char op[64];
    int index;
    int flag;
};

struct NodeGeometryForeachGeometryElementOutput4_3_0 {
    NodeForeachGeometryElementInputItems4_3_0 input_items;
    NodeForeachGeometryElementMainItems4_3_0 main_items;
    NodeForeachGeometryElementGenerationItems4_3_0 generation_items;
    int inspection_index;
    uint8_t domain;
    char _pad[3];
};

struct SDNA_Struct4_3_0 {
    short type_index;
    short members_num;
    SDNA_StructMember4_3_0 members[];
};

struct bActionGroup4_3_0 {
    bActionGroup4_3_0 *next, *prev;
    ListBase2_93_0 channels;
    int fcurve_range_start;
    int fcurve_range_length;
    ActionChannelBag4_3_0 *channel_bag;
    int flag;
    int customCol;
    char name[64];
    ThemeWireColor2_93_0 cs;
};

struct bAction4_3_0 {
    ID4_1_0 id;
    ActionLayer4_2_0 **layer_array;
    int layer_array_num;
    int layer_active_index;
    ActionSlot4_3_0 **slot_array;
    int slot_array_num;
    int32_t last_slot_handle;
    ActionStripKeyframeData4_3_0 **strip_keyframe_data_array;
    int strip_keyframe_data_array_num;
    char _pad0[4];
    ListBase2_93_0 curves;
    ListBase2_93_0 chanbase;
    ListBase2_93_0 groups;
    ListBase2_93_0 markers;
    int flag;
    int active_marker;
    int idroot;
    char _pad1[4];
    float frame_start, frame_end;
    PreviewImage4_2_0 *preview;
};

struct SpaceAction4_3_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D4_0_0 v2d;
    bAction4_3_0 *action;
    int32_t action_slot_handle;
    char _pad2[4];
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

struct NlaStrip4_3_0 {
    NlaStrip4_3_0 *next, *prev;
    ListBase2_93_0 strips;
    bAction4_3_0 *act;
    int32_t action_slot_handle;
    char action_slot_name[66];
    char _pad0[2];
    ListBase2_93_0 fcurves;
    ListBase2_93_0 modifiers;
    char name[64];
    float influence;
    float strip_time;
    float start, end;
    float actstart, actend;
    float repeat;
    float scale;
    float blendin, blendout;
    short blendmode;
    short extendmode;
    char _pad1[2];
    short type;
    void *speaker_handle;
    int flag;
    char _pad2[4];
    NlaStrip4_3_0 *orig_strip;
    void *_pad3;
};

struct AnimData4_3_0 {
    bAction4_3_0 *action;
    int32_t slot_handle;
    char slot_name[66];
    uint8_t _pad0[2];
    bAction4_3_0 *tmpact;
    int32_t tmp_slot_handle;
    char tmp_slot_name[66];
    uint8_t _pad1[2];
    ListBase2_93_0 nla_tracks;
    NlaTrack2_93_0 *act_track;
    NlaStrip4_3_0 *actstrip;
    ListBase2_93_0 drivers;
    ListBase2_93_0 overrides;
    FCurve2_93_0 **driver_array;
    int flag;
    short act_blendmode;
    short act_extendmode;
    float act_influence;
    uint8_t _pad2[4];
};

struct Brush4_3_0 {
    ID4_1_0 id;
    BrushClone2_93_0 clone;
    CurveMapping2_93_0 *curve;
    MTex4_0_0 mtex;
    MTex4_0_0 mask_mtex;
    Brush4_3_0 *toggle_brush;
    void *icon_imbuf;
    PreviewImage4_2_0 *preview;
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
    char has_unsaved_changes;
    char falloff_shape;
    float falloff_angle;
    char sculpt_brush_type;
    char vertex_brush_type;
    char weight_brush_type;
    char image_brush_type;
    char mask_tool;
    char gpencil_brush_type;
    char gpencil_vertex_brush_type;
    char gpencil_sculpt_brush_type;
    char gpencil_weight_brush_type;
    char curves_sculpt_brush_type;
    char _pad1[6];
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
    BrushGpencilSettings4_3_0 *gpencil_settings;
    BrushCurvesSculptSettings4_2_0 *curves_sculpt_settings;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    CurveMapping2_93_0 *automasking_cavity_curve;
};

struct Camera4_3_0 {
    ID4_1_0 id;
    AnimData4_3_0 *adt;
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
    float central_cylindrical_range_u_min;
    float central_cylindrical_range_u_max;
    float central_cylindrical_range_v_min;
    float central_cylindrical_range_v_max;
    float central_cylindrical_radius;
    float _pad2;
    Ipo4_0_0 *ipo;
    Object4_2_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings3_3_0 dof;
    ListBase2_93_0 bg_images;
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct CurvesGeometry4_3_0 {
    int *curve_offsets;
    CustomData4_0_0 point_data;
    CustomData4_0_0 curve_data;
    int point_num;
    int curve_num;
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    void *runtime;
};

struct Curves4_3_0 {
    ID4_1_0 id;
    AnimData4_3_0 *adt;
    CurvesGeometry4_3_0 geometry;
    int flag;
    int attributes_active_index_legacy;
    Material4_2_0 **mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    Object4_2_0 *surface;
    char *surface_uv_map;
    void *batch_cache;
};

struct GreasePencilLayerTreeGroup4_3_0 {
    GreasePencilLayerTreeNode4_0_0 base;
    ListBase2_93_0 children;
    int8_t color_tag;
    char _pad[7];
    void *runtime;
};

struct Light4_3_0 {
    ID4_1_0 id;
    AnimData4_3_0 *adt;
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
    short pr_texture, use_nodes;
    float clipsta;
    float clipend_deprecated;
    float cascade_max_dist;
    float cascade_exponent;
    float cascade_fade;
    int cascade_count;
    float diff_fac;
    float spec_fac;
    float transmission_fac;
    float volume_fac;
    float att_dist;
    float shadow_filter_radius;
    float shadow_maximum_resolution;
    float shadow_jitter_overblur;
    PreviewImage4_2_0 *preview;
    bNodeTree4_3_0 *nodetree;
    Ipo4_0_0 *ipo;
    float energy_deprecated;
    float _pad2;
};

struct BevelModifierData4_3_0 {
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
    char edge_weight_name[64];
    char vertex_weight_name[64];
};

struct NodesModifierData4_3_0 {
    ModifierData4_1_0 modifier;
    bNodeTree4_3_0 *node_group;
    NodesModifierSettings2_93_0 settings;
    char *bake_directory;
    int8_t flag;
    int8_t bake_target;
    char _pad[2];
    int bakes_num;
    NodesModifierBake4_3_0 *bakes;
    char _pad2[4];
    int panels_num;
    NodesModifierPanel4_1_0 *panels;
    void *runtime;
};

struct GreasePencilLineartModifierData4_3_0 {
    ModifierData4_1_0 modifier;
    uint16_t edge_types;
    char source_type;
    char use_multiple_levels;
    short level_start;
    short level_end;
    Object4_2_0 *source_camera;
    Object4_2_0 *light_contour_object;
    Object4_2_0 *source_object;
    Collection4_2_0 *source_collection;
    Material4_2_0 *target_material;
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
    void *runtime;
};

struct bNode4_3_0 {
    bNode4_3_0 *next, *prev;
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
    int8_t warning_propagation;
    char _pad[7];
    ID4_1_0 *id;
    void *storage;
    IDProperty3_5_0 *prop;
    bNode4_3_0 *parent;
    float locx, locy;
    float width, height;
    float offsetx, offsety;
    char label[64];
    float color[3];
    int num_panel_states;
    bNodePanelState4_0_0 *panel_states_array;
    void *runtime;
};

struct bNodeTree4_3_0 {
    ID4_1_0 id;
    AnimData4_3_0 *adt;
    ID4_1_0 *owner_id;
    void *typeinfo;
    char idname[64];
    char *description;
    bGPdata3_0_0 *gpd;
    float view_center[2];
    ListBase2_93_0 nodes, links;
    int type;
    int cur_index;
    int flag;
    int chunksize;
    int execution_mode;
    int precision;
    int color_tag;
    int default_group_node_width;
    rctf2_93_0 viewer_border;
    ListBase2_93_0 inputs_legacy , outputs_legacy;
    bNodeTreeInterface4_0_0 tree_interface;
    void *previews;
    bNodeInstanceKey2_93_0 active_viewer_key;
    int nested_node_refs_num;
    bNestedNodeRef4_0_0 *nested_node_refs;
    GeometryNodeAssetTraits4_0_0 *geometry_node_asset_traits;
    PreviewImage4_2_0 *preview;
    void *runtime;
};

struct ImageFormatData4_3_0 {
    char imtype;
    char depth;
    char planes;
    char flag;
    char quality;
    char compress;
    char exr_codec;
    char cineon_flag;
    short cineon_white, cineon_black;
    float cineon_gamma;
    char jp2_flag;
    char jp2_codec;
    char tiff_codec;
    char _pad[4];
    char views_format;
    Stereo3dFormat2_93_0 stereo3d_format;
    char color_management;
    char _pad1[7];
    ColorManagedViewSettings4_3_0 view_settings;
    ColorManagedDisplaySettings2_93_0 display_settings;
    ColorManagedColorspaceSettings2_93_0 linear_colorspace_settings;
};

struct NodeImageMultiFile4_3_0 {
    char base_path[1024];
    ImageFormatData4_3_0 format;
    int sfra , efra;
    int active_input;
    char save_as_render;
    char _pad[3];
};

struct NodeTexGabor4_3_0 {
    NodeTexBase2_93_0 base;
    char type;
    char _pad[7];
};

struct ToolSystemBrushBindings4_3_0 {
    AssetWeakReference3_6_0 *main_brush_asset_reference;
    ListBase2_93_0 active_brush_per_brush_type;
};

struct Paint4_3_0 {
    Brush4_3_0 *brush;
    AssetWeakReference3_6_0 *brush_asset_reference;
    Brush4_3_0 *eraser_brush;
    AssetWeakReference3_6_0 *eraser_brush_asset_reference;
    ToolSystemBrushBindings4_3_0 tool_brush_bindings;
    Palette2_93_0 *palette;
    CurveMapping2_93_0 *cavity_curve;
    void *paint_cursor;
    unsigned char paint_cursor_col[4];
    int flags;
    int num_input_samples_deprecated;
    int symmetry_flags;
    float tile_offset[3];
    char _pad2[4];
    Paint_Runtime4_3_0 runtime;
};

struct BakeData4_3_0 {
    ImageFormatData4_3_0 im_format;
    char filepath[1024];
    short width, height;
    short margin, flag;
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
    Object4_2_0* cage_object;
};

struct RenderData4_3_0 {
    ImageFormatData4_3_0 im_format;
    void* _pad;
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
    BakeData4_3_0 bake;
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
    char use_old_cpu_compositor;
    char _pad10[7];
};

struct ImagePaintSettings4_3_0 {
    Paint4_3_0 paint;
    short flag, missing_data;
    short seam_bleed, normal_angle;
    short screen_grab_size[2];
    int mode;
    Image4_2_0* stencil;
    Image4_2_0* clone;
    Image4_2_0* canvas;
    float stencil_col[3];
    float dither;
    int interp;
    char _pad[4];
};

struct ToolSettings4_3_0 {
    VPaint4_3_0 *vpaint;
    VPaint4_3_0 *wpaint;
    Sculpt4_3_0 *sculpt;
    UvSculpt4_2_0 uvsculpt;
    GpPaint4_3_0 *gp_paint;
    GpVertexPaint4_3_0 *gp_vertexpaint;
    GpSculptPaint4_3_0 *gp_sculptpaint;
    GpWeightPaint4_3_0 *gp_weightpaint;
    CurvesSculpt4_3_0 *curves_sculpt;
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
    int uvcalc_iterations;
    float uvcalc_weight_factor;
    char uvcalc_weight_group[64];
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
    ImagePaintSettings4_3_0 imapaint;
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

struct SceneEEVEE4_3_0 {
    int flag;
    int gi_diffuse_bounces;
    int gi_cubemap_resolution;
    int gi_visibility_resolution;
    float gi_glossy_clamp;
    int gi_irradiance_pool_size;
    char _pad0[4];
    int taa_samples;
    int taa_render_samples;
    float volumetric_start;
    float volumetric_end;
    int volumetric_tile_size;
    int volumetric_samples;
    float volumetric_sample_distribution;
    float volumetric_light_clamp;
    int volumetric_shadow_samples;
    int volumetric_ray_depth;
    float gtao_distance;
    float gtao_thickness;
    float gtao_focus;
    int gtao_resolution;
    int fast_gi_step_count;
    int fast_gi_ray_count;
    float fast_gi_quality;
    float fast_gi_distance;
    float fast_gi_thickness_near;
    float fast_gi_thickness_far;
    char fast_gi_method;
    char _pad1[3];
    float bokeh_overblur;
    float bokeh_max_size;
    float bokeh_threshold;
    float bokeh_neighbor_max;
    int motion_blur_samples;
    int motion_blur_max;
    int motion_blur_steps;
    int motion_blur_position_deprecated;
    float motion_blur_shutter_deprecated;
    float motion_blur_depth_scale;
    int shadow_cube_size_deprecated;
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

struct AssetShelf4_3_0 {
    AssetShelf4_3_0 *next, *prev;
    char idname[64];
    void *type;
    AssetShelfSettings4_0_0 settings;
    short preferred_row_count;
    short instance_flag;
    char _pad[4];
};

struct Sequence4_3_0 {
    Sequence4_3_0 *next, *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag, type;
    int len;
    float start;
    float startofs, endofs;
    float startstill, endstill;
    int machine;
    int startdisp, enddisp;
    float sat;
    float mul;
    short streamindex;
    short _pad;
    int multicam_source;
    int clip_flag;
    Strip3_6_0 *strip;
    Ipo4_0_0 *ipo;
    Scene4_3_0 *scene;
    Object4_2_0 *scene_camera;
    MovieClip4_1_0 *clip;
    Mask3_6_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence4_3_0 *seq1, *seq2;
    void *_pad7;
    int _pad8[2];
    ListBase2_93_0 seqbase;
    ListBase2_93_0 channels;
    ListBase2_93_0 connections;
    bSound3_3_0 *sound;
    void *scene_sound;
    float volume;
    float pitch , pan;
    float strobe;
    float sound_offset;
    char _pad4[4];
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

struct SpaceSpreadsheet4_3_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    ListBase2_93_0 columns;
    ListBase2_93_0 row_filters;
    ViewerPath3_4_0 viewer_path;
    SpreadsheetInstanceID4_3_0 *instance_ids;
    int instance_ids_num;
    uint8_t filter_flag;
    uint8_t geometry_component_type;
    uint8_t attribute_domain;
    uint8_t object_eval_state;
    int active_layer_index;
    uint32_t flag;
    void *runtime;
};

struct uiStyle4_3_0 {
    uiStyle4_3_0 *next, *prev;
    char name[64];
    uiFontStyle4_1_0 paneltitle;
    uiFontStyle4_1_0 grouplabel;
    uiFontStyle4_1_0 widget;
    uiFontStyle4_1_0 tooltip;
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

struct ThemeUI4_3_0 {
    uiWidgetColors2_93_0 wcol_regular, wcol_tool, wcol_toolbar_item, wcol_text;
    uiWidgetColors2_93_0 wcol_radio, wcol_option, wcol_toggle;
    uiWidgetColors2_93_0 wcol_num, wcol_numslider, wcol_tab;
    uiWidgetColors2_93_0 wcol_menu, wcol_pulldown, wcol_menu_back, wcol_menu_item, wcol_tooltip;
    uiWidgetColors2_93_0 wcol_box, wcol_scroll, wcol_progress, wcol_list_item, wcol_pie_menu;
    uiWidgetStateColors2_93_0 wcol_state;
    unsigned char widget_emboss[4];
    float menu_shadow_fac;
    short menu_shadow_width;
    unsigned char editor_border[4];
    unsigned char editor_outline[4];
    unsigned char editor_outline_active[4];
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
    unsigned char icon_autokey[4];
    char _pad3[4];
    float icon_border_intensity;
    float panel_roundness;
    char _pad2[4];
};

struct ThemeSpace4_3_0 {
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
    unsigned char before_current_frame[4], after_current_frame[4];
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
    unsigned char nodeclass_output[4], nodeclass_filter[4];
    unsigned char nodeclass_vector[4], nodeclass_texture[4];
    unsigned char nodeclass_shader[4], nodeclass_script[4];
    unsigned char nodeclass_pattern[4], nodeclass_layout[4];
    unsigned char nodeclass_geometry[4], nodeclass_attribute[4];
    unsigned char node_zone_simulation[4];
    unsigned char node_zone_repeat[4];
    unsigned char node_zone_foreach_geometry_element[4];
    unsigned char simulated_frames[4];
    unsigned char movie[4], movieclip[4], mask[4], image[4], scene[4], audio[4];
    unsigned char effect[4], transition[4], meta[4], text_strip[4], color_strip[4];
    unsigned char active_strip[4], selected_strip[4];
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

struct UserDef4_3_0 {
    int versionfile, subversionfile;
    int flag;
    unsigned int dupflag;
    char pref_flag;
    char savetime;
    char mouse_emulate_3_button_modifier;
    char trackpad_scroll_direction;
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
    ListBase2_93_0 asset_shelves_settings;
    char keyconfigstr[64];
    short active_asset_library;
    short active_extension_repo;
    char extension_flag;
    uint8_t network_timeout;
    uint8_t network_connection_limit;
    char _pad14[3];
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
    int gpu_preferred_index;
    uint32_t gpu_preferred_vendor_id;
    uint32_t gpu_preferred_device_id;
    char _pad16[4];
    short gpu_backend;
    short max_shader_compilation_subprocesses;
    short playback_fps_samples;
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
    int sequencer_editor_flag;
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
    UserDef_Experimental4_3_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct ForeachGeometryElementZoneViewerPathElem4_3_0 {
    ViewerPathElem4_0_0 base;
    int zone_output_node_id;
    int index;
};

struct GreasePencilDrawing4_3_0 {
    GreasePencilDrawingBase4_0_0 base;
    CurvesGeometry4_3_0 geometry;
    void *runtime;
};

struct Sculpt4_3_0 {
    Paint4_3_0 paint;
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
    Object4_2_0 *gravity_object;
};

struct CurvesSculpt4_3_0 {
    Paint4_3_0 paint;
};

struct GpPaint4_3_0 {
    Paint4_3_0 paint;
    int flag;
    int mode;
};

struct GpVertexPaint4_3_0 {
    Paint4_3_0 paint;
    int flag;
    char _pad[4];
};

struct GpSculptPaint4_3_0 {
    Paint4_3_0 paint;
    int flag;
    char _pad[4];
};

struct GpWeightPaint4_3_0 {
    Paint4_3_0 paint;
    int flag;
    char _pad[4];
};

struct VPaint4_3_0 {
    Paint4_3_0 paint;
    char flag;
    char _pad[3];
    int radial_symm[3];
};

struct Scene4_3_0 {
    ID4_1_0 id;
    AnimData4_3_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object4_2_0 *camera;
    World4_2_0 *world;
    Scene4_3_0 *set;
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
    bNodeTree4_3_0 *nodetree;
    Editing4_3_0 *ed;
    ToolSettings4_3_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData4_3_0 r;
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
    MovieClip4_1_0 *clip;
    PhysicsSettings3_0_0 physics_settings;
    void *_pad8;
    CustomData_MeshMasks2_93_0 customdata_mask;
    CustomData_MeshMasks2_93_0 customdata_mask_modal;
    ColorManagedViewSettings4_3_0 view_settings;
    ColorManagedDisplaySettings2_93_0 display_settings;
    ColorManagedColorspaceSettings2_93_0 sequencer_colorspace_settings;
    RigidBodyWorld2_93_0 *rigidbody_world;
    PreviewImage4_2_0 *preview;
    ListBase2_93_0 view_layers;
    Collection4_2_0 *master_collection;
    IDProperty3_5_0 *layer_properties;
    int simulation_frame_start;
    int simulation_frame_end;
    SceneDisplay2_93_0 display;
    SceneEEVEE4_3_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
    SceneHydra4_0_0 hydra;
    void *runtime;
    void *_pad9;
};

struct Editing4_3_0 {
    ListBase2_93_0 *seqbasep;
    ListBase2_93_0 *displayed_channels;
    void *_pad0;
    ListBase2_93_0 seqbase;
    ListBase2_93_0 metastack;
    ListBase2_93_0 channels;
    Sequence4_3_0 *act_seq;
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
    EditingRuntime4_3_0 runtime;
};

struct bTheme4_3_0 {
    bTheme4_3_0 *next, *prev;
    char name[64];
    char filepath[1024];
    ThemeUI4_3_0 tui;
    ThemeSpace4_3_0 space_properties;
    ThemeSpace4_3_0 space_view3d;
    ThemeSpace4_3_0 space_file;
    ThemeSpace4_3_0 space_graph;
    ThemeSpace4_3_0 space_info;
    ThemeSpace4_3_0 space_action;
    ThemeSpace4_3_0 space_nla;
    ThemeSpace4_3_0 space_sequencer;
    ThemeSpace4_3_0 space_image;
    ThemeSpace4_3_0 space_text;
    ThemeSpace4_3_0 space_outliner;
    ThemeSpace4_3_0 space_node;
    ThemeSpace4_3_0 space_preferences;
    ThemeSpace4_3_0 space_console;
    ThemeSpace4_3_0 space_clip;
    ThemeSpace4_3_0 space_topbar;
    ThemeSpace4_3_0 space_statusbar;
    ThemeSpace4_3_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

struct View3D4_3_0 {
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
    Object4_2_0 *camera, *ob_center;
    rctf2_93_0 render_border;
    View3D4_3_0 *localvd;
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
    bGPdata3_0_0 *gpd;
    short stereo3d_flag;
    char stereo3d_camera;
    char _pad4;
    float stereo3d_convergence_factor;
    float stereo3d_volume_alpha;
    float stereo3d_convergence_alpha;
    View3DShading3_5_0 shading;
    View3DOverlay4_3_0 overlay;
    ViewerPath3_4_0 viewer_path;
    View3D_Runtime3_0_0 runtime;
};

struct NodeImageFile4_3_0 {
    char name[1024];
    ImageFormatData4_3_0 im_format;
    int sfra, efra;
};

struct NodeImageMultiFileSocket4_3_0 {
    short use_render_format;
    short use_node_format;
    char save_as_render;
    char _pad1[3];
    char path[1024];
    ImageFormatData4_3_0 format;
    char layer[30];
    char _pad2[2];
};

#endif
