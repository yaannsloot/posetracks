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

Various structs from Blender v3.4 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_4_0_H
#define MAKESDNA_TYPES_3_4_0_H

#include "makesdna_types_3_3_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_1_0.h"

struct BrushGpencilSettings3_4_0;
struct Brush3_4_0;
struct Collection3_4_0;
struct TimeGpencilModifierSegment3_4_0;
struct TimeGpencilModifierData3_4_0;
struct OutlineGpencilModifierData3_4_0;
struct ID3_4_0;
struct ImageTile3_4_0;
struct Image3_4_0;
struct Base3_4_0;
struct LightProbe3_4_0;
struct MVert3_4_0;
struct MEdge3_4_0;
struct MPoly3_4_0;
struct MetaBall3_4_0;
struct BooleanModifierData3_4_0;
struct NodeGeometryCurveSample3_4_0;
struct NodeGeometrySampleIndex3_4_0;
struct NodeGeometryViewer3_4_0;
struct NodeGeometryDistributePointsInVolume3_4_0;
struct NodeShaderMix3_4_0;
struct BakeData3_4_0;
struct Sculpt3_4_0;
struct Scene3_4_0;
struct SpaceOutliner3_4_0;
struct SpaceImage3_4_0;
struct SpaceSpreadsheet3_4_0;
struct SolidLight3_4_0;
struct UserDef_Experimental3_4_0;
struct View3DOverlay3_4_0;
struct View3D3_4_0;
struct ViewerPathElem3_4_0;
struct IDViewerPathElem3_4_0;
struct ModifierViewerPathElem3_4_0;
struct NodeViewerPathElem3_4_0;
struct ViewerPath3_4_0;
struct wmWindowManager3_4_0;
struct WorkSpace3_4_0;
struct bAction3_4_0;
struct IdAdtTemplate3_4_0;
struct bArmature3_4_0;
struct Palette3_4_0;
struct PaintCurve3_4_0;
struct CacheFile3_4_0;
struct Camera3_4_0;
struct Curves3_4_0;
struct Ipo3_4_0;
struct Key3_4_0;
struct Mask3_4_0;
struct MovieClip3_4_0;
struct ParticleSettings3_4_0;
struct RenderData3_4_0;
struct bScreen3_4_0;
struct bSound3_4_0;
struct Script3_4_0;
struct Speaker3_4_0;
struct Text3_4_0;
struct UserDef3_4_0;
struct VFont3_4_0;
struct Volume3_4_0;

struct BrushGpencilSettings3_4_0 {
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
    float hardeness;
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
    char _pad1[4];
    Material2_93_0 *material;
    Material2_93_0 *material_alt;
};

struct TimeGpencilModifierSegment3_4_0 {
    char name[64];
    TimeGpencilModifierData3_4_0 *gpmd;
    int seg_start;
    int seg_end;
    int seg_mode;
    int seg_repeat;
};

struct Base3_4_0 {
    Base3_4_0 *next, *prev;
    Object3_0_0 *object;
    Base3_4_0 *base_orig;
    unsigned int lay;
    short flag;
    short flag_from_collection;
    short flag_legacy;
    unsigned short local_view_bits;
    unsigned short local_collections_bits;
    char _pad1[2];
};

struct MVert3_4_0 {
    float co[3];
    char flag_legacy;
    char bweight_legacy;
    char _pad[2];
};

struct MEdge3_4_0 {
    unsigned int v1, v2;
    char crease_legacy;
    char bweight_legacy;
    short flag;
};

struct MPoly3_4_0 {
    int loopstart;
    int totloop;
    short mat_nr_legacy;
    char flag, _pad;
};

struct NodeGeometryCurveSample3_4_0 {
    uint8_t mode;
    int8_t use_all_curves;
    int8_t data_type;
    char _pad[1];
};

struct NodeGeometrySampleIndex3_4_0 {
    int8_t data_type;
    int8_t domain;
    int8_t clamp;
    char _pad[1];
};

struct NodeGeometryViewer3_4_0 {
    int8_t data_type;
    int8_t domain;
};

struct NodeGeometryDistributePointsInVolume3_4_0 {
    uint8_t mode;
};

struct NodeShaderMix3_4_0 {
    int8_t data_type;
    int8_t factor_mode;
    int8_t clamp_factor;
    int8_t clamp_result;
    int8_t blend_type;
    char _pad[3];
};

struct SolidLight3_4_0 {
    int flag;
    float smooth;
    float col[4], spec[4], vec[4];
};

struct UserDef_Experimental3_4_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
    char show_asset_debug_info;
    char no_asset_indexing;
    char use_viewport_debug;
    char SANITIZE_AFTER_HERE;
    char use_new_curves_tools;
    char use_new_point_cloud_type;
    char use_full_frame_compositor;
    char use_sculpt_tools_tilt;
    char use_extended_asset_browser;
    char use_override_templates;
    char enable_eevee_next;
    char use_sculpt_texture_paint;
    char use_draw_manager_acquire_lock;
    char use_realtime_compositor;
    char _pad[7];
};

struct View3DOverlay3_4_0 {
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
};

struct ViewerPathElem3_4_0 {
    ViewerPathElem3_4_0 *next, *prev;
    int type;
    char _pad[4];
};

struct IDViewerPathElem3_4_0 {
    ViewerPathElem3_4_0 base;
    ID3_4_0 *id;
};

struct ModifierViewerPathElem3_4_0 {
    ViewerPathElem3_4_0 base;
    char *modifier_name;
};

struct NodeViewerPathElem3_4_0 {
    ViewerPathElem3_4_0 base;
    char *node_name;
};

struct ID3_4_0 {
    void* next, * prev;
    ID3_4_0* newid;
    Library2_93_0* lib;
    AssetMetaData3_0_0* asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    unsigned int recalc;
    unsigned int recalc_up_to_undo_push;
    unsigned int recalc_after_undo_push;
    unsigned int session_uuid;
    IDProperty3_0_0* properties;
    IDOverrideLibrary3_2_0* override_library;
    ID3_4_0* orig_id;
    void* py_instance;
    LibraryWeakReference3_0_0* library_weak_reference;
    ID_Runtime3_2_0 runtime;
};

struct Brush3_4_0 {
    ID3_4_0 id;
    BrushClone2_93_0 clone;
    CurveMapping2_93_0 *curve;
    MTex3_0_0 mtex;
    MTex3_0_0 mask_mtex;
    Brush3_4_0 *toggle_brush;
    void *icon_imbuf;
    PreviewImage2_93_0 *preview;
    ColorBand2_93_0 *gradient;
    PaintCurve3_4_0 *paint_curve;
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
    BrushGpencilSettings3_4_0 *gpencil_settings;
    BrushCurvesSculptSettings3_3_0 *curves_sculpt_settings;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    CurveMapping2_93_0 *automasking_cavity_curve;
};

struct Collection3_4_0 {
    ID3_4_0 id;
    ID3_4_0 *owner_id;
    ListBase2_93_0 gobject;
    ListBase2_93_0 children;
    PreviewImage2_93_0 *preview;
    unsigned int layer;
    float instance_offset[3];
    short flag;
    short tag;
    short lineart_usage;
    unsigned char lineart_flags;
    unsigned char lineart_intersection_mask;
    unsigned char lineart_intersection_priority;
    char _pad[5];
    int16_t color_tag;
    ListBase2_93_0 object_cache;
    ListBase2_93_0 object_cache_instanced;
    ListBase2_93_0 parents;
    SceneCollection2_93_0 *collection;
    ViewLayer3_2_0 *view_layer;
};

struct TimeGpencilModifierData3_4_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    int layer_pass;
    int flag;
    int offset;
    float frame_scale;
    int mode;
    int sfra, efra;
    char _pad[4];
    TimeGpencilModifierSegment3_4_0 *segments;
    int segments_len;
    int segment_active_index;
};

struct OutlineGpencilModifierData3_4_0 {
    GpencilModifierData2_93_0 modifier;
    Object3_0_0 *object;
    Material2_93_0 *material;
    char layername[64];
    int pass_index;
    int flag;
    int thickness;
    float sample_length;
    int subdiv;
    int layer_pass;
    Material2_93_0 *outline_material;
};

struct ImageTile3_4_0 {
    ImageTile3_4_0 *next, *prev;
    ImageTile_Runtime3_3_0 runtime;
    int tile_number;
    int gen_x, gen_y;
    char gen_type, gen_flag;
    short gen_depth;
    float gen_color[4];
    char label[64];
};

struct Image3_4_0 {
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
    char _pad2[4];
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

struct LightProbe3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    char type;
    char flag;
    char attenuation_type;
    char parallax_type;
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
    char _pad1[4];
    Object3_0_0 *parallax_ob;
    Image3_4_0 *image;
    Collection3_4_0 *visibility_grp;
    float distfalloff, distgridinf;
};

struct MetaBall3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 disp;
    ListBase2_93_0 *editelems;
    Ipo3_4_0 *ipo;
    Material2_93_0 **mat;
    char flag, flag2;
    short totcol;
    char texflag;
    char _pad[2];
    char needs_flush_to_id;
    float loc[3];
    float size[3];
    float rot[3];
    float wiresize, rendersize;
    float thresh;
    MetaElem2_93_0 *lastelem;
};

struct BooleanModifierData3_4_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *object;
    Collection3_4_0 *collection;
    float double_threshold;
    char operation;
    char solver;
    char material_mode;
    char flag;
    char bm_flag;
    char _pad[7];
};

struct BakeData3_4_0 {
    ImageFormatData3_2_0 im_format;
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
    Object3_0_0 *cage_object;
};

struct Sculpt3_4_0 {
    Paint2_93_0 paint;
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
    float automasking_start_normal_limit, automasking_start_normal_falloff;
    float automasking_view_normal_limit, automasking_view_normal_falloff;
    CurveMapping2_93_0 *automasking_cavity_curve;
    CurveMapping2_93_0 *automasking_cavity_curve_op;
    Object3_0_0 *gravity_object;
};

struct RenderData3_4_0 {
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

struct Scene3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object3_0_0 *camera;
    World2_93_0 *world;
    Scene3_4_0 *set;
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
    RenderData3_4_0 r;
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
    MovieClip3_4_0 *clip;
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
    Collection3_4_0 *master_collection;
    SceneCollection2_93_0 *collection;
    IDProperty3_0_0 *layer_properties;
    void *_pad9;
    SceneDisplay2_93_0 display;
    SceneEEVEE2_93_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
};

struct SpaceOutliner3_4_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
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

struct SpaceImage3_4_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Image3_4_0 *image;
    ImageUser3_0_0 iuser;
    Scopes2_93_0 scopes;
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
    char _pad1[2];
    int flag;
    float uv_opacity;
    int tile_grid_shape[2];
    int custom_grid_subdiv[2];
    MaskSpaceInfo3_3_0 mask_info;
    SpaceImageOverlay2_93_0 overlay;
};

struct ViewerPath3_4_0 {
    ListBase2_93_0 path;
};

struct SpaceSpreadsheet3_4_0 {
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
    uint32_t flag;
    void *runtime;
};

struct View3D3_4_0 {
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
    Object3_0_0 *camera, *ob_center;
    rctf2_93_0 render_border;
    View3D3_4_0 *localvd;
    char ob_center_bone[64];
    unsigned short local_view_uuid;
    char _pad6[2];
    int layact;
    unsigned short local_collections_uuid;
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
    View3DShading2_93_0 shading;
    View3DOverlay3_4_0 overlay;
    ViewerPath3_4_0 viewer_path;
    View3D_Runtime3_0_0 runtime;
};

struct wmWindowManager3_4_0 {
    ID3_4_0 id;
    wmWindow3_3_0 *windrawable;
    wmWindow3_3_0 *winactive;
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
    wmXrData2_93_0 xr;
};

struct WorkSpace3_4_0 {
    ID3_4_0 id;
    ListBase2_93_0 layouts;
    ListBase2_93_0 hook_layout_relations;
    ListBase2_93_0 owner_ids;
    ListBase2_93_0 tools;
    Scene3_4_0 *pin_scene;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    char *status_text;
    AssetLibraryReference3_0_0 asset_library_ref;
    ViewerPath3_4_0 viewer_path;
};

struct bAction3_4_0 {
    ID3_4_0 id;
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

struct IdAdtTemplate3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
};

struct bArmature3_4_0 {
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
    unsigned int layer_used;
    unsigned int layer, layer_protected;
    float axes_position;
};

struct Palette3_4_0 {
    ID3_4_0 id;
    ListBase2_93_0 colors;
    int active_color;
    char _pad[4];
};

struct PaintCurve3_4_0 {
    ID3_4_0 id;
    PaintCurvePoint2_93_0 *points;
    int tot_points;
    int add_index;
};

struct CacheFile3_4_0 {
    ID3_4_0 id;
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

struct Camera3_4_0 {
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
    Ipo3_4_0 *ipo;
    Object3_0_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings3_3_0 dof;
    ListBase2_93_0 bg_images;
    char sensor_fit;
    char _pad[7];
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct Curves3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
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

struct Ipo3_4_0 {
    ID3_4_0 id;
    ListBase2_93_0 curve;
    rctf2_93_0 cur;
    short blocktype, showkey;
    short muteipo;
    char _pad[2];
};

struct Key3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    KeyBlock2_93_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    ListBase2_93_0 block;
    Ipo3_4_0 *ipo;
    ID3_4_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct Mask3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra, efra;
    int flag;
    char _pad[4];
};

struct MovieClip3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    char filepath[1024];
    int source;
    int lastframe;
    int lastsize[2];
    float aspx, aspy;
    void *anim;
    void *cache;
    bGPdata3_0_0 *gpd;
    MovieTracking2_93_0 tracking;
    void *tracking_context;
    MovieClipProxy2_93_0 proxy;
    int flag;
    int len;
    int start_frame;
    int frame_offset;
    ColorManagedColorspaceSettings2_93_0 colorspace_settings;
    MovieClip_Runtime2_93_0 runtime;
};

struct ParticleSettings3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    BoidSettings2_93_0 *boids;
    SPHFluidSettings2_93_0 *fluid;
    EffectorWeights3_0_0 *effector_weights;
    Collection3_4_0 *collision_group;
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
    Collection3_4_0 *instance_collection;
    ListBase2_93_0 instance_weights;
    Collection3_4_0 *force_group;
    Object3_0_0 *instance_object;
    Object3_0_0 *bb_ob;
    Ipo3_4_0 *ipo;
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

struct bScreen3_4_0 {
    ID3_4_0 id;
    ListBase2_93_0 vertbase;
    ListBase2_93_0 edgebase;
    ListBase2_93_0 areabase;
    ListBase2_93_0 regionbase;
    Scene3_4_0 *scene;
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
    ARegion2_93_0 *active_region;
    void *animtimer;
    void  *context;
    void *tool_tip;
    PreviewImage2_93_0 *preview;
};

struct bSound3_4_0 {
    ID3_4_0 id;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    void *handle;
    PackedFile2_93_0 *newpackedfile;
    Ipo3_4_0 *ipo;
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

struct Script3_4_0 {
    ID3_4_0 id;
    void *py_draw;
    void *py_event;
    void *py_button;
    void *py_browsercallback;
    void *py_globaldict;
    int flags, lastspace;
    char scriptname[1024];
    char scriptarg[256];
};

struct Speaker3_4_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    bSound3_4_0 *sound;
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

struct Text3_4_0 {
    ID3_4_0 id;
    char *filepath;
    void *compiled;
    int flags;
    char _pad0[4];
    ListBase2_93_0 lines;
    TextLine2_93_0 *curl, *sell;
    int curc, selc;
    double mtime;
};

struct UserDef3_4_0 {
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
    char pythondir[768];
    char sounddir[768];
    char i18ndir[768];
    char image_editor[1024];
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
    float dpi_fac;
    float inv_dpi_fac;
    float pixelsize;
    int virtual_pixel;
    int scrollback;
    char node_margin;
    char _pad2[1];
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
    ListBase2_93_0 user_menus;
    ListBase2_93_0 asset_libraries;
    char keyconfigstr[64];
    short undosteps;
    char _pad1[2];
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
    char _pad7[6];
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
    UserDef_Experimental3_4_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct VFont3_4_0 {
    ID3_4_0 id;
    char filepath[1024];
    void *data;
    PackedFile2_93_0 *packedfile;
    PackedFile2_93_0 *temp_pf;
};

struct Volume3_4_0 {
    ID3_4_0 id;
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
    Volume_Runtime3_2_0 runtime;
};

#endif
