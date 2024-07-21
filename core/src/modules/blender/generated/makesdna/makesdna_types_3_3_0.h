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

Various structs from Blender v3.3 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_3_0_H
#define MAKESDNA_TYPES_3_3_0_H

#include "makesdna_types_3_1_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_2_93_0.h"

struct BrushCurvesSculptSettings3_3_0;
struct CameraDOFSettings3_3_0;
struct Collection3_3_0;
struct CurvesGeometry3_3_0;
struct Curves3_3_0;
struct NoiseGpencilModifierData3_3_0;
struct LineartGpencilModifierData3_3_0;
struct Library_Runtime3_3_0;
struct ImagePackedFile3_3_0;
struct ImageTile_Runtime3_3_0;
struct Image3_3_0;
struct MaterialLineArt3_3_0;
struct SurfaceDeformModifierData3_3_0;
struct bNodeSocket3_3_0;
struct bNode3_3_0;
struct NodeCMPCombSepColor3_3_0;
struct NodeGeometryMeshToVolume3_3_0;
struct NodeGeometryUVUnwrap3_3_0;
struct NodeCombSepColor3_3_0;
struct ObjectLineArt3_3_0;
struct PointCloud3_3_0;
struct Sculpt3_3_0;
struct ToolSettings3_3_0;
struct uiList3_3_0;
struct Sequence3_3_0;
struct bSound3_3_0;
struct MaskSpaceInfo3_3_0;
struct SpaceImage3_3_0;
struct SpreadsheetRowFilter3_3_0;
struct ThemeUI3_3_0;
struct UserDef_Experimental3_3_0;
struct UserDef3_3_0;
struct wmWindow3_3_0;
struct WorkSpace3_3_0;
struct Camera3_3_0;
struct ImageTile3_3_0;
struct SpaceClip3_3_0;
struct bTheme3_3_0;

struct BrushCurvesSculptSettings3_3_0 {
    int add_amount;
    int points_per_curve;
    uint32_t flag;
    float minimum_length;
    float curve_length;
    float minimum_distance;
    int density_add_attempts;
    uint8_t density_mode;
    char _pad[7];
};

struct CameraDOFSettings3_3_0 {
    Object3_0_0 *focus_object;
    char focus_subtarget[64];
    float focus_distance;
    float aperture_fstop;
    float aperture_rotation;
    float aperture_ratio;
    int aperture_blades;
    short flag;
    char _pad[2];
};

struct Library_Runtime3_3_0 {
    void *name_map;
};

struct ImagePackedFile3_3_0 {
    ImagePackedFile3_3_0 *next, *prev;
    PackedFile2_93_0 *packedfile;
    int view;
    int tile_number;
    char filepath[1024];
};

struct ImageTile_Runtime3_3_0 {
    int tilearray_layer;
    int _pad;
    int tilearray_offset[2];
    int tilearray_size[2];
};

struct MaterialLineArt3_3_0 {
    int flags;
    unsigned char material_mask_bits;
    unsigned char mat_occlusion;
    unsigned char intersection_priority;
    char _pad;
};

struct NodeCMPCombSepColor3_3_0 {
    uint8_t mode;
    uint8_t ycc_mode;
};

struct NodeGeometryMeshToVolume3_3_0 {
    uint8_t resolution_mode;
};

struct NodeGeometryUVUnwrap3_3_0 {
    uint8_t method;
};

struct NodeCombSepColor3_3_0 {
    int8_t mode;
};

struct ObjectLineArt3_3_0 {
    short usage;
    short flags;
    float crease_threshold;
    unsigned char intersection_priority;
    char _pad[7];
};

struct uiList3_3_0 {
    uiList3_3_0 *next, *prev;
    void *type;
    char list_id[64];
    int layout_type;
    int flag;
    int list_scroll;
    int list_grip;
    int list_last_len;
    int list_last_activei;
    char filter_byname[64];
    int filter_flag;
    int filter_sort_flag;
    IDProperty3_0_0 *properties;
    uiListDyn2_93_0 *dyn_data;
};

struct MaskSpaceInfo3_3_0 {
    Mask2_93_0 *mask;
    char draw_flag;
    char draw_type;
    char overlay_mode;
    char _pad3[1];
    float blend_factor;
};

struct SpreadsheetRowFilter3_3_0 {
    SpreadsheetRowFilter3_3_0 *next, *prev;
    char column_name[64];
    uint8_t operation;
    uint8_t flag;
    char _pad0[2];
    int value_int;
    char *value_string;
    float value_float;
    float threshold;
    float value_float2[2];
    float value_float3[3];
    float value_color[4];
    char _pad1[4];
};

struct UserDef_Experimental3_3_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
    char show_asset_debug_info;
    char no_asset_indexing;
    char SANITIZE_AFTER_HERE;
    char _pad0;
    char use_new_curves_tools;
    char use_new_point_cloud_type;
    char use_full_frame_compositor;
    char use_sculpt_tools_tilt;
    char use_extended_asset_browser;
    char use_override_templates;
    char enable_eevee_next;
    char use_sculpt_texture_paint;
    char use_draw_manager_acquire_lock;
};

struct ImageTile3_3_0 {
    ImageTile3_3_0 *next, *prev;
    ImageTile_Runtime3_3_0 runtime;
    char _pad[4];
    int tile_number;
    char label[64];
};

struct Collection3_3_0 {
    ID3_2_0 id;
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

struct CurvesGeometry3_3_0 {
    int *curve_offsets;
    CustomData3_0_0 point_data;
    CustomData3_0_0 curve_data;
    int point_num;
    int curve_num;
    void *runtime;
};

struct Curves3_3_0 {
    ID3_2_0 id;
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

struct NoiseGpencilModifierData3_3_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
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
    int layer_pass;
    int seed;
    CurveMapping2_93_0 *curve_intensity;
};

struct LineartGpencilModifierData3_3_0 {
    GpencilModifierData2_93_0 modifier;
    uint16_t edge_types;
    char source_type;
    char use_multiple_levels;
    short level_start;
    short level_end;
    Object3_0_0 *source_camera;
    Object3_0_0 *light_contour_object;
    Object3_0_0 *source_object;
    Collection3_3_0 *source_collection;
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
    void *cache;
    void *la_data_ptr;
};

struct Image3_3_0 {
    ID3_2_0 id;
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
    int gen_x, gen_y;
    char gen_type, gen_flag;
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

struct SurfaceDeformModifierData3_3_0 {
    ModifierData3_1_0 modifier;
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

struct bNodeSocket3_3_0 {
    bNodeSocket3_3_0 *next, *prev;
    IDProperty3_0_0 *prop;
    char identifier[64];
    char name[64];
    void *storage;
    short type;
    short flag;
    short limit;
    short in_out;
    void *typeinfo;
    char idname[64];
    float locx, locy;
    void *default_value;
    short stack_index;
    short stack_type;
    char display_shape;
    char attribute_domain;
    short total_inputs;
    char label[64];
    char description[64];
    char *default_attribute_name;
    void *cache;
    int own_index;
    int to_index;
    bNodeSocket3_3_0 *groupsock;
    bNodeLink2_93_0 *link;
    bNodeStack2_93_0 ns;
    void *runtime;
};

struct bNode3_3_0 {
    bNode3_3_0 *next, *prev;
    IDProperty3_0_0 *prop;
    void *typeinfo;
    char idname[64];
    char name[64];
    int flag;
    short type;
    short done, level;
    uint8_t need_exec;
    char _pad2[1];
    float color[3];
    ListBase2_93_0 inputs, outputs;
    bNode3_3_0 *parent;
    ID3_2_0 *id;
    void *storage;
    bNode3_3_0 *original;
    ListBase2_93_0 internal_links;
    float locx, locy;
    float width, height;
    float miniwidth;
    float offsetx, offsety;
    float anim_init_locx;
    float anim_ofsx;
    int update;
    char label[64];
    short custom1, custom2;
    float custom3, custom4;
    char _pad1[4];
    rctf2_93_0 totr;
    rctf2_93_0 prvr;
    short preview_xsize, preview_ysize;
    short tmp_flag;
    char _pad0;
    char iter_flag;
    void *runtime;
};

struct PointCloud3_3_0 {
    ID3_2_0 id;
    AnimData2_93_0 *adt;
    int flag;
    int totpoint;
    CustomData3_0_0 pdata;
    int attributes_active_index;
    int _pad4;
    Material2_93_0 **mat;
    short totcol;
    short _pad3[3];
    void *batch_cache;
};

struct Sculpt3_3_0 {
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
    char _pad[4];
    Object3_0_0 *gravity_object;
};

struct ToolSettings3_3_0 {
    VPaint2_93_0 *vpaint;
    VPaint2_93_0 *wpaint;
    Sculpt3_3_0 *sculpt;
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
    char _pad1[1];
    short snap_mode;
    char snap_node_mode;
    char snap_uv_mode;
    short snap_flag;
    short snap_flag_node;
    short snap_flag_seq;
    short snap_uv_flag;
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
};

struct Sequence3_3_0 {
    Sequence3_3_0 *next, *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag, type;
    int len;
    float start;
    float startofs, endofs;
    float startstill, endstill;
    int machine;
    int _pad3;
    int startdisp, enddisp;
    float sat;
    float mul;
    float _pad;
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    Strip2_93_0 *strip;
    Ipo2_93_0 *ipo;
    Scene2_93_0 *scene;
    Object3_0_0 *scene_camera;
    MovieClip2_93_0 *clip;
    Mask2_93_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence3_3_0 *seq1, *seq2, *seq3;
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
    char _pad4[2];
    int cache_flag;
    int sfra;
    char views_format;
    char _pad1[3];
    Stereo3dFormat2_93_0 *stereo3d_format;
    IDProperty3_0_0 *prop;
    ListBase2_93_0 modifiers;
    float media_playback_rate;
    float speed_factor;
    SequenceRuntime2_93_0 runtime;
};

struct bSound3_3_0 {
    ID3_2_0 id;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    void *handle;
    PackedFile2_93_0 *newpackedfile;
    Ipo2_93_0 *ipo;
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

struct SpaceImage3_3_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Image3_3_0 *image;
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
    char pixel_snap_mode;
    char lock;
    char dt_uv;
    char dt_uvstretch;
    char around;
    char gizmo_flag;
    char _pad1[3];
    int flag;
    float uv_opacity;
    int tile_grid_shape[2];
    int custom_grid_subdiv;
    char _pad3[4];
    MaskSpaceInfo3_3_0 mask_info;
    SpaceImageOverlay2_93_0 overlay;
};

struct ThemeUI3_3_0 {
    uiWidgetColors2_93_0 wcol_regular, wcol_tool, wcol_toolbar_item, wcol_text;
    uiWidgetColors2_93_0 wcol_radio, wcol_option, wcol_toggle;
    uiWidgetColors2_93_0 wcol_num, wcol_numslider, wcol_tab;
    uiWidgetColors2_93_0 wcol_menu, wcol_pulldown, wcol_menu_back, wcol_menu_item, wcol_tooltip;
    uiWidgetColors2_93_0 wcol_box, wcol_scroll, wcol_progress, wcol_list_item, wcol_pie_menu;
    uiWidgetColors2_93_0 wcol_view_item;
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

struct UserDef3_3_0 {
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
    SolidLight2_93_0 light_param[4];
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
    UserDef_Experimental3_3_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct wmWindow3_3_0 {
    wmWindow3_3_0 *next, *prev;
    void *ghostwin;
    void *gpuctx;
    wmWindow3_3_0 *parent;
    Scene2_93_0 *scene;
    Scene2_93_0 *new_scene;
    char view_layer_name[64];
    Scene2_93_0 *unpinned_scene;
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
    char addmousemove;
    char tag_cursor_refresh;
    char event_queue_check_click;
    char event_queue_check_drag;
    char event_queue_check_drag_handled;
    char _pad0[1];
    short pie_event_type_lock;
    short pie_event_type_last;
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

struct WorkSpace3_3_0 {
    ID3_2_0 id;
    ListBase2_93_0 layouts;
    ListBase2_93_0 hook_layout_relations;
    ListBase2_93_0 owner_ids;
    ListBase2_93_0 tools;
    Scene2_93_0 *pin_scene;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    char *status_text;
    AssetLibraryReference3_0_0 asset_library_ref;
};

struct Camera3_3_0 {
    ID3_2_0 id;
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
    Ipo2_93_0 *ipo;
    Object3_0_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings3_3_0 dof;
    ListBase2_93_0 bg_images;
    char sensor_fit;
    char _pad[7];
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct SpaceClip3_3_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char _pad1[4];
    float xof, yof;
    float xlockof, ylockof;
    float zoom;
    MovieClipUser2_93_0 user;
    MovieClip2_93_0 *clip;
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

struct bTheme3_3_0 {
    bTheme3_3_0 *next, *prev;
    char name[32];
    ThemeUI3_3_0 tui;
    ThemeSpace3_0_0 space_properties;
    ThemeSpace3_0_0 space_view3d;
    ThemeSpace3_0_0 space_file;
    ThemeSpace3_0_0 space_graph;
    ThemeSpace3_0_0 space_info;
    ThemeSpace3_0_0 space_action;
    ThemeSpace3_0_0 space_nla;
    ThemeSpace3_0_0 space_sequencer;
    ThemeSpace3_0_0 space_image;
    ThemeSpace3_0_0 space_text;
    ThemeSpace3_0_0 space_outliner;
    ThemeSpace3_0_0 space_node;
    ThemeSpace3_0_0 space_preferences;
    ThemeSpace3_0_0 space_console;
    ThemeSpace3_0_0 space_clip;
    ThemeSpace3_0_0 space_topbar;
    ThemeSpace3_0_0 space_statusbar;
    ThemeSpace3_0_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

#endif
