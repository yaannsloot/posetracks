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

Various structs from Blender v3.6 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_6_0_H
#define MAKESDNA_TYPES_3_6_0_H

#include "makesdna_types_3_3_0.h"
#include "makesdna_types_2_93_0.h"
#include "makesdna_types_3_5_0.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_3_4_0.h"
#include "makesdna_types_3_2_0.h"
#include "makesdna_types_3_1_0.h"

struct DriverTarget3_6_0;
struct Collection_Runtime3_6_0;
struct CharInfo3_6_0;
struct CustomDataLayer3_6_0;
struct PartEff3_6_0;
struct Image3_6_0;
struct LightProbe3_6_0;
struct LightProbeVisibilityData3_6_0;
struct LightProbeConnectivityData3_6_0;
struct LightProbeBlockData3_6_0;
struct LightProbeObjectCache3_6_0;
struct Mask3_6_0;
struct MLoopTri3_6_0;
struct MEdge3_6_0;
struct MPoly3_6_0;
struct NodesModifierData3_6_0;
struct NodeSimulationItem3_6_0;
struct NodeGeometrySimulationInput3_6_0;
struct NodeGeometrySampleVolume3_6_0;
struct RenderData3_6_0;
struct ToolSettings3_6_0;
struct ARegion3_6_0;
struct StripElem3_6_0;
struct StripProxy3_6_0;
struct Strip3_6_0;
struct SpreadsheetRowFilter3_6_0;
struct MovieTrackingTrack3_6_0;
struct MovieTrackingSettings3_6_0;
struct MovieTrackingStabilization3_6_0;
struct ThemeSpace3_6_0;
struct bUserMenuItem_Op3_6_0;
struct bUserAssetLibrary3_6_0;
struct bUserScriptDirectory3_6_0;
struct vec2i3_6_0;
struct View3DOverlay3_6_0;
struct DriverVar3_6_0;
struct Collection3_6_0;
struct Scene3_6_0;
struct MovieTracking3_6_0;
struct bTheme3_6_0;
struct MovieClip3_6_0;

struct DriverTarget3_6_0 {
    ID3_4_0 *id;
    char *rna_path;
    char pchan_name[64];
    short transChan;
    char rotation_mode;
    char _pad[7];
    short flag;
    int idtype;
    int context_property;
    int _pad1;
};

struct CharInfo3_6_0 {
    float kern;
    short mat_nr;
    char flag;
    char _pad[1];
};

struct CustomDataLayer3_6_0 {
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
    const void *sharing_info;
};

struct PartEff3_6_0 {
    PartEff3_6_0 *next, *prev;
    short type, flag, buttype;
    short stype, vertgroup, userjit;
    float sta, end, lifetime;
    int totpart, totkey, seed;
    float normfac, obfac, randfac, texfac, randlife;
    float force[3];
    float damp;
    float nabla, vectsize, maxlen, defvec[3];
    char _pad[4];
    float mult[4], life[4];
    short child[4], mat[4];
    short texmap, curmult;
    short staticstep, omat, timetex, speedtex, flag2, flag2neg;
    short disp, vertgroup_v;
    char vgroupname[64], vgroupname_v[64];
    float imat[4][4];
    Particle3_0_0 *keys;
    Collection3_6_0 *group;
};

struct LightProbeVisibilityData3_6_0 {
    float *L0;
    float *L1_a;
    float *L1_b;
    float *L1_c;
};

struct LightProbeConnectivityData3_6_0 {
    uint8_t *bitmask;
};

struct LightProbeBlockData3_6_0 {
    int offset[3];
    int level;
};

struct LightProbeObjectCache3_6_0 {
    int cache_type;
    char shared;
    char dirty;
    char _pad0[2];
    void *grid_static_cache;
};

struct MLoopTri3_6_0 {
    unsigned int tri[3];
};

struct MEdge3_6_0 {
    unsigned int v1, v2;
    char crease_legacy;
    char bweight_legacy;
    short flag_legacy;
};

struct MPoly3_6_0 {
    int loopstart;
    int totloop;
    short mat_nr_legacy;
    char flag_legacy, _pad;
};

struct NodeSimulationItem3_6_0 {
    char *name;
    short socket_type;
    short attribute_domain;
    int identifier;
};

struct NodeGeometrySimulationInput3_6_0 {
    int32_t output_node_id;
};

struct NodeGeometrySampleVolume3_6_0 {
    int8_t grid_type;
    int8_t interpolation_mode;
};

struct StripElem3_6_0 {
    char filename[256];
    int orig_width, orig_height;
    float orig_fps;
};

struct StripProxy3_6_0 {
    char dirpath[768];
    char filename[256];
    void *anim;
    short tc;
    short quality;
    short build_size_flags;
    short build_tc_flags;
    short build_flags;
    char storage;
    char _pad[5];
};

struct SpreadsheetRowFilter3_6_0 {
    SpreadsheetRowFilter3_6_0 *next, *prev;
    char column_name[64];
    uint8_t operation;
    uint8_t flag;
    char _pad0[2];
    int value_int;
    int value_int2[2];
    char *value_string;
    float value_float;
    float threshold;
    float value_float2[2];
    float value_float3[3];
    float value_color[4];
    char _pad1[4];
};

struct MovieTrackingTrack3_6_0 {
    MovieTrackingTrack3_6_0 *next, *prev;
    char name[64];
    float pat_min_legacy[2], pat_max_legacy[2];
    float search_min_legacy[2], search_max_legacy[2];
    float offset[2];
    int markersnr;
    int _pad;
    MovieTrackingMarker2_93_0 *markers;
    float bundle_pos[3];
    float error;
    int flag, pat_flag, search_flag;
    float color[3];
    short frames_limit;
    short margin;
    short pattern_match;
    short motion_model;
    int algorithm_flag;
    float minimum_correlation;
    bGPdata3_0_0 *gpd;
    float weight;
    float weight_stab;
};

struct MovieTrackingSettings3_6_0 {
    short default_motion_model;
    short default_algorithm_flag;
    float default_minimum_correlation;
    short default_pattern_size;
    short default_search_size;
    short default_frames_limit;
    short default_margin;
    short default_pattern_match;
    short default_flag;
    float default_weight;
    short motion_flag;
    short speed;
    int keyframe1_legacy;
    int keyframe2_legacy;
    int reconstruction_flag;
    int refine_camera_intrinsics;
    float dist;
    int clean_frames, clean_action;
    float clean_error;
    float object_distance;
};

struct MovieTrackingStabilization3_6_0 {
    int flag;
    int tot_track, act_track;
    int tot_rot_track, act_rot_track;
    float maxscale;
    MovieTrackingTrack3_6_0 *rot_track_legacy;
    int anchor_frame;
    float target_pos[2];
    float target_rot;
    float scale;
    float locinf, scaleinf, rotinf;
    int filter;
    int _pad;
};

struct bUserAssetLibrary3_6_0 {
    bUserAssetLibrary3_6_0 *next, *prev;
    char name[64];
    char path[1024];
    short import_method;
    short flag;
    char _pad0[4];
};

struct bUserScriptDirectory3_6_0 {
    bUserScriptDirectory3_6_0 *next, *prev;
    char name[64];
    char dir_path[768];
};

struct vec2i3_6_0 {
    int x, y;
};

struct View3DOverlay3_6_0 {
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
    char _pad[4];
};

struct DriverVar3_6_0 {
    DriverVar3_6_0 *next, *prev;
    char name[64];
    DriverTarget3_6_0 targets[8];
    char num_targets;
    char type;
    short flag;
    float curval;
};

struct Collection_Runtime3_6_0 {
    ID3_4_0 *owner_id;
    ListBase2_93_0 object_cache;
    ListBase2_93_0 object_cache_instanced;
    ListBase2_93_0 parents;
    void *gobject_hash;
    uint8_t tag;
    char _pad0[7];
};

struct Image3_6_0 {
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
    int offset_x, offset_y;
    int active_tile_index;
    ListBase2_93_0 tiles;
    ListBase2_93_0 views;
    Stereo3dFormat2_93_0 *stereo3d_format;
    Image_Runtime3_1_0 runtime;
};

struct LightProbe3_6_0 {
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
    Image3_6_0 *image;
    Collection3_6_0 *visibility_grp;
};

struct Mask3_6_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    ListBase2_93_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra, efra;
    int flag;
    char _pad[4];
};

struct NodesModifierData3_6_0 {
    ModifierData3_5_0 modifier;
    void *node_group;
    NodesModifierSettings2_93_0 settings;
    char *simulation_bake_directory;
    void *_pad;
    void *runtime_eval_log;
    void *simulation_cache;
};

struct RenderData3_6_0 {
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
    CurveMapping2_93_0 mblur_shutter_curve;
};

struct ToolSettings3_6_0 {
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
    short snap_mode_tools;
    char plane_axis;
    char plane_depth;
    char plane_orient;
    char use_plane_axis_auto;
    char _pad7[2];
};

struct ARegion3_6_0 {
    ARegion3_6_0 *next, *prev;
    View2D2_93_0 v2d;
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

struct Strip3_6_0 {
    Strip3_6_0 *next, *prev;
    int us, done;
    int startstill, endstill;
    StripElem3_6_0 *stripdata;
    char dirpath[768];
    StripProxy3_6_0 *proxy;
    StripCrop2_93_0 *crop;
    StripTransform3_2_0 *transform;
    StripColorBalance3_0_0 *color_balance;
    ColorManagedColorspaceSettings2_93_0 colorspace_settings;
};

struct ThemeSpace3_6_0 {
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

struct bUserMenuItem_Op3_6_0 {
    bUserMenuItem2_93_0 item;
    char op_idname[64];
    IDProperty3_5_0 *prop;
    char op_prop_enum[64];
    char opcontext;
    char _pad0[7];
};

struct Collection3_6_0 {
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
    Collection_Runtime3_6_0 runtime;
};

struct Scene3_6_0 {
    ID3_4_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    Object3_0_0 *camera;
    World2_93_0 *world;
    Scene3_6_0 *set;
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
    ToolSettings3_6_0 *toolsettings;
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
    bGPdata3_0_0 *gpd;
    MovieClip3_6_0 *clip;
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
    Collection3_6_0 *master_collection;
    SceneCollection2_93_0 *collection;
    IDProperty3_5_0 *layer_properties;
    void *_pad9;
    SceneDisplay2_93_0 display;
    SceneEEVEE3_5_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
};

struct MovieTracking3_6_0 {
    MovieTrackingSettings3_6_0 settings;
    MovieTrackingCamera3_5_0 camera;
    ListBase2_93_0 tracks_legacy;
    ListBase2_93_0 plane_tracks_legacy;
    MovieTrackingReconstruction2_93_0 reconstruction_legacy;
    MovieTrackingStabilization3_6_0 stabilization;
    MovieTrackingTrack3_6_0 *act_track_legacy;
    MovieTrackingPlaneTrack2_93_0 *act_plane_track_legacy;
    ListBase2_93_0 objects;
    int objectnr, tot_object;
    MovieTrackingStats2_93_0 *stats;
    MovieTrackingDopesheet2_93_0 dopesheet;
};

struct bTheme3_6_0 {
    bTheme3_6_0 *next, *prev;
    char name[32];
    ThemeUI3_3_0 tui;
    ThemeSpace3_6_0 space_properties;
    ThemeSpace3_6_0 space_view3d;
    ThemeSpace3_6_0 space_file;
    ThemeSpace3_6_0 space_graph;
    ThemeSpace3_6_0 space_info;
    ThemeSpace3_6_0 space_action;
    ThemeSpace3_6_0 space_nla;
    ThemeSpace3_6_0 space_sequencer;
    ThemeSpace3_6_0 space_image;
    ThemeSpace3_6_0 space_text;
    ThemeSpace3_6_0 space_outliner;
    ThemeSpace3_6_0 space_node;
    ThemeSpace3_6_0 space_preferences;
    ThemeSpace3_6_0 space_console;
    ThemeSpace3_6_0 space_clip;
    ThemeSpace3_6_0 space_topbar;
    ThemeSpace3_6_0 space_statusbar;
    ThemeSpace3_6_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    ThemeStripColor3_0_0 strip_color[9];
    int active_theme_area;
};

struct MovieClip3_6_0 {
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
    MovieTracking3_6_0 tracking;
    void *tracking_context;
    MovieClipProxy2_93_0 proxy;
    int flag;
    int len;
    int start_frame;
    int frame_offset;
    ColorManagedColorspaceSettings2_93_0 colorspace_settings;
    MovieClip_Runtime2_93_0 runtime;
};

#endif
