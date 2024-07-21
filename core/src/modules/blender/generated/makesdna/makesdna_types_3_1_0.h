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

Various structs from Blender v3.1 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_1_0_H
#define MAKESDNA_TYPES_3_1_0_H

#include "makesdna_types_2_93_4.h"
#include "makesdna_types_3_0_0.h"
#include "makesdna_types_2_93_0.h"

struct bAction3_1_0;
struct CacheFileLayer3_1_0;
struct CacheFile3_1_0;
struct Curve3_1_0;
struct FileGlobal3_1_0;
struct LengthGpencilModifierData3_1_0;
struct ShrinkwrapGpencilModifierData3_1_0;
struct IDOverrideLibrary3_1_0;
struct Image_Runtime3_1_0;
struct Image3_1_0;
struct MVert3_1_0;
struct Mesh3_1_0;
struct MetaBall3_1_0;
struct ModifierData3_1_0;
struct SubsurfRuntimeData3_1_0;
struct CastModifierData3_1_0;
struct bNodeSocket3_1_0;
struct bNode3_1_0;
struct NodeConvertColorSpace3_1_0;
struct NodeMapRange3_1_0;
struct NodeAccumulateField3_1_0;
struct NodeGeometryExtrudeMesh3_1_0;
struct NodeGeometryCurvePrimitiveArc3_1_0;
struct NodeGeometryStringToCurves3_1_0;
struct NodeFunctionCompare3_1_0;
struct FFMpegCodecData3_1_0;
struct BakeData3_1_0;
struct RenderData3_1_0;
struct ARegion_Runtime3_1_0;
struct Sequence3_1_0;
struct TextVars3_1_0;
struct uiFont3_1_0;
struct uiFontStyle3_1_0;
struct UserDef_Experimental3_1_0;
struct UserDef3_1_0;
struct View3DOverlay3_1_0;
struct MappingInfoModifierData3_1_0;
struct SubsurfModifierData3_1_0;
struct LatticeModifierData3_1_0;
struct CurveModifierData3_1_0;
struct BuildModifierData3_1_0;
struct MaskModifierData3_1_0;
struct ArrayModifierData3_1_0;
struct MirrorModifierData3_1_0;
struct EdgeSplitModifierData3_1_0;
struct BevelModifierData3_1_0;
struct FluidModifierData3_1_0;
struct DisplaceModifierData3_1_0;
struct UVProjectModifierData3_1_0;
struct DecimateModifierData3_1_0;
struct SmoothModifierData3_1_0;
struct WaveModifierData3_1_0;
struct HookModifierData3_1_0;
struct SoftbodyModifierData3_1_0;
struct ClothModifierData3_1_0;
struct CollisionModifierData3_1_0;
struct SurfaceModifierData3_1_0;
struct BooleanModifierData3_1_0;
struct ParticleSystemModifierData3_1_0;
struct ParticleInstanceModifierData3_1_0;
struct ExplodeModifierData3_1_0;
struct MultiresModifierData3_1_0;
struct FluidsimModifierData3_1_0;
struct SmokeModifierData3_1_0;
struct ShrinkwrapModifierData3_1_0;
struct SimpleDeformModifierData3_1_0;
struct ShapeKeyModifierData3_1_0;
struct SolidifyModifierData3_1_0;
struct ScrewModifierData3_1_0;
struct OceanModifierData3_1_0;
struct WarpModifierData3_1_0;
struct WeightVGEditModifierData3_1_0;
struct WeightVGMixModifierData3_1_0;
struct WeightVGProximityModifierData3_1_0;
struct DynamicPaintModifierData3_1_0;
struct RemeshModifierData3_1_0;
struct SkinModifierData3_1_0;
struct TriangulateModifierData3_1_0;
struct LaplacianSmoothModifierData3_1_0;
struct UVWarpModifierData3_1_0;
struct MeshCacheModifierData3_1_0;
struct LaplacianDeformModifierData3_1_0;
struct WireframeModifierData3_1_0;
struct WeldModifierData3_1_0;
struct DataTransferModifierData3_1_0;
struct NormalEditModifierData3_1_0;
struct MeshSeqCacheModifierData3_1_0;
struct SurfaceDeformModifierData3_1_0;
struct WeightedNormalModifierData3_1_0;
struct NodesModifierData3_1_0;
struct MeshToVolumeModifierData3_1_0;
struct VolumeDisplaceModifierData3_1_0;
struct VolumeToMeshModifierData3_1_0;
struct Scene3_1_0;
struct ARegion3_1_0;
struct uiStyle3_1_0;
struct View3D3_1_0;

struct CacheFileLayer3_1_0 {
    CacheFileLayer3_1_0 *next, *prev;
    char filepath[1024];
    int flag;
    int _pad;
};

struct FileGlobal3_1_0 {
    char subvstr[4];
    short subversion;
    short minversion, minsubversion;
    char _pad[6];
    bScreen2_93_0 *curscreen;
    Scene3_1_0 *curscene;
    ViewLayer2_93_0 *cur_view_layer;
    void *_pad1;
    int fileflags;
    int globalf;
    uint64_t build_commit_timestamp;
    char build_hash[16];
    char filepath[1024];
};

struct Image_Runtime3_1_0 {
    void *cache_mutex;
    void *partial_update_register;
    void *partial_update_user;
};

struct MVert3_1_0 {
    float co[3];
    char flag, bweight;
    char _pad[2];
};

struct SubsurfRuntimeData3_1_0 {
    void *subdiv;
    char set_by_draw_code;
    char _pad[7];
};

struct NodeConvertColorSpace3_1_0 {
    char from_color_space[64];
    char to_color_space[64];
};

struct NodeMapRange3_1_0 {
    uint8_t data_type;
    uint8_t interpolation_type;
    uint8_t clamp;
    char _pad[5];
};

struct NodeAccumulateField3_1_0 {
    uint8_t data_type;
    uint8_t domain;
};

struct NodeGeometryExtrudeMesh3_1_0 {
    uint8_t mode;
};

struct NodeGeometryCurvePrimitiveArc3_1_0 {
    uint8_t mode;
};

struct NodeGeometryStringToCurves3_1_0 {
    uint8_t overflow;
    uint8_t align_x;
    uint8_t align_y;
    uint8_t pivot_mode;
};

struct NodeFunctionCompare3_1_0 {
    int8_t operation;
    int8_t data_type;
    int8_t mode;
    char _pad[1];
};

struct FFMpegCodecData3_1_0 {
    int type;
    int codec;
    int audio_codec;
    int video_bitrate;
    int audio_bitrate;
    int audio_mixrate;
    int audio_channels;
    float audio_volume;
    int gop_size;
    int max_b_frames;
    int flags;
    int constant_rate_factor;
    int ffmpeg_preset;
    int rc_min_rate;
    int rc_max_rate;
    int rc_buffer_size;
    int mux_packet_size;
    int mux_rate;
    void *_pad1;
};

struct TextVars3_1_0 {
    char text[512];
    VFont2_93_0 *text_font;
    int text_blf_id;
    float text_size;
    float color[4], shadow_color[4], box_color[4];
    float loc[2];
    float wrap_width;
    float box_margin;
    char flag;
    char align, align_y;
    char _pad[5];
};

struct uiFont3_1_0 {
    uiFont3_1_0 *next, *prev;
    char filepath[1024];
    short blf_id;
    short uifont_id;
};

struct uiFontStyle3_1_0 {
    short uifont_id;
    char _pad1[2];
    float points;
    short italic, bold;
    short shadow;
    short shadx, shady;
    char _pad0[2];
    float shadowalpha;
    float shadowcolor;
    char _pad2[4];
};

struct UserDef_Experimental3_1_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char no_proxy_to_override_conversion;
    char use_cycles_debug;
    char use_geometry_nodes_legacy;
    char show_asset_debug_info;
    char no_asset_indexing;
    char SANITIZE_AFTER_HERE;
    char use_new_hair_type;
    char use_new_point_cloud_type;
    char use_full_frame_compositor;
    char use_sculpt_vertex_colors;
    char use_sculpt_tools_tilt;
    char use_extended_asset_browser;
    char use_override_templates;
    char _pad[1];
};

struct View3DOverlay3_1_0 {
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
    char _pad[4];
};

struct uiStyle3_1_0 {
    uiStyle3_1_0 *next, *prev;
    char name[64];
    uiFontStyle3_1_0 paneltitle;
    uiFontStyle3_1_0 grouplabel;
    uiFontStyle3_1_0 widgetlabel;
    uiFontStyle3_1_0 widget;
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

struct bAction3_1_0 {
    ID3_0_0 id;
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

struct CacheFile3_1_0 {
    ID3_0_0 id;
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

struct Curve3_1_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 nurb;
    EditNurb2_93_0 *editnurb;
    Object3_0_0 *bevobj, *taperobj, *textoncurve;
    Ipo2_93_0 *ipo;
    Key2_93_0 *key;
    Material2_93_0 **mat;
    CurveProfile2_93_0 *bevel_profile;
    float loc[3];
    float size[3];
    short type;
    char texflag;
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
    VFont2_93_0 *vfont;
    VFont2_93_0 *vfontb;
    VFont2_93_0 *vfonti;
    VFont2_93_0 *vfontbi;
    TextBox2_93_0 *tb;
    int totbox, actbox;
    CharInfo2_93_0 *strinfo;
    CharInfo2_93_0 curinfo;
    float ctime;
    float bevfac1, bevfac2;
    char bevfac1_mapping, bevfac2_mapping;
    char _pad2[6];
    float fsize_realtime;
    void *curve_eval;
    void *batch_cache;
};

struct LengthGpencilModifierData3_1_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    int pass_index;
    int flag;
    int layer_pass;
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
};

struct ShrinkwrapGpencilModifierData3_1_0 {
    GpencilModifierData2_93_0 modifier;
    Object3_0_0 *target;
    Object3_0_0 *aux_target;
    Material2_93_0 *material;
    char layername[64];
    char vgname[64];
    int pass_index;
    int flag;
    int layer_pass;
    float keep_dist;
    short shrink_type;
    char shrink_opts;
    char shrink_mode;
    float proj_limit;
    char proj_axis;
    char subsurf_levels;
    char _pad[6];
    float smooth_factor;
    int smooth_step;
    void *cache_data;
};

struct IDOverrideLibrary3_1_0 {
    ID3_0_0 *reference;
    ListBase2_93_0 properties;
    ID3_0_0 *storage;
    void *runtime;
    void *_pad_0;
    unsigned int flag;
    char _pad_1[4];
};

struct Image3_1_0 {
    ID3_0_0 id;
    char filepath[1024];
    void *cache;
    void *gputexture[3][2][2];
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

struct Mesh3_1_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    Ipo2_93_0 *ipo;
    Key2_93_0 *key;
    Material2_93_0 **mat;
    MVert3_1_0 *mvert;
    MEdge2_93_0 *medge;
    MPoly2_93_0 *mpoly;
    MLoop2_93_0 *mloop;
    int totvert;
    int totedge;
    int totpoly;
    int totloop;
    CustomData3_0_0 vdata, edata, pdata, ldata;
    MDeformVert2_93_0 *dvert;
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    MLoopUV2_93_0 *mloopuv;
    MLoopCol2_93_0 *mloopcol;
    void *edit_mesh;
    MSelect2_93_0 *mselect;
    int totselect;
    int act_face;
    Mesh3_1_0 *texcomesh;
    float loc[3];
    float size[3];
    char texflag;
    char editflag;
    short flag;
    float smoothresh;
    char cd_flag;
    char symmetry;
    short totcol;
    char remesh_mode;
    char subdiv;
    char subdivr;
    char subsurftype;
    MTFace2_93_0 *mtface;
    TFace2_93_0 *tface;
    MCol2_93_0 *mcol;
    MFace2_93_0 *mface;
    CustomData3_0_0 fdata;
    int totface;
    float remesh_voxel_size;
    float remesh_voxel_adaptivity;
    int face_sets_color_seed;
    int face_sets_color_default;
    char _pad1[4];
    void *_pad2;
    Mesh_Runtime3_0_0 runtime;
};

struct MetaBall3_1_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 disp;
    ListBase2_93_0 *editelems;
    Ipo2_93_0 *ipo;
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
    void *batch_cache;
};

struct ModifierData3_1_0 {
    ModifierData3_1_0 *next, *prev;
    int type, mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
    SessionUUID2_93_0 session_uuid;
    void *runtime;
};

struct CastModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag;
    short type;
    void *_pad1;
};

struct bNodeSocket3_1_0 {
    bNodeSocket3_1_0 *next, *prev;
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
    void *cache;
    int own_index;
    int to_index;
    bNodeSocket3_1_0 *groupsock;
    bNodeLink2_93_0 *link;
    bNodeStack2_93_0 ns;
    const void *declaration;
    uint32_t changed_flag;
    char _pad[4];
};

struct bNode3_1_0 {
    bNode3_1_0 *next, *prev;
    IDProperty3_0_0 *prop;
    void *typeinfo;
    char idname[64];
    char name[64];
    int flag;
    short type;
    short done, level;
    uint8_t need_exec;
    char _pad2[5];
    uint32_t changed_flag;
    float color[3];
    ListBase2_93_0 inputs, outputs;
    bNode3_1_0 *parent;
    ID3_0_0 *id;
    void *storage;
    bNode3_1_0 *original;
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
    char branch_tag;
    char iter_flag;
    float ssr_id;
    float sss_id;
    void *declaration;
};

struct BakeData3_1_0 {
    ImageFormatData2_93_0 im_format;
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
    char _pad[5];
    Object3_0_0 *cage_object;
};

struct RenderData3_1_0 {
    ImageFormatData2_93_0 im_format;
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
    int line_thickness_mode;
    float unit_line_thickness;
    char engine[32];
    char _pad2[2];
    short perf_flag;
    BakeData3_1_0 bake;
    int _pad8;
    short preview_pixel_size;
    short _pad4;
    ListBase2_93_0 views;
    short actview;
    short views_format;
    short hair_type, hair_subdiv;
    CurveMapping2_93_0 mblur_shutter_curve;
};

struct ARegion_Runtime3_1_0 {
    const char *category;
    rcti2_93_0 visible_rect;
    int offset_x, offset_y;
    void *block_name_map;
};

struct Sequence3_1_0 {
    Sequence3_1_0 *next, *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag, type;
    int len;
    int start;
    int startofs, endofs;
    int startstill, endstill;
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
    Scene3_1_0 *scene;
    Object3_0_0 *scene_camera;
    MovieClip2_93_0 *clip;
    Mask2_93_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence3_1_0 *seq1, *seq2, *seq3;
    ListBase2_93_0 seqbase;
    bSound2_93_4 *sound;
    void *scene_sound;
    float volume;
    float pitch, pan;
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
    SequenceRuntime2_93_0 runtime;
};

struct UserDef3_1_0 {
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
    short opensubdiv_compute_type;
    short _pad6;
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
    UserDef_Experimental3_1_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct MappingInfoModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
};

struct SubsurfModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    short subdivType, levels, renderLevels, flags;
    short uv_smooth;
    short quality;
    short boundary_smooth;
    char _pad[2];
    void *emCache, *mCache;
};

struct LatticeModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct CurveModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
    void *_pad1;
};

struct BuildModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    float start, length;
    short flag;
    short randomize;
    int seed;
};

struct MaskModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
    void *_pad1;
};

struct ArrayModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct MirrorModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct EdgeSplitModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    float split_angle;
    int flags;
};

struct BevelModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct FluidModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    FluidDomainSettings3_0_0 *domain;
    FluidFlowSettings2_93_0 *flow;
    FluidEffectorSettings2_93_0 *effector;
    float time;
    int type;
    void *_pad1;
};

struct DisplaceModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
    float strength;
    int direction;
    char defgrp_name[64];
    float midlevel;
    int space;
    short flag;
    char _pad[6];
};

struct UVProjectModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *projectors[10];
    char _pad2[4];
    int num_projectors;
    float aspectx, aspecty;
    float scalex, scaley;
    char uvlayer_name[64];
    int uvlayer_tmp;
    char _pad[4];
};

struct DecimateModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct SmoothModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    float fac;
    char defgrp_name[64];
    short flag, repeat;
};

struct WaveModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
    Object3_0_0 *objectcenter;
    char defgrp_name[64];
    short flag;
    char _pad[2];
    float startx, starty, height, width;
    float narrow, speed, damp, falloff;
    float timeoffs, lifetime;
    char _pad1[4];
    void *_pad2;
};

struct HookModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    int totindex;
    float force;
    char name[64];
    void *_pad1;
};

struct SoftbodyModifierData3_1_0 {
    ModifierData3_1_0 modifier;
};

struct ClothModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct CollisionModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    MVert3_1_0 *x;
    MVert3_1_0 *xnew;
    MVert3_1_0 *xold;
    MVert3_1_0 *current_xnew;
    MVert3_1_0 *current_x;
    MVert3_1_0 *current_v;
    MVertTri2_93_0 *tri;
    unsigned int mvert_num;
    unsigned int tri_num;
    float time_x, time_xnew;
    char is_static;
    char _pad[7];
    void *bvhtree;
};

struct SurfaceModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    MVert3_1_0 *x;
    MVert3_1_0 *v;
    Mesh3_1_0 *mesh;
    void *bvhtree;
    int cfra, numverts;
};

struct BooleanModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *object;
    Collection3_0_0 *collection;
    float double_threshold;
    char operation;
    char solver;
    char flag;
    char bm_flag;
};

struct ParticleSystemModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    void *psys;
    Mesh3_1_0 *mesh_final;
    Mesh3_1_0 *mesh_original;
    int totdmvert, totdmedge, totdmface;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct ParticleInstanceModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Object3_0_0 *ob;
    short psys, flag, axis, space;
    float position, random_position;
    float rotation, random_rotation;
    float particle_amount, particle_offset;
    char index_layer_name[64];
    char value_layer_name[64];
    void *_pad1;
};

struct ExplodeModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    int *facepa;
    short flag, vgroup;
    float protect;
    char uvname[64];
    void *_pad1;
};

struct MultiresModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    char lvl, sculptlvl, renderlvl, totlvl;
    char simple;
    char flags, _pad[2];
    short quality;
    short uv_smooth;
    short boundary_smooth;
    char _pad2[2];
};

struct FluidsimModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    FluidsimSettings2_93_0 *fss;
    void *_pad1;
};

struct SmokeModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    int type;
    int _pad;
};

struct ShrinkwrapModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct SimpleDeformModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct ShapeKeyModifierData3_1_0 {
    ModifierData3_1_0 modifier;
};

struct SolidifyModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct ScrewModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct OceanModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    char foamlayername[64];
    char spraylayername[64];
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

struct WarpModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
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
    char _pad[6];
    void *_pad1;
};

struct WeightVGEditModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    char mask_tex_uvlayer_name[64];
    char _pad0[4];
    void *_pad1;
};

struct WeightVGMixModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    char mask_tex_uvlayer_name[64];
    char flag;
    char _pad1[3];
};

struct WeightVGProximityModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    char mask_tex_uvlayer_name[64];
    float min_dist, max_dist;
    short falloff_type;
    char _pad0[2];
};

struct DynamicPaintModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    DynamicPaintBrushSettings2_93_0 *brush;
    int type;
    char _pad[4];
};

struct RemeshModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct SkinModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct TriangulateModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    float lambda, lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag, repeat;
};

struct UVWarpModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    char uvlayer_name[64];
};

struct MeshCacheModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct LaplacianDeformModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    char anchor_grp_name[64];
    int total_verts, repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag, mat_ofs;
    char _pad[4];
};

struct WeldModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct DataTransferModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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
    int layers_select_src[4];
    int layers_select_dst[4];
    int mix_mode;
    float mix_factor;
    char defgrp_name[64];
    int flags;
    void *_pad2;
};

struct NormalEditModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct MeshSeqCacheModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    CacheFile3_1_0 *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
};

struct SurfaceDeformModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    void *depsgraph;
    Object3_0_0 *target;
    SDefVert3_0_0 *verts;
    float falloff;
    unsigned int num_mesh_verts, num_bind_verts, numpoly;
    int flags;
    float mat[4][4];
    float strength;
    char defgrp_name[64];
    void *_pad1;
};

struct WeightedNormalModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    char defgrp_name[64];
    char mode, flag;
    short weight;
    float thresh;
};

struct NodesModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    void *node_group;
    NodesModifierSettings2_93_0 settings;
    void *runtime_eval_log;
    void *_pad1;
};

struct MeshToVolumeModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct VolumeDisplaceModifierData3_1_0 {
    ModifierData3_1_0 modifier;
    Tex2_93_0 *texture;
    Object3_0_0 *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

struct VolumeToMeshModifierData3_1_0 {
    ModifierData3_1_0 modifier;
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

struct Scene3_1_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    Object3_0_0 *camera;
    World2_93_0 *world;
    Scene3_1_0 *set;
    ListBase2_93_0 base;
    Base2_93_0 *basact;
    void *_pad1;
    View3DCursor2_93_0 cursor;
    unsigned int lay;
    int layact;
    char _pad2[4];
    short flag;
    char use_nodes;
    char _pad3[1];
    void *nodetree;
    Editing3_0_0 *ed;
    ToolSettings3_0_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData3_1_0 r;
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
    MovieClip2_93_0 *clip;
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
    Collection3_0_0 *master_collection;
    SceneCollection2_93_0 *collection;
    IDProperty3_0_0 *layer_properties;
    void *_pad9;
    SceneDisplay2_93_0 display;
    SceneEEVEE2_93_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
};

struct ARegion3_1_0 {
    ARegion3_1_0 *next, *prev;
    View2D2_93_0 v2d;
    rcti2_93_0 winrct;
    rcti2_93_0 drawrct;
    short winx, winy;
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

struct View3D3_1_0 {
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
    View3D3_1_0 *localvd;
    char ob_center_bone[64];
    unsigned short local_view_uuid;
    char _pad6[2];
    int layact;
    unsigned short local_collections_uuid;
    short _pad7[3];
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
    View3DOverlay3_1_0 overlay;
    View3D_Runtime3_0_0 runtime;
};

#endif
