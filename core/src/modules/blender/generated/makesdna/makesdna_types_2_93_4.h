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

Various structs from Blender v2.93.4 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_2_93_4_H
#define MAKESDNA_TYPES_2_93_4_H

#include <stdint.h>
#include "makesdna_types_2_93_0.h"

struct bGPDstroke2_93_4;
struct Mesh2_93_4;
struct ModifierData2_93_4;
struct LatticeModifierData2_93_4;
struct CurveModifierData2_93_4;
struct MaskModifierData2_93_4;
struct MirrorModifierData2_93_4;
struct BevelModifierData2_93_4;
struct FluidModifierData2_93_4;
struct CastModifierData2_93_4;
struct WaveModifierData2_93_4;
struct HookModifierData2_93_4;
struct ParticleSystemModifierData2_93_4;
struct ParticleInstanceModifierData2_93_4;
struct ExplodeModifierData2_93_4;
struct FluidsimModifierData2_93_4;
struct SimpleDeformModifierData2_93_4;
struct ScrewModifierData2_93_4;
struct WarpModifierData2_93_4;
struct WeightVGEditModifierData2_93_4;
struct DataTransferModifierData2_93_4;
struct NormalEditModifierData2_93_4;
struct MeshSeqCacheModifierData2_93_4;
struct SurfaceDeformModifierData2_93_4;
struct MeshToVolumeModifierData2_93_4;
struct VolumeToMeshModifierData2_93_4;
struct Object_Runtime2_93_4;
struct Object2_93_4;
struct bSound2_93_4;
struct MappingInfoModifierData2_93_4;
struct SubsurfModifierData2_93_4;
struct BuildModifierData2_93_4;
struct ArrayModifierData2_93_4;
struct EdgeSplitModifierData2_93_4;
struct DisplaceModifierData2_93_4;
struct UVProjectModifierData2_93_4;
struct DecimateModifierData2_93_4;
struct SmoothModifierData2_93_4;
struct SoftbodyModifierData2_93_4;
struct ClothModifierData2_93_4;
struct CollisionModifierData2_93_4;
struct SurfaceModifierData2_93_4;
struct BooleanModifierData2_93_4;
struct MultiresModifierData2_93_4;
struct SmokeModifierData2_93_4;
struct ShrinkwrapModifierData2_93_4;
struct ShapeKeyModifierData2_93_4;
struct SolidifyModifierData2_93_4;
struct OceanModifierData2_93_4;
struct WeightVGMixModifierData2_93_4;
struct WeightVGProximityModifierData2_93_4;
struct DynamicPaintModifierData2_93_4;
struct RemeshModifierData2_93_4;
struct SkinModifierData2_93_4;
struct TriangulateModifierData2_93_4;
struct LaplacianSmoothModifierData2_93_4;
struct UVWarpModifierData2_93_4;
struct MeshCacheModifierData2_93_4;
struct LaplacianDeformModifierData2_93_4;
struct WireframeModifierData2_93_4;
struct WeldModifierData2_93_4;
struct WeightedNormalModifierData2_93_4;
struct NodesModifierData2_93_4;
struct VolumeDisplaceModifierData2_93_4;

struct bGPDstroke2_93_4 {
    bGPDstroke2_93_4 *next, *prev;
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
    float hardeness;
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
    bGPDstroke_Runtime2_93_0 runtime;
    void *_pad5;
};

struct Mesh2_93_4 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    Ipo2_93_0 *ipo;
    Key2_93_0 *key;
    Material2_93_0 **mat;
    MSelect2_93_0 *mselect;
    MPoly2_93_0 *mpoly;
    MLoop2_93_0 *mloop;
    MLoopUV2_93_0 *mloopuv;
    MLoopCol2_93_0 *mloopcol;
    MFace2_93_0 *mface;
    MTFace2_93_0 *mtface;
    TFace2_93_0 *tface;
    MVert2_93_0 *mvert;
    MEdge2_93_0 *medge;
    MDeformVert2_93_0 *dvert;
    MCol2_93_0 *mcol;
    Mesh2_93_4 *texcomesh;
    void *edit_mesh;
    CustomData2_93_0 vdata, edata, fdata;
    CustomData2_93_0 pdata, ldata;
    int totvert, totedge, totface, totselect;
    int totpoly, totloop;
    int attributes_active_index;
    int _pad3;
    int act_face;
    float loc[3];
    float size[3];
    short texflag, flag;
    float smoothresh;
    char cd_flag, _pad;
    char subdiv , subdivr;
    char subsurftype;
    char editflag;
    short totcol;
    float remesh_voxel_size;
    float remesh_voxel_adaptivity;
    char remesh_mode;
    char symmetry;
    char _pad1[2];
    int face_sets_color_seed;
    int face_sets_color_default;
    void *_pad2;
    Mesh_Runtime2_93_0 runtime;
};

struct ModifierData2_93_4 {
    ModifierData2_93_4 *next, *prev;
    int type, mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
    ModifierData2_93_4 *orig_modifier_data;
    SessionUUID2_93_0 session_uuid;
    void *runtime;
    void *_pad1;
};

struct LatticeModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct CurveModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
    void *_pad1;
};

struct MaskModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
    void *_pad1;
};

struct MirrorModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    short axis;
    short flag;
    float tolerance;
    float bisect_threshold;
    char _pad[4];
    float uv_offset[2];
    float uv_offset_copy[2];
    Object2_93_4 *mirror_ob;
    void *_pad1;
};

struct BevelModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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

struct FluidModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    FluidDomainSettings2_93_0 *domain;
    FluidFlowSettings2_93_0 *flow;
    FluidEffectorSettings2_93_0 *effector;
    float time;
    int type;
    void *_pad1;
};

struct CastModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag, type;
    void *_pad1;
};

struct WaveModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Tex2_93_0 *texture;
    Object2_93_4 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
    Object2_93_4 *objectcenter;
    char defgrp_name[64];
    short flag;
    char _pad[2];
    float startx, starty, height, width;
    float narrow, speed, damp, falloff;
    float timeoffs, lifetime;
    char _pad1[4];
    void *_pad2;
};

struct HookModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
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

struct ParticleSystemModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    void *psys;
    Mesh2_93_4 *mesh_final;
    Mesh2_93_4 *mesh_original;
    int totdmvert, totdmedge, totdmface;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct ParticleInstanceModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *ob;
    short psys, flag, axis, space;
    float position, random_position;
    float rotation, random_rotation;
    float particle_amount, particle_offset;
    char index_layer_name[64];
    char value_layer_name[64];
    void *_pad1;
};

struct ExplodeModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    int *facepa;
    short flag, vgroup;
    float protect;
    char uvname[64];
    void *_pad1;
};

struct FluidsimModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    FluidsimSettings2_93_0 *fss;
    void *_pad1;
};

struct SimpleDeformModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *origin;
    char vgroup_name[64];
    float factor;
    float limit[2];
    char mode;
    char axis;
    char deform_axis;
    char flag;
    void *_pad1;
};

struct ScrewModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *ob_axis;
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

struct WarpModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Tex2_93_0 *texture;
    Object2_93_4 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
    Object2_93_4 *object_from;
    Object2_93_4 *object_to;
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

struct WeightVGEditModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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
    Object2_93_4 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[64];
    char _pad0[4];
    void *_pad1;
};

struct DataTransferModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *ob_source;
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

struct NormalEditModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char defgrp_name[64];
    Object2_93_4 *target;
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

struct MeshSeqCacheModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    CacheFile2_93_0 *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
    MeshCacheVertexVelocity2_93_0 *vertex_velocities;
    int num_vertices;
    float velocity_delta;
    float last_lookup_time;
    int _pad1;
    void *_pad2;
};

struct SurfaceDeformModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    void *depsgraph;
    Object2_93_4 *target;
    SDefVert2_93_0 *verts;
    float falloff;
    unsigned int numverts, numpoly;
    int flags;
    float mat[4][4];
    float strength;
    char _pad[4];
    char defgrp_name[64];
    void *_pad1;
};

struct MeshToVolumeModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
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

struct VolumeToMeshModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
    float threshold;
    float adaptivity;
    uint32_t flag;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char grid_name[64];
    void *_pad1;
};

struct Object_Runtime2_93_4 {
    CustomData_MeshMasks2_93_0 last_data_mask;
    char last_need_mapping;
    char _pad0[3];
    float parent_display_origin[3];
    int select_id;
    char _pad1[3];
    char is_data_eval_owned;
    BoundBox2_93_0 *bb;
    ID2_93_0 *data_orig;
    ID2_93_0 *data_eval;
    void *geometry_set_eval;
    void *geometry_set_previews;
    Mesh2_93_4 *mesh_deform_eval;
    bGPdata2_93_0 *gpd_orig;
    bGPdata2_93_0 *gpd_eval;
    Mesh2_93_4 *object_as_temp_mesh;
    Curve2_93_0 *object_as_temp_curve;
    void *curve_cache;
    unsigned short local_collections_bits;
    short _pad2[3];
    void *_pad3;
};

struct Object2_93_4 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    void *sculpt;
    short type, partype;
    int par1, par2, par3;
    char parsubstr[64];
    Object2_93_4 *parent, *track;
    Object2_93_4 *proxy, *proxy_group, *proxy_from;
    Ipo2_93_0 *ipo;
    bAction2_93_0 *action;
    bAction2_93_0 *poselib;
    bPose2_93_0 *pose;
    void *data;
    bGPdata2_93_0 *gpd;
    bAnimVizSettings2_93_0 avs;
    bMotionPath2_93_0 *mpath;
    void *_pad0;
    ListBase2_93_0 constraintChannels;
    ListBase2_93_0 effect;
    ListBase2_93_0 defbase;
    ListBase2_93_0 modifiers;
    ListBase2_93_0 greasepencil_modifiers;
    ListBase2_93_0 fmaps;
    ListBase2_93_0 shader_fx;
    int mode;
    int restore_mode;
    Material2_93_0 **mat;
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
    float obmat[4][4];
    float parentinv[4][4];
    float constinv[4][4];
    float imat[4][4];
    float imat_ren[4][4];
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
    unsigned short actfmap;
    char _pad2[2];
    float color[4];
    short softflag;
    char restrictflag;
    char shapeflag;
    short shapenr;
    char _pad3[2];
    ListBase2_93_0 constraints;
    ListBase2_93_0 nlastrips;
    ListBase2_93_0 hooks;
    ListBase2_93_0 particlesystem;
    PartDeflect2_93_0 *pd;
    SoftBody2_93_0 *soft;
    Collection2_93_0 *instance_collection;
    FluidsimSettings2_93_0 *fluidsimSettings;
    ListBase2_93_0 pc_ids;
    RigidBodyOb2_93_0 *rigidbody_object;
    RigidBodyCon2_93_0 *rigidbody_constraint;
    float ima_ofs[2];
    ImageUser2_93_0 *iuser;
    char empty_image_visibility_flag;
    char empty_image_depth;
    char empty_image_flag;
    char _pad8[5];
    PreviewImage2_93_0 *preview;
    ObjectLineArt2_93_0 lineart;
    void *_pad9;
    Object_Runtime2_93_4 runtime;
};

struct bSound2_93_4 {
    ID2_93_0 id;
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
};

struct MappingInfoModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Tex2_93_0 *texture;
    Object2_93_4 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
};

struct SubsurfModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    short subdivType, levels, renderLevels, flags;
    short uv_smooth;
    short quality;
    short boundary_smooth;
    char _pad[2];
    void *emCache, *mCache;
};

struct BuildModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    float start, length;
    short flag;
    short randomize;
    int seed;
};

struct ArrayModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *start_cap;
    Object2_93_4 *end_cap;
    Object2_93_4 *curve_ob;
    Object2_93_4 *offset_ob;
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

struct EdgeSplitModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    float split_angle;
    int flags;
};

struct DisplaceModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Tex2_93_0 *texture;
    Object2_93_4 *map_object;
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

struct UVProjectModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *projectors[10];
    char _pad2[4];
    int num_projectors;
    float aspectx, aspecty;
    float scalex, scaley;
    char uvlayer_name[64];
    int uvlayer_tmp;
    char _pad[4];
};

struct DecimateModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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

struct SmoothModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    float fac;
    char defgrp_name[64];
    short flag, repeat;
};

struct SoftbodyModifierData2_93_4 {
    ModifierData2_93_4 modifier;
};

struct ClothModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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

struct CollisionModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    MVert2_93_0 *x;
    MVert2_93_0 *xnew;
    MVert2_93_0 *xold;
    MVert2_93_0 *current_xnew;
    MVert2_93_0 *current_x;
    MVert2_93_0 *current_v;
    MVertTri2_93_0 *tri;
    unsigned int mvert_num;
    unsigned int tri_num;
    float time_x, time_xnew;
    char is_static;
    char _pad[7];
    void *bvhtree;
};

struct SurfaceModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    MVert2_93_0 *x;
    MVert2_93_0 *v;
    Mesh2_93_4 *mesh;
    void *bvhtree;
    int cfra, numverts;
};

struct BooleanModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *object;
    Collection2_93_0 *collection;
    float double_threshold;
    char operation;
    char solver;
    char flag;
    char bm_flag;
};

struct MultiresModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char lvl, sculptlvl, renderlvl, totlvl;
    char simple;
    char flags, _pad[2];
    short quality;
    short uv_smooth;
    short boundary_smooth;
    char _pad2[2];
};

struct SmokeModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    int type;
    int _pad;
};

struct ShrinkwrapModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Object2_93_4 *target;
    Object2_93_4 *auxTarget;
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

struct ShapeKeyModifierData2_93_4 {
    ModifierData2_93_4 modifier;
};

struct SolidifyModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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

struct OceanModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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

struct WeightVGMixModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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
    Object2_93_4 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[64];
    char flag;
    char _pad1[3];
};

struct WeightVGProximityModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char defgrp_name[64];
    CurveMapping2_93_0 *cmap_curve;
    int proximity_mode;
    int proximity_flags;
    Object2_93_4 *proximity_ob_target;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex2_93_0 *mask_texture;
    Object2_93_4 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[64];
    float min_dist, max_dist;
    short falloff_type;
    char _pad0[2];
};

struct DynamicPaintModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    DynamicPaintBrushSettings2_93_0 *brush;
    int type;
    char _pad[4];
};

struct RemeshModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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

struct SkinModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct TriangulateModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    float lambda, lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag, repeat;
};

struct UVWarpModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char axis_u, axis_v;
    short flag;
    float center[2];
    float offset[2];
    float scale[2];
    float rotation;
    Object2_93_4 *object_src;
    char bone_src[64];
    Object2_93_4 *object_dst;
    char bone_dst[64];
    char vgroup_name[64];
    char uvlayer_name[64];
};

struct MeshCacheModifierData2_93_4 {
    ModifierData2_93_4 modifier;
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
    char _pad[7];
    float frame_start;
    float frame_scale;
    float eval_frame;
    float eval_time;
    float eval_factor;
    char filepath[1024];
};

struct LaplacianDeformModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char anchor_grp_name[64];
    int total_verts, repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag, mat_ofs;
    char _pad[4];
};

struct WeldModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct WeightedNormalModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    char defgrp_name[64];
    char mode, flag;
    short weight;
    float thresh;
};

struct NodesModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    void *node_group;
    NodesModifierSettings2_93_0 settings;
};

struct VolumeDisplaceModifierData2_93_4 {
    ModifierData2_93_4 modifier;
    Tex2_93_0 *texture;
    Object2_93_4 *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

#endif
