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

Various structs from Blender v3.0 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_3_0_0_H
#define MAKESDNA_TYPES_3_0_0_H

#include <stdint.h>
#include "makesdna_types_2_93_4.h"
#include "makesdna_types_2_93_0.h"

struct bPoseChannel3_0_0;
struct Bone3_0_0;
struct AssetFilterSettings3_0_0;
struct AssetMetaData3_0_0;
struct AssetLibraryReference3_0_0;
struct AssetHandle3_0_0;
struct BoidRuleAverageSpeed3_0_0;
struct BrushGpencilSettings3_0_0;
struct CacheObjectPath3_0_0;
struct CacheFile3_0_0;
struct Collection3_0_0;
struct bTransLikeConstraint3_0_0;
struct bRotLimitConstraint3_0_0;
struct Curve3_0_0;
struct CustomDataLayer3_0_0;
struct CustomData3_0_0;
struct Effect3_0_0;
struct BuildEff3_0_0;
struct Particle3_0_0;
struct FluidDomainSettings3_0_0;
struct LengthGpencilModifierData3_0_0;
struct DashGpencilModifierSegment3_0_0;
struct DashGpencilModifierData3_0_0;
struct OffsetGpencilModifierData3_0_0;
struct WeightProxGpencilModifierData3_0_0;
struct WeightAngleGpencilModifierData3_0_0;
struct LineartGpencilModifierData3_0_0;
struct bGPdata_Runtime3_0_0;
struct bGPdata3_0_0;
struct IDPropertyUIData3_0_0;
struct IDPropertyUIDataInt3_0_0;
struct IDPropertyUIDataFloat3_0_0;
struct IDPropertyUIDataString3_0_0;
struct IDPropertyUIDataID3_0_0;
struct IDProperty3_0_0;
struct ID3_0_0;
struct LibraryWeakReference3_0_0;
struct ImageUser3_0_0;
struct ImageTile_RuntimeTextureSlot3_0_0;
struct ImageTile_Runtime3_0_0;
struct ImageTile3_0_0;
struct Image3_0_0;
struct IpoCurve3_0_0;
struct Lattice3_0_0;
struct MaskLayer3_0_0;
struct MaterialLineArt3_0_0;
struct Mesh_Runtime3_0_0;
struct Mesh3_0_0;
struct MirrorModifierData3_0_0;
struct MeshCacheModifierData3_0_0;
struct MeshSeqCacheModifierData3_0_0;
struct SDefVert3_0_0;
struct SurfaceDeformModifierData3_0_0;
struct NodesModifierData3_0_0;
struct bNodeSocket3_0_0;
struct bNode3_0_0;
struct bNodeSocketValueTexture3_0_0;
struct bNodeSocketValueMaterial3_0_0;
struct NodeShaderPrincipled3_0_0;
struct NodeDenoise3_0_0;
struct NodeRandomValue3_0_0;
struct NodeAttributeVectorRotate3_0_0;
struct NodeAttributeCurveMap3_0_0;
struct NodeInputBool3_0_0;
struct NodeInputInt3_0_0;
struct NodeInputColor3_0_0;
struct NodeGeometryProximity3_0_0;
struct NodeAttributeConvert3_0_0;
struct NodeGeometrySubdivisionSurface3_0_0;
struct NodeSwitch3_0_0;
struct NodeGeometryCurveSplineType3_0_0;
struct NodeGeometrySetCurveHandlePositions3_0_0;
struct NodeGeometryCurveSetHandles3_0_0;
struct NodeGeometryCurveSelectHandles3_0_0;
struct NodeGeometryCurvePrimitiveLine3_0_0;
struct NodeGeometryCurvePrimitiveBezierSegment3_0_0;
struct NodeGeometryCurvePrimitiveCircle3_0_0;
struct NodeGeometryCurvePrimitiveQuad3_0_0;
struct NodeGeometryCurveResample3_0_0;
struct NodeGeometryCurveSubdivide3_0_0;
struct NodeGeometryCurveFillet3_0_0;
struct NodeGeometryCurveTrim3_0_0;
struct NodeGeometryCurveToPoints3_0_0;
struct NodeGeometryCurveSample3_0_0;
struct NodeGeometryAttributeTransfer3_0_0;
struct NodeGeometryTransferAttribute3_0_0;
struct NodeGeometryRaycast3_0_0;
struct NodeGeometryCurveFill3_0_0;
struct NodeGeometryMeshToPoints3_0_0;
struct NodeGeometryAttributeCapture3_0_0;
struct NodeGeometryStringToCurves3_0_0;
struct NodeGeometryDeleteGeometry3_0_0;
struct NodeGeometrySeparateGeometry3_0_0;
struct NodeGeometryImageTexture3_0_0;
struct NodeGeometryViewer3_0_0;
struct EffectorWeights3_0_0;
struct SoftBody3_0_0;
struct Object_Runtime3_0_0;
struct Object3_0_0;
struct BoidParticle3_0_0;
struct ChildParticle3_0_0;
struct ParticleDupliWeight3_0_0;
struct ParticleSettings3_0_0;
struct RenderData3_0_0;
struct ParticleEditSettings3_0_0;
struct SequencerToolSettings3_0_0;
struct ToolSettings3_0_0;
struct PhysicsSettings3_0_0;
struct Panel_Runtime3_0_0;
struct uiList3_0_0;
struct StripElem3_0_0;
struct StripTransform3_0_0;
struct StripColorBalance3_0_0;
struct Sequence3_0_0;
struct EditingRuntime3_0_0;
struct Editing3_0_0;
struct SpeedControlVars3_0_0;
struct SequencerPreviewOverlay3_0_0;
struct SequencerTimelineOverlay3_0_0;
struct SpaceSeqRuntime3_0_0;
struct SpaceSeq3_0_0;
struct FileSelectParams3_0_0;
struct FileAssetSelectParams3_0_0;
struct FileDirEntryArr3_0_0;
struct SpaceImage3_0_0;
struct SpaceNodeOverlay3_0_0;
struct SpaceNode3_0_0;
struct SpreadsheetColumn3_0_0;
struct SpaceSpreadsheet3_0_0;
struct SpreadsheetRowFilter3_0_0;
struct MTex3_0_0;
struct TexMapping3_0_0;
struct MovieTrackingDopesheetChannel3_0_0;
struct uiFontStyle3_0_0;
struct ThemeUI3_0_0;
struct ThemeSpace3_0_0;
struct ThemeStripColor3_0_0;
struct bTheme3_0_0;
struct UserDef_Experimental3_0_0;
struct UserDef3_0_0;
struct bUUID3_0_0;
struct RegionView3D3_0_0;
struct View3DOverlay3_0_0;
struct View3D_Runtime3_0_0;
struct wmWindowManager3_0_0;
struct bToolRef_Runtime3_0_0;
struct WorkSpace3_0_0;
struct XrSessionSettings3_0_0;
struct XrActionMapBinding3_0_0;
struct XrActionMapItem3_0_0;
struct XrActionMap3_0_0;
struct bAction3_0_0;
struct IdAdtTemplate3_0_0;
struct bArmature3_0_0;
struct Brush3_0_0;
struct Palette3_0_0;
struct PaintCurve3_0_0;
struct CameraBGImage3_0_0;
struct Camera3_0_0;
struct Library3_0_0;
struct Ipo3_0_0;
struct Key3_0_0;
struct LightProbe3_0_0;
struct Light3_0_0;
struct FreestyleLineStyle3_0_0;
struct Mask3_0_0;
struct Material3_0_0;
struct MetaBall3_0_0;
struct MovieClip3_0_0;
struct NodeTexBase3_0_0;
struct NodeTexSky3_0_0;
struct NodeTexImage3_0_0;
struct NodeTexChecker3_0_0;
struct NodeTexBrick3_0_0;
struct NodeTexEnvironment3_0_0;
struct NodeTexGradient3_0_0;
struct NodeTexNoise3_0_0;
struct NodeTexVoronoi3_0_0;
struct NodeTexMusgrave3_0_0;
struct NodeTexWave3_0_0;
struct NodeTexMagic3_0_0;
struct NodeShaderTexPointDensity3_0_0;
struct NodeCryptomatte3_0_0;
struct Scene3_0_0;
struct bScreen3_0_0;
struct Panel3_0_0;
struct ColorBalanceModifierData3_0_0;
struct Simulation3_0_0;
struct bSound3_0_0;
struct Script3_0_0;
struct Speaker3_0_0;
struct Tex3_0_0;
struct Text3_0_0;
struct uiStyle3_0_0;
struct VFont3_0_0;
struct View3D3_0_0;
struct Volume3_0_0;
struct wmXrData3_0_0;
struct World3_0_0;

struct AssetLibraryReference3_0_0 {
    short type;
    char _pad1[2];
    int custom_library_index;
};

struct AssetHandle3_0_0 {
    const FileDirEntry2_93_0 *file_data;
};

struct BrushGpencilSettings3_0_0 {
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
    short fill_leak;
    int8_t caps_type;
    char _pad;
    int flag2;
    int fill_simplylvl;
    int fill_draw_mode;
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
    Material3_0_0 *material;
};

struct CacheObjectPath3_0_0 {
    CacheObjectPath3_0_0 *next, *prev;
    char path[4096];
};

struct bTransLikeConstraint3_0_0 {
    Object3_0_0 *tar;
    int flag;
    char mix_mode;
    char _pad[3];
    char subtarget[64];
};

struct bRotLimitConstraint3_0_0 {
    float xmin, xmax;
    float ymin, ymax;
    float zmin, zmax;
    short flag;
    short flag2;
    char euler_order;
    char _pad[3];
};

struct CustomDataLayer3_0_0 {
    int type;
    int offset;
    int flag;
    int active;
    int active_rnd;
    int active_clone;
    int active_mask;
    int uid;
    char name[64];
    void *data;
    const void *anonymous_id;
};

struct CustomData3_0_0 {
    CustomDataLayer3_0_0 *layers;
    int typemap[52];
    char _pad[4];
    int totlayer, maxlayer;
    int totsize;
    void *pool;
    CustomDataExternal2_93_0 *external;
};

struct Effect3_0_0 {
    Effect3_0_0 *next, *prev;
    short type, flag, buttype;
    char _pad0[2];
};

struct BuildEff3_0_0 {
    BuildEff3_0_0 *next, *prev;
    short type, flag, buttype;
    char _pad0[2];
    float len, sfra;
};

struct Particle3_0_0 {
    float co[3], no[3];
    float time, lifetime;
    short mat_nr;
    char _pad0[2];
};

struct DashGpencilModifierSegment3_0_0 {
    char name[64];
    DashGpencilModifierData3_0_0 *dmd;
    int dash;
    int gap;
    float radius;
    float opacity;
    int mat_nr;
    int _pad;
};

struct bGPdata_Runtime3_0_0 {
    void *sbuffer;
    void *sbuffer_stroke_batch;
    void *sbuffer_fill_batch;
    bGPDstroke2_93_4 *sbuffer_gps;
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
    bGPDcontrolpoint2_93_0 *cp_points;
    Brush3_0_0 *sbuffer_brush;
    void *gpencil_cache;
    void *lineart_cache;
};

struct IDPropertyUIData3_0_0 {
    char *description;
    int rna_subtype;
    char _pad[4];
};

struct ID3_0_0 {
    void *next, *prev;
    ID3_0_0 *newid;
    Library3_0_0 *lib;
    AssetMetaData3_0_0 *asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    int recalc;
    int recalc_up_to_undo_push;
    int recalc_after_undo_push;
    unsigned int session_uuid;
    IDProperty3_0_0 *properties;
    IDOverrideLibrary2_93_0 *override_library;
    ID3_0_0 *orig_id;
    void *py_instance;
    LibraryWeakReference3_0_0 *library_weak_reference;
};

struct LibraryWeakReference3_0_0 {
    char library_filepath[1024];
    char library_id_name[66];
    char _pad[2];
};

struct ImageUser3_0_0 {
    Scene3_0_0 *scene;
    int framenr;
    int frames;
    int offset, sfra;
    char cycl;
    char multiview_eye;
    short pass;
    int tile;
    short multi_index, view, layer;
    short flag;
};

struct ImageTile_RuntimeTextureSlot3_0_0 {
    int tilearray_layer;
    int _pad;
    int tilearray_offset[2];
    int tilearray_size[2];
};

struct MaterialLineArt3_0_0 {
    int flags;
    unsigned char material_mask_bits;
    unsigned char mat_occlusion;
    unsigned char _pad[2];
};

struct SDefVert3_0_0 {
    SDefBind2_93_0 *binds;
    unsigned int numbinds;
    unsigned int vertex_idx;
};

struct bNodeSocketValueTexture3_0_0 {
    Tex3_0_0 *value;
};

struct bNodeSocketValueMaterial3_0_0 {
    Material3_0_0 *value;
};

struct NodeShaderPrincipled3_0_0 {
    char use_subsurface_auto_radius;
    char _pad[3];
};

struct NodeDenoise3_0_0 {
    char hdr;
    char prefilter;
};

struct NodeRandomValue3_0_0 {
    uint8_t data_type;
};

struct NodeAttributeVectorRotate3_0_0 {
    uint8_t mode;
    uint8_t input_type_vector;
    uint8_t input_type_center;
    uint8_t input_type_axis;
    uint8_t input_type_angle;
    uint8_t input_type_rotation;
    char _pad[2];
};

struct NodeAttributeCurveMap3_0_0 {
    uint8_t data_type;
    char _pad[7];
    CurveMapping2_93_0 *curve_vec;
    CurveMapping2_93_0 *curve_rgb;
};

struct NodeInputBool3_0_0 {
    uint8_t boolean;
};

struct NodeInputInt3_0_0 {
    int integer;
};

struct NodeInputColor3_0_0 {
    float color[4];
};

struct NodeGeometryProximity3_0_0 {
    uint8_t target_element;
};

struct NodeAttributeConvert3_0_0 {
    int8_t data_type;
    int8_t domain;
};

struct NodeGeometrySubdivisionSurface3_0_0 {
    uint8_t uv_smooth;
    uint8_t boundary_smooth;
};

struct NodeSwitch3_0_0 {
    uint8_t input_type;
};

struct NodeGeometryCurveSplineType3_0_0 {
    uint8_t spline_type;
};

struct NodeGeometrySetCurveHandlePositions3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurveSetHandles3_0_0 {
    uint8_t handle_type;
    uint8_t mode;
};

struct NodeGeometryCurveSelectHandles3_0_0 {
    uint8_t handle_type;
    uint8_t mode;
};

struct NodeGeometryCurvePrimitiveLine3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurvePrimitiveBezierSegment3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurvePrimitiveCircle3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurvePrimitiveQuad3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurveResample3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurveSubdivide3_0_0 {
    uint8_t cuts_type;
};

struct NodeGeometryCurveFillet3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurveTrim3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurveToPoints3_0_0 {
    uint8_t mode;
};

struct NodeGeometryCurveSample3_0_0 {
    uint8_t mode;
};

struct NodeGeometryAttributeTransfer3_0_0 {
    int8_t domain;
    uint8_t mapping;
};

struct NodeGeometryTransferAttribute3_0_0 {
    int8_t data_type;
    int8_t domain;
    uint8_t mode;
    char _pad[1];
};

struct NodeGeometryRaycast3_0_0 {
    uint8_t mapping;
    int8_t data_type;
    uint8_t input_type_ray_direction;
    uint8_t input_type_ray_length;
};

struct NodeGeometryCurveFill3_0_0 {
    uint8_t mode;
};

struct NodeGeometryMeshToPoints3_0_0 {
    uint8_t mode;
};

struct NodeGeometryAttributeCapture3_0_0 {
    int8_t data_type;
    int8_t domain;
};

struct NodeGeometryStringToCurves3_0_0 {
    uint8_t overflow;
    uint8_t align_x;
    uint8_t align_y;
    char _pad[1];
};

struct NodeGeometryDeleteGeometry3_0_0 {
    int8_t domain;
    int8_t mode;
};

struct NodeGeometrySeparateGeometry3_0_0 {
    int8_t domain;
};

struct NodeGeometryImageTexture3_0_0 {
    int interpolation;
    int extension;
};

struct NodeGeometryViewer3_0_0 {
    int8_t data_type;
};

struct EffectorWeights3_0_0 {
    Collection3_0_0 *group;
    float weight[14];
    float global_gravity;
    short flag;
    char _pad[2];
};

struct ChildParticle3_0_0 {
    int num;
    int parent;
    int pa[4];
    float w[4];
    float fuv[4], foffset;
    char _pad0[4];
};

struct ParticleDupliWeight3_0_0 {
    ParticleDupliWeight3_0_0 *next, *prev;
    Object3_0_0 *ob;
    short count;
    short flag;
    short index;
    char _pad0[2];
};

struct SequencerToolSettings3_0_0 {
    int fit_method;
    short snap_mode;
    short snap_flag;
    int overlap_mode;
    int snap_distance;
    int pivot_point;
};

struct PhysicsSettings3_0_0 {
    float gravity[3];
    int flag, quick_cache_step;
    char _pad0[4];
};

struct Panel_Runtime3_0_0 {
    int region_ofsx;
    char _pad[4];
    void *custom_data_ptr;
    void *block;
    void *context;
};

struct uiList3_0_0 {
    uiList3_0_0 *next, *prev;
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
    const char *custom_activate_opname;
    const char *custom_drag_opname;
    IDProperty3_0_0 *properties;
    uiListDyn2_93_0 *dyn_data;
};

struct StripElem3_0_0 {
    char name[256];
    int orig_width, orig_height;
    float orig_fps;
};

struct StripTransform3_0_0 {
    int xofs;
    int yofs;
    float scale_x;
    float scale_y;
    float rotation;
    float origin[2];
};

struct StripColorBalance3_0_0 {
    int method;
    float lift[3];
    float gamma[3];
    float gain[3];
    float slope[3];
    float offset[3];
    float power[3];
    int flag;
    char _pad[4];
};

struct EditingRuntime3_0_0 {
    void *sequence_lookup;
};

struct SpeedControlVars3_0_0 {
    float *frameMap;
    float globalSpeed;
    int flags;
    int speed_control_type;
    float speed_fader;
    float speed_fader_length;
    float speed_fader_frame_number;
};

struct SequencerPreviewOverlay3_0_0 {
    int flag;
    char _pad0[4];
};

struct SequencerTimelineOverlay3_0_0 {
    int flag;
    char _pad0[4];
};

struct FileSelectParams3_0_0 {
    char title[96];
    char dir[1090];
    char file[256];
    char renamefile[256];
    short rename_flag;
    char _pad[4];
    const ID3_0_0 *rename_id;
    void *_pad3;
    char filter_glob[256];
    char filter_search[64];
    uint64_t filter_id;
    int active_file;
    int highlight_file;
    int sel_first;
    int sel_last;
    unsigned short thumbnail_size;
    char _pad1[2];
    short type;
    short flag;
    short sort;
    short display;
    char details_flags;
    char _pad2[3];
    int filter;
    short recursion_level;
    char _pad4[2];
};

struct SpaceNodeOverlay3_0_0 {
    int flag;
};

struct SpreadsheetColumn3_0_0 {
    SpreadsheetColumn3_0_0 *next, *prev;
    SpreadsheetColumnID2_93_0 *id;
    uint8_t data_type;
    char _pad0[7];
    char *display_name;
};

struct SpreadsheetRowFilter3_0_0 {
    SpreadsheetRowFilter3_0_0 *next, *prev;
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

struct MTex3_0_0 {
    short texco, mapto, maptoneg, blendtype;
    Object3_0_0 *object;
    Tex3_0_0 *tex;
    char uvname[64];
    char projx, projy, projz, mapping;
    char brush_map_mode, brush_angle_mode;
    char _pad[2];
    float ofs[3], size[3], rot, random_angle;
    char _pad0[2];
    short colormodel;
    short normapspace, which_output;
    float r, g, b, k;
    float def_var;
    float colfac, varfac;
    float norfac, dispfac, warpfac;
    float colspecfac, mirrfac, alphafac;
    float difffac, specfac, emitfac, hardfac;
    float raymirrfac, translfac, ambfac;
    float colemitfac, colreflfac, coltransfac;
    float densfac, scatterfac, reflfac;
    float timefac, lengthfac, clumpfac, dampfac;
    float kinkfac, kinkampfac, roughfac, padensfac, gravityfac;
    float lifefac, sizefac, ivelfac, fieldfac;
    float twistfac;
    float shadowfac;
    float zenupfac, zendownfac, blendfac;
};

struct TexMapping3_0_0 {
    float loc[3];
    float rot[3];
    float size[3];
    int flag;
    char projx, projy, projz, mapping;
    int type;
    float mat[4][4];
    float min[3], max[3];
    Object3_0_0 *ob;
};

struct MovieTrackingDopesheetChannel3_0_0 {
    MovieTrackingDopesheetChannel3_0_0 *next, *prev;
    MovieTrackingTrack2_93_0 *track;
    char _pad[4];
    char name[64];
    int tot_segment;
    int *segments;
    int max_segment, total_frames;
    int first_not_disabled_marker_framenr, last_not_disabled_marker_framenr;
};

struct uiFontStyle3_0_0 {
    short uifont_id;
    short points;
    short italic, bold;
    short shadow;
    short shadx, shady;
    char _pad0[2];
    float shadowalpha;
    float shadowcolor;
};

struct ThemeStripColor3_0_0 {
    unsigned char color[4];
};

struct UserDef_Experimental3_0_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char no_proxy_to_override_conversion;
    char use_cycles_debug;
    char use_geometry_nodes_legacy;
    char show_asset_debug_info;
    char SANITIZE_AFTER_HERE;
    char use_new_hair_type;
    char use_new_point_cloud_type;
    char use_full_frame_compositor;
    char use_sculpt_vertex_colors;
    char use_sculpt_tools_tilt;
    char use_extended_asset_browser;
    char use_override_templates;
    char _pad[2];
};

struct bUUID3_0_0 {
    uint32_t time_low;
    uint16_t time_mid;
    uint16_t time_hi_and_version;
    uint8_t clock_seq_hi_and_reserved;
    uint8_t clock_seq_low;
    uint8_t node[6];
};

struct RegionView3D3_0_0 {
    float winmat[4][4];
    float viewmat[4][4];
    float viewinv[4][4];
    float persmat[4][4];
    float persinv[4][4];
    float viewcamtexcofac[4];
    float viewmatob[4][4];
    float persmatob[4][4];
    float clip[6][4];
    float clip_local[6][4];
    BoundBox2_93_0 *clipbb;
    RegionView3D3_0_0 *localvd;
    void *render_engine;
    void *sms;
    void *smooth_timer;
    float twmat[4][4];
    float tw_axis_min[3], tw_axis_max[3];
    float tw_axis_matrix[3][3];
    float gridview;
    float viewquat[4];
    float dist;
    float camdx, camdy;
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

struct View3DOverlay3_0_0 {
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

struct View3D_Runtime3_0_0 {
    void *properties_storage;
    int flag;
    char _pad1[4];
    void *local_stats;
};

struct bToolRef_Runtime3_0_0 {
    int cursor;
    char keymap[64];
    char gizmo_group[64];
    char data_block[64];
    char keymap_fallback[64];
    char op[64];
    int index;
    int flag;
};

struct XrActionMapBinding3_0_0 {
    XrActionMapBinding3_0_0 *next, *prev;
    char name[64];
    char profile[256];
    char component_path0[192];
    char component_path1[192];
    float float_threshold;
    short axis_flag;
    char _pad[2];
    float pose_location[3];
    float pose_rotation[3];
};

struct IDPropertyUIDataInt3_0_0 {
    IDPropertyUIData3_0_0 base;
    int *default_array;
    int default_array_len;
    char _pad[4];
    int min;
    int max;
    int soft_min;
    int soft_max;
    int step;
    int default_value;
};

struct IDPropertyUIDataFloat3_0_0 {
    IDPropertyUIData3_0_0 base;
    double *default_array;
    int default_array_len;
    char _pad[4];
    float step;
    int precision;
    double min;
    double max;
    double soft_min;
    double soft_max;
    double default_value;
};

struct IDPropertyUIDataString3_0_0 {
    IDPropertyUIData3_0_0 base;
    char *default_value;
};

struct IDPropertyUIDataID3_0_0 {
    IDPropertyUIData3_0_0 base;
};

struct ImageTile_Runtime3_0_0 {
    ImageTile_RuntimeTextureSlot3_0_0 slots[2];
};

struct FileAssetSelectParams3_0_0 {
    FileSelectParams3_0_0 base_params;
    AssetLibraryReference3_0_0 asset_library_ref;
    short asset_catalog_visibility;
    char _pad[6];
    bUUID3_0_0 catalog_id;
    short import_type;
    char _pad2[6];
};

struct IdAdtTemplate3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
};

struct PaintCurve3_0_0 {
    ID3_0_0 id;
    PaintCurvePoint2_93_0 *points;
    int tot_points;
    int add_index;
};

struct Library3_0_0 {
    ID3_0_0 id;
    void *filedata;
    char filepath[1024];
    char filepath_abs[1024];
    Library3_0_0 *parent;
    PackedFile2_93_0 *packedfile;
    int temp_index;
    short versionfile, subversionfile;
};

struct LightProbe3_0_0 {
    ID3_0_0 id;
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
    Image3_0_0 *image;
    Collection3_0_0 *visibility_grp;
    float distfalloff, distgridinf;
    char _pad[8];
};

struct Light3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    short type, flag;
    int mode;
    float r, g, b, k;
    float shdwr, shdwg, shdwb, shdwpad;
    float energy, dist, spotsize, spotblend;
    float att1, att2;
    float coeff_const, coeff_lin, coeff_quad;
    char _pad0[4];
    CurveMapping2_93_0 *curfalloff;
    short falloff_type;
    char _pad2[2];
    float clipsta, clipend;
    float bias;
    float soft;
    float bleedbias;
    float bleedexp;
    short bufsize, samp, buffers, filtertype;
    char bufflag, buftype;
    short area_shape;
    float area_size, area_sizey, area_sizez;
    float area_spread;
    float sun_angle;
    short texact, shadhalostep;
    Ipo3_0_0 *ipo;
    short pr_texture, use_nodes;
    char _pad6[4];
    float cascade_max_dist;
    float cascade_exponent;
    float cascade_fade;
    int cascade_count;
    float contact_dist;
    float contact_bias;
    float contact_spread;
    float contact_thickness;
    float diff_fac, volume_fac;
    float spec_fac, att_dist;
    PreviewImage2_93_0 *preview;
    void *nodetree;
};

struct Simulation3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    void *nodetree;
    uint32_t flag;
    char _pad[4];
};

struct bSound3_0_0 {
    ID3_0_0 id;
    char filepath[1024];
    PackedFile2_93_0 *packedfile;
    void *handle;
    PackedFile2_93_0 *newpackedfile;
    Ipo3_0_0 *ipo;
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

struct Script3_0_0 {
    ID3_0_0 id;
    void *py_draw;
    void *py_event;
    void *py_button;
    void *py_browsercallback;
    void *py_globaldict;
    int flags, lastspace;
    char scriptname[1024];
    char scriptarg[256];
};

struct Speaker3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    bSound3_0_0 *sound;
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

struct Tex3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
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
    int frames, offset, sfra;
    float checkerdist, nabla;
    char _pad1[4];
    ImageUser3_0_0 iuser;
    void *nodetree;
    Ipo3_0_0 *ipo;
    Image3_0_0 *ima;
    ColorBand2_93_0 *coba;
    PreviewImage2_93_0 *preview;
    char use_nodes;
    char _pad[7];
};

struct uiStyle3_0_0 {
    uiStyle3_0_0 *next, *prev;
    char name[64];
    uiFontStyle3_0_0 paneltitle;
    uiFontStyle3_0_0 grouplabel;
    uiFontStyle3_0_0 widgetlabel;
    uiFontStyle3_0_0 widget;
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

struct VFont3_0_0 {
    ID3_0_0 id;
    char filepath[1024];
    void *data;
    PackedFile2_93_0 *packedfile;
    PackedFile2_93_0 *temp_pf;
};

struct bPoseChannel3_0_0 {
    bPoseChannel3_0_0 *next, *prev;
    IDProperty3_0_0 *prop;
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
    Bone3_0_0 *bone;
    bPoseChannel3_0_0 *parent;
    bPoseChannel3_0_0 *child;
    ListBase2_93_0 iktree;
    ListBase2_93_0 siktree;
    bMotionPath2_93_0 *mpath;
    Object3_0_0 *custom;
    bPoseChannel3_0_0 *custom_tx;
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
    bPoseChannel3_0_0 *bbone_prev;
    bPoseChannel3_0_0 *bbone_next;
    void *temp;
    bPoseChannelDrawData2_93_0 *draw_data;
    bPoseChannel3_0_0 *orig_pchan;
    bPoseChannel_Runtime2_93_0 runtime;
};

struct Bone3_0_0 {
    Bone3_0_0 *next, *prev;
    IDProperty3_0_0 *prop;
    Bone3_0_0 *parent;
    ListBase2_93_0 childbase;
    char name[64];
    float roll;
    float head[3];
    float tail[3];
    float bone_mat[3][3];
    int flag;
    char inherit_scale_mode;
    char _pad[7];
    float arm_head[3];
    float arm_tail[3];
    float arm_mat[4][4];
    float arm_roll;
    float dist, weight;
    float xwidth, length, zwidth;
    float rad_head, rad_tail;
    float roll1, roll2;
    float curve_in_x, curve_in_z;
    float curve_out_x, curve_out_z;
    float ease1, ease2;
    float scale_in_x , scale_in_z;
    float scale_out_x , scale_out_z;
    float scale_in[3], scale_out[3];
    float size[3];
    int layer;
    short segments;
    char bbone_prev_type;
    char bbone_next_type;
    int bbone_flag;
    short bbone_prev_flag;
    short bbone_next_flag;
    Bone3_0_0 *bbone_prev;
    Bone3_0_0 *bbone_next;
};

struct AssetFilterSettings3_0_0 {
    ListBase2_93_0 tags;
    uint64_t id_types;
};

struct AssetMetaData3_0_0 {
    void *local_type_info;
    IDProperty3_0_0 *properties;
    bUUID3_0_0 catalog_id;
    char catalog_simple_name[64];
    char *author;
    char *description;
    ListBase2_93_0 tags;
    short active_tag;
    short tot_tags;
    char _pad[4];
};

struct BoidRuleAverageSpeed3_0_0 {
    BoidRule2_93_0 rule;
    float wander, level, speed;
    char _pad0[4];
};

struct CacheFile3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 object_paths;
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
    char _pad2[7];
    char velocity_unit;
    char velocity_name[64];
    void *handle;
    char handle_filepath[1024];
    void *handle_readers;
};

struct Collection3_0_0 {
    ID3_0_0 id;
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
    char _pad[6];
    int16_t color_tag;
    ListBase2_93_0 object_cache;
    ListBase2_93_0 object_cache_instanced;
    ListBase2_93_0 parents;
    SceneCollection2_93_0 *collection;
    ViewLayer2_93_0 *view_layer;
};

struct Curve3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 nurb;
    EditNurb2_93_0 *editnurb;
    Object3_0_0 *bevobj, *taperobj, *textoncurve;
    Ipo3_0_0 *ipo;
    Key3_0_0 *key;
    Material3_0_0 **mat;
    CurveProfile2_93_0 *bevel_profile;
    float loc[3];
    float size[3];
    short type;
    short texflag;
    char _pad0[6];
    short twist_mode;
    float twist_smooth, smallcaps_scale;
    int pathlen;
    short bevresol, totcol;
    int flag;
    float width, ext1, ext2;
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
    VFont3_0_0 *vfont;
    VFont3_0_0 *vfontb;
    VFont3_0_0 *vfonti;
    VFont3_0_0 *vfontbi;
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

struct FluidDomainSettings3_0_0 {
    FluidModifierData2_93_4 *fmd;
    void *fluid;
    void *fluid_old;
    void *fluid_mutex;
    Collection3_0_0 *fluid_group;
    Collection3_0_0 *force_group;
    Collection3_0_0 *effector_group;
    void *tex_density;
    void *tex_color;
    void *tex_wt;
    void *tex_shadow;
    void *tex_flame;
    void *tex_flame_coba;
    void *tex_coba;
    void *tex_field;
    void *tex_velocity_x;
    void *tex_velocity_y;
    void *tex_velocity_z;
    void *tex_flags;
    void *tex_range_field;
    Object3_0_0 *guide_parent;
    EffectorWeights3_0_0 *effector_weights;
    float p0[3];
    float p1[3];
    float dp0[3];
    float cell_size[3];
    float global_size[3];
    float prev_loc[3];
    int shift[3];
    float shift_f[3];
    float obj_shift_f[3];
    float imat[4][4];
    float obmat[4][4];
    float fluidmat[4][4];
    float fluidmat_wt[4][4];
    int base_res[3];
    int res_min[3];
    int res_max[3];
    int res[3];
    int total_cells;
    float dx;
    float scale;
    int boundary_width;
    float gravity_final[3];
    int adapt_margin;
    int adapt_res;
    float adapt_threshold;
    int maxres;
    int solver_res;
    int border_collisions;
    int flags;
    float gravity[3];
    int active_fields;
    short type;
    char _pad2[6];
    float alpha;
    float beta;
    int diss_speed;
    float vorticity;
    float active_color[3];
    int highres_sampling;
    float burning_rate, flame_smoke, flame_vorticity;
    float flame_ignition, flame_max_temp;
    float flame_smoke_color[3];
    float noise_strength;
    float noise_pos_scale;
    float noise_time_anim;
    int res_noise[3];
    int noise_scale;
    char _pad3[4];
    float particle_randomness;
    int particle_number;
    int particle_minimum;
    int particle_maximum;
    float particle_radius;
    float particle_band_width;
    float fractions_threshold;
    float fractions_distance;
    float flip_ratio;
    int sys_particle_maximum;
    short simulation_method;
    char _pad4[6];
    float viscosity_value;
    char _pad5[4];
    float surface_tension;
    float viscosity_base;
    int viscosity_exponent;
    float mesh_concave_upper;
    float mesh_concave_lower;
    float mesh_particle_radius;
    int mesh_smoothen_pos;
    int mesh_smoothen_neg;
    int mesh_scale;
    short mesh_generator;
    char _pad6[2];
    int particle_type;
    int particle_scale;
    float sndparticle_tau_min_wc;
    float sndparticle_tau_max_wc;
    float sndparticle_tau_min_ta;
    float sndparticle_tau_max_ta;
    float sndparticle_tau_min_k;
    float sndparticle_tau_max_k;
    int sndparticle_k_wc;
    int sndparticle_k_ta;
    float sndparticle_k_b;
    float sndparticle_k_d;
    float sndparticle_l_min;
    float sndparticle_l_max;
    int sndparticle_potential_radius;
    int sndparticle_update_radius;
    char sndparticle_boundary;
    char sndparticle_combined_export;
    char _pad7[6];
    float guide_alpha;
    int guide_beta;
    float guide_vel_factor;
    int guide_res[3];
    short guide_source;
    char _pad8[2];
    int cache_frame_start;
    int cache_frame_end;
    int cache_frame_pause_data;
    int cache_frame_pause_noise;
    int cache_frame_pause_mesh;
    int cache_frame_pause_particles;
    int cache_frame_pause_guide;
    int cache_frame_offset;
    int cache_flag;
    char cache_mesh_format;
    char cache_data_format;
    char cache_particle_format;
    char cache_noise_format;
    char cache_directory[1024];
    char error[64];
    short cache_type;
    char cache_id[4];
    char _pad9[2];
    float dt;
    float time_total;
    float time_per_frame;
    float frame_length;
    float time_scale;
    float cfl_condition;
    int timesteps_minimum;
    int timesteps_maximum;
    float slice_per_voxel;
    float slice_depth;
    float display_thickness;
    float grid_scale;
    ColorBand2_93_0 *coba;
    float vector_scale;
    float gridlines_lower_bound;
    float gridlines_upper_bound;
    float gridlines_range_color[4];
    char axis_slice_method;
    char slice_axis;
    char show_gridlines;
    char draw_velocity;
    char vector_draw_type;
    char vector_field;
    char vector_scale_with_magnitude;
    char vector_draw_mac_components;
    char use_coba;
    char coba_field;
    char interp_method;
    char gridlines_color_field;
    char gridlines_cell_filter;
    char _pad10[7];
    int openvdb_compression;
    float clipping;
    char openvdb_data_depth;
    char _pad11[7];
    int viewsettings;
    char _pad12[4];
    void *point_cache[2];
    ListBase2_93_0 ptcaches[2];
    int cache_comp;
    int cache_high_comp;
    char cache_file_format;
    char _pad13[7];
};

struct LengthGpencilModifierData3_0_0 {
    GpencilModifierData2_93_0 modifier;
    Material3_0_0 *material;
    char layername[64];
    int pass_index;
    int flag;
    int layer_pass;
    float start_fac, end_fac;
    float overshoot_fac;
    int mode;
    float point_density;
    float segment_influence;
    float max_angle;
};

struct DashGpencilModifierData3_0_0 {
    GpencilModifierData2_93_0 modifier;
    Material3_0_0 *material;
    char layername[64];
    int pass_index;
    int flag;
    int layer_pass;
    int dash_offset;
    DashGpencilModifierSegment3_0_0 *segments;
    int segments_len;
    int segment_active_index;
};

struct OffsetGpencilModifierData3_0_0 {
    GpencilModifierData2_93_0 modifier;
    Material3_0_0 *material;
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
    int layer_pass;
};

struct WeightProxGpencilModifierData3_0_0 {
    GpencilModifierData2_93_0 modifier;
    char target_vgname[64];
    Material3_0_0 *material;
    char layername[64];
    char vgname[64];
    int pass_index;
    int flag;
    float min_weight;
    int layer_pass;
    float dist_start;
    float dist_end;
    Object3_0_0 *object;
};

struct WeightAngleGpencilModifierData3_0_0 {
    GpencilModifierData2_93_0 modifier;
    char target_vgname[64];
    Material3_0_0 *material;
    char layername[64];
    char vgname[64];
    int pass_index;
    int flag;
    float min_weight;
    int layer_pass;
    short axis;
    short space;
    float angle;
};

struct LineartGpencilModifierData3_0_0 {
    GpencilModifierData2_93_0 modifier;
    short edge_types;
    char source_type;
    char use_multiple_levels;
    short level_start;
    short level_end;
    Object3_0_0 *source_camera;
    Object3_0_0 *source_object;
    Collection3_0_0 *source_collection;
    Material3_0_0 *target_material;
    char target_layer[64];
    char source_vertex_group[64];
    char vgname[64];
    float overscan;
    float opacity;
    short thickness;
    unsigned char mask_switches;
    unsigned char material_mask_bits;
    unsigned char intersection_mask;
    char _pad[3];
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
    void *cache;
    void *render_buffer_ptr;
};

struct bGPdata3_0_0 {
    ID3_0_0 id;
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
    Material3_0_0 **mat;
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
    bGPdata_Runtime3_0_0 runtime;
};

struct IDProperty3_0_0 {
    IDProperty3_0_0 *next, *prev;
    char type, subtype;
    short flag;
    char name[64];
    int saved;
    IDPropertyData2_93_0 data;
    int len;
    int totallen;
    IDPropertyUIData3_0_0 *ui_data;
};

struct ImageTile3_0_0 {
    ImageTile3_0_0 *next, *prev;
    ImageTile_Runtime3_0_0 runtime;
    char _pad[4];
    int tile_number;
    char label[64];
};

struct Image3_0_0 {
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
    ListBase2_93_0 gpu_refresh_areas;
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
};

struct IpoCurve3_0_0 {
    IpoCurve3_0_0 *next, *prev;
    BPoint2_93_0 *bp;
    BezTriple2_93_0 *bezt;
    rctf2_93_0 maxrct, totrct;
    short blocktype, adrcode, vartype;
    short totvert;
    short ipo, extrap;
    short flag;
    char _pad0[2];
    float ymin, ymax;
    unsigned int bitmask;
    float slide_min, slide_max;
    float curval;
    IpoDriver2_93_0 *driver;
};

struct Lattice3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    short pntsu, pntsv, pntsw, flag;
    short opntsu, opntsv, opntsw;
    char _pad2[3];
    char typeu, typev, typew;
    int actbp;
    float fu, fv, fw, du, dv, dw;
    BPoint2_93_0 *def;
    Ipo3_0_0 *ipo;
    Key3_0_0 *key;
    MDeformVert2_93_0 *dvert;
    char vgroup[64];
    ListBase2_93_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad0[4];
    EditLatt2_93_0 *editlatt;
    void *batch_cache;
};

struct MaskLayer3_0_0 {
    MaskLayer3_0_0 *next, *prev;
    char name[64];
    ListBase2_93_0 splines;
    ListBase2_93_0 splines_shapes;
    MaskSpline2_93_0 *act_spline;
    MaskSplinePoint2_93_0 *act_point;
    float alpha;
    char blend;
    char blend_flag;
    char falloff;
    char _pad[7];
    char flag;
    char visibility_flag;
};

struct Mesh_Runtime3_0_0 {
    Mesh3_0_0 *mesh_eval;
    void *eval_mutex;
    void *edit_data;
    void *batch_cache;
    void *subdiv_ccg;
    void *_pad1;
    int subdiv_ccg_tot_level;
    char _pad2[4];
    int64_t cd_dirty_vert;
    int64_t cd_dirty_edge;
    int64_t cd_dirty_loop;
    int64_t cd_dirty_poly;
    MLoopTri_Store2_93_0 looptris;
    void *bvh_cache;
    void *shrinkwrap_data;
    char deformed_only;
    char is_original;
    char wrapper_type;
    char wrapper_type_finalize;
    char _pad[4];
    CustomData_MeshMasks2_93_0 cd_mask_extra;
    void *render_mutex;
    void *_pad3;
};

struct Mesh3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    Ipo3_0_0 *ipo;
    Key3_0_0 *key;
    Material3_0_0 **mat;
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
    ListBase2_93_0 vertex_group_names;
    MCol2_93_0 *mcol;
    Mesh3_0_0 *texcomesh;
    void *edit_mesh;
    CustomData3_0_0 vdata, edata, fdata;
    CustomData3_0_0 pdata, ldata;
    int totvert, totedge, totface, totselect;
    int totpoly, totloop;
    int attributes_active_index;
    int vertex_group_active_index;
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
    Mesh_Runtime3_0_0 runtime;
};

struct MirrorModifierData3_0_0 {
    ModifierData2_93_4 modifier;
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

struct MeshCacheModifierData3_0_0 {
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
    char defgrp_name[64];
    char _pad[7];
    float frame_start;
    float frame_scale;
    float eval_frame;
    float eval_time;
    float eval_factor;
    char filepath[1024];
};

struct MeshSeqCacheModifierData3_0_0 {
    ModifierData2_93_4 modifier;
    CacheFile3_0_0 *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
};

struct SurfaceDeformModifierData3_0_0 {
    ModifierData2_93_4 modifier;
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

struct NodesModifierData3_0_0 {
    ModifierData2_93_4 modifier;
    void *node_group;
    NodesModifierSettings2_93_0 settings;
    void *runtime_eval_log;
    void *_pad1;
};

struct bNodeSocket3_0_0 {
    bNodeSocket3_0_0 *next, *prev, *new_sock;
    IDProperty3_0_0 *prop;
    char identifier[64];
    char name[64];
    void *storage;
    short type, flag;
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
    bNodeSocket3_0_0 *groupsock;
    bNodeLink2_93_0 *link;
    bNodeStack2_93_0 ns;
};

struct bNode3_0_0 {
    bNode3_0_0 *next, *prev, *new_node;
    IDProperty3_0_0 *prop;
    void *typeinfo;
    char idname[64];
    char name[64];
    int flag;
    short type;
    short done, level;
    uint8_t need_exec;
    char _pad[1];
    float color[3];
    ListBase2_93_0 inputs, outputs;
    bNode3_0_0 *parent;
    ID3_0_0 *id;
    void *storage;
    bNode3_0_0 *original;
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
    rctf2_93_0 butr;
    rctf2_93_0 prvr;
    short preview_xsize, preview_ysize;
    short tmp_flag;
    char branch_tag;
    char iter_flag;
    void *block;
    float ssr_id;
    float sss_id;
    void *declaration;
};

struct SoftBody3_0_0 {
    int totpoint, totspring;
    void *bpoint;
    void *bspring;
    char _pad;
    char msg_lock;
    short msg_value;
    float nodemass;
    char namedVG_Mass[64];
    float grav;
    float mediafrict;
    float rklimit;
    float physics_speed;
    float goalspring;
    float goalfrict;
    float mingoal;
    float maxgoal;
    float defgoal;
    short vertgroup;
    char namedVG_Softgoal[64];
    short fuzzyness;
    float inspring;
    float infrict;
    char namedVG_Spring_K[64];
    char _pad1[6];
    char local, solverflags;
    SBVertex2_93_0 **keys;
    int totpointkey, totkey;
    float secondspring;
    float colball;
    float balldamp;
    float ballstiff;
    short sbc_mode;
    short aeroedge;
    short minloops;
    short maxloops;
    short choke;
    short solver_ID;
    short plastic;
    short springpreload;
    void *scratch;
    float shearstiff;
    float inpush;
    SoftBody_Shared2_93_0 *shared;
    void *pointcache;
    ListBase2_93_0 ptcaches;
    Collection3_0_0 *collision_group;
    EffectorWeights3_0_0 *effector_weights;
    float lcom[3];
    float lrot[3][3];
    float lscale[3][3];
    int last_frame;
};

struct Object_Runtime3_0_0 {
    CustomData_MeshMasks2_93_0 last_data_mask;
    char last_need_mapping;
    char _pad0[3];
    float parent_display_origin[3];
    int select_id;
    char _pad1[3];
    char is_data_eval_owned;
    double overlay_mode_transfer_start_time;
    BoundBox2_93_0 *bb;
    ID3_0_0 *data_orig;
    ID3_0_0 *data_eval;
    void *geometry_set_eval;
    Mesh3_0_0 *mesh_deform_eval;
    bGPdata3_0_0 *gpd_orig;
    bGPdata3_0_0 *gpd_eval;
    Mesh3_0_0 *object_as_temp_mesh;
    Curve3_0_0 *object_as_temp_curve;
    void *curve_cache;
    unsigned short local_collections_bits;
    short _pad2[3];
};

struct Object3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    void *sculpt;
    short type, partype;
    int par1, par2, par3;
    char parsubstr[64];
    Object3_0_0 *parent, *track;
    Object3_0_0 *proxy, *proxy_group, *proxy_from;
    Ipo3_0_0 *ipo;
    bAction3_0_0 *action;
    bAction3_0_0 *poselib;
    bPose2_93_0 *pose;
    void *data;
    bGPdata3_0_0 *gpd;
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
    Material3_0_0 **mat;
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
    Collection3_0_0 *instance_collection;
    FluidsimSettings2_93_0 *fluidsimSettings;
    ListBase2_93_0 pc_ids;
    RigidBodyOb2_93_0 *rigidbody_object;
    RigidBodyCon2_93_0 *rigidbody_constraint;
    float ima_ofs[2];
    ImageUser3_0_0 *iuser;
    char empty_image_visibility_flag;
    char empty_image_depth;
    char empty_image_flag;
    char _pad8[5];
    PreviewImage2_93_0 *preview;
    ObjectLineArt2_93_0 lineart;
    void *_pad9;
    Object_Runtime3_0_0 runtime;
};

struct BoidParticle3_0_0 {
    Object3_0_0 *ground;
    BoidData2_93_0 data;
    float gravity[3];
    float wander[3];
    char _pad0[4];
};

struct ParticleSettings3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    BoidSettings2_93_0 *boids;
    SPHFluidSettings2_93_0 *fluid;
    EffectorWeights3_0_0 *effector_weights;
    Collection3_0_0 *collision_group;
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
    int child_nbr, ren_child_nbr;
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
    Collection3_0_0 *instance_collection;
    ListBase2_93_0 instance_weights;
    Collection3_0_0 *force_group;
    Object3_0_0 *instance_object;
    Object3_0_0 *bb_ob;
    Ipo3_0_0 *ipo;
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

struct RenderData3_0_0 {
    ImageFormatData2_93_0 im_format;
    AviCodecData2_93_0 *avicodecdata;
    FFMpegCodecData2_93_0 ffcodecdata;
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
    short bake_filter, bake_samples;
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
    BakeData2_93_0 bake;
    int _pad8;
    short preview_pixel_size;
    short _pad4;
    ListBase2_93_0 views;
    short actview;
    short views_format;
    short hair_type, hair_subdiv;
    CurveMapping2_93_0 mblur_shutter_curve;
};

struct ParticleEditSettings3_0_0 {
    short flag;
    short totrekey;
    short totaddkey;
    short brushtype;
    ParticleBrushData2_93_0 brush[7];
    void *paintcursor;
    float emitterdist;
    char _pad0[4];
    int selectmode;
    int edittype;
    int draw_step, fade_frames;
    Scene3_0_0 *scene;
    Object3_0_0 *object;
    Object3_0_0 *shape_object;
};

struct ToolSettings3_0_0 {
    VPaint2_93_0 *vpaint;
    VPaint2_93_0 *wpaint;
    Sculpt2_93_0 *sculpt;
    UvSculpt2_93_0 *uvsculpt;
    GpPaint2_93_0 *gp_paint;
    GpVertexPaint2_93_0 *gp_vertexpaint;
    GpSculptPaint2_93_0 *gp_sculptpaint;
    GpWeightPaint2_93_0 *gp_weightpaint;
    float vgroup_weight;
    float doublimit;
    char automerge;
    char object_flag;
    short selectmode;
    char unwrapper;
    char uvcalc_flag;
    char uv_flag;
    char uv_selectmode;
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
    char snap_mode;
    char snap_node_mode;
    char snap_uv_mode;
    char snap_flag;
    char snap_uv_flag;
    char snap_target;
    char snap_transform_mode_flag;
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
    char _pad2[1];
    char uv_sculpt_settings;
    char uv_relax_method;
    short sculpt_paint_settings;
    char workspace_tool_type;
    char _pad5[1];
    int sculpt_paint_unified_size;
    float sculpt_paint_unified_unprojected_radius;
    float sculpt_paint_unified_alpha;
    UnifiedPaintSettings2_93_0 unified_paint_settings;
    CurvePaintSettings2_93_0 curve_paint_settings;
    MeshStatVis2_93_0 statvis;
    float normal_vector[3];
    char _pad6[4];
    CurveProfile2_93_0 *custom_bevel_profile_preset;
    SequencerToolSettings3_0_0 *sequencer_tool_settings;
};

struct Sequence3_0_0 {
    Sequence3_0_0 *next, *prev;
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
    char tmp_tag;
    char _pad[3];
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    Strip2_93_0 *strip;
    Ipo3_0_0 *ipo;
    Scene3_0_0 *scene;
    Object3_0_0 *scene_camera;
    MovieClip3_0_0 *clip;
    Mask3_0_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence3_0_0 *seq1, *seq2, *seq3;
    ListBase2_93_0 seqbase;
    bSound3_0_0 *sound;
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

struct Editing3_0_0 {
    ListBase2_93_0 *seqbasep;
    ListBase2_93_0 seqbase;
    ListBase2_93_0 metastack;
    Sequence3_0_0 *act_seq;
    char act_imagedir[1024];
    char act_sounddir[1024];
    char proxy_dir[1024];
    int proxy_storage;
    int overlay_frame_ofs, overlay_frame_abs;
    int overlay_frame_flag;
    rctf2_93_0 overlay_frame_rect;
    void *cache;
    float recycle_max_cost;
    int cache_flag;
    void *prefetch_job;
    int64_t disk_cache_timestamp;
    EditingRuntime3_0_0 runtime;
    void *_pad1;
};

struct SpaceSeqRuntime3_0_0 {
    rctf2_93_0 last_thumbnail_area;
    void *last_displayed_thumbnails;
};

struct SpaceSeq3_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
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
    SequencerScopes2_93_0 scopes;
    SequencerPreviewOverlay3_0_0 preview_overlay;
    SequencerTimelineOverlay3_0_0 timeline_overlay;
    char multiview_eye;
    char _pad2[7];
    SpaceSeqRuntime3_0_0 runtime;
};

struct FileDirEntryArr3_0_0 {
    ListBase2_93_0 entries;
    int nbr_entries;
    int nbr_entries_filtered;
    char root[1024];
};

struct SpaceImage3_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Image3_0_0 *image;
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
    char sticky;
    char dt_uvstretch;
    char around;
    char _pad1[3];
    int flag;
    float uv_opacity;
    int tile_grid_shape[2];
    int custom_grid_subdiv;
    char _pad3[4];
    MaskSpaceInfo2_93_0 mask_info;
    SpaceImageOverlay2_93_0 overlay;
};

struct SpaceNode3_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
    ID3_0_0 *id, *from;
    short flag;
    char insert_ofs_dir;
    char _pad1;
    float xof, yof;
    float zoom;
    ListBase2_93_0 treepath;
    void *edittree;
    void *nodetree;
    char tree_idname[64];
    int treetype;
    short texfrom;
    short shaderfrom;
    bGPdata3_0_0 *gpd;
    SpaceNodeOverlay3_0_0 overlay;
    char _pad2[4];
    void *runtime;
};

struct SpaceSpreadsheet3_0_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    ListBase2_93_0 columns;
    ListBase2_93_0 row_filters;
    ListBase2_93_0 context_path;
    uint8_t filter_flag;
    uint8_t geometry_component_type;
    uint8_t attribute_domain;
    uint8_t object_eval_state;
    uint32_t flag;
    void *runtime;
};

struct ThemeUI3_0_0 {
    uiWidgetColors2_93_0 wcol_regular, wcol_tool, wcol_toolbar_item, wcol_text;
    uiWidgetColors2_93_0 wcol_radio, wcol_option, wcol_toggle;
    uiWidgetColors2_93_0 wcol_num, wcol_numslider, wcol_tab;
    uiWidgetColors2_93_0 wcol_menu, wcol_pulldown, wcol_menu_back, wcol_menu_item, wcol_tooltip;
    uiWidgetColors2_93_0 wcol_box, wcol_scroll, wcol_progress, wcol_list_item, wcol_pie_menu;
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

struct ThemeSpace3_0_0 {
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

struct bTheme3_0_0 {
    bTheme3_0_0 *next, *prev;
    char name[32];
    ThemeUI3_0_0 tui;
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

struct UserDef3_0_0 {
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
    char _pad10[2];
    char file_preview_type;
    char statusbar_flag;
    WalkNavigation2_93_0 walk_navigation;
    UserDef_SpaceData2_93_0 space_data;
    UserDef_FileSpaceData2_93_0 file_space_data;
    UserDef_Experimental3_0_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct XrSessionSettings3_0_0 {
    View3DShading2_93_0 shading;
    float base_scale;
    char _pad[3];
    char base_pose_type;
    Object3_0_0* base_pose_object;
    float base_pose_location[3];
    float base_pose_angle;
    char draw_flags;
    char controller_draw_style;
    char _pad2[2];
    float clip_start, clip_end;
    int flag;
};

struct wmXrData3_0_0 {
    void* runtime;
    XrSessionSettings3_0_0 session_settings;
};

struct wmWindowManager3_0_0 {
    ID3_0_0 id;
    wmWindow2_93_0 *windrawable;
    wmWindow2_93_0 *winactive;
    ListBase2_93_0 windows;
    short initialized;
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    ListBase2_93_0 operators;
    ListBase2_93_0 notifier_queue;
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
    wmXrData3_0_0 xr;
};

struct WorkSpace3_0_0 {
    ID3_0_0 id;
    ListBase2_93_0 layouts;
    ListBase2_93_0 hook_layout_relations;
    ListBase2_93_0 owner_ids;
    ListBase2_93_0 tools;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    char *status_text;
    AssetLibraryReference3_0_0 asset_library_ref;
};

struct XrActionMapItem3_0_0 {
    XrActionMapItem3_0_0 *next, *prev;
    char name[64];
    char type;
    char _pad[7];
    char user_path0[64];
    char user_path1[64];
    char op[64];
    IDProperty3_0_0 *op_properties;
    void *op_properties_ptr;
    short op_flag;
    short action_flag;
    short haptic_flag;
    short pose_flag;
    char haptic_name[64];
    float haptic_duration;
    float haptic_frequency;
    float haptic_amplitude;
    short selbinding;
    char _pad3[2];
    ListBase2_93_0 bindings;
};

struct XrActionMap3_0_0 {
    XrActionMap3_0_0 *next, *prev;
    char name[64];
    ListBase2_93_0 items;
    short selitem;
    char _pad[6];
};

struct bAction3_0_0 {
    ID3_0_0 id;
    ListBase2_93_0 curves;
    ListBase2_93_0 chanbase;
    ListBase2_93_0 groups;
    ListBase2_93_0 markers;
    int flag;
    int active_marker;
    int idroot;
    char _pad[4];
    PreviewImage2_93_0 *preview;
};

struct bArmature3_0_0 {
    ID3_0_0 id;
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

struct Brush3_0_0 {
    ID3_0_0 id;
    BrushClone2_93_0 clone;
    CurveMapping2_93_0 *curve;
    MTex3_0_0 mtex;
    MTex3_0_0 mask_mtex;
    Brush3_0_0 *toggle_brush;
    void *icon_imbuf;
    PreviewImage2_93_0 *preview;
    ColorBand2_93_0 *gradient;
    PaintCurve3_0_0 *paint_curve;
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
    BrushGpencilSettings3_0_0 *gpencil_settings;
};

struct Palette3_0_0 {
    ID3_0_0 id;
    ListBase2_93_0 colors;
    int active_color;
    char _pad[4];
};

struct CameraBGImage3_0_0 {
    CameraBGImage3_0_0 *next, *prev;
    Image3_0_0 *ima;
    ImageUser3_0_0 iuser;
    MovieClip3_0_0 *clip;
    MovieClipUser2_93_0 cuser;
    float offset[2], scale, rotation;
    float alpha;
    short flag;
    short source;
};

struct Camera3_0_0 {
    ID3_0_0 id;
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
    Ipo3_0_0 *ipo;
    Object3_0_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings2_93_0 dof;
    ListBase2_93_0 bg_images;
    char sensor_fit;
    char _pad[7];
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct Ipo3_0_0 {
    ID3_0_0 id;
    ListBase2_93_0 curve;
    rctf2_93_0 cur;
    short blocktype, showkey;
    short muteipo;
    char _pad[2];
};

struct Key3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    KeyBlock2_93_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    ListBase2_93_0 block;
    Ipo3_0_0 *ipo;
    ID3_0_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct FreestyleLineStyle3_0_0 {
    ID3_0_0 id;
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
    MTex3_0_0 *mtex[18];
    void *nodetree;
    ListBase2_93_0 color_modifiers;
    ListBase2_93_0 alpha_modifiers;
    ListBase2_93_0 thickness_modifiers;
    ListBase2_93_0 geometry_modifiers;
};

struct Mask3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra, efra;
    int flag;
    char _pad[4];
};

struct Material3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    short flag;
    char _pad1[2];
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
    void *nodetree;
    Ipo3_0_0 *ipo;
    PreviewImage2_93_0 *preview;
    float line_col[4];
    short line_priority;
    short vcol_alpha;
    short paint_active_slot;
    short paint_clone_slot;
    short tot_slots;
    char _pad2[2];
    float alpha_threshold;
    float refract_depth;
    char blend_method;
    char blend_shadow;
    char blend_flag;
    char _pad3[1];
    TexPaintSlot2_93_0 *texpaintslot;
    ListBase2_93_0 gpumaterial;
    MaterialGPencilStyle2_93_0 *gp_style;
    MaterialLineArt3_0_0 lineart;
};

struct MetaBall3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 disp;
    ListBase2_93_0 *editelems;
    Ipo3_0_0 *ipo;
    Material3_0_0 **mat;
    char flag, flag2;
    short totcol;
    short texflag;
    char _pad[1];
    char needs_flush_to_id;
    float loc[3];
    float size[3];
    float rot[3];
    float wiresize, rendersize;
    float thresh;
    MetaElem2_93_0 *lastelem;
    void *batch_cache;
};

struct MovieClip3_0_0 {
    ID3_0_0 id;
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

struct NodeTexBase3_0_0 {
    TexMapping3_0_0 tex_mapping;
    ColorMapping2_93_0 color_mapping;
};

struct NodeTexSky3_0_0 {
    NodeTexBase3_0_0 base;
    int sky_model;
    float sun_direction[3];
    float turbidity;
    float ground_albedo;
    float sun_size;
    float sun_intensity;
    float sun_elevation;
    float sun_rotation;
    float altitude;
    float air_density;
    float dust_density;
    float ozone_density;
    char sun_disc;
    char _pad[7];
};

struct NodeTexImage3_0_0 {
    NodeTexBase3_0_0 base;
    ImageUser3_0_0 iuser;
    int color_space;
    int projection;
    float projection_blend;
    int interpolation;
    int extension;
    char _pad[4];
};

struct NodeTexChecker3_0_0 {
    NodeTexBase3_0_0 base;
};

struct NodeTexBrick3_0_0 {
    NodeTexBase3_0_0 base;
    int offset_freq, squash_freq;
    float offset, squash;
};

struct NodeTexEnvironment3_0_0 {
    NodeTexBase3_0_0 base;
    ImageUser3_0_0 iuser;
    int color_space;
    int projection;
    int interpolation;
    char _pad[4];
};

struct NodeTexGradient3_0_0 {
    NodeTexBase3_0_0 base;
    int gradient_type;
    char _pad[4];
};

struct NodeTexNoise3_0_0 {
    NodeTexBase3_0_0 base;
    int dimensions;
    char _pad[4];
};

struct NodeTexVoronoi3_0_0 {
    NodeTexBase3_0_0 base;
    int dimensions;
    int feature;
    int distance;
    int coloring;
};

struct NodeTexMusgrave3_0_0 {
    NodeTexBase3_0_0 base;
    int musgrave_type;
    int dimensions;
};

struct NodeTexWave3_0_0 {
    NodeTexBase3_0_0 base;
    int wave_type;
    int bands_direction;
    int rings_direction;
    int wave_profile;
};

struct NodeTexMagic3_0_0 {
    NodeTexBase3_0_0 base;
    int depth;
    char _pad[4];
};

struct NodeShaderTexPointDensity3_0_0 {
    NodeTexBase3_0_0 base;
    short point_source;
    char _pad[2];
    int particle_system;
    float radius;
    int resolution;
    short space;
    short interpolation;
    short color_source;
    short ob_color_source;
    char vertex_attribute_name[64];
    PointDensity2_93_0 pd;
    int cached_resolution;
    char _pad2[4];
};

struct NodeCryptomatte3_0_0 {
    ImageUser3_0_0 iuser;
    ListBase2_93_0 entries;
    char layer_name[64];
    char *matte_id;
    int num_inputs;
    char _pad[4];
    NodeCryptomatte_Runtime2_93_0 runtime;
};

struct Scene3_0_0 {
    ID3_0_0 id;
    AnimData2_93_0 *adt;
    Object3_0_0 *camera;
    World3_0_0 *world;
    Scene3_0_0 *set;
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
    RenderData3_0_0 r;
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
    MovieClip3_0_0 *clip;
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

struct bScreen3_0_0 {
    ID3_0_0 id;
    ListBase2_93_0 vertbase;
    ListBase2_93_0 edgebase;
    ListBase2_93_0 areabase;
    ListBase2_93_0 regionbase;
    Scene3_0_0 *scene;
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

struct Panel3_0_0 {
    Panel3_0_0 *next, *prev;
    void *type;
    void *layout;
    char panelname[64];
    char drawname[64];
    int ofsx, ofsy;
    int sizex, sizey;
    int blocksizex, blocksizey;
    short labelofs;
    short flag, runtime_flag;
    char _pad[6];
    int sortorder;
    void *activedata;
    ListBase2_93_0 children;
    Panel_Runtime3_0_0 runtime;
};

struct ColorBalanceModifierData3_0_0 {
    SequenceModifierData2_93_0 modifier;
    StripColorBalance3_0_0 color_balance;
    float color_multiply;
};

struct Text3_0_0 {
    ID3_0_0 id;
    char *filepath;
    void *compiled;
    int flags;
    char _pad0[4];
    ListBase2_93_0 lines;
    TextLine2_93_0 *curl, *sell;
    int curc, selc;
    double mtime;
};

struct View3D3_0_0 {
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
    View3D3_0_0 *localvd;
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
    View3DOverlay3_0_0 overlay;
    View3D_Runtime3_0_0 runtime;
};

struct Volume3_0_0 {
    ID3_0_0 id;
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
    Material3_0_0 **mat;
    short totcol;
    short _pad2[3];
    VolumeRender2_93_0 render;
    VolumeDisplay2_93_0 display;
    void *batch_cache;
    Volume_Runtime2_93_0 runtime;
};

struct World3_0_0 {
    ID3_0_0 id;
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
    char _pad3[6];
    Ipo3_0_0 *ipo;
    short pr_texture, use_nodes;
    char _pad[4];
    PreviewImage2_93_0 *preview;
    void *nodetree;
    ListBase2_93_0 gpumaterial;
};

#endif
