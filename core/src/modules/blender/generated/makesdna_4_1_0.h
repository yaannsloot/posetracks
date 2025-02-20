/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef MAKESDNA_4_1_0_H
#define MAKESDNA_4_1_0_H

#include "makesdna_3_6_0.h"
#include "makesdna_4_0_0.h"

struct AnimData4_1_0;
struct ArmatureModifierData4_1_0;
struct ArrayModifierData4_1_0;
struct BakeData4_1_0;
struct Base4_1_0;
struct BevelModifierData4_1_0;
struct BoneCollection4_1_0;
struct BoneCollectionReference4_1_0;
struct BooleanModifierData4_1_0;
struct BoundBox4_1_0;
struct BrightContrastModifierData4_1_0;
struct Brush4_1_0;
struct BrushClone4_1_0;
struct BuildModifierData4_1_0;
struct CacheFile4_1_0;
struct Camera4_1_0;
struct CameraBGImage4_1_0;
struct CastModifierData4_1_0;
struct ClothModifierData4_1_0;
struct Collection4_1_0;
struct CollectionChild4_1_0;
struct Collection_Runtime4_1_0;
struct CollisionModifierData4_1_0;
struct ColorBalanceModifierData4_1_0;
struct CorrectiveSmoothModifierData4_1_0;
struct Curve4_1_0;
struct CurveModifierData4_1_0;
struct Curves4_1_0;
struct CurvesSculpt4_1_0;
struct DataTransferModifierData4_1_0;
struct DecimateModifierData4_1_0;
struct DisplaceModifierData4_1_0;
struct DriverTarget4_1_0;
struct DynamicPaintModifierData4_1_0;
struct EdgeSplitModifierData4_1_0;
struct EditLatt4_1_0;
struct Editing4_1_0;
struct ExplodeModifierData4_1_0;
struct FileAssetSelectParams4_1_0;
struct FileHandler4_1_0;
struct FileSelectParams4_1_0;
struct FluidModifierData4_1_0;
struct FluidsimModifierData4_1_0;
struct FreestyleLineStyle4_1_0;
struct GP_Sculpt_Guide4_1_0;
struct GP_Sculpt_Settings4_1_0;
struct GpPaint4_1_0;
struct GpSculptPaint4_1_0;
struct GpVertexPaint4_1_0;
struct GpWeightPaint4_1_0;
struct GreasePencil4_1_0;
struct GreasePencilColorModifierData4_1_0;
struct GreasePencilDrawingReference4_1_0;
struct GreasePencilMirrorModifierData4_1_0;
struct GreasePencilModifierInfluenceData4_1_0;
struct GreasePencilNoiseModifierData4_1_0;
struct GreasePencilOffsetModifierData4_1_0;
struct GreasePencilOpacityModifierData4_1_0;
struct GreasePencilSmoothModifierData4_1_0;
struct GreasePencilSubdivModifierData4_1_0;
struct GreasePencilThickModifierData4_1_0;
struct GreasePencilTintModifierData4_1_0;
struct HookModifierData4_1_0;
struct ID4_1_0;
struct IDOverrideLibrary4_1_0;
struct IDOverrideLibraryPropertyOperation4_1_0;
struct IDPropertyUIDataEnumItem4_1_0;
struct IDPropertyUIDataInt4_1_0;
struct IDViewerPathElem4_1_0;
struct IdAdtTemplate4_1_0;
struct Image4_1_0;
struct ImagePaintSettings4_1_0;
struct IndexSwitchItem4_1_0;
struct Ipo4_1_0;
struct KS_Path4_1_0;
struct Key4_1_0;
struct LaplacianDeformModifierData4_1_0;
struct LaplacianSmoothModifierData4_1_0;
struct Lattice4_1_0;
struct LatticeModifierData4_1_0;
struct LayoutPanelState4_1_0;
struct Library4_1_0;
struct Light4_1_0;
struct LightProbe4_1_0;
struct MBoolProperty4_1_0;
struct MInt8Property4_1_0;
struct MTex4_1_0;
struct MappingInfoModifierData4_1_0;
struct Mask4_1_0;
struct MaskLayer4_1_0;
struct MaskModifierData4_1_0;
struct MaskParent4_1_0;
struct MaskSpline4_1_0;
struct MaskSplinePoint4_1_0;
struct Material4_1_0;
struct Mesh4_1_0;
struct MeshCacheModifierData4_1_0;
struct MeshSeqCacheModifierData4_1_0;
struct MeshToVolumeModifierData4_1_0;
struct MetaBall4_1_0;
struct MetaStack4_1_0;
struct MirrorModifierData4_1_0;
struct ModifierData4_1_0;
struct MovieClip4_1_0;
struct MovieTracking4_1_0;
struct MovieTrackingObject4_1_0;
struct MovieTrackingPlaneTrack4_1_0;
struct MultiresModifierData4_1_0;
struct NlaStrip4_1_0;
struct NodeEnumDefinition4_1_0;
struct NodeEnumItem4_1_0;
struct NodeGeometryBake4_1_0;
struct NodeGeometryBakeItem4_1_0;
struct NodeIndexSwitch4_1_0;
struct NodeKeyingScreenData4_1_0;
struct NodeKuwaharaData4_1_0;
struct NodeMenuSwitch4_1_0;
struct NodesModifierBake4_1_0;
struct NodesModifierData4_1_0;
struct NodesModifierDataBlock4_1_0;
struct NodesModifierPanel4_1_0;
struct NormalEditModifierData4_1_0;
struct ObHook4_1_0;
struct Object4_1_0;
struct OceanModifierData4_1_0;
struct Paint4_1_0;
struct PaintCurve4_1_0;
struct PaintModeSettings4_1_0;
struct Palette4_1_0;
struct Panel4_1_0;
struct ParticleInstanceModifierData4_1_0;
struct ParticleSettings4_1_0;
struct ParticleSystemModifierData4_1_0;
struct PointCloud4_1_0;
struct PointDensity4_1_0;
struct RaytraceEEVEE4_1_0;
struct RegionView3D4_1_0;
struct RemeshModifierData4_1_0;
struct ReportList4_1_0;
struct SceneEEVEE4_1_0;
struct Scopes4_1_0;
struct ScrArea4_1_0;
struct ScrewModifierData4_1_0;
struct Script4_1_0;
struct Sculpt4_1_0;
struct SeqRetimingKey4_1_0;
struct Sequence4_1_0;
struct SequenceModifierData4_1_0;
struct SequenceRuntime4_1_0;
struct SequencerMaskModifierData4_1_0;
struct SequencerTonemapModifierData4_1_0;
struct SessionUID4_1_0;
struct ShapeKeyModifierData4_1_0;
struct ShrinkwrapModifierData4_1_0;
struct SimpleDeformModifierData4_1_0;
struct SkinModifierData4_1_0;
struct SmokeModifierData4_1_0;
struct SmoothModifierData4_1_0;
struct SoftbodyModifierData4_1_0;
struct SolidifyModifierData4_1_0;
struct SoundEqualizerModifierData4_1_0;
struct SpaceAction4_1_0;
struct SpaceClip4_1_0;
struct SpaceConsole4_1_0;
struct SpaceFile4_1_0;
struct SpaceGraph4_1_0;
struct SpaceImage4_1_0;
struct SpaceNla4_1_0;
struct SpaceNode4_1_0;
struct SpaceProperties4_1_0;
struct SpaceScript4_1_0;
struct SpaceSeq4_1_0;
struct SpaceSpreadsheet4_1_0;
struct SpaceText4_1_0;
struct Speaker4_1_0;
struct SubsurfModifierData4_1_0;
struct SurfaceDeformModifierData4_1_0;
struct SurfaceModifierData4_1_0;
struct Tex4_1_0;
struct TexMapping4_1_0;
struct Text4_1_0;
struct ThemeSpace4_1_0;
struct TimeMarker4_1_0;
struct TreeStore4_1_0;
struct TreeStoreElem4_1_0;
struct TriangulateModifierData4_1_0;
struct UVProjectModifierData4_1_0;
struct UVWarpModifierData4_1_0;
struct UnifiedPaintSettings4_1_0;
struct UserDef_Experimental4_1_0;
struct UvSculpt4_1_0;
struct VFont4_1_0;
struct VPaint4_1_0;
struct View3DOverlay4_1_0;
struct ViewLayer4_1_0;
struct Volume4_1_0;
struct VolumeDisplaceModifierData4_1_0;
struct VolumeToMeshModifierData4_1_0;
struct WarpModifierData4_1_0;
struct WaveModifierData4_1_0;
struct WeightVGEditModifierData4_1_0;
struct WeightVGMixModifierData4_1_0;
struct WeightVGProximityModifierData4_1_0;
struct WeightedNormalModifierData4_1_0;
struct WeldModifierData4_1_0;
struct WhiteBalanceModifierData4_1_0;
struct WireframeModifierData4_1_0;
struct WorkSpace4_1_0;
struct WorkSpaceInstanceHook4_1_0;
struct World4_1_0;
struct XrSessionSettings4_1_0;
struct bAction4_1_0;
struct bArmature4_1_0;
struct bArmature_Runtime4_1_0;
struct bDopeSheet4_1_0;
struct bGPdata4_1_0;
struct bGPdata_Runtime4_1_0;
struct bNode4_1_0;
struct bNodeLink4_1_0;
struct bNodeSocket4_1_0;
struct bNodeSocketValueImage4_1_0;
struct bNodeSocketValueMenu4_1_0;
struct bNodeSocketValueObject4_1_0;
struct bNodeSocketValueTexture4_1_0;
struct bNodeTree4_1_0;
struct bNodeTreeInterfaceSocket4_1_0;
struct bNodeTreePath4_1_0;
struct bPose4_1_0;
struct bPoseChannel4_1_0;
struct bPoseChannel_Runtime4_1_0;
struct bScreen4_1_0;
struct bSound4_1_0;
struct bUserExtensionRepo4_1_0;
struct uiFontStyle4_1_0;
struct uiStyle4_1_0;
struct vec3i4_1_0;
struct wmOperator4_1_0;
struct wmWindow4_1_0;
struct wmWindowManager4_1_0;
struct wmXrData4_1_0;

struct vec3i4_1_0 {
    int x;
    int y;
    int z;
};

struct bArmature_Runtime4_1_0 {
    int active_collection_index;
    unsigned char _pad0[4];
    struct BoneCollection4_1_0 *active_collection;
};

struct BoundBox4_1_0 {
    float vec[3][8];
};

struct Base4_1_0 {
    struct Base4_1_0 *next;
    struct Base4_1_0 *prev;
    struct Object4_1_0 *object;
    struct Base4_1_0 *base_orig;
    unsigned int lay;
    short flag;
    short flag_from_collection;
    short flag_legacy;
    unsigned short local_view_bits;
    unsigned short local_collections_bits;
    char _pad1[2];
};

struct MetaStack4_1_0 {
    struct MetaStack4_1_0 *next;
    struct MetaStack4_1_0 *prev;
    struct ListBase3_6_0 *oldbasep;
    struct ListBase3_6_0 *old_channels;
    struct Sequence4_1_0 *parseq;
    int disp_range[2];
};

struct MTex4_1_0 {
    short texco;
    short mapto;
    short blendtype;
    char _pad2[2];
    struct Object4_1_0 *object;
    struct Tex4_1_0 *tex;
    char uvname[68];
    char projx;
    char projy;
    char projz;
    char mapping;
    char brush_map_mode;
    char brush_angle_mode;
    short which_output;
    float ofs[3];
    float size[3];
    float rot;
    float random_angle;
    float r;
    float g;
    float b;
    float k;
    float def_var;
    float colfac;
    float alphafac;
    float timefac;
    float lengthfac;
    float clumpfac;
    float dampfac;
    float kinkfac;
    float kinkampfac;
    float roughfac;
    float padensfac;
    float gravityfac;
    float lifefac;
    float sizefac;
    float ivelfac;
    float fieldfac;
    float twistfac;
};

struct ObHook4_1_0 {
    struct ObHook4_1_0 *next;
    struct ObHook4_1_0 *prev;
    struct Object4_1_0 *parent;
    float parentinv[4][4];
    float mat[4][4];
    float cent[3];
    float falloff;
    char name[64];
    int *indexar;
    int totindex;
    int curindex;
    short type;
    short active;
    float force;
};

struct TreeStoreElem4_1_0 {
    short type;
    short nr;
    short flag;
    short used;
    struct ID4_1_0 *id;
};

struct TreeStore4_1_0 {
    int totelem;
    int usedelem;
    struct TreeStoreElem4_1_0 *data;
};

struct TimeMarker4_1_0 {
    struct TimeMarker4_1_0 *next;
    struct TimeMarker4_1_0 *prev;
    int frame;
    char name[64];
    unsigned int flag;
    struct Object4_1_0 *camera;
    struct IDProperty3_6_0 *prop;
};

struct ModifierData4_1_0 {
    struct ModifierData4_1_0 *next;
    struct ModifierData4_1_0 *prev;
    int type;
    int mode;
    float execution_time;
    short flag;
    short ui_expand_flag;
    unsigned short layout_panel_open_flag;
    char _pad[2];
    int persistent_uid;
    char name[64];
    char *error;
    void *runtime;
};

struct bNodeLink4_1_0 {
    struct bNodeLink4_1_0 *next;
    struct bNodeLink4_1_0 *prev;
    struct bNode4_1_0 *fromnode;
    struct bNode4_1_0 *tonode;
    struct bNodeSocket4_1_0 *fromsock;
    struct bNodeSocket4_1_0 *tosock;
    int flag;
    int multi_input_socket_index;
};

struct TexMapping4_1_0 {
    float loc[3];
    float rot[3];
    float size[3];
    int flag;
    char projx;
    char projy;
    char projz;
    char mapping;
    int type;
    float mat[4][4];
    float min[3];
    float max[3];
    struct Object4_1_0 *ob;
};

struct BrushClone4_1_0 {
    struct Image4_1_0 *image;
    float offset[2];
    float alpha;
    char _pad[4];
};

struct bGPdata_Runtime4_1_0 {
    void *sbuffer;
    void *sbuffer_position_buf;
    void *sbuffer_color_buf;
    void *sbuffer_batch;
    struct bGPDstroke4_0_0 *sbuffer_gps;
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
    struct bGPDcontrolpoint3_6_0 *cp_points;
    struct Brush4_1_0 *sbuffer_brush;
    void *gpencil_cache;
    void *lineart_cache;
    void *update_cache;
};

struct PointDensity4_1_0 {
    short flag;
    short falloff_type;
    float falloff_softness;
    float radius;
    short source;
    char _pad0[2];
    short color_source;
    short ob_color_source;
    int totpoints;
    struct Object4_1_0 *object;
    int psys;
    short psys_cache_space;
    short ob_cache_space;
    char vertex_attribute_name[68];
    char _pad1[4];
    void *point_tree;
    float *point_data;
    float noise_size;
    short noise_depth;
    short noise_influence;
    short noise_basis;
    char _pad2[6];
    float noise_fac;
    float speed_scale;
    float falloff_speed_scale;
    char _pad3[4];
    void *coba;
    void *falloff_curve;
};

struct uiFontStyle4_1_0 {
    short uifont_id;
    char _pad1[2];
    float points;
    short italic;
    short bold;
    short shadow;
    short shadx;
    short shady;
    char _pad0[2];
    float shadowalpha;
    float shadowcolor;
    int character_weight;
};

struct DriverTarget4_1_0 {
    struct ID4_1_0 *id;
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

struct KS_Path4_1_0 {
    struct KS_Path4_1_0 *next;
    struct KS_Path4_1_0 *prev;
    struct ID4_1_0 *id;
    char group[64];
    int idtype;
    short groupmode;
    short flag;
    char *rna_path;
    int array_index;
    short keyingflag;
    short keyingoverride;
};

struct FileSelectParams4_1_0 {
    char title[96];
    char dir[1090];
    char file[256];
    char renamefile[256];
    short rename_flag;
    char _pad[4];
    const  struct ID4_1_0 *rename_id;
    void *_pad3;
    char filter_glob[256];
    char filter_search[64];
    unsigned long long filter_id;
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

struct RegionView3D4_1_0 {
    float winmat[4][4];
    float viewmat[4][4];
    float viewinv[4][4];
    float persmat[4][4];
    float persinv[4][4];
    float viewcamtexcofac[4];
    float viewmatob[4][4];
    float persmatob[4][4];
    float clip[4][6];
    float clip_local[4][6];
    struct BoundBox4_1_0 *clipbb;
    struct RegionView3D4_1_0 *localvd;
    void *view_render;
    void *sms;
    void *smooth_timer;
    float twmat[4][4];
    float tw_axis_min[3];
    float tw_axis_max[3];
    float tw_axis_matrix[3][3];
    float gridview;
    float viewquat[4];
    float dist;
    float camdx;
    float camdy;
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

struct EditLatt4_1_0 {
    struct Lattice4_1_0 *latt;
    int shapenr;
    char needs_flush_to_id;
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

struct MaskParent4_1_0 {
    int id_type;
    int type;
    struct ID4_1_0 *id;
    char parent[64];
    char sub_parent[64];
    float parent_orig[2];
    float parent_corners_orig[2][4];
};

struct NodeKeyingScreenData4_1_0 {
    char tracking_object[64];
    float smoothness;
};

struct SequenceModifierData4_1_0 {
    struct SequenceModifierData4_1_0 *next;
    struct SequenceModifierData4_1_0 *prev;
    int type;
    int flag;
    char name[64];
    int mask_input_type;
    int mask_time;
    struct Sequence4_1_0 *mask_sequence;
    void *mask_id;
};

struct MovieTrackingPlaneTrack4_1_0 {
    struct MovieTrackingPlaneTrack4_1_0 *next;
    struct MovieTrackingPlaneTrack4_1_0 *prev;
    char name[64];
    struct MovieTrackingTrack3_6_0 **point_tracks;
    int point_tracksnr;
    char _pad[4];
    struct MovieTrackingPlaneMarker3_6_0 *markers;
    int markersnr;
    int flag;
    struct Image4_1_0 *image;
    float image_opacity;
    int last_marker;
};

struct IDOverrideLibraryPropertyOperation4_1_0 {
    struct IDOverrideLibraryPropertyOperation4_1_0 *next;
    struct IDOverrideLibraryPropertyOperation4_1_0 *prev;
    short operation;
    short flag;
    short tag;
    char _pad0[2];
    char *subitem_reference_name;
    char *subitem_local_name;
    int subitem_reference_index;
    int subitem_local_index;
    struct ID4_1_0 *subitem_reference_id;
    struct ID4_1_0 *subitem_local_id;
};

struct SessionUID4_1_0 {
    unsigned long long uid_;
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

struct GP_Sculpt_Guide4_1_0 {
    char use_guide;
    char use_snapping;
    char reference_point;
    char type;
    char _pad2[4];
    float angle;
    float angle_snap;
    float spacing;
    float location[3];
    struct Object4_1_0 *reference_object;
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

struct WorkSpaceInstanceHook4_1_0 {
    struct WorkSpace4_1_0 *active;
    struct WorkSpaceLayout3_6_0 *act_layout;
    struct WorkSpace4_1_0 *temp_workspace_store;
    struct WorkSpaceLayout3_6_0 *temp_layout_store;
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

struct bNodeSocketValueObject4_1_0 {
    struct Object4_1_0 *value;
};

struct bNodeSocketValueImage4_1_0 {
    struct Image4_1_0 *value;
};

struct MBoolProperty4_1_0 {
    int b;
};

struct bNodeSocketValueTexture4_1_0 {
    struct Tex4_1_0 *value;
};

struct MInt8Property4_1_0 {
    int i;
};

struct bUserExtensionRepo4_1_0 {
    struct bUserExtensionRepo4_1_0 *next;
    struct bUserExtensionRepo4_1_0 *prev;
    char name[64];
    char module[48];
    char custom_dirpath[1024];
    char remote_path[1024];
    int flag;
    char _pad0[4];
};

struct BoneCollectionReference4_1_0 {
    struct BoneCollectionReference4_1_0 *next;
    struct BoneCollectionReference4_1_0 *prev;
    struct BoneCollection4_1_0 *bcoll;
};

struct NodesModifierBake4_1_0 {
    int id;
    unsigned int flag;
    unsigned char bake_mode;
    char _pad[7];
    char *directory;
    int frame_start;
    int frame_end;
    int data_blocks_num;
    int active_data_block;
    struct NodesModifierDataBlock4_1_0 *data_blocks;
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

struct IDPropertyUIDataEnumItem4_1_0 {
    char *identifier;
    char *name;
    char *description;
    int value;
    int icon;
};

struct NodesModifierDataBlock4_1_0 {
    char *id_name;
    char *lib_name;
    void *id;
    int id_type;
    char _pad[4];
};

struct NodesModifierPanel4_1_0 {
    int id;
    unsigned int flag;
};

struct GreasePencilModifierInfluenceData4_1_0 {
    int flag;
    char _pad1[4];
    char layer_name[64];
    void *material;
    int layer_pass;
    int material_pass;
    char vertex_group_name[64];
    void *custom_curve;
    void *_pad2;
};

struct bNodeSocketValueMenu4_1_0 {
    int value;
    int runtime_flag;
    const  void *enum_items;
};

struct NodeEnumItem4_1_0 {
    char *name;
    char *description;
    int identifier;
    char _pad[4];
};

struct NodeEnumDefinition4_1_0 {
    struct NodeEnumItem4_1_0 *items_array;
    short items_num;
    short active_index;
    unsigned int next_identifier;
};

struct IndexSwitchItem4_1_0 {
    int identifier;
};

struct NodeIndexSwitch4_1_0 {
    struct IndexSwitchItem4_1_0 *items;
    int items_num;
    int data_type;
    int next_identifier;
    char _pad[4];
};

struct NodeGeometryBakeItem4_1_0 {
    char *name;
    short socket_type;
    short attribute_domain;
    int identifier;
    int flag;
    char _pad[4];
};

struct NodeGeometryBake4_1_0 {
    struct NodeGeometryBakeItem4_1_0 *items;
    int items_num;
    int next_identifier;
    int active_index;
    char _pad[4];
};

struct LayoutPanelState4_1_0 {
    struct LayoutPanelState4_1_0 *next;
    struct LayoutPanelState4_1_0 *prev;
    char *idname;
    unsigned char flag;
    char _pad[7];
};

struct FileHandler4_1_0 {
    void *type;
};

struct bPoseChannel_Runtime4_1_0 {
    struct SessionUID4_1_0 session_uid;
    struct DualQuat3_6_0 deform_dual_quat;
    int bbone_segments;
    float bbone_arc_length_reciprocal;
    char _pad1[4];
    void *bbone_rest_mats;
    void *bbone_pose_mats;
    void *bbone_deform_mats;
    struct DualQuat3_6_0 *bbone_dual_quats;
    struct bPoseChannel_BBoneSegmentBoundary4_0_0 *bbone_segment_boundaries;
    void *_pad;
};

struct bPose4_1_0 {
    struct ListBase3_6_0 chanbase;
    void *chanhash;
    struct bPoseChannel4_1_0 **chan_array;
    short flag;
    char _pad[2];
    float ctime;
    float stride_offset[3];
    float cyclic_offset[3];
    struct ListBase3_6_0 agroups;
    int active_group;
    int iksolver;
    void *ikdata;
    void *ikparam;
    struct bAnimVizSettings3_6_0 avs;
};

struct bDopeSheet4_1_0 {
    struct ID4_1_0 *source;
    struct ListBase3_6_0 chanbase;
    void *filter_grp;
    char searchstr[64];
    int filterflag;
    int filterflag2;
    int flag;
    int renameIndex;
};

struct ScrArea4_1_0 {
    struct ScrArea4_1_0 *next;
    struct ScrArea4_1_0 *prev;
    struct ScrVert3_6_0 *v1;
    struct ScrVert3_6_0 *v2;
    struct ScrVert3_6_0 *v3;
    struct ScrVert3_6_0 *v4;
    struct bScreen4_1_0 *full;
    struct rcti3_6_0 totrct;
    char spacetype;
    char butspacetype;
    short butspacetype_subtype;
    short winx;
    short winy;
    char headertype;
    char do_refresh;
    short flag;
    short region_active_win;
    char _pad[2];
    void *type;
    struct ScrGlobalAreaData3_6_0 *global;
    struct ListBase3_6_0 spacedata;
    struct ListBase3_6_0 regionbase;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 actionzones;
    struct ScrArea_Runtime3_6_0 runtime;
};

struct SequenceRuntime4_1_0 {
    struct SessionUID4_1_0 session_uid;
};

struct Editing4_1_0 {
    struct ListBase3_6_0 *seqbasep;
    struct ListBase3_6_0 *displayed_channels;
    void *_pad0;
    struct ListBase3_6_0 seqbase;
    struct ListBase3_6_0 metastack;
    struct ListBase3_6_0 channels;
    struct Sequence4_1_0 *act_seq;
    char act_imagedir[1024];
    char act_sounddir[1024];
    char proxy_dir[1024];
    int proxy_storage;
    int overlay_frame_ofs;
    int overlay_frame_abs;
    int overlay_frame_flag;
    struct rctf3_6_0 overlay_frame_rect;
    void *cache;
    float recycle_max_cost;
    int cache_flag;
    void *prefetch_job;
    long long disk_cache_timestamp;
    struct EditingRuntime3_6_0 runtime;
    void *_pad1;
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
    float minmax[2][3];
    struct Histogram3_6_0 hist;
    float *waveform_1;
    float *waveform_2;
    float *waveform_3;
    float *vecscope;
    float *vecscope_rgb;
};

struct SpaceText4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    void *text;
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
    void *runtime;
};

struct Panel4_1_0 {
    struct Panel4_1_0 *next;
    struct Panel4_1_0 *prev;
    void *type;
    void *layout;
    char panelname[64];
    char *drawname;
    int ofsx;
    int ofsy;
    int sizex;
    int sizey;
    int blocksizex;
    int blocksizey;
    short labelofs;
    short flag;
    short runtime_flag;
    char _pad[6];
    int sortorder;
    void *activedata;
    struct ListBase3_6_0 children;
    struct ListBase3_6_0 layout_panel_states;
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
    struct uiPanelColors3_6_0 panelcolors;
    struct ThemeAssetShelf4_0_0 asset_shelf;
    unsigned char shade1[4];
    unsigned char shade2[4];
    unsigned char hilite[4];
    unsigned char grid[4];
    unsigned char view_overlay[4];
    unsigned char wire[4];
    unsigned char wire_edit[4];
    unsigned char select[4];
    unsigned char lamp[4];
    unsigned char speaker[4];
    unsigned char empty[4];
    unsigned char camera[4];
    unsigned char active[4];
    unsigned char group[4];
    unsigned char group_active[4];
    unsigned char transform[4];
    unsigned char vertex[4];
    unsigned char vertex_select[4];
    unsigned char vertex_active[4];
    unsigned char vertex_bevel[4];
    unsigned char vertex_unreferenced[4];
    unsigned char edge[4];
    unsigned char edge_select[4];
    unsigned char edge_mode_select[4];
    unsigned char edge_seam[4];
    unsigned char edge_sharp[4];
    unsigned char edge_facesel[4];
    unsigned char edge_crease[4];
    unsigned char edge_bevel[4];
    unsigned char face[4];
    unsigned char face_select[4];
    unsigned char face_mode_select[4];
    unsigned char face_retopology[4];
    unsigned char face_back[4];
    unsigned char face_front[4];
    unsigned char face_dot[4];
    unsigned char extra_edge_len[4];
    unsigned char extra_edge_angle[4];
    unsigned char extra_face_angle[4];
    unsigned char extra_face_area[4];
    unsigned char normal[4];
    unsigned char vertex_normal[4];
    unsigned char loop_normal[4];
    unsigned char bone_solid[4];
    unsigned char bone_pose[4];
    unsigned char bone_pose_active[4];
    unsigned char bone_locked_weight[4];
    unsigned char strip[4];
    unsigned char strip_select[4];
    unsigned char cframe[4];
    unsigned char time_keyframe[4];
    unsigned char time_gp_keyframe[4];
    unsigned char freestyle_edge_mark[4];
    unsigned char freestyle_face_mark[4];
    unsigned char time_scrub_background[4];
    unsigned char time_marker_line[4];
    unsigned char time_marker_line_selected[4];
    unsigned char nurb_uline[4];
    unsigned char nurb_vline[4];
    unsigned char act_spline[4];
    unsigned char nurb_sel_uline[4];
    unsigned char nurb_sel_vline[4];
    unsigned char lastsel_point[4];
    unsigned char handle_free[4];
    unsigned char handle_auto[4];
    unsigned char handle_vect[4];
    unsigned char handle_align[4];
    unsigned char handle_auto_clamped[4];
    unsigned char handle_sel_free[4];
    unsigned char handle_sel_auto[4];
    unsigned char handle_sel_vect[4];
    unsigned char handle_sel_align[4];
    unsigned char handle_sel_auto_clamped[4];
    unsigned char ds_channel[4];
    unsigned char ds_subchannel[4];
    unsigned char ds_ipoline[4];
    unsigned char keytype_keyframe[4];
    unsigned char keytype_extreme[4];
    unsigned char keytype_breakdown[4];
    unsigned char keytype_jitter[4];
    unsigned char keytype_movehold[4];
    unsigned char keytype_keyframe_select[4];
    unsigned char keytype_extreme_select[4];
    unsigned char keytype_breakdown_select[4];
    unsigned char keytype_jitter_select[4];
    unsigned char keytype_movehold_select[4];
    unsigned char keyborder[4];
    unsigned char keyborder_select[4];
    char _pad4[3];
    unsigned char console_output[4];
    unsigned char console_input[4];
    unsigned char console_info[4];
    unsigned char console_error[4];
    unsigned char console_cursor[4];
    unsigned char console_select[4];
    unsigned char vertex_size;
    unsigned char edge_width;
    unsigned char outline_width;
    unsigned char obcenter_dia;
    unsigned char facedot_size;
    unsigned char noodle_curving;
    unsigned char grid_levels;
    char _pad5[2];
    float dash_alpha;
    unsigned char syntaxl[4];
    unsigned char syntaxs[4];
    unsigned char syntaxb[4];
    unsigned char syntaxn[4];
    unsigned char syntaxv[4];
    unsigned char syntaxc[4];
    unsigned char syntaxd[4];
    unsigned char syntaxr[4];
    unsigned char line_numbers[4];
    char _pad6[3];
    unsigned char nodeclass_output[4];
    unsigned char nodeclass_filter[4];
    unsigned char nodeclass_vector[4];
    unsigned char nodeclass_texture[4];
    unsigned char nodeclass_shader[4];
    unsigned char nodeclass_script[4];
    unsigned char nodeclass_pattern[4];
    unsigned char nodeclass_layout[4];
    unsigned char nodeclass_geometry[4];
    unsigned char nodeclass_attribute[4];
    unsigned char node_zone_simulation[4];
    unsigned char node_zone_repeat[4];
    unsigned char simulated_frames[4];
    unsigned char movie[4];
    unsigned char movieclip[4];
    unsigned char mask[4];
    unsigned char image[4];
    unsigned char scene[4];
    unsigned char audio[4];
    unsigned char effect[4];
    unsigned char transition[4];
    unsigned char meta[4];
    unsigned char text_strip[4];
    unsigned char color_strip[4];
    unsigned char active_strip[4];
    unsigned char selected_strip[4];
    char _pad7[1];
    float keyframe_scale_fac;
    unsigned char editmesh_active[4];
    unsigned char handle_vertex[4];
    unsigned char handle_vertex_select[4];
    unsigned char handle_vertex_size;
    unsigned char clipping_border_3d[4];
    unsigned char marker_outline[4];
    unsigned char marker[4];
    unsigned char act_marker[4];
    unsigned char sel_marker[4];
    unsigned char dis_marker[4];
    unsigned char lock_marker[4];
    unsigned char bundle_solid[4];
    unsigned char path_before[4];
    unsigned char path_after[4];
    unsigned char path_keyframe_before[4];
    unsigned char path_keyframe_after[4];
    unsigned char camera_path[4];
    unsigned char camera_passepartout[4];
    unsigned char _pad1[2];
    unsigned char gp_vertex_size;
    unsigned char gp_vertex[4];
    unsigned char gp_vertex_select[4];
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
    unsigned char nla_transition[4];
    unsigned char nla_transition_sel[4];
    unsigned char nla_meta[4];
    unsigned char nla_meta_sel[4];
    unsigned char nla_sound[4];
    unsigned char nla_sound_sel[4];
    unsigned char info_selected[4];
    unsigned char info_selected_text[4];
    unsigned char info_error[4];
    unsigned char info_error_text[4];
    unsigned char info_warning[4];
    unsigned char info_warning_text[4];
    unsigned char info_info[4];
    unsigned char info_info_text[4];
    unsigned char info_debug[4];
    unsigned char info_debug_text[4];
    unsigned char info_property[4];
    unsigned char info_property_text[4];
    unsigned char info_operator[4];
    unsigned char info_operator_text[4];
    unsigned char paint_curve_pivot[4];
    unsigned char paint_curve_handle[4];
    unsigned char metadatabg[4];
    unsigned char metadatatext[4];
};

struct SpaceScript4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct Script4_1_0 *script;
    short flags;
    short menunr;
    char _pad1[4];
    void *but_refs;
};

struct SubsurfModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    short subdivType;
    short levels;
    short renderLevels;
    short flags;
    short uv_smooth;
    short quality;
    short boundary_smooth;
    char _pad[2];
    void *emCache;
    void *mCache;
};

struct LatticeModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct CurveModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
    void *_pad1;
};

struct BuildModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float start;
    float length;
    short flag;
    short randomize;
    int seed;
};

struct MirrorModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    short axis;
    short flag;
    float tolerance;
    float bisect_threshold;
    unsigned char use_correct_order_on_merge;
    char _pad[3];
    float uv_offset[2];
    float uv_offset_copy[2];
    void *mirror_ob;
    void *_pad1;
};

struct DecimateModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float percent;
    short iter;
    char delimit;
    char symmetry_axis;
    float angle;
    char defgrp_name[64];
    float defgrp_factor;
    short flag;
    short mode;
    int face_count;
};

struct WaveModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *texture;
    void *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    void *objectcenter;
    char defgrp_name[64];
    short flag;
    char _pad2[2];
    float startx;
    float starty;
    float height;
    float width;
    float narrow;
    float speed;
    float damp;
    float falloff;
    float timeoffs;
    float lifetime;
    char _pad3[4];
    void *_pad4;
};

struct ArmatureModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    short deformflag;
    short multi;
    char _pad2[4];
    void *object;
    float *(vert_coords_prev[3]);
    char defgrp_name[64];
};

struct HookModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    char subtarget[64];
    char flag;
    char falloff_type;
    char _pad[6];
    float parentinv[4][4];
    float cent[3];
    float falloff;
    void *curfalloff;
    int *indexar;
    int indexar_num;
    float force;
    char name[64];
    void *_pad1;
};

struct SoftbodyModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
};

struct BooleanModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    void *collection;
    float double_threshold;
    char operation;
    char solver;
    char material_mode;
    char flag;
    char bm_flag;
    char _pad[7];
};

struct ArrayModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *start_cap;
    void *end_cap;
    void *curve_ob;
    void *offset_ob;
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

struct bNodeSocket4_1_0 {
    struct bNodeSocket4_1_0 *next;
    struct bNodeSocket4_1_0 *prev;
    struct IDProperty3_6_0 *prop;
    char identifier[64];
    char name[64];
    void *storage;
    short type;
    short flag;
    short limit;
    short in_out;
    void *typeinfo;
    char idname[64];
    void *default_value;
    short stack_index;
    char display_shape;
    char attribute_domain;
    char _pad[4];
    char label[64];
    char short_label[64];
    char description[64];
    char *default_attribute_name;
    int own_index;
    int to_index;
    struct bNodeLink4_1_0 *link;
    struct bNodeStack3_6_0 ns;
    void *runtime;
};

struct bNode4_1_0 {
    struct bNode4_1_0 *next;
    struct bNode4_1_0 *prev;
    struct ListBase3_6_0 inputs;
    struct ListBase3_6_0 outputs;
    char name[64];
    int identifier;
    int flag;
    char idname[64];
    void *typeinfo;
    short type;
    short ui_order;
    short custom1;
    short custom2;
    float custom3;
    float custom4;
    struct ID4_1_0 *id;
    void *storage;
    struct IDProperty3_6_0 *prop;
    struct bNode4_1_0 *parent;
    float locx;
    float locy;
    float width;
    float height;
    float offsetx;
    float offsety;
    char label[64];
    float color[3];
    int num_panel_states;
    struct bNodePanelState4_0_0 *panel_states_array;
    void *runtime;
};

struct EdgeSplitModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float split_angle;
    int flags;
};

struct DisplaceModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *texture;
    void *map_object;
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
    struct ModifierData4_1_0 modifier;
    void *projectors[10];
    char _pad2[4];
    int projectors_num;
    float aspectx;
    float aspecty;
    float scalex;
    float scaley;
    char uvlayer_name[68];
    int uvlayer_tmp;
};

struct Paint4_1_0 {
    void *brush;
    struct PaintToolSlot3_6_0 *tool_slots;
    int tool_slots_len;
    char _pad1[4];
    void *palette;
    void *cavity_curve;
    void *paint_cursor;
    unsigned char paint_cursor_col[4];
    int flags;
    int num_input_samples_deprecated;
    int symmetry_flags;
    float tile_offset[3];
    char _pad2[4];
    struct Paint_Runtime3_6_0 runtime;
};

struct SmoothModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float fac;
    char defgrp_name[64];
    short flag;
    short repeat;
};

struct CastModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag;
    short type;
    void *_pad1;
};

struct BevelModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
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
    void *custom_profile;
    void *_pad2;
};

struct ClothModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *clothObject;
    void *sim_parms;
    void *coll_parms;
    void *point_cache;
    struct ListBase3_6_0 ptcaches;
    void *hairdata;
    float hair_grid_min[3];
    float hair_grid_max[3];
    int hair_grid_res[3];
    float hair_grid_cellsize;
    void *solver_result;
};

struct CollisionModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float *(x[3]);
    float *(xnew[3]);
    float *(xold[3]);
    float *(current_xnew[3]);
    float *(current_x[3]);
    float *(current_v[3]);
    int *(vert_tris[3]);
    unsigned int mvert_num;
    unsigned int tri_num;
    float time_x;
    float time_xnew;
    char is_static;
    char _pad[7];
    void *bvhtree;
};

struct ParticleSystemModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *psys;
    void *mesh_final;
    void *mesh_original;
    int totdmvert;
    int totdmedge;
    int totdmface;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct ParticleInstanceModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *ob;
    short psys;
    short flag;
    short axis;
    short space;
    float position;
    float random_position;
    float rotation;
    float random_rotation;
    float particle_amount;
    float particle_offset;
    char index_layer_name[68];
    char value_layer_name[68];
    void *_pad1;
};

struct ExplodeModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    int *facepa;
    short flag;
    short vgroup;
    float protect;
    char uvname[68];
    char _pad1[4];
    void *_pad2;
};

struct MaskModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
    void *_pad1;
};

struct FluidsimModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *fss;
    void *_pad1;
};

struct ShrinkwrapModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *target;
    void *auxTarget;
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
    struct ModifierData4_1_0 modifier;
    void *origin;
    char vgroup_name[64];
    float factor;
    float limit[2];
    char mode;
    char axis;
    char deform_axis;
    char flag;
    void *_pad1;
};

struct SurfaceModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct SurfaceModifierData_Runtime3_6_0 runtime;
};

struct uiStyle4_1_0 {
    struct uiStyle4_1_0 *next;
    struct uiStyle4_1_0 *prev;
    char name[64];
    struct uiFontStyle4_1_0 paneltitle;
    struct uiFontStyle4_1_0 grouplabel;
    struct uiFontStyle4_1_0 widgetlabel;
    struct uiFontStyle4_1_0 widget;
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

struct NlaStrip4_1_0 {
    struct NlaStrip4_1_0 *next;
    struct NlaStrip4_1_0 *prev;
    struct ListBase3_6_0 strips;
    struct bAction4_1_0 *act;
    struct ListBase3_6_0 fcurves;
    struct ListBase3_6_0 modifiers;
    char name[64];
    float influence;
    float strip_time;
    float start;
    float end;
    float actstart;
    float actend;
    float repeat;
    float scale;
    float blendin;
    float blendout;
    short blendmode;
    short extendmode;
    char _pad1[2];
    short type;
    void *speaker_handle;
    int flag;
    char _pad2[4];
    struct NlaStrip4_1_0 *orig_strip;
    void *_pad3;
};

struct AnimData4_1_0 {
    struct bAction4_1_0 *action;
    struct bAction4_1_0 *tmpact;
    struct ListBase3_6_0 nla_tracks;
    struct NlaTrack3_6_0 *act_track;
    struct NlaStrip4_1_0 *actstrip;
    struct ListBase3_6_0 drivers;
    struct ListBase3_6_0 overrides;
    struct FCurve3_6_0 **driver_array;
    int flag;
    char _pad[4];
    short act_blendmode;
    short act_extendmode;
    float act_influence;
};

struct SmokeModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    int type;
    int _pad;
};

struct MultiresModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char lvl;
    char sculptlvl;
    char renderlvl;
    char totlvl;
    char simple;
    char flags;
    char _pad[2];
    short quality;
    short uv_smooth;
    short boundary_smooth;
    char _pad2[2];
};

struct ShapeKeyModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
};

struct SpaceFile4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char browse_mode;
    char _pad1[1];
    short tags;
    int scroll_offset;
    struct FileSelectParams4_1_0 *params;
    struct FileAssetSelectParams4_1_0 *asset_params;
    void *_pad2;
    void *files;
    struct ListBase3_6_0 *folders_prev;
    struct ListBase3_6_0 *folders_next;
    struct ListBase3_6_0 folder_histories;
    void *op;
    void *smoothscroll_timer;
    void *previews_timer;
    void *layout;
    short recentnr;
    short bookmarknr;
    short systemnr;
    short system_bookmarknr;
    void *runtime;
};

struct SpaceConsole4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct ListBase3_6_0 scrollback;
    struct ListBase3_6_0 history;
    char prompt[256];
    char language[32];
    int lheight;
    int history_index;
    int sel_start;
    int sel_end;
};

struct ReportList4_1_0 {
    struct ListBase3_6_0 list;
    int printlevel;
    int storelevel;
    int flag;
    char _pad[4];
    void *reporttimer;
    void *lock;
};

struct wmOperator4_1_0 {
    struct wmOperator4_1_0 *next;
    struct wmOperator4_1_0 *prev;
    char idname[64];
    struct IDProperty3_6_0 *properties;
    void *type;
    void *customdata;
    void *py_instance;
    void *ptr;
    struct ReportList4_1_0 *reports;
    struct ListBase3_6_0 macro;
    struct wmOperator4_1_0 *opm;
    void *layout;
    short flag;
    char _pad[6];
};

struct SolidifyModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
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
    struct ModifierData4_1_0 modifier;
    void *ob_axis;
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

struct MappingInfoModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *texture;
    void *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
};

struct WarpModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *texture;
    void *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
    void *object_from;
    void *object_to;
    char bone_from[64];
    char bone_to[64];
    void *curfalloff;
    char defgrp_name[64];
    float strength;
    float falloff_radius;
    char flag;
    char falloff_type;
    char _pad2[6];
    void *_pad3;
};

struct WeightVGEditModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char defgrp_name[64];
    short edit_flags;
    short falloff_type;
    float default_weight;
    void *cmap_curve;
    float add_threshold;
    float rem_threshold;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    void *mask_texture;
    void *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    void *_pad1;
};

struct WeightVGMixModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
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
    void *mask_texture;
    void *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    char _pad1[4];
    char flag;
    char _pad2[3];
};

struct WeightVGProximityModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char defgrp_name[64];
    void *cmap_curve;
    int proximity_mode;
    int proximity_flags;
    void *proximity_ob_target;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    void *mask_texture;
    void *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[68];
    char _pad1[4];
    float min_dist;
    float max_dist;
    short falloff_type;
    char _pad0[2];
};

struct OceanModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
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

struct DynamicPaintModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *canvas;
    void *brush;
    int type;
    char _pad[4];
};

struct RemeshModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
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

struct MovieTrackingObject4_1_0 {
    struct MovieTrackingObject4_1_0 *next;
    struct MovieTrackingObject4_1_0 *prev;
    char name[64];
    int flag;
    float scale;
    struct ListBase3_6_0 tracks;
    struct ListBase3_6_0 plane_tracks;
    struct MovieTrackingTrack3_6_0 *active_track;
    struct MovieTrackingPlaneTrack4_1_0 *active_plane_track;
    struct MovieTrackingReconstruction3_6_0 reconstruction;
    int keyframe1;
    int keyframe2;
};

struct MaskSplinePoint4_1_0 {
    struct BezTriple3_6_0 bezt;
    char _pad[4];
    int tot_uw;
    struct MaskSplinePointUW3_6_0 *uw;
    struct MaskParent4_1_0 parent;
};

struct MaskSpline4_1_0 {
    struct MaskSpline4_1_0 *next;
    struct MaskSpline4_1_0 *prev;
    short flag;
    char offset_mode;
    char weight_interp;
    int tot_point;
    struct MaskSplinePoint4_1_0 *points;
    struct MaskParent4_1_0 parent;
    struct MaskSplinePoint4_1_0 *points_deform;
};

struct MaskLayer4_1_0 {
    struct MaskLayer4_1_0 *next;
    struct MaskLayer4_1_0 *prev;
    char name[64];
    struct ListBase3_6_0 splines;
    struct ListBase3_6_0 splines_shapes;
    struct MaskSpline4_1_0 *act_spline;
    struct MaskSplinePoint4_1_0 *act_point;
    float alpha;
    char blend;
    char blend_flag;
    char falloff;
    char _pad[7];
    char flag;
    char visibility_flag;
};

struct SkinModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct ColorBalanceModifierData4_1_0 {
    struct SequenceModifierData4_1_0 modifier;
    struct StripColorBalance3_6_0 color_balance;
    float color_multiply;
};

struct BrightContrastModifierData4_1_0 {
    struct SequenceModifierData4_1_0 modifier;
    float bright;
    float contrast;
};

struct TriangulateModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float lambda;
    float lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag;
    short repeat;
};

struct UVWarpModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char axis_u;
    char axis_v;
    short flag;
    float center[2];
    float offset[2];
    float scale[2];
    float rotation;
    void *object_src;
    char bone_src[64];
    void *object_dst;
    char bone_dst[64];
    char vgroup_name[64];
    char uvlayer_name[68];
    char _pad[4];
};

struct MeshCacheModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
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

struct bNodeTreePath4_1_0 {
    struct bNodeTreePath4_1_0 *next;
    struct bNodeTreePath4_1_0 *prev;
    struct bNodeTree4_1_0 *nodetree;
    struct bNodeInstanceKey3_6_0 parent_key;
    char _pad[4];
    float view_center[2];
    char node_name[64];
    char display_name[64];
};

struct SequencerMaskModifierData4_1_0 {
    struct SequenceModifierData4_1_0 modifier;
};

struct LaplacianDeformModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char anchor_grp_name[64];
    int verts_num;
    int repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag;
    short mat_ofs;
    char _pad[4];
};

struct DataTransferModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *ob_source;
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
    struct ModifierData4_1_0 modifier;
    char defgrp_name[64];
    void *target;
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

struct CorrectiveSmoothModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float *(bind_coords[3]);
    unsigned int bind_coords_num;
    float lambda;
    float scale;
    short repeat;
    short flag;
    char smooth_type;
    char rest_source;
    char _pad[6];
    char defgrp_name[64];
    struct CorrectiveSmoothDeltaCache3_6_0 delta_cache;
};

struct WhiteBalanceModifierData4_1_0 {
    struct SequenceModifierData4_1_0 modifier;
    float white_value[3];
    char _pad[4];
};

struct SequencerTonemapModifierData4_1_0 {
    struct SequenceModifierData4_1_0 modifier;
    float key;
    float offset;
    float gamma;
    float intensity;
    float contrast;
    float adaptation;
    float correction;
    int type;
};

struct MeshSeqCacheModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
};

struct SurfaceDeformModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *depsgraph;
    void *target;
    struct SDefVert3_6_0 *verts;
    void *_pad1;
    float falloff;
    unsigned int mesh_verts_num;
    unsigned int bind_verts_num;
    unsigned int target_verts_num;
    unsigned int target_polys_num;
    int flags;
    float mat[4][4];
    float strength;
    char defgrp_name[64];
    int _pad2;
};

struct IDOverrideLibrary4_1_0 {
    struct ID4_1_0 *reference;
    struct ListBase3_6_0 properties;
    struct ID4_1_0 *hierarchy_root;
    struct ID4_1_0 *storage;
    struct IDOverrideLibraryRuntime3_6_0 *runtime;
    unsigned int flag;
    char _pad_1[4];
};

struct CameraBGImage4_1_0 {
    struct CameraBGImage4_1_0 *next;
    struct CameraBGImage4_1_0 *prev;
    struct Image4_1_0 *ima;
    struct ImageUser3_6_0 iuser;
    struct MovieClip4_1_0 *clip;
    struct MovieClipUser3_6_0 cuser;
    float offset[2];
    float scale;
    float rotation;
    float alpha;
    short flag;
    short source;
};

struct CollectionChild4_1_0 {
    struct CollectionChild4_1_0 *next;
    struct CollectionChild4_1_0 *prev;
    struct Collection4_1_0 *collection;
    struct CollectionLightLinking4_0_0 light_linking;
    int _pad;
};

struct Collection_Runtime4_1_0 {
    struct ListBase3_6_0 object_cache;
    struct ListBase3_6_0 object_cache_instanced;
    struct ListBase3_6_0 parents;
    void *gobject_hash;
    unsigned char tag;
    char _pad0[7];
};

struct WeightedNormalModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    char defgrp_name[64];
    char mode;
    char flag;
    short weight;
    float thresh;
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
    struct RaytraceEEVEE4_1_0 ray_tracing_options;
    void *light_cache;
    void *light_cache_data;
    char light_cache_info[128];
    float overscan;
    float light_threshold;
};

struct FluidModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *domain;
    void *flow;
    void *effector;
    float time;
    int type;
    void *_pad1;
};

struct WeldModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct GP_Sculpt_Settings4_1_0 {
    void *paintcursor;
    int flag;
    int lock_axis;
    float isect_threshold;
    char _pad[4];
    void *cur_falloff;
    void *cur_primitive;
    struct GP_Sculpt_Guide4_1_0 guide;
};

struct XrSessionSettings4_1_0 {
    struct View3DShading3_6_0 shading;
    float base_scale;
    char _pad[3];
    char base_pose_type;
    struct Object4_1_0 *base_pose_object;
    float base_pose_location[3];
    float base_pose_angle;
    char draw_flags;
    char controller_draw_style;
    char _pad2[2];
    float clip_start;
    float clip_end;
    int flag;
    int object_type_exclude_viewport;
    int object_type_exclude_select;
};

struct MeshToVolumeModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    float interior_band_width;
    float density;
    char _pad2[4];
    void *_pad3;
};

struct VolumeDisplaceModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *texture;
    void *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

struct VolumeToMeshModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *object;
    float threshold;
    float adaptivity;
    unsigned int flag;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char grid_name[64];
    void *_pad1;
};

struct NodesModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    void *node_group;
    struct NodesModifierSettings3_6_0 settings;
    char *bake_directory;
    signed char flag;
    char _pad[3];
    int bakes_num;
    struct NodesModifierBake4_1_0 *bakes;
    char _pad2[4];
    int panels_num;
    struct NodesModifierPanel4_1_0 *panels;
    void *runtime;
};

struct FileAssetSelectParams4_1_0 {
    struct FileSelectParams4_1_0 base_params;
    struct AssetLibraryReference3_6_0 asset_library_ref;
    short asset_catalog_visibility;
    char _pad[6];
    struct bUUID3_6_0 catalog_id;
    short import_method;
    char _pad2[6];
};

struct IDPropertyUIDataInt4_1_0 {
    struct IDPropertyUIData3_6_0 base;
    int *default_array;
    int default_array_len;
    int min;
    int max;
    int soft_min;
    int soft_max;
    int step;
    int default_value;
    int enum_items_num;
    struct IDPropertyUIDataEnumItem4_1_0 *enum_items;
};

struct PaintModeSettings4_1_0 {
    char canvas_source;
    char _pad[7];
    struct Image4_1_0 *canvas_image;
    struct ImageUser3_6_0 image_user;
};

struct IDViewerPathElem4_1_0 {
    struct ViewerPathElem4_0_0 base;
    struct ID4_1_0 *id;
};

struct BoneCollection4_1_0 {
    struct BoneCollection4_1_0 *next;
    struct BoneCollection4_1_0 *prev;
    char name[64];
    struct ListBase3_6_0 bones;
    unsigned char flags;
    unsigned char _pad0[7];
    int child_index;
    int child_count;
    struct IDProperty3_6_0 *prop;
};

struct GreasePencilDrawingReference4_1_0 {
    struct GreasePencilDrawingBase4_0_0 base;
    struct GreasePencil4_1_0 *id_reference;
};

struct bNodeTreeInterfaceSocket4_1_0 {
    struct bNodeTreeInterfaceItem4_0_0 item;
    char *name;
    char *description;
    char *socket_type;
    int flag;
    short attribute_domain;
    short default_input;
    char *default_attribute_name;
    char *identifier;
    void *socket_data;
    struct IDProperty3_6_0 *properties;
};

struct SoundEqualizerModifierData4_1_0 {
    struct SequenceModifierData4_1_0 modifier;
    struct ListBase3_6_0 graphics;
};

struct GreasePencilOpacityModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    char color_mode;
    char _pad1[3];
    float color_factor;
    float hardness_factor;
    void *_pad2;
};

struct GreasePencilSubdivModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    int type;
    int level;
    char _pad[8];
    void *_pad1;
};

struct GreasePencilColorModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    char color_mode;
    char _pad1[3];
    float hsv[3];
    void *_pad2;
};

struct GreasePencilTintModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    short flag;
    char color_mode;
    char tint_mode;
    float factor;
    float radius;
    float color[3];
    void *object;
    void *color_ramp;
    void *_pad;
};

struct GreasePencilSmoothModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float factor;
    int step;
    char _pad[4];
    void *_pad1;
};

struct GreasePencilOffsetModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
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
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
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
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    void *object;
    int flag;
    char _pad[4];
};

struct GreasePencilThickModifierData4_1_0 {
    struct ModifierData4_1_0 modifier;
    struct GreasePencilModifierInfluenceData4_1_0 influence;
    int flag;
    float thickness_fac;
    float thickness;
    char _pad[4];
    void *_pad1;
};

struct NodeMenuSwitch4_1_0 {
    struct NodeEnumDefinition4_1_0 enum_definition;
    unsigned char data_type;
    char _pad[7];
};

struct ID4_1_0 {
    void *next;
    void *prev;
    struct ID4_1_0 *newid;
    struct Library4_1_0 *lib;
    void *asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    unsigned int recalc;
    unsigned int recalc_up_to_undo_push;
    unsigned int recalc_after_undo_push;
    unsigned int session_uid;
    struct IDProperty3_6_0 *properties;
    struct IDOverrideLibrary4_1_0 *override_library;
    struct ID4_1_0 *orig_id;
    void *py_instance;
    struct LibraryWeakReference3_6_0 *library_weak_reference;
    struct ID_Runtime3_6_0 runtime;
};

struct bPoseChannel4_1_0 {
    struct bPoseChannel4_1_0 *next;
    struct bPoseChannel4_1_0 *prev;
    struct IDProperty3_6_0 *prop;
    struct ListBase3_6_0 constraints;
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
    struct Bone4_0_0 *bone;
    struct bPoseChannel4_1_0 *parent;
    struct bPoseChannel4_1_0 *child;
    struct ListBase3_6_0 iktree;
    struct ListBase3_6_0 siktree;
    struct bMotionPath3_6_0 *mpath;
    struct Object4_1_0 *custom;
    struct bPoseChannel4_1_0 *custom_tx;
    float custom_scale;
    float custom_scale_xyz[3];
    float custom_translation[3];
    float custom_rotation_euler[3];
    float loc[3];
    float size[3];
    float eul[3];
    float quat[4];
    float rotAxis[3];
    float rotAngle;
    short rotmode;
    char _pad[2];
    float chan_mat[4][4];
    float pose_mat[4][4];
    float disp_mat[4][4];
    float disp_tail_mat[4][4];
    float constinv[4][4];
    float pose_head[3];
    float pose_tail[3];
    float limitmin[3];
    float limitmax[3];
    float stiffness[3];
    float ikstretch;
    float ikrotweight;
    float iklinweight;
    float roll1;
    float roll2;
    float curve_in_x;
    float curve_in_z;
    float curve_out_x;
    float curve_out_z;
    float ease1;
    float ease2;
    float scale_in_x;
    float scale_in_z;
    float scale_out_x;
    float scale_out_z;
    float scale_in[3];
    float scale_out[3];
    struct bPoseChannel4_1_0 *bbone_prev;
    struct bPoseChannel4_1_0 *bbone_next;
    void *temp;
    struct bPoseChannelDrawData3_6_0 *draw_data;
    struct bPoseChannel4_1_0 *orig_pchan;
    struct BoneColor4_0_0 color;
    struct bPoseChannel_Runtime4_1_0 runtime;
};

struct SpaceAction4_1_0 {
    void *next;
    void *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct bAction4_1_0 *action;
    struct bDopeSheet4_1_0 ads;
    float timeslide;
    short flag;
    char mode;
    char mode_prev;
    char autosnap;
    char cache_display;
    char _pad1[6];
    struct SpaceAction_Runtime3_6_0 runtime;
};

struct Sequence4_1_0 {
    struct Sequence4_1_0 *next;
    struct Sequence4_1_0 *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag;
    int type;
    int len;
    float start;
    float startofs;
    float endofs;
    float startstill;
    float endstill;
    int machine;
    int _pad;
    int startdisp;
    int enddisp;
    float sat;
    float mul;
    float _pad1;
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    struct Strip3_6_0 *strip;
    void *ipo;
    void *scene;
    void *scene_camera;
    void *clip;
    void *mask;
    struct ListBase3_6_0 anims;
    float effect_fader;
    float speed_fader;
    struct Sequence4_1_0 *seq1;
    struct Sequence4_1_0 *seq2;
    struct Sequence4_1_0 *seq3;
    struct ListBase3_6_0 seqbase;
    struct ListBase3_6_0 channels;
    void *sound;
    void *scene_sound;
    float volume;
    float pitch;
    float pan;
    float strobe;
    void *effectdata;
    int anim_startofs;
    int anim_endofs;
    int blend_mode;
    float blend_opacity;
    signed char color_tag;
    char alpha_mode;
    char _pad2[2];
    int cache_flag;
    int sfra;
    char views_format;
    char _pad3[3];
    void *stereo3d_format;
    void *prop;
    struct ListBase3_6_0 modifiers;
    float media_playback_rate;
    float speed_factor;
    struct SeqRetimingKey4_1_0 *retiming_keys;
    void *_pad5;
    int retiming_keys_num;
    char _pad6[4];
    struct SequenceRuntime4_1_0 runtime;
};

struct SpaceSeq4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    float xof;
    float yof;
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
    void *gpd;
    struct SequencerPreviewOverlay3_6_0 preview_overlay;
    struct SequencerTimelineOverlay3_6_0 timeline_overlay;
    char multiview_eye;
    char _pad2[7];
    void *runtime;
};

struct SpaceImage4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct Image4_1_0 *image;
    struct ImageUser3_6_0 iuser;
    struct Scopes4_1_0 scopes;
    struct Histogram3_6_0 sample_line_hist;
    void *gpd;
    float cursor[2];
    float xof;
    float yof;
    float zoom;
    float centx;
    float centy;
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
    struct MaskSpaceInfo3_6_0 mask_info;
    struct SpaceImageOverlay3_6_0 overlay;
};

struct SpaceNla4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    short autosnap;
    short flag;
    char _pad[4];
    struct bDopeSheet4_1_0 *ads;
    struct View2D4_0_0 v2d;
};

struct SpaceNode4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct ID4_1_0 *id;
    struct ID4_1_0 *from;
    short flag;
    char insert_ofs_dir;
    char _pad1;
    float xof;
    float yof;
    float zoom;
    struct ListBase3_6_0 treepath;
    struct bNodeTree4_1_0 *edittree;
    struct bNodeTree4_1_0 *nodetree;
    char tree_idname[64];
    int treetype;
    short texfrom;
    char shaderfrom;
    char geometry_nodes_type;
    struct bNodeTree4_1_0 *geometry_nodes_tool_tree;
    void *gpd;
    struct SpaceNodeOverlay4_0_0 overlay;
    void *runtime;
};

struct ImagePaintSettings4_1_0 {
    struct Paint4_1_0 paint;
    short flag;
    short missing_data;
    short seam_bleed;
    short normal_angle;
    short screen_grab_size[2];
    int mode;
    struct Image4_1_0 *stencil;
    struct Image4_1_0 *clone;
    struct Image4_1_0 *canvas;
    float stencil_col[3];
    float dither;
    int interp;
    char _pad[4];
};

struct Sculpt4_1_0 {
    struct Paint4_1_0 paint;
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
    float automasking_start_normal_limit;
    float automasking_start_normal_falloff;
    float automasking_view_normal_limit;
    float automasking_view_normal_falloff;
    void *automasking_cavity_curve;
    void *automasking_cavity_curve_op;
    struct Object4_1_0 *gravity_object;
};

struct VPaint4_1_0 {
    struct Paint4_1_0 paint;
    char flag;
    char _pad[3];
    int radial_symm[3];
};

struct wmXrData4_1_0 {
    void *runtime;
    struct XrSessionSettings4_1_0 session_settings;
};

struct wmWindow4_1_0 {
    struct wmWindow4_1_0 *next;
    struct wmWindow4_1_0 *prev;
    void *ghostwin;
    void *gpuctx;
    struct wmWindow4_1_0 *parent;
    void *scene;
    void *new_scene;
    char view_layer_name[64];
    void *unpinned_scene;
    void *workspace_hook;
    struct ScrAreaMap3_6_0 global_areas;
    struct bScreen4_1_0 *screen;
    int winid;
    short posx;
    short posy;
    short sizex;
    short sizey;
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
    const  void *ime_data;
    char ime_data_is_composing;
    char _pad1[7];
    struct ListBase3_6_0 event_queue;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 modalhandlers;
    struct ListBase3_6_0 gesture;
    void *stereo3d_format;
    struct ListBase3_6_0 drawcalls;
    void *cursor_keymap_status;
    unsigned long long eventstate_prev_press_time_ms;
};

struct MovieTracking4_1_0 {
    struct MovieTrackingSettings3_6_0 settings;
    struct MovieTrackingCamera3_6_0 camera;
    struct ListBase3_6_0 tracks_legacy;
    struct ListBase3_6_0 plane_tracks_legacy;
    struct MovieTrackingReconstruction3_6_0 reconstruction_legacy;
    struct MovieTrackingStabilization3_6_0 stabilization;
    struct MovieTrackingTrack3_6_0 *act_track_legacy;
    struct MovieTrackingPlaneTrack4_1_0 *act_plane_track_legacy;
    struct ListBase3_6_0 objects;
    int objectnr;
    int tot_object;
    struct MovieTrackingStats3_6_0 *stats;
    struct MovieTrackingDopesheet3_6_0 dopesheet;
};

struct SpaceClip4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char gizmo_flag;
    char _pad1[3];
    float xof;
    float yof;
    float xlockof;
    float ylockof;
    float zoom;
    struct MovieClipUser3_6_0 user;
    struct MovieClip4_1_0 *clip;
    struct MovieClipScopes3_6_0 scopes;
    int flag;
    short mode;
    short view;
    int path_length;
    float loc[2];
    float scale;
    float angle;
    char _pad[4];
    float stabmat[4][4];
    float unistabmat[4][4];
    int postproc_flag;
    short gpencil_src;
    char _pad2[2];
    int around;
    char _pad4[4];
    float cursor[2];
    struct MaskSpaceInfo3_6_0 mask_info;
};

struct UvSculpt4_1_0 {
    struct Paint4_1_0 paint;
};

struct BakeData4_1_0 {
    struct ImageFormatData3_6_0 im_format;
    char filepath[1024];
    short width;
    short height;
    short margin;
    short flag;
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
    struct Object4_1_0 *cage_object;
};

struct ViewLayer4_1_0 {
    struct ViewLayer4_1_0 *next;
    struct ViewLayer4_1_0 *prev;
    char name[64];
    short flag;
    char _pad[6];
    struct ListBase3_6_0 object_bases;
    void *stats;
    struct Base4_1_0 *basact;
    struct ListBase3_6_0 layer_collections;
    struct LayerCollection4_0_0 *active_collection;
    int layflag;
    int passflag;
    float pass_alpha_threshold;
    short cryptomatte_flag;
    short cryptomatte_levels;
    char _pad1[4];
    int samples;
    void *mat_override;
    struct IDProperty3_6_0 *id_properties;
    struct FreestyleConfig3_6_0 freestyle_config;
    struct ViewLayerEEVEE3_6_0 eevee;
    struct ListBase3_6_0 aovs;
    struct ViewLayerAOV3_6_0 *active_aov;
    struct ListBase3_6_0 lightgroups;
    struct ViewLayerLightgroup3_6_0 *active_lightgroup;
    struct ListBase3_6_0 drawdata;
    struct Base4_1_0 **object_bases_array;
    void *object_bases_hash;
};

struct GpPaint4_1_0 {
    struct Paint4_1_0 paint;
    int flag;
    int mode;
};

struct SpaceProperties4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    short space_subtype;
    short mainb;
    short mainbo;
    short mainbuser;
    short preview;
    char _pad[4];
    char flag;
    char outliner_sync;
    void *path;
    int pathflag;
    int dataicon;
    struct ID4_1_0 *pinid;
    void *texuser;
    void *runtime;
};

struct SpaceGraph4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D4_0_0 v2d;
    struct bDopeSheet4_1_0 *ads;
    short mode;
    short autosnap;
    int flag;
    float cursorTime;
    float cursorVal;
    int around;
    char _pad[4];
    struct SpaceGraph_Runtime3_6_0 runtime;
};

struct GpVertexPaint4_1_0 {
    struct Paint4_1_0 paint;
    int flag;
    char _pad[4];
};

struct GpSculptPaint4_1_0 {
    struct Paint4_1_0 paint;
    int flag;
    char _pad[4];
};

struct GpWeightPaint4_1_0 {
    struct Paint4_1_0 paint;
    int flag;
    char _pad[4];
};

struct SpaceSpreadsheet4_1_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct ListBase3_6_0 columns;
    struct ListBase3_6_0 row_filters;
    struct ViewerPath3_6_0 viewer_path;
    unsigned char filter_flag;
    unsigned char geometry_component_type;
    unsigned char attribute_domain;
    unsigned char object_eval_state;
    int active_layer_index;
    unsigned int flag;
    char _pad1[4];
    void *runtime;
};

struct CurvesSculpt4_1_0 {
    struct Paint4_1_0 paint;
};

struct Library4_1_0 {
    struct ID4_1_0 id;
    void *filedata;
    char filepath[1024];
    char filepath_abs[1024];
    struct Library4_1_0 *parent;
    void *packedfile;
    unsigned short tag;
    char _pad_0[6];
    int temp_index;
    short versionfile;
    short subversionfile;
    struct Library_Runtime3_6_0 runtime;
};

struct bAction4_1_0 {
    struct ID4_1_0 id;
    struct ListBase3_6_0 curves;
    struct ListBase3_6_0 chanbase;
    struct ListBase3_6_0 groups;
    struct ListBase3_6_0 markers;
    int flag;
    int active_marker;
    int idroot;
    char _pad[4];
    float frame_start;
    float frame_end;
    struct PreviewImage3_6_0 *preview;
};

struct bArmature4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct ListBase3_6_0 bonebase;
    void *bonehash;
    void *_pad1;
    struct ListBase3_6_0 *edbo;
    struct Bone4_0_0 *act_bone;
    void *act_edbone;
    char needs_flush_to_id;
    char _pad0[3];
    int flag;
    int drawtype;
    short deformflag;
    short pathflag;
    struct ListBase3_6_0 collections_legacy;
    struct BoneCollection4_1_0 **collection_array;
    int collection_array_num;
    int collection_root_count;
    char active_collection_name[64];
    unsigned int layer_used;
    unsigned int layer;
    unsigned int layer_protected;
    float axes_position;
    struct bArmature_Runtime4_1_0 runtime;
};

struct Camera4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    char type;
    char dtx;
    short flag;
    float passepartalpha;
    float clip_start;
    float clip_end;
    float lens;
    float ortho_scale;
    float drawsize;
    float sensor_x;
    float sensor_y;
    float shiftx;
    float shifty;
    float dof_distance;
    char sensor_fit;
    char panorama_type;
    char _pad[2];
    float fisheye_fov;
    float fisheye_lens;
    float latitude_min;
    float latitude_max;
    float longitude_min;
    float longitude_max;
    float fisheye_polynomial_k0;
    float fisheye_polynomial_k1;
    float fisheye_polynomial_k2;
    float fisheye_polynomial_k3;
    float fisheye_polynomial_k4;
    void *ipo;
    void *dof_ob;
    struct GPUDOFSettings3_6_0 gpu_dof;
    struct CameraDOFSettings3_6_0 dof;
    struct ListBase3_6_0 bg_images;
    struct CameraStereoSettings3_6_0 stereo;
    struct Camera_Runtime3_6_0 runtime;
};

struct Ipo4_1_0 {
    struct ID4_1_0 id;
    struct ListBase3_6_0 curve;
    struct rctf3_6_0 cur;
    short blocktype;
    short showkey;
    short muteipo;
    char _pad[2];
};

struct Object4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    void *sculpt;
    short type;
    short partype;
    int par1;
    int par2;
    int par3;
    char parsubstr[64];
    struct Object4_1_0 *parent;
    struct Object4_1_0 *track;
    struct Object4_1_0 *proxy;
    struct Object4_1_0 *proxy_group;
    struct Object4_1_0 *proxy_from;
    void *ipo;
    struct bAction4_1_0 *action;
    struct bAction4_1_0 *poselib;
    struct bPose4_1_0 *pose;
    void *data;
    void *gpd;
    struct bAnimVizSettings3_6_0 avs;
    struct bMotionPath3_6_0 *mpath;
    void *_pad0;
    struct ListBase3_6_0 constraintChannels;
    struct ListBase3_6_0 effect;
    struct ListBase3_6_0 defbase;
    struct ListBase3_6_0 fmaps;
    struct ListBase3_6_0 modifiers;
    struct ListBase3_6_0 greasepencil_modifiers;
    struct ListBase3_6_0 shader_fx;
    int mode;
    int restore_mode;
    void *mat;
    char *matbits;
    int totcol;
    int actcol;
    float loc[3];
    float dloc[3];
    float scale[3];
    float dsize[3];
    float dscale[3];
    float rot[3];
    float drot[3];
    float quat[4];
    float dquat[4];
    float rotAxis[3];
    float drotAxis[3];
    float rotAngle;
    float drotAngle;
    float object_to_world[4][4];
    float world_to_object[4][4];
    float parentinv[4][4];
    float constinv[4][4];
    unsigned int lay;
    short flag;
    short colbits;
    short transflag;
    short protectflag;
    short trackflag;
    short upflag;
    short nlaflag;
    char _pad1;
    char duplicator_visibility_flag;
    short base_flag;
    unsigned short base_local_view_bits;
    unsigned short col_group;
    unsigned short col_mask;
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
    struct ListBase3_6_0 constraints;
    struct ListBase3_6_0 nlastrips;
    struct ListBase3_6_0 hooks;
    struct ListBase3_6_0 particlesystem;
    void *pd;
    void *soft;
    void *instance_collection;
    void *fluidsimSettings;
    struct ListBase3_6_0 pc_ids;
    void *rigidbody_object;
    void *rigidbody_constraint;
    float ima_ofs[2];
    struct ImageUser3_6_0 *iuser;
    char empty_image_visibility_flag;
    char empty_image_depth;
    char empty_image_flag;
    unsigned char modifier_flag;
    char _pad8[4];
    struct PreviewImage3_6_0 *preview;
    struct ObjectLineArt3_6_0 lineart;
    void *lightgroup;
    struct LightLinking4_0_0 *light_linking;
    void *lightprobe_cache;
    void *runtime;
};

struct Curve4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct ListBase3_6_0 nurb;
    struct EditNurb3_6_0 *editnurb;
    void *bevobj;
    void *taperobj;
    void *textoncurve;
    void *ipo;
    void *key;
    void *mat;
    void *bevel_profile;
    float texspace_location[3];
    float texspace_size[3];
    short type;
    char texspace_flag;
    char _pad0[7];
    short twist_mode;
    float twist_smooth;
    float smallcaps_scale;
    int pathlen;
    short bevresol;
    short totcol;
    int flag;
    float offset;
    float extrude;
    float bevel_radius;
    short resolu;
    short resolv;
    short resolu_ren;
    short resolv_ren;
    int actnu;
    int actvert;
    char overflow;
    char spacemode;
    char align_y;
    char bevel_mode;
    char taper_radius_mode;
    char _pad;
    short lines;
    float spacing;
    float linedist;
    float shear;
    float fsize;
    float wordspace;
    float ulpos;
    float ulheight;
    float xof;
    float yof;
    float linewidth;
    int pos;
    int selstart;
    int selend;
    int len_char32;
    int len;
    char *str;
    void *editfont;
    char family[64];
    void *vfont;
    void *vfontb;
    void *vfonti;
    void *vfontbi;
    struct TextBox3_6_0 *tb;
    int totbox;
    int actbox;
    struct CharInfo3_6_0 *strinfo;
    struct CharInfo3_6_0 curinfo;
    float ctime;
    float bevfac1;
    float bevfac2;
    char bevfac1_mapping;
    char bevfac2_mapping;
    char _pad2[6];
    float fsize_realtime;
    const  void *curve_eval;
    char edit_data_from_original;
    char _pad3[7];
    void *batch_cache;
};

struct Image4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    char filepath[1024];
    void *cache;
    void *gputexture[2][3];
    struct ListBase3_6_0 anims;
    void *rr;
    struct ListBase3_6_0 renderslots;
    short render_slot;
    short last_render_slot;
    int flag;
    short source;
    short type;
    int lastframe;
    int gpuframenr;
    short gpuflag;
    short gpu_pass;
    short gpu_layer;
    short gpu_view;
    short seam_margin;
    char _pad2[2];
    void *packedfile;
    struct ListBase3_6_0 packedfiles;
    struct PreviewImage3_6_0 *preview;
    int lastused;
    int gen_x;
    int gen_y;
    char gen_type;
    char gen_flag;
    short gen_depth;
    float gen_color[4];
    float aspx;
    float aspy;
    struct ColorManagedColorspaceSettings3_6_0 colorspace_settings;
    char alpha_mode;
    char _pad;
    char eye;
    char views_format;
    int offset_x;
    int offset_y;
    int active_tile_index;
    struct ListBase3_6_0 tiles;
    struct ListBase3_6_0 views;
    void *stereo3d_format;
    struct Image_Runtime3_6_0 runtime;
};

struct Key4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct KeyBlock3_6_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    struct ListBase3_6_0 block;
    void *ipo;
    struct ID4_1_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct Lattice4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    short pntsu;
    short pntsv;
    short pntsw;
    short flag;
    short opntsu;
    short opntsv;
    short opntsw;
    char _pad2[3];
    char typeu;
    char typev;
    char typew;
    int actbp;
    float fu;
    float fv;
    float fw;
    float du;
    float dv;
    float dw;
    void *def;
    void *ipo;
    void *key;
    void *dvert;
    char vgroup[64];
    struct ListBase3_6_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad0[4];
    struct EditLatt4_1_0 *editlatt;
    void *batch_cache;
};

struct Material4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    short flag;
    char surface_render_method;
    char _pad1[1];
    float r;
    float g;
    float b;
    float a;
    float specr;
    float specg;
    float specb;
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
    void *ipo;
    struct PreviewImage3_6_0 *preview;
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
    struct TexPaintSlot3_6_0 *texpaintslot;
    struct ListBase3_6_0 gpumaterial;
    struct MaterialGPencilStyle3_6_0 *gp_style;
    struct MaterialLineArt3_6_0 lineart;
};

struct Mesh4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    void *ipo;
    void *key;
    void *mat;
    int verts_num;
    int edges_num;
    int faces_num;
    int corners_num;
    int *face_offset_indices;
    struct CustomData4_0_0 vert_data;
    struct CustomData4_0_0 edge_data;
    struct CustomData4_0_0 face_data;
    struct CustomData4_0_0 corner_data;
    struct ListBase3_6_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    void *edit_mesh;
    void *mselect;
    int totselect;
    int act_face;
    struct Mesh4_1_0 *texcomesh;
    float texspace_location[3];
    float texspace_size[3];
    char texspace_flag;
    char editflag;
    unsigned short flag;
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
    void *mpoly;
    void *mloop;
    void *mvert;
    void *medge;
    void *dvert;
    void *mtface;
    void *tface;
    void *mcol;
    void *mface;
    struct CustomData4_0_0 fdata_legacy;
    int totface_legacy;
    char _pad1[4];
    void *runtime;
};

struct MetaBall4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct ListBase3_6_0 elems;
    struct ListBase3_6_0 *editelems;
    void *ipo;
    void *mat;
    char flag;
    char flag2;
    short totcol;
    char texspace_flag;
    char _pad[2];
    char needs_flush_to_id;
    float texspace_location[3];
    float texspace_size[3];
    float wiresize;
    float rendersize;
    float thresh;
    char _pad0[4];
    struct MetaElem3_6_0 *lastelem;
};

struct bScreen4_1_0 {
    struct ID4_1_0 id;
    struct ListBase3_6_0 vertbase;
    struct ListBase3_6_0 edgebase;
    struct ListBase3_6_0 areabase;
    struct ListBase3_6_0 regionbase;
    void *scene;
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
    struct ARegion4_0_0 *active_region;
    void *animtimer;
    void *context;
    void *tool_tip;
    struct PreviewImage3_6_0 *preview;
};

struct bSound4_1_0 {
    struct ID4_1_0 id;
    char filepath[1024];
    void *packedfile;
    void *handle;
    void *newpackedfile;
    void *ipo;
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

struct Tex4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    float noisesize;
    float turbul;
    float bright;
    float contrast;
    float saturation;
    float rfac;
    float gfac;
    float bfac;
    float filtersize;
    char _pad2[4];
    float mg_H;
    float mg_lacunarity;
    float mg_octaves;
    float mg_offset;
    float mg_gain;
    float dist_amount;
    float ns_outscale;
    float vn_w1;
    float vn_w2;
    float vn_w3;
    float vn_w4;
    float vn_mexp;
    short vn_distm;
    short vn_coltype;
    short noisedepth;
    short noisetype;
    short noisebasis;
    short noisebasis2;
    short imaflag;
    short flag;
    short type;
    short stype;
    float cropxmin;
    float cropymin;
    float cropxmax;
    float cropymax;
    int texfilter;
    int afmax;
    short xrepeat;
    short yrepeat;
    short extend;
    short _pad0;
    int len;
    int frames;
    int offset;
    int sfra;
    float checkerdist;
    float nabla;
    char _pad1[4];
    struct ImageUser3_6_0 iuser;
    void *nodetree;
    void *ipo;
    struct Image4_1_0 *ima;
    void *coba;
    struct PreviewImage3_6_0 *preview;
    char use_nodes;
    char _pad[7];
};

struct Text4_1_0 {
    struct ID4_1_0 id;
    char *filepath;
    void *compiled;
    int flags;
    char _pad0[4];
    struct ListBase3_6_0 lines;
    struct TextLine3_6_0 *curl;
    struct TextLine3_6_0 *sell;
    int curc;
    int selc;
    double mtime;
};

struct VFont4_1_0 {
    struct ID4_1_0 id;
    char filepath[1024];
    void *data;
    void *packedfile;
    void *temp_pf;
};

struct World4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    char _pad0[4];
    short texact;
    short mistype;
    float horr;
    float horg;
    float horb;
    float exposure;
    float exp;
    float range;
    short mode;
    char _pad2[6];
    float misi;
    float miststa;
    float mistdist;
    float misthi;
    float aodist;
    float aoenergy;
    short flag;
    char _pad3[2];
    int probe_resolution;
    void *ipo;
    short pr_texture;
    short use_nodes;
    char _pad[4];
    struct PreviewImage3_6_0 *preview;
    void *nodetree;
    void *lightgroup;
    struct ListBase3_6_0 gpumaterial;
};

struct Brush4_1_0 {
    struct ID4_1_0 id;
    struct BrushClone4_1_0 clone;
    void *curve;
    struct MTex4_1_0 mtex;
    struct MTex4_1_0 mask_mtex;
    struct Brush4_1_0 *toggle_brush;
    void *icon_imbuf;
    struct PreviewImage3_6_0 *preview;
    void *gradient;
    struct PaintCurve4_1_0 *paint_curve;
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
    struct BrushGpencilSettings4_0_0 *gpencil_settings;
    struct BrushCurvesSculptSettings3_6_0 *curves_sculpt_settings;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    void *automasking_cavity_curve;
};

struct ParticleSettings4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct BoidSettings3_6_0 *boids;
    struct SPHFluidSettings3_6_0 *fluid;
    void *effector_weights;
    void *collision_group;
    int flag;
    char _pad1[4];
    short type;
    short from;
    short distr;
    short texact;
    short phystype;
    short rotmode;
    short avemode;
    short reactevent;
    int draw;
    float draw_size;
    short draw_as;
    short childtype;
    char _pad2[4];
    short ren_as;
    short subframes;
    short draw_col;
    short draw_step;
    short ren_step;
    short hair_step;
    short keys_step;
    short adapt_angle;
    short adapt_pix;
    short disp;
    short omat;
    short interpolation;
    short integrator;
    short rotfrom;
    short kink;
    short kink_axis;
    short bb_align;
    short bb_uv_split;
    short bb_anim;
    short bb_split_offset;
    float bb_tilt;
    float bb_rand_tilt;
    float bb_offset[2];
    float bb_size[2];
    float bb_vel_head;
    float bb_vel_tail;
    float color_vec_max;
    float sta;
    float end;
    float lifetime;
    float randlife;
    float timetweak;
    float courant_target;
    float jitfac;
    float eff_hair;
    float grid_rand;
    float ps_offset[1];
    int totpart;
    int userjit;
    int grid_res;
    int effector_amount;
    short time_flag;
    char _pad0[6];
    float normfac;
    float obfac;
    float randfac;
    float partfac;
    float tanfac;
    float tanphase;
    float reactfac;
    float ob_vel[3];
    float avefac;
    float phasefac;
    float randrotfac;
    float randphasefac;
    float mass;
    float size;
    float randsize;
    float acc[3];
    float dragfac;
    float brownfac;
    float dampfac;
    float randlength;
    int child_flag;
    char _pad3[4];
    int child_percent;
    int child_render_percent;
    float parents;
    float childsize;
    float childrandsize;
    float childrad;
    float childflat;
    float clumpfac;
    float clumppow;
    float kink_amp;
    float kink_freq;
    float kink_shape;
    float kink_flat;
    float kink_amp_clump;
    int kink_extra_steps;
    char _pad4[4];
    float kink_axis_random;
    float kink_amp_random;
    float rough1;
    float rough1_size;
    float rough2;
    float rough2_size;
    float rough2_thres;
    float rough_end;
    float rough_end_shape;
    float clength;
    float clength_thres;
    float parting_fac;
    float parting_min;
    float parting_max;
    float branch_thres;
    float draw_line[2];
    float path_start;
    float path_end;
    int trail_count;
    int keyed_loops;
    void *clumpcurve;
    void *roughcurve;
    float clump_noise_size;
    float bending_random;
    void *mtex[18];
    void *instance_collection;
    struct ListBase3_6_0 instance_weights;
    void *force_group;
    void *instance_object;
    void *bb_ob;
    void *ipo;
    void *pd;
    void *pd2;
    short use_modifier_stack;
    char _pad5[2];
    short shape_flag;
    char _pad6[2];
    float twist;
    char _pad8[4];
    float shape;
    float rad_root;
    float rad_tip;
    float rad_scale;
    void *twistcurve;
    void *_pad7;
};

struct Script4_1_0 {
    struct ID4_1_0 id;
    void *py_draw;
    void *py_event;
    void *py_button;
    void *py_browsercallback;
    void *py_globaldict;
    int flags;
    int lastspace;
    char scriptname[1024];
    char scriptarg[256];
};

struct bGPdata4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct ListBase3_6_0 layers;
    int flag;
    int curve_edit_resolution;
    float curve_edit_threshold;
    float curve_edit_corner_angle;
    struct ListBase3_6_0 palettes;
    struct ListBase3_6_0 vertex_group_names;
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
    void *mat;
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
    struct bGPgrid3_6_0 grid;
    struct bGPdata_Runtime4_1_0 runtime;
};

struct IdAdtTemplate4_1_0 {
    struct ID4_1_0 id;
    struct AnimData4_1_0 *adt;
};

struct wmWindowManager4_1_0 {
    struct ID4_1_0 id;
    struct wmWindow4_1_0 *windrawable;
    struct wmWindow4_1_0 *winactive;
    struct ListBase3_6_0 windows;
    unsigned char init_flag;
    char _pad0[1];
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    struct ListBase3_6_0 operators;
    struct ListBase3_6_0 notifier_queue;
    void *notifier_queue_set;
    void *_pad1;
    struct ReportList4_1_0 reports;
    struct ListBase3_6_0 jobs;
    struct ListBase3_6_0 paintcursors;
    struct ListBase3_6_0 drags;
    struct ListBase3_6_0 keyconfigs;
    struct wmKeyConfig3_6_0 *defaultconf;
    struct wmKeyConfig3_6_0 *addonconf;
    struct wmKeyConfig3_6_0 *userconf;
    struct ListBase3_6_0 timers;
    void *autosavetimer;
    void *undo_stack;
    char is_interface_locked;
    char _pad[7];
    void *message_bus;
    struct wmXrData4_1_0 xr;
};

struct Speaker4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    void *sound;
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

struct MovieClip4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    char filepath[1024];
    int source;
    int _pad;
    int lastsize[2];
    float aspx;
    float aspy;
    void *anim;
    void *cache;
    void *gpd;
    struct MovieTracking4_1_0 tracking;
    void *tracking_context;
    struct MovieClipProxy3_6_0 proxy;
    int flag;
    int len;
    int start_frame;
    int frame_offset;
    struct ColorManagedColorspaceSettings3_6_0 colorspace_settings;
    struct MovieClip_Runtime3_6_0 runtime;
};

struct Mask4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    struct ListBase3_6_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra;
    int efra;
    int flag;
    char _pad[4];
};

struct FreestyleLineStyle4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    float r;
    float g;
    float b;
    float alpha;
    float thickness;
    int thickness_position;
    float thickness_ratio;
    int flag;
    int caps;
    int chaining;
    unsigned int rounds;
    float split_length;
    float min_angle;
    float max_angle;
    float min_length;
    float max_length;
    unsigned int chain_count;
    unsigned short split_dash1;
    unsigned short split_gap1;
    unsigned short split_dash2;
    unsigned short split_gap2;
    unsigned short split_dash3;
    unsigned short split_gap3;
    int sort_key;
    int integration_type;
    float texstep;
    short texact;
    short pr_texture;
    short use_nodes;
    char _pad[6];
    unsigned short dash1;
    unsigned short gap1;
    unsigned short dash2;
    unsigned short gap2;
    unsigned short dash3;
    unsigned short gap3;
    int panel;
    void *mtex[18];
    void *nodetree;
    struct ListBase3_6_0 color_modifiers;
    struct ListBase3_6_0 alpha_modifiers;
    struct ListBase3_6_0 thickness_modifiers;
    struct ListBase3_6_0 geometry_modifiers;
};

struct Palette4_1_0 {
    struct ID4_1_0 id;
    struct ListBase3_6_0 colors;
    int active_color;
    char _pad[4];
};

struct PaintCurve4_1_0 {
    struct ID4_1_0 id;
    struct PaintCurvePoint3_6_0 *points;
    int tot_points;
    int add_index;
};

struct CacheFile4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct ListBase3_6_0 object_paths;
    struct ListBase3_6_0 layers;
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

struct Collection4_1_0 {
    struct ID4_1_0 id;
    struct ID4_1_0 *owner_id;
    struct ListBase3_6_0 gobject;
    struct ListBase3_6_0 children;
    struct PreviewImage3_6_0 *preview;
    unsigned int layer;
    float instance_offset[3];
    unsigned char flag;
    signed char color_tag;
    char _pad0[2];
    unsigned char lineart_usage;
    unsigned char lineart_flags;
    unsigned char lineart_intersection_mask;
    unsigned char lineart_intersection_priority;
    void *view_layer;
    struct Collection_Runtime4_1_0 runtime;
};

struct LightProbe4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    char type;
    char flag;
    char attenuation_type;
    char parallax_type;
    char grid_flag;
    char _pad0[3];
    float distinf;
    float distpar;
    float falloff;
    float clipsta;
    float clipend;
    float vis_bias;
    float vis_bleedbias;
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
    void *visibility_grp;
    float data_display_size;
    char _pad1[4];
};

struct Light4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    short type;
    short flag;
    int mode;
    float r;
    float g;
    float b;
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
    float shdwr;
    float shdwg;
    float shdwb;
    short pr_texture;
    short use_nodes;
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
    float diff_fac;
    float volume_fac;
    float spec_fac;
    float att_dist;
    float shadow_softness_factor;
    float shadow_trace_distance;
    float _pad3;
    struct PreviewImage3_6_0 *preview;
    void *nodetree;
    void *ipo;
    float energy_deprecated;
    float _pad2;
};

struct WorkSpace4_1_0 {
    struct ID4_1_0 id;
    struct ListBase3_6_0 layouts;
    struct ListBase3_6_0 hook_layout_relations;
    struct ListBase3_6_0 owner_ids;
    struct ListBase3_6_0 tools;
    void *pin_scene;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    char *status_text;
    struct AssetLibraryReference3_6_0 asset_library_ref;
    struct ViewerPath3_6_0 viewer_path;
};

struct PointCloud4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    int flag;
    int totpoint;
    struct CustomData4_0_0 pdata;
    int attributes_active_index;
    int _pad4;
    void *mat;
    short totcol;
    short _pad3[3];
    void *runtime;
    void *batch_cache;
};

struct Volume4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    char filepath[1024];
    void *packedfile;
    char is_sequence;
    char sequence_mode;
    char _pad1[2];
    int frame_start;
    int frame_duration;
    int frame_offset;
    int flag;
    int active_grid;
    void *mat;
    short totcol;
    short _pad2[3];
    struct VolumeRender3_6_0 render;
    struct VolumeDisplay3_6_0 display;
    char velocity_grid[64];
    char _pad3[3];
    char velocity_unit;
    float velocity_scale;
    void *batch_cache;
    void *runtime;
};

struct Curves4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct CurvesGeometry4_0_0 geometry;
    int flag;
    int attributes_active_index;
    void *mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    struct Object4_1_0 *surface;
    char *surface_uv_map;
    void *batch_cache;
};

struct bNodeTree4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct ID4_1_0 *owner_id;
    void *typeinfo;
    char idname[64];
    void *gpd;
    float view_center[2];
    struct ListBase3_6_0 nodes;
    struct ListBase3_6_0 links;
    int type;
    int cur_index;
    int flag;
    short edit_quality;
    short render_quality;
    int chunksize;
    int execution_mode;
    int precision;
    char _pad[4];
    struct rctf3_6_0 viewer_border;
    struct ListBase3_6_0 inputs_legacy;
    struct ListBase3_6_0 outputs_legacy;
    struct bNodeTreeInterface4_0_0 tree_interface;
    void *previews;
    struct bNodeInstanceKey3_6_0 active_viewer_key;
    int nested_node_refs_num;
    struct bNestedNodeRef4_0_0 *nested_node_refs;
    struct GeometryNodeAssetTraits4_0_0 *geometry_node_asset_traits;
    struct PreviewImage3_6_0 *preview;
    void *runtime;
};

struct GreasePencil4_1_0 {
    struct ID4_1_0 id;
    void *adt;
    struct GreasePencilDrawingBase4_0_0 **drawing_array;
    int drawing_array_num;
    char _pad[4];
    struct GreasePencilLayerTreeGroup4_0_0 *root_group_ptr;
    struct CustomData4_0_0 layers_data;
    int attributes_active_index;
    char _pad2[4];
    struct GreasePencilLayer4_0_0 *active_layer;
    void *material_array;
    short material_array_num;
    char _pad3[2];
    unsigned int flag;
    struct ListBase3_6_0 vertex_group_names;
    int vertex_group_active_index;
    char _pad4[4];
    struct GreasePencilOnionSkinningSettings4_0_0 onion_skinning_settings;
    void *runtime;
};

#endif