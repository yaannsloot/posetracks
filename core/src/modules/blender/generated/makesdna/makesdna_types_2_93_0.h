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

Various structs from Blender v2.93 makesdna headers.

This file and others like it (makesdna_types_*.h) were generated using
gen_headers.py, which can be found in the project's root directory.
*/

#ifndef MAKESDNA_TYPES_2_93_0_H
#define MAKESDNA_TYPES_2_93_0_H

#include <stdint.h>

#pragma warning( disable : 4200 )

struct bMotionPathVert2_93_0;
struct bMotionPath2_93_0;
struct bAnimVizSettings2_93_0;
struct bPoseChannelDrawData2_93_0;
struct bPoseChannel_Runtime2_93_0;
struct bPoseChannel2_93_0;
struct bPose2_93_0;
struct bIKParam2_93_0;
struct bItasc2_93_0;
struct bActionGroup2_93_0;
struct bAction2_93_0;
struct bDopeSheet2_93_0;
struct SpaceAction_Runtime2_93_0;
struct SpaceAction2_93_0;
struct bActionChannel2_93_0;
struct FModifier2_93_0;
struct FMod_Generator2_93_0;
struct FMod_FunctionGenerator2_93_0;
struct FCM_EnvelopeData2_93_0;
struct FMod_Envelope2_93_0;
struct FMod_Cycles2_93_0;
struct FMod_Python2_93_0;
struct FMod_Limits2_93_0;
struct FMod_Noise2_93_0;
struct FMod_Stepped2_93_0;
struct DriverTarget2_93_0;
struct DriverVar2_93_0;
struct ChannelDriver2_93_0;
struct FPoint2_93_0;
struct FCurve2_93_0;
struct NlaStrip2_93_0;
struct NlaTrack2_93_0;
struct KS_Path2_93_0;
struct KeyingSet2_93_0;
struct AnimOverride2_93_0;
struct AnimData2_93_0;
struct IdAdtTemplate2_93_0;
struct Bone2_93_0;
struct bArmature2_93_0;
struct AssetTag2_93_0;
struct AssetMetaData2_93_0;
struct BoidRule2_93_0;
struct BoidRuleGoalAvoid2_93_0;
struct BoidRuleAvoidCollision2_93_0;
struct BoidRuleFollowLeader2_93_0;
struct BoidRuleAverageSpeed2_93_0;
struct BoidRuleFight2_93_0;
struct BoidData2_93_0;
struct BoidState2_93_0;
struct BoidSettings2_93_0;
struct BrushClone2_93_0;
struct BrushGpencilSettings2_93_0;
struct Brush2_93_0;
struct tPaletteColorHSV2_93_0;
struct PaletteColor2_93_0;
struct Palette2_93_0;
struct PaintCurvePoint2_93_0;
struct PaintCurve2_93_0;
struct AlembicObjectPath2_93_0;
struct CacheFile2_93_0;
struct CameraStereoSettings2_93_0;
struct CameraBGImage2_93_0;
struct CameraDOFSettings2_93_0;
struct Camera_Runtime2_93_0;
struct Camera2_93_0;
struct ClothSimSettings2_93_0;
struct ClothCollSettings2_93_0;
struct CollectionObject2_93_0;
struct CollectionChild2_93_0;
struct Collection2_93_0;
struct CurveMapPoint2_93_0;
struct CurveMap2_93_0;
struct CurveMapping2_93_0;
struct Histogram2_93_0;
struct Scopes2_93_0;
struct ColorManagedViewSettings2_93_0;
struct ColorManagedDisplaySettings2_93_0;
struct ColorManagedColorspaceSettings2_93_0;
struct bConstraintChannel2_93_0;
struct bConstraint2_93_0;
struct bConstraintTarget2_93_0;
struct bPythonConstraint2_93_0;
struct bKinematicConstraint2_93_0;
struct bSplineIKConstraint2_93_0;
struct bArmatureConstraint2_93_0;
struct bTrackToConstraint2_93_0;
struct bRotateLikeConstraint2_93_0;
struct bLocateLikeConstraint2_93_0;
struct bSizeLikeConstraint2_93_0;
struct bSameVolumeConstraint2_93_0;
struct bTransLikeConstraint2_93_0;
struct bMinMaxConstraint2_93_0;
struct bActionConstraint2_93_0;
struct bLockTrackConstraint2_93_0;
struct bDampTrackConstraint2_93_0;
struct bFollowPathConstraint2_93_0;
struct bStretchToConstraint2_93_0;
struct bRigidBodyJointConstraint2_93_0;
struct bClampToConstraint2_93_0;
struct bChildOfConstraint2_93_0;
struct bTransformConstraint2_93_0;
struct bPivotConstraint2_93_0;
struct bLocLimitConstraint2_93_0;
struct bRotLimitConstraint2_93_0;
struct bSizeLimitConstraint2_93_0;
struct bDistLimitConstraint2_93_0;
struct bShrinkwrapConstraint2_93_0;
struct bFollowTrackConstraint2_93_0;
struct bCameraSolverConstraint2_93_0;
struct bObjectSolverConstraint2_93_0;
struct bTransformCacheConstraint2_93_0;
struct CurveProfilePoint2_93_0;
struct CurveProfile2_93_0;
struct BevPoint2_93_0;
struct BevList2_93_0;
struct BezTriple2_93_0;
struct BPoint2_93_0;
struct Nurb2_93_0;
struct CharInfo2_93_0;
struct TextBox2_93_0;
struct EditNurb2_93_0;
struct Curve2_93_0;
struct CustomDataLayer2_93_0;
struct CustomDataExternal2_93_0;
struct CustomData2_93_0;
struct CustomData_MeshMasks2_93_0;
struct DynamicPaintRuntime2_93_0;
struct DynamicPaintSurface2_93_0;
struct DynamicPaintCanvasSettings2_93_0;
struct DynamicPaintBrushSettings2_93_0;
struct Effect2_93_0;
struct BuildEff2_93_0;
struct Particle2_93_0;
struct PartEff2_93_0;
struct WaveEff2_93_0;
struct FileGlobal2_93_0;
struct FluidDomainVertexVelocity2_93_0;
struct FluidDomainSettings2_93_0;
struct FluidFlowSettings2_93_0;
struct FluidEffectorSettings2_93_0;
struct FreestyleLineSet2_93_0;
struct FreestyleModuleConfig2_93_0;
struct FreestyleConfig2_93_0;
struct GpencilModifierData2_93_0;
struct NoiseGpencilModifierData2_93_0;
struct SubdivGpencilModifierData2_93_0;
struct ThickGpencilModifierData2_93_0;
struct TimeGpencilModifierData2_93_0;
struct ColorGpencilModifierData2_93_0;
struct OpacityGpencilModifierData2_93_0;
struct ArrayGpencilModifierData2_93_0;
struct BuildGpencilModifierData2_93_0;
struct LatticeGpencilModifierData2_93_0;
struct MirrorGpencilModifierData2_93_0;
struct HookGpencilModifierData2_93_0;
struct SimplifyGpencilModifierData2_93_0;
struct OffsetGpencilModifierData2_93_0;
struct SmoothGpencilModifierData2_93_0;
struct MultiplyGpencilModifierData2_93_0;
struct TintGpencilModifierData2_93_0;
struct TextureGpencilModifierData2_93_0;
struct LineartGpencilModifierData2_93_0;
struct bGPDcontrolpoint2_93_0;
struct bGPDspoint_Runtime2_93_0;
struct bGPDspoint2_93_0;
struct bGPDtriangle2_93_0;
struct bGPDpalettecolor2_93_0;
struct bGPDpalette2_93_0;
struct bGPDcurve_point2_93_0;
struct bGPDcurve2_93_0;
struct bGPDstroke_Runtime2_93_0;
struct bGPDstroke2_93_0;
struct bGPDframe_Runtime2_93_0;
struct bGPDframe2_93_0;
struct bGPDlayer_Mask2_93_0;
struct bGPDlayer_Runtime2_93_0;
struct bGPDlayer2_93_0;
struct bGPdata_Runtime2_93_0;
struct bGPgrid2_93_0;
struct bGPdata2_93_0;
struct GPUDOFSettings2_93_0;
struct HairCurve2_93_0;
struct HairMapping2_93_0;
struct DrawDataList2_93_0;
struct IDPropertyData2_93_0;
struct IDProperty2_93_0;
struct IDOverrideLibraryPropertyOperation2_93_0;
struct IDOverrideLibraryProperty2_93_0;
struct IDOverrideLibrary2_93_0;
struct ID2_93_0;
struct Library2_93_0;
struct PreviewImage2_93_0;
struct ImageUser2_93_0;
struct ImageAnim2_93_0;
struct ImageView2_93_0;
struct ImagePackedFile2_93_0;
struct RenderSlot2_93_0;
struct ImageTile_Runtime2_93_0;
struct ImageTile2_93_0;
struct Image2_93_0;
struct IpoDriver2_93_0;
struct IpoCurve2_93_0;
struct Ipo2_93_0;
struct KeyBlock2_93_0;
struct Key2_93_0;
struct EditLatt2_93_0;
struct Lattice2_93_0;
struct Base2_93_0;
struct LayerCollection2_93_0;
struct ViewLayerEEVEE2_93_0;
struct ViewLayerAOV2_93_0;
struct ViewLayer2_93_0;
struct SceneCollection2_93_0;
struct LightProbe2_93_0;
struct LightProbeCache2_93_0;
struct LightGridCache2_93_0;
struct LightCacheTexture2_93_0;
struct LightCache2_93_0;
struct Light2_93_0;
struct LineStyleModifier2_93_0;
struct LineStyleColorModifier_AlongStroke2_93_0;
struct LineStyleAlphaModifier_AlongStroke2_93_0;
struct LineStyleThicknessModifier_AlongStroke2_93_0;
struct LineStyleColorModifier_DistanceFromCamera2_93_0;
struct LineStyleAlphaModifier_DistanceFromCamera2_93_0;
struct LineStyleThicknessModifier_DistanceFromCamera2_93_0;
struct LineStyleColorModifier_DistanceFromObject2_93_0;
struct LineStyleAlphaModifier_DistanceFromObject2_93_0;
struct LineStyleThicknessModifier_DistanceFromObject2_93_0;
struct LineStyleColorModifier_Curvature_3D2_93_0;
struct LineStyleAlphaModifier_Curvature_3D2_93_0;
struct LineStyleThicknessModifier_Curvature_3D2_93_0;
struct LineStyleColorModifier_Noise2_93_0;
struct LineStyleAlphaModifier_Noise2_93_0;
struct LineStyleThicknessModifier_Noise2_93_0;
struct LineStyleColorModifier_CreaseAngle2_93_0;
struct LineStyleAlphaModifier_CreaseAngle2_93_0;
struct LineStyleThicknessModifier_CreaseAngle2_93_0;
struct LineStyleColorModifier_Tangent2_93_0;
struct LineStyleAlphaModifier_Tangent2_93_0;
struct LineStyleThicknessModifier_Tangent2_93_0;
struct LineStyleColorModifier_Material2_93_0;
struct LineStyleAlphaModifier_Material2_93_0;
struct LineStyleThicknessModifier_Material2_93_0;
struct LineStyleGeometryModifier_Sampling2_93_0;
struct LineStyleGeometryModifier_BezierCurve2_93_0;
struct LineStyleGeometryModifier_SinusDisplacement2_93_0;
struct LineStyleGeometryModifier_SpatialNoise2_93_0;
struct LineStyleGeometryModifier_PerlinNoise1D2_93_0;
struct LineStyleGeometryModifier_PerlinNoise2D2_93_0;
struct LineStyleGeometryModifier_BackboneStretcher2_93_0;
struct LineStyleGeometryModifier_TipRemover2_93_0;
struct LineStyleGeometryModifier_Polygonalization2_93_0;
struct LineStyleGeometryModifier_GuidingLines2_93_0;
struct LineStyleGeometryModifier_Blueprint2_93_0;
struct LineStyleGeometryModifier_2DOffset2_93_0;
struct LineStyleGeometryModifier_2DTransform2_93_0;
struct LineStyleGeometryModifier_Simplification2_93_0;
struct LineStyleThicknessModifier_Calligraphy2_93_0;
struct FreestyleLineStyle2_93_0;
struct Link2_93_0;
struct LinkData2_93_0;
struct ListBase2_93_0;
struct Mask2_93_0;
struct MaskParent2_93_0;
struct MaskSplinePointUW2_93_0;
struct MaskSplinePoint2_93_0;
struct MaskSpline2_93_0;
struct MaskLayerShape2_93_0;
struct MaskLayerShapeElem2_93_0;
struct MaskLayer2_93_0;
struct TexPaintSlot2_93_0;
struct MaterialGPencilStyle2_93_0;
struct MaterialLineArt2_93_0;
struct Material2_93_0;
struct MVert2_93_0;
struct MEdge2_93_0;
struct MPoly2_93_0;
struct MLoop2_93_0;
struct MSelect2_93_0;
struct MLoopTri2_93_0;
struct MVertTri2_93_0;
struct MFloatProperty2_93_0;
struct MIntProperty2_93_0;
struct MStringProperty2_93_0;
struct MBoolProperty2_93_0;
struct MDeformWeight2_93_0;
struct MDeformVert2_93_0;
struct MVertSkin2_93_0;
struct MLoopUV2_93_0;
struct MLoopCol2_93_0;
struct MPropCol2_93_0;
struct GridPaintMask2_93_0;
struct OrigSpaceFace2_93_0;
struct OrigSpaceLoop2_93_0;
struct FreestyleEdge2_93_0;
struct FreestyleFace2_93_0;
struct MFace2_93_0;
struct MTFace2_93_0;
struct MCol2_93_0;
struct MRecast2_93_0;
struct MLoopTri_Store2_93_0;
struct Mesh_Runtime2_93_0;
struct Mesh2_93_0;
struct TFace2_93_0;
struct MetaElem2_93_0;
struct MetaBall2_93_0;
struct ModifierData2_93_0;
struct MappingInfoModifierData2_93_0;
struct SubsurfModifierData2_93_0;
struct LatticeModifierData2_93_0;
struct CurveModifierData2_93_0;
struct BuildModifierData2_93_0;
struct MaskModifierData2_93_0;
struct ArrayModifierData2_93_0;
struct MirrorModifierData2_93_0;
struct EdgeSplitModifierData2_93_0;
struct BevelModifierData2_93_0;
struct FluidModifierData2_93_0;
struct DisplaceModifierData2_93_0;
struct UVProjectModifierData2_93_0;
struct DecimateModifierData2_93_0;
struct SmoothModifierData2_93_0;
struct CastModifierData2_93_0;
struct WaveModifierData2_93_0;
struct HookModifierData2_93_0;
struct SoftbodyModifierData2_93_0;
struct ClothModifierData2_93_0;
struct CollisionModifierData2_93_0;
struct SurfaceModifierData2_93_0;
struct BooleanModifierData2_93_0;
struct MDefInfluence2_93_0;
struct MDefCell2_93_0;
struct ParticleSystemModifierData2_93_0;
struct ParticleInstanceModifierData2_93_0;
struct ExplodeModifierData2_93_0;
struct MultiresModifierData2_93_0;
struct FluidsimModifierData2_93_0;
struct SmokeModifierData2_93_0;
struct ShrinkwrapModifierData2_93_0;
struct SimpleDeformModifierData2_93_0;
struct ShapeKeyModifierData2_93_0;
struct SolidifyModifierData2_93_0;
struct ScrewModifierData2_93_0;
struct OceanModifierData2_93_0;
struct WarpModifierData2_93_0;
struct WeightVGEditModifierData2_93_0;
struct WeightVGMixModifierData2_93_0;
struct WeightVGProximityModifierData2_93_0;
struct DynamicPaintModifierData2_93_0;
struct RemeshModifierData2_93_0;
struct SkinModifierData2_93_0;
struct TriangulateModifierData2_93_0;
struct LaplacianSmoothModifierData2_93_0;
struct UVWarpModifierData2_93_0;
struct MeshCacheModifierData2_93_0;
struct LaplacianDeformModifierData2_93_0;
struct WireframeModifierData2_93_0;
struct WeldModifierData2_93_0;
struct DataTransferModifierData2_93_0;
struct NormalEditModifierData2_93_0;
struct MeshCacheVertexVelocity2_93_0;
struct MeshSeqCacheModifierData2_93_0;
struct SDefBind2_93_0;
struct SDefVert2_93_0;
struct SurfaceDeformModifierData2_93_0;
struct WeightedNormalModifierData2_93_0;
struct NodesModifierSettings2_93_0;
struct NodesModifierData2_93_0;
struct MeshToVolumeModifierData2_93_0;
struct VolumeDisplaceModifierData2_93_0;
struct VolumeToMeshModifierData2_93_0;
struct MovieClipUser2_93_0;
struct MovieClipProxy2_93_0;
struct MovieClip_RuntimeGPUTexture2_93_0;
struct MovieClip_Runtime2_93_0;
struct MovieClip2_93_0;
struct MovieClipScopes2_93_0;
struct bActionModifier2_93_0;
struct bActionStrip2_93_0;
struct bNodeStack2_93_0;
struct bNodeSocket2_93_0;
struct bNode2_93_0;
struct bNodeInstanceKey2_93_0;
struct bNodeInstanceHashEntry2_93_0;
struct bNodePreview2_93_0;
struct bNodeLink2_93_0;
struct bNodeSocketValueInt2_93_0;
struct bNodeSocketValueFloat2_93_0;
struct bNodeSocketValueBoolean2_93_0;
struct bNodeSocketValueVector2_93_0;
struct bNodeSocketValueRGBA2_93_0;
struct bNodeSocketValueString2_93_0;
struct bNodeSocketValueObject2_93_0;
struct bNodeSocketValueImage2_93_0;
struct bNodeSocketValueCollection2_93_0;
struct NodeFrame2_93_0;
struct NodeImageAnim2_93_0;
struct ColorCorrectionData2_93_0;
struct NodeColorCorrection2_93_0;
struct NodeBokehImage2_93_0;
struct NodeBoxMask2_93_0;
struct NodeEllipseMask2_93_0;
struct NodeImageLayer2_93_0;
struct NodeBlurData2_93_0;
struct NodeDBlurData2_93_0;
struct NodeBilateralBlurData2_93_0;
struct NodeAntiAliasingData2_93_0;
struct NodeHueSat2_93_0;
struct NodeImageFile2_93_0;
struct NodeImageMultiFile2_93_0;
struct NodeImageMultiFileSocket2_93_0;
struct NodeChroma2_93_0;
struct NodeTwoXYs2_93_0;
struct NodeTwoFloats2_93_0;
struct NodeVertexCol2_93_0;
struct NodeDefocus2_93_0;
struct NodeScriptDict2_93_0;
struct NodeGlare2_93_0;
struct NodeTonemap2_93_0;
struct NodeLensDist2_93_0;
struct NodeColorBalance2_93_0;
struct NodeColorspill2_93_0;
struct NodeDilateErode2_93_0;
struct NodeMask2_93_0;
struct NodeSetAlpha2_93_0;
struct NodeTexBase2_93_0;
struct NodeTexSky2_93_0;
struct NodeTexImage2_93_0;
struct NodeTexChecker2_93_0;
struct NodeTexBrick2_93_0;
struct NodeTexEnvironment2_93_0;
struct NodeTexGradient2_93_0;
struct NodeTexNoise2_93_0;
struct NodeTexVoronoi2_93_0;
struct NodeTexMusgrave2_93_0;
struct NodeTexWave2_93_0;
struct NodeTexMagic2_93_0;
struct NodeShaderAttribute2_93_0;
struct NodeShaderVectTransform2_93_0;
struct NodeShaderTexPointDensity2_93_0;
struct TexNodeOutput2_93_0;
struct NodeKeyingScreenData2_93_0;
struct NodeKeyingData2_93_0;
struct NodeTrackPosData2_93_0;
struct NodeTranslateData2_93_0;
struct NodePlaneTrackDeformData2_93_0;
struct NodeShaderScript2_93_0;
struct NodeShaderTangent2_93_0;
struct NodeShaderNormalMap2_93_0;
struct NodeShaderUVMap2_93_0;
struct NodeShaderVertexColor2_93_0;
struct NodeShaderTexIES2_93_0;
struct NodeShaderOutputAOV2_93_0;
struct NodeSunBeams2_93_0;
struct CryptomatteEntry2_93_0;
struct CryptomatteLayer2_93_0;
struct NodeCryptomatte_Runtime2_93_0;
struct NodeCryptomatte2_93_0;
struct NodeDenoise2_93_0;
struct NodeAttributeClamp2_93_0;
struct NodeAttributeCompare2_93_0;
struct NodeAttributeMapRange2_93_0;
struct NodeAttributeMath2_93_0;
struct NodeAttributeMix2_93_0;
struct NodeAttributeRandomize2_93_0;
struct NodeAttributeVectorMath2_93_0;
struct NodeAttributeColorRamp2_93_0;
struct NodeInputVector2_93_0;
struct NodeInputString2_93_0;
struct NodeGeometryRotatePoints2_93_0;
struct NodeGeometryAlignRotationToVector2_93_0;
struct NodeGeometryPointScale2_93_0;
struct NodeGeometryPointTranslate2_93_0;
struct NodeGeometryObjectInfo2_93_0;
struct NodeGeometryPointInstance2_93_0;
struct NodeGeometryPointsToVolume2_93_0;
struct NodeGeometryCollectionInfo2_93_0;
struct NodeGeometryAttributeProximity2_93_0;
struct NodeGeometryVolumeToMesh2_93_0;
struct NodeAttributeCombineXYZ2_93_0;
struct NodeAttributeSeparateXYZ2_93_0;
struct NodeAttributeConvert2_93_0;
struct NodeGeometryMeshCircle2_93_0;
struct NodeGeometryMeshCylinder2_93_0;
struct NodeGeometryMeshCone2_93_0;
struct NodeGeometryMeshLine2_93_0;
struct FluidVertexVelocity2_93_0;
struct FluidsimSettings2_93_0;
struct PartDeflect2_93_0;
struct EffectorWeights2_93_0;
struct SBVertex2_93_0;
struct SoftBody_Shared2_93_0;
struct SoftBody2_93_0;
struct bDeformGroup2_93_0;
struct bFaceMap2_93_0;
struct BoundBox2_93_0;
struct Object_Runtime2_93_0;
struct ObjectLineArt2_93_0;
struct Object2_93_0;
struct ObHook2_93_0;
struct TreeStoreElem2_93_0;
struct TreeStore2_93_0;
struct PackedFile2_93_0;
struct HairKey2_93_0;
struct ParticleKey2_93_0;
struct BoidParticle2_93_0;
struct ParticleSpring2_93_0;
struct ChildParticle2_93_0;
struct ParticleTarget2_93_0;
struct ParticleDupliWeight2_93_0;
struct ParticleData2_93_0;
struct SPHFluidSettings2_93_0;
struct ParticleSettings2_93_0;
struct PTCacheExtra2_93_0;
struct PTCacheMem2_93_0;
struct RigidBodyWorld_Shared2_93_0;
struct RigidBodyWorld2_93_0;
struct RigidBodyOb_Shared2_93_0;
struct RigidBodyOb2_93_0;
struct RigidBodyCon2_93_0;
struct AviCodecData2_93_0;
struct FFMpegCodecData2_93_0;
struct AudioData2_93_0;
struct SceneRenderLayer2_93_0;
struct SceneRenderView2_93_0;
struct Stereo3dFormat2_93_0;
struct ImageFormatData2_93_0;
struct BakeData2_93_0;
struct RenderData2_93_0;
struct RenderProfile2_93_0;
struct TimeMarker2_93_0;
struct Paint_Runtime2_93_0;
struct PaintToolSlot2_93_0;
struct Paint2_93_0;
struct ImagePaintSettings2_93_0;
struct ParticleBrushData2_93_0;
struct ParticleEditSettings2_93_0;
struct Sculpt2_93_0;
struct UvSculpt2_93_0;
struct GpPaint2_93_0;
struct GpVertexPaint2_93_0;
struct GpSculptPaint2_93_0;
struct GpWeightPaint2_93_0;
struct VPaint2_93_0;
struct GP_Sculpt_Guide2_93_0;
struct GP_Sculpt_Settings2_93_0;
struct GP_Interpolate_Settings2_93_0;
struct UnifiedPaintSettings2_93_0;
struct CurvePaintSettings2_93_0;
struct MeshStatVis2_93_0;
struct SequencerToolSettings2_93_0;
struct ToolSettings2_93_0;
struct UnitSettings2_93_0;
struct PhysicsSettings2_93_0;
struct DisplaySafeAreas2_93_0;
struct SceneDisplay2_93_0;
struct SceneEEVEE2_93_0;
struct SceneGpencil2_93_0;
struct TransformOrientationSlot2_93_0;
struct Scene2_93_0;
struct bScreen2_93_0;
struct ScrVert2_93_0;
struct ScrEdge2_93_0;
struct ScrAreaMap2_93_0;
struct Panel_Runtime2_93_0;
struct Panel2_93_0;
struct PanelCategoryDyn2_93_0;
struct PanelCategoryStack2_93_0;
struct uiListDyn2_93_0;
struct uiList2_93_0;
struct TransformOrientation2_93_0;
struct uiPreview2_93_0;
struct ScrGlobalAreaData2_93_0;
struct ScrArea_Runtime2_93_0;
struct ScrArea2_93_0;
struct ARegion_Runtime2_93_0;
struct ARegion2_93_0;
struct SDNA_StructMember2_93_0;
struct SDNA_Struct2_93_0;
struct BHead2_93_0;
struct BHead82_93_0;
struct StripAnim2_93_0;
struct StripElem2_93_0;
struct StripCrop2_93_0;
struct StripTransform2_93_0;
struct StripColorBalance2_93_0;
struct StripProxy2_93_0;
struct Strip2_93_0;
struct SequenceRuntime2_93_0;
struct Sequence2_93_0;
struct MetaStack2_93_0;
struct Editing2_93_0;
struct WipeVars2_93_0;
struct GlowVars2_93_0;
struct TransformVars2_93_0;
struct SolidColorVars2_93_0;
struct SpeedControlVars2_93_0;
struct GaussianBlurVars2_93_0;
struct TextVars2_93_0;
struct ColorMixVars2_93_0;
struct SequenceModifierData2_93_0;
struct ColorBalanceModifierData2_93_0;
struct CurvesModifierData2_93_0;
struct HueCorrectModifierData2_93_0;
struct BrightContrastModifierData2_93_0;
struct SequencerMaskModifierData2_93_0;
struct WhiteBalanceModifierData2_93_0;
struct SequencerTonemapModifierData2_93_0;
struct SequencerScopes2_93_0;
struct SessionUUID2_93_0;
struct ShaderFxData2_93_0;
struct ShaderFxData_Runtime2_93_0;
struct BlurShaderFxData2_93_0;
struct ColorizeShaderFxData2_93_0;
struct FlipShaderFxData2_93_0;
struct GlowShaderFxData2_93_0;
struct PixelShaderFxData2_93_0;
struct RimShaderFxData2_93_0;
struct ShadowShaderFxData2_93_0;
struct SwirlShaderFxData2_93_0;
struct WaveShaderFxData2_93_0;
struct Simulation2_93_0;
struct bSound2_93_0;
struct SpaceLink2_93_0;
struct SpaceInfo2_93_0;
struct SpaceProperties2_93_0;
struct SpaceOutliner2_93_0;
struct SpaceGraph_Runtime2_93_0;
struct SpaceGraph2_93_0;
struct SpaceNla2_93_0;
struct SpaceSeq2_93_0;
struct MaskSpaceInfo2_93_0;
struct FileSelectAssetLibraryUID2_93_0;
struct FileSelectParams2_93_0;
struct FileAssetSelectParams2_93_0;
struct FileFolderHistory2_93_0;
struct SpaceFile2_93_0;
struct AssetUUID2_93_0;
struct AssetUUIDList2_93_0;
struct FileDirEntryRevision2_93_0;
struct FileDirEntryVariant2_93_0;
struct FileDirEntry2_93_0;
struct FileDirEntryArr2_93_0;
struct SpaceImageOverlay2_93_0;
struct SpaceImage2_93_0;
struct SpaceText_Runtime2_93_0;
struct SpaceText2_93_0;
struct Script2_93_0;
struct SpaceScript2_93_0;
struct bNodeTreePath2_93_0;
struct SpaceNode2_93_0;
struct ConsoleLine2_93_0;
struct SpaceConsole2_93_0;
struct SpaceUserPref2_93_0;
struct SpaceClip2_93_0;
struct SpaceTopBar2_93_0;
struct SpaceStatusBar2_93_0;
struct SpreadsheetColumnID2_93_0;
struct SpreadsheetColumn2_93_0;
struct SpreadsheetContext2_93_0;
struct SpreadsheetContextObject2_93_0;
struct SpreadsheetContextModifier2_93_0;
struct SpreadsheetContextNode2_93_0;
struct SpaceSpreadsheet2_93_0;
struct Speaker2_93_0;
struct MTex2_93_0;
struct CBData2_93_0;
struct ColorBand2_93_0;
struct PointDensity2_93_0;
struct Tex2_93_0;
struct TexMapping2_93_0;
struct ColorMapping2_93_0;
struct TextLine2_93_0;
struct Text2_93_0;
struct MovieReconstructedCamera2_93_0;
struct MovieTrackingCamera2_93_0;
struct MovieTrackingMarker2_93_0;
struct MovieTrackingTrack2_93_0;
struct MovieTrackingPlaneMarker2_93_0;
struct MovieTrackingPlaneTrack2_93_0;
struct MovieTrackingSettings2_93_0;
struct MovieTrackingStabilization2_93_0;
struct MovieTrackingReconstruction2_93_0;
struct MovieTrackingObject2_93_0;
struct MovieTrackingStats2_93_0;
struct MovieTrackingDopesheetChannel2_93_0;
struct MovieTrackingDopesheetCoverageSegment2_93_0;
struct MovieTrackingDopesheet2_93_0;
struct MovieTracking2_93_0;
struct uiFont2_93_0;
struct uiFontStyle2_93_0;
struct uiStyle2_93_0;
struct uiWidgetColors2_93_0;
struct uiWidgetStateColors2_93_0;
struct uiPanelColors2_93_0;
struct ThemeUI2_93_0;
struct ThemeSpace2_93_0;
struct ThemeWireColor2_93_0;
struct ThemeCollectionColor2_93_0;
struct bTheme2_93_0;
struct bAddon2_93_0;
struct bPathCompare2_93_0;
struct bUserMenu2_93_0;
struct bUserMenuItem2_93_0;
struct bUserMenuItem_Op2_93_0;
struct bUserMenuItem_Menu2_93_0;
struct bUserMenuItem_Prop2_93_0;
struct bUserAssetLibrary2_93_0;
struct SolidLight2_93_0;
struct WalkNavigation2_93_0;
struct UserDef_Runtime2_93_0;
struct UserDef_SpaceData2_93_0;
struct UserDef_FileSpaceData2_93_0;
struct UserDef_Experimental2_93_0;
struct UserDef2_93_0;
struct vec2s2_93_0;
struct vec2f2_93_0;
struct vec3f2_93_0;
struct rcti2_93_0;
struct rctf2_93_0;
struct DualQuat2_93_0;
struct VFont2_93_0;
struct View2D2_93_0;
struct RegionView3D2_93_0;
struct View3DCursor2_93_0;
struct View3DShading2_93_0;
struct View3DOverlay2_93_0;
struct View3D_Runtime2_93_0;
struct View3D2_93_0;
struct Volume_Runtime2_93_0;
struct VolumeDisplay2_93_0;
struct VolumeRender2_93_0;
struct Volume2_93_0;
struct Report2_93_0;
struct ReportList2_93_0;
struct ReportTimerInfo2_93_0;
struct wmXrData2_93_0;
struct wmWindowManager2_93_0;
struct wmWindow2_93_0;
struct wmOperatorTypeMacro2_93_0;
struct wmKeyMapItem2_93_0;
struct wmKeyMapDiffItem2_93_0;
struct wmKeyConfigPref2_93_0;
struct wmKeyConfig2_93_0;
struct wmOperator2_93_0;
struct bToolRef_Runtime2_93_0;
struct bToolRef2_93_0;
struct WorkSpaceLayout2_93_0;
struct wmOwnerID2_93_0;
struct WorkSpace2_93_0;
struct WorkSpaceDataRelation2_93_0;
struct WorkSpaceInstanceHook2_93_0;
struct World2_93_0;
struct XrSessionSettings2_93_0;

struct bMotionPathVert2_93_0 {
    float co[3];
    int flag;
};

struct bMotionPath2_93_0 {
    bMotionPathVert2_93_0 *points;
    int length;
    int start_frame;
    int end_frame;
    float color[3];
    int line_thickness;
    int flag;
    void *points_vbo;
    void *batch_line;
    void *batch_points;
    void *_pad;
};

struct bAnimVizSettings2_93_0 {
    short recalc;
    short path_type;
    short path_step;
    short path_viewflag;
    short path_bakeflag;
    char _pad[6];
    int path_sf, path_ef;
    int path_bc, path_ac;
};

struct bPoseChannelDrawData2_93_0 {
    float solid_color[4];
    float wire_color[4];
    int bbone_matrix_len;
    float bbone_matrix[0][4][4];
};

struct bIKParam2_93_0 {
    int iksolver;
};

struct bItasc2_93_0 {
    int iksolver;
    float precision;
    short numiter;
    short numstep;
    float minstep;
    float maxstep;
    short solver;
    short flag;
    float feedback;
    float maxvel;
    float dampmax;
    float dampeps;
};

struct SpaceAction_Runtime2_93_0 {
    char flag;
    char _pad0[7];
};

struct FModifier2_93_0 {
    FModifier2_93_0 *next, *prev;
    FCurve2_93_0 *curve;
    void *data;
    char name[64];
    short type;
    short flag;
    short ui_expand_flag;
    char _pad[6];
    float influence;
    float sfra;
    float efra;
    float blendin;
    float blendout;
};

struct FMod_Generator2_93_0 {
    float *coefficients;
    unsigned int arraysize;
    int poly_order;
    int mode;
    int flag;
};

struct FMod_FunctionGenerator2_93_0 {
    float amplitude;
    float phase_multiplier;
    float phase_offset;
    float value_offset;
    int type;
    int flag;
};

struct FCM_EnvelopeData2_93_0 {
    float min, max;
    float time;
    short f1;
    short f2;
};

struct FMod_Envelope2_93_0 {
    FCM_EnvelopeData2_93_0 *data;
    int totvert;
    float midval;
    float min, max;
};

struct FMod_Cycles2_93_0 {
    short before_mode;
    short after_mode;
    short before_cycles;
    short after_cycles;
};

struct FMod_Python2_93_0 {
    Text2_93_0 *script;
    IDProperty2_93_0 *prop;
};

struct FMod_Noise2_93_0 {
    float size;
    float strength;
    float phase;
    float offset;
    short depth;
    short modification;
};

struct FMod_Stepped2_93_0 {
    float step_size;
    float offset;
    float start_frame;
    float end_frame;
    int flag;
};

struct DriverTarget2_93_0 {
    ID2_93_0 *id;
    char *rna_path;
    char pchan_name[64];
    short transChan;
    char rotation_mode;
    char _pad[7];
    short flag;
    int idtype;
};

struct FPoint2_93_0 {
    float vec[2];
    int flag;
    char _pad[4];
};

struct KS_Path2_93_0 {
    KS_Path2_93_0 *next, *prev;
    ID2_93_0 *id;
    char group[64];
    int idtype;
    short groupmode;
    short flag;
    char *rna_path;
    int array_index;
    short keyingflag;
    short keyingoverride;
};

struct AnimOverride2_93_0 {
    AnimOverride2_93_0 *next, *prev;
    char *rna_path;
    int array_index;
    float value;
};

struct AssetTag2_93_0 {
    AssetTag2_93_0 *next, *prev;
    char name[64];
};

struct BoidRule2_93_0 {
    BoidRule2_93_0 *next, *prev;
    int type, flag;
    char name[32];
};

struct BoidData2_93_0 {
    float health, acc[3];
    short state_id, mode;
};

struct BrushClone2_93_0 {
    Image2_93_0 *image;
    float offset[2];
    float alpha;
    char _pad[4];
};

struct BrushGpencilSettings2_93_0 {
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
    char _pad2[2];
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
    char _pad3[4];
    CurveMapping2_93_0 *curve_sensitivity;
    CurveMapping2_93_0 *curve_strength;
    CurveMapping2_93_0 *curve_jitter;
    CurveMapping2_93_0 *curve_rand_pressure;
    CurveMapping2_93_0 *curve_rand_strength;
    CurveMapping2_93_0 *curve_rand_uv;
    CurveMapping2_93_0 *curve_rand_hue;
    CurveMapping2_93_0 *curve_rand_saturation;
    CurveMapping2_93_0 *curve_rand_value;
    Material2_93_0 *material;
};

struct tPaletteColorHSV2_93_0 {
    float rgb[3];
    float value;
    float h;
    float s;
    float v;
};

struct PaletteColor2_93_0 {
    PaletteColor2_93_0 *next, *prev;
    float rgb[3];
    float value;
};

struct AlembicObjectPath2_93_0 {
    AlembicObjectPath2_93_0 *next, *prev;
    char path[4096];
};

struct CameraStereoSettings2_93_0 {
    float interocular_distance;
    float convergence_distance;
    short convergence_mode;
    short pivot;
    short flag;
    char _pad[2];
    float pole_merge_angle_from;
    float pole_merge_angle_to;
};

struct CameraDOFSettings2_93_0 {
    Object2_93_0 *focus_object;
    float focus_distance;
    float aperture_fstop;
    float aperture_rotation;
    float aperture_ratio;
    int aperture_blades;
    short flag;
    char _pad[2];
};

struct Camera_Runtime2_93_0 {
    float drw_corners[2][4][2];
    float drw_tria[2][2];
    float drw_depth[2];
    float drw_focusmat[4][4];
    float drw_normalmat[4][4];
};

struct ClothSimSettings2_93_0 {
    void *cache;
    float mingoal;
    float Cdis;
    float Cvi;
    float gravity[3];
    float dt;
    float mass;
    float structural;
    float shear;
    float bending;
    float max_bend;
    float max_struct;
    float max_shear;
    float max_sewing;
    float avg_spring_len;
    float timescale;
    float time_scale;
    float maxgoal;
    float eff_force_scale;
    float eff_wind_scale;
    float sim_time_old;
    float defgoal;
    float goalspring;
    float goalfrict;
    float velocity_smooth;
    float density_target;
    float density_strength;
    float collider_friction;
    float vel_damping;
    float shrink_min;
    float shrink_max;
    float uniform_pressure_force;
    float target_volume;
    float pressure_factor;
    float fluid_density;
    short vgroup_pressure;
    char _pad7[6];
    float bending_damping;
    float voxel_cell_size;
    int stepsPerFrame;
    int flags;
    int preroll;
    int maxspringlen;
    short solver_type;
    short vgroup_bend;
    short vgroup_mass;
    short vgroup_struct;
    short vgroup_shrink;
    short shapekey_rest;
    short presets;
    short reset;
    EffectorWeights2_93_0 *effector_weights;
    short bending_model;
    short vgroup_shear;
    float tension;
    float compression;
    float max_tension;
    float max_compression;
    float tension_damp;
    float compression_damp;
    float shear_damp;
    float internal_spring_max_length;
    float internal_spring_max_diversion;
    short vgroup_intern;
    char _pad1[2];
    float internal_tension;
    float internal_compression;
    float max_internal_tension;
    float max_internal_compression;
    char _pad0[4];
};

struct ClothCollSettings2_93_0 {
    void *collision_list;
    float epsilon;
    float self_friction;
    float friction;
    float damping;
    float selfepsilon;
    float repel_force;
    float distance_repel;
    int flags;
    short self_loop_count;
    short loop_count;
    char _pad[4];
    Collection2_93_0 *group;
    short vgroup_selfcol;
    short vgroup_objcol;
    char _pad2[4];
    float clamp;
    float self_clamp;
};

struct CollectionObject2_93_0 {
    CollectionObject2_93_0 *next, *prev;
    Object2_93_0 *ob;
};

struct CollectionChild2_93_0 {
    CollectionChild2_93_0 *next, *prev;
    Collection2_93_0 *collection;
};

struct CurveMapPoint2_93_0 {
    float x, y;
    short flag, shorty;
};

struct CurveMap2_93_0 {
    short totpoint;
    short flag;
    float range;
    float mintable, maxtable;
    float ext_in[2], ext_out[2];
    CurveMapPoint2_93_0 *curve;
    CurveMapPoint2_93_0 *table;
    CurveMapPoint2_93_0 *premultable;
    float premul_ext_in[2];
    float premul_ext_out[2];
};

struct Histogram2_93_0 {
    int channels;
    int x_resolution;
    float data_luma[256];
    float data_r[256];
    float data_g[256];
    float data_b[256];
    float data_a[256];
    float xmax, ymax;
    short mode;
    short flag;
    int height;
    float co[2][2];
};

struct ColorManagedViewSettings2_93_0 {
    int flag;
    char _pad[4];
    char look[64];
    char view_transform[64];
    float exposure;
    float gamma;
    CurveMapping2_93_0 *curve_mapping;
    void *_pad2;
};

struct ColorManagedDisplaySettings2_93_0 {
    char display_device[64];
};

struct ColorManagedColorspaceSettings2_93_0 {
    char name[64];
};

struct bConstraintChannel2_93_0 {
    bConstraintChannel2_93_0 *next, *prev;
    Ipo2_93_0 *ipo;
    short flag;
    char name[30];
};

struct bConstraint2_93_0 {
    bConstraint2_93_0 *next, *prev;
    void *data;
    short type;
    short flag;
    char ownspace;
    char tarspace;
    short ui_expand_flag;
    Object2_93_0 *space_object;
    char space_subtarget[64];
    char name[64];
    float enforce;
    float headtail;
    Ipo2_93_0 *ipo;
    float lin_error;
    float rot_error;
};

struct bConstraintTarget2_93_0 {
    bConstraintTarget2_93_0 *next, *prev;
    Object2_93_0 *tar;
    char subtarget[64];
    float matrix[4][4];
    short space;
    short flag;
    short type;
    short rotOrder;
    float weight;
    char _pad[4];
};

struct bKinematicConstraint2_93_0 {
    Object2_93_0 *tar;
    short iterations;
    short flag;
    short rootbone;
    short max_rootbone;
    char subtarget[64];
    Object2_93_0 *poletar;
    char polesubtarget[64];
    float poleangle;
    float weight;
    float orientweight;
    float grabtarget[3];
    short type;
    short mode;
    float dist;
};

struct bSplineIKConstraint2_93_0 {
    Object2_93_0 *tar;
    float *points;
    short numpoints;
    short chainlen;
    short flag;
    short xzScaleMode;
    short yScaleMode;
    short _pad[3];
    float bulge;
    float bulge_min;
    float bulge_max;
    float bulge_smooth;
};

struct bTrackToConstraint2_93_0 {
    Object2_93_0 *tar;
    int reserved1;
    int reserved2;
    int flags;
    char _pad[4];
    char subtarget[64];
};

struct bRotateLikeConstraint2_93_0 {
    Object2_93_0 *tar;
    int flag;
    char euler_order;
    char mix_mode;
    char _pad[2];
    char subtarget[64];
};

struct bLocateLikeConstraint2_93_0 {
    Object2_93_0 *tar;
    int flag;
    int reserved1;
    char subtarget[64];
};

struct bSizeLikeConstraint2_93_0 {
    Object2_93_0 *tar;
    int flag;
    float power;
    char subtarget[64];
};

struct bSameVolumeConstraint2_93_0 {
    char free_axis;
    char mode;
    char _pad[2];
    float volume;
};

struct bTransLikeConstraint2_93_0 {
    Object2_93_0 *tar;
    char mix_mode;
    char _pad[7];
    char subtarget[64];
};

struct bMinMaxConstraint2_93_0 {
    Object2_93_0 *tar;
    int minmaxflag;
    float offset;
    int flag;
    char subtarget[64];
    int _pad;
};

struct bActionConstraint2_93_0 {
    Object2_93_0 *tar;
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
    bAction2_93_0 *act;
    char subtarget[64];
};

struct bLockTrackConstraint2_93_0 {
    Object2_93_0 *tar;
    int trackflag;
    int lockflag;
    char subtarget[64];
};

struct bDampTrackConstraint2_93_0 {
    Object2_93_0 *tar;
    int trackflag;
    char _pad[4];
    char subtarget[64];
};

struct bFollowPathConstraint2_93_0 {
    Object2_93_0 *tar;
    float offset;
    float offset_fac;
    int followflag;
    short trackflag;
    short upflag;
};

struct bStretchToConstraint2_93_0 {
    Object2_93_0 *tar;
    int flag;
    int volmode;
    int plane;
    float orglength;
    float bulge;
    float bulge_min;
    float bulge_max;
    float bulge_smooth;
    char subtarget[64];
};

struct bRigidBodyJointConstraint2_93_0 {
    Object2_93_0 *tar;
    Object2_93_0 *child;
    int type;
    float pivX;
    float pivY;
    float pivZ;
    float axX;
    float axY;
    float axZ;
    float minLimit[6];
    float maxLimit[6];
    float extraFz;
    short flag;
    char _pad[6];
};

struct bClampToConstraint2_93_0 {
    Object2_93_0 *tar;
    int flag;
    int flag2;
};

struct bChildOfConstraint2_93_0 {
    Object2_93_0 *tar;
    int flag;
    char _pad[4];
    float invmat[4][4];
    char subtarget[64];
};

struct bTransformConstraint2_93_0 {
    Object2_93_0 *tar;
    char subtarget[64];
    short from, to;
    char map[3];
    char expo;
    char from_rotation_mode;
    char to_euler_order;
    char mix_mode_loc;
    char mix_mode_rot;
    char mix_mode_scale;
    char _pad[3];
    float from_min[3];
    float from_max[3];
    float to_min[3];
    float to_max[3];
    float from_min_rot[3];
    float from_max_rot[3];
    float to_min_rot[3];
    float to_max_rot[3];
    float from_min_scale[3];
    float from_max_scale[3];
    float to_min_scale[3];
    float to_max_scale[3];
};

struct bPivotConstraint2_93_0 {
    Object2_93_0 *tar;
    char subtarget[64];
    float offset[3];
    short rotAxis;
    short flag;
};

struct bLocLimitConstraint2_93_0 {
    float xmin, xmax;
    float ymin, ymax;
    float zmin, zmax;
    short flag;
    short flag2;
};

struct bRotLimitConstraint2_93_0 {
    float xmin, xmax;
    float ymin, ymax;
    float zmin, zmax;
    short flag;
    short flag2;
};

struct bSizeLimitConstraint2_93_0 {
    float xmin, xmax;
    float ymin, ymax;
    float zmin, zmax;
    short flag;
    short flag2;
};

struct bDistLimitConstraint2_93_0 {
    Object2_93_0 *tar;
    char subtarget[64];
    float dist;
    float soft;
    short flag;
    short mode;
    char _pad[4];
};

struct bShrinkwrapConstraint2_93_0 {
    Object2_93_0 *target;
    float dist;
    short shrinkType;
    char projAxis;
    char projAxisSpace;
    float projLimit;
    char shrinkMode;
    char flag;
    char trackAxis;
    char _pad;
};

struct bFollowTrackConstraint2_93_0 {
    MovieClip2_93_0 *clip;
    char track[64];
    int flag;
    int frame_method;
    char object[64];
    Object2_93_0 *camera;
    Object2_93_0 *depth_ob;
};

struct bCameraSolverConstraint2_93_0 {
    MovieClip2_93_0 *clip;
    int flag;
    char _pad[4];
};

struct bObjectSolverConstraint2_93_0 {
    MovieClip2_93_0 *clip;
    int flag;
    char _pad[4];
    char object[64];
    float invmat[4][4];
    Object2_93_0 *camera;
};

struct bTransformCacheConstraint2_93_0 {
    CacheFile2_93_0 *cache_file;
    char object_path[1024];
    void *reader;
    char reader_object_path[1024];
};

struct CurveProfilePoint2_93_0 {
    float x, y;
    short flag;
    char h1, h2;
    float h1_loc[2];
    float h2_loc[2];
    char _pad[4];
    CurveProfile2_93_0 *profile;
};

struct BevPoint2_93_0 {
    float vec[3], tilt, radius, weight, offset;
    float sina, cosa;
    float dir[3], tan[3], quat[4];
    short dupe_tag;
};

struct BevList2_93_0 {
    BevList2_93_0 *next, *prev;
    int nr, dupe_nr;
    int poly, hole;
    int charidx;
    int *segbevcount;
    float *seglen;
    BevPoint2_93_0 *bevpoints;
};

struct BezTriple2_93_0 {
    float vec[3][3];
    float tilt;
    float weight;
    float radius;
    char ipo;
    uint8_t h1, h2;
    uint8_t f1, f2, f3;
    char hide;
    char easing;
    float back;
    float amplitude, period;
    char auto_handle_type;
    char _pad[3];
};

struct BPoint2_93_0 {
    float vec[4];
    float tilt;
    float weight;
    uint8_t f1;
    char _pad1[1];
    short hide;
    float radius;
    char _pad[4];
};

struct Nurb2_93_0 {
    Nurb2_93_0 *next, *prev;
    short type;
    short mat_nr;
    short hide, flag;
    int pntsu, pntsv;
    char _pad[4];
    short resolu, resolv;
    short orderu, orderv;
    short flagu, flagv;
    float *knotsu, *knotsv;
    BPoint2_93_0 *bp;
    BezTriple2_93_0 *bezt;
    short tilt_interp;
    short radius_interp;
    int charidx;
};

struct CharInfo2_93_0 {
    short kern;
    short mat_nr;
    char flag;
    char _pad[3];
};

struct TextBox2_93_0 {
    float x, y, w, h;
};

struct CustomDataLayer2_93_0 {
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
};

struct CustomDataExternal2_93_0 {
    char filename[1024];
};

struct CustomData2_93_0 {
    CustomDataLayer2_93_0 *layers;
    int typemap[51];
    int totlayer, maxlayer;
    int totsize;
    void *pool;
    CustomDataExternal2_93_0 *external;
};

struct CustomData_MeshMasks2_93_0 {
    uint64_t vmask;
    uint64_t emask;
    uint64_t fmask;
    uint64_t pmask;
    uint64_t lmask;
};

struct DynamicPaintRuntime2_93_0 {
    Mesh2_93_0 *canvas_mesh;
    Mesh2_93_0 *brush_mesh;
};

struct DynamicPaintBrushSettings2_93_0 {
    DynamicPaintModifierData2_93_0 *pmd;
    void *psys;
    int flags;
    int collision;
    float r, g, b, alpha;
    float wetness;
    float particle_radius, particle_smooth;
    float paint_distance;
    ColorBand2_93_0 *paint_ramp;
    ColorBand2_93_0 *vel_ramp;
    short proximity_falloff;
    short wave_type;
    short ray_dir;
    char _pad[2];
    float wave_factor, wave_clamp;
    float max_velocity, smudge_strength;
};

struct Effect2_93_0 {
    Effect2_93_0 *next, *prev;
    short type, flag, buttype, rt;
};

struct BuildEff2_93_0 {
    BuildEff2_93_0 *next, *prev;
    short type, flag, buttype, rt;
    float len, sfra;
};

struct Particle2_93_0 {
    float co[3], no[3];
    float time, lifetime;
    short mat_nr, rt;
};

struct PartEff2_93_0 {
    PartEff2_93_0 *next, *prev;
    short type, flag, buttype, stype, vertgroup, userjit;
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
    Particle2_93_0 *keys;
    Collection2_93_0 *group;
};

struct WaveEff2_93_0 {
    WaveEff2_93_0 *next, *prev;
    short type, flag, buttype, stype;
    float startx, starty, height, width;
    float narrow, speed, minfac, damp;
    float timeoffs, lifetime;
};

struct FileGlobal2_93_0 {
    char subvstr[4];
    short subversion;
    short minversion, minsubversion;
    char _pad[6];
    bScreen2_93_0 *curscreen;
    Scene2_93_0 *curscene;
    ViewLayer2_93_0 *cur_view_layer;
    void *_pad1;
    int fileflags;
    int globalf;
    uint64_t build_commit_timestamp;
    char build_hash[16];
    char filename[1024];
};

struct FluidDomainVertexVelocity2_93_0 {
    float vel[3];
};

struct FluidFlowSettings2_93_0 {
    FluidModifierData2_93_0 *fmd;
    Mesh2_93_0 *mesh;
    void *psys;
    Tex2_93_0 *noise_texture;
    float *verts_old;
    int numverts;
    float vel_multi;
    float vel_normal;
    float vel_random;
    float vel_coord[3];
    char _pad1[4];
    float density;
    float color[3];
    float fuel_amount;
    float temperature;
    float volume_density;
    float surface_distance;
    float particle_size;
    int subframes;
    float texture_size;
    float texture_offset;
    char _pad2[4];
    char uvlayer_name[64];
    short vgroup_density;
    short type;
    short behavior;
    short source;
    short texture_type;
    short _pad3[3];
    int flags;
};

struct FluidEffectorSettings2_93_0 {
    FluidModifierData2_93_0 *fmd;
    Mesh2_93_0 *mesh;
    float *verts_old;
    int numverts;
    float surface_distance;
    int flags;
    int subframes;
    short type;
    char _pad1[6];
    float vel_multi;
    short guide_mode;
    char _pad2[2];
};

struct FreestyleLineSet2_93_0 {
    FreestyleLineSet2_93_0 *next, *prev;
    char name[64];
    int flags;
    int selection;
    short qi;
    char _pad1[2];
    int qi_start, qi_end;
    int edge_types, exclude_edge_types;
    char _pad2[4];
    Collection2_93_0 *group;
    FreestyleLineStyle2_93_0 *linestyle;
};

struct FreestyleModuleConfig2_93_0 {
    FreestyleModuleConfig2_93_0 *next, *prev;
    Text2_93_0 *script;
    short is_displayed;
    char _pad[6];
};

struct GpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 *next, *prev;
    int type, mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
};

struct bGPDcontrolpoint2_93_0 {
    float x, y, z;
    float color[4];
    int size;
};

struct bGPDspoint_Runtime2_93_0 {
    bGPDspoint2_93_0 *pt_orig;
    int idx_orig;
    char _pad0[4];
};

struct bGPDtriangle2_93_0 {
    unsigned int verts[3];
};

struct bGPDpalettecolor2_93_0 {
    bGPDpalettecolor2_93_0 *next, *prev;
    char info[64];
    float color[4];
    float fill[4];
    short flag;
    char _pad[6];
};

struct bGPDcurve2_93_0 {
    bGPDcurve_point2_93_0 *curve_points;
    int tot_curve_points;
    short flag;
    char _pad[2];
};

struct bGPDstroke_Runtime2_93_0 {
    char tmp_layerinfo[128];
    float multi_frame_falloff;
    int stroke_start;
    int fill_start;
    int curve_start;
    bGPDstroke2_93_0 *gps_orig;
    void *_pad2;
};

struct bGPDframe_Runtime2_93_0 {
    int frameid;
    int onion_id;
    bGPDframe2_93_0 *gpf_orig;
};

struct bGPDlayer_Mask2_93_0 {
    bGPDlayer_Mask2_93_0 *next, *prev;
    char name[128];
    short flag;
    short sort_index;
    char _pad[4];
};

struct bGPDlayer_Runtime2_93_0 {
    int icon_id;
    char _pad[4];
    bGPDlayer2_93_0 *gpl_orig;
};

struct bGPdata_Runtime2_93_0 {
    void *sbuffer;
    void *sbuffer_stroke_batch;
    void *sbuffer_fill_batch;
    bGPDstroke2_93_0 *sbuffer_gps;
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
    Brush2_93_0 *sbuffer_brush;
    void *gpencil_cache;
};

struct bGPgrid2_93_0 {
    float color[3];
    float scale[2];
    float offset[2];
    char _pad1[4];
    int lines;
    char _pad[4];
};

struct GPUDOFSettings2_93_0 {
    float focus_distance;
    float fstop;
    float focal_length;
    float sensor;
    float rotation;
    float ratio;
    int num_blades;
    int high_quality;
};

struct HairCurve2_93_0 {
    int firstpoint;
    int numpoints;
};

struct HairMapping2_93_0 {
    float uv[2];
    int poly;
};

struct DrawDataList2_93_0 {
    void *first, *last;
};

struct IDOverrideLibraryPropertyOperation2_93_0 {
    IDOverrideLibraryPropertyOperation2_93_0 *next, *prev;
    short operation;
    short flag;
    short tag;
    char _pad0[2];
    char *subitem_reference_name;
    char *subitem_local_name;
    int subitem_reference_index;
    int subitem_local_index;
};

struct ID2_93_0 {
    void *next, *prev;
    ID2_93_0 *newid;
    Library2_93_0 *lib;
    AssetMetaData2_93_0 *asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    int recalc;
    int recalc_up_to_undo_push;
    int recalc_after_undo_push;
    unsigned int session_uuid;
    IDProperty2_93_0 *properties;
    IDOverrideLibrary2_93_0 *override_library;
    ID2_93_0 *orig_id;
    void *py_instance;
    void *_pad1;
};

struct PreviewImage2_93_0 {
    unsigned int w[2];
    unsigned int h[2];
    short flag[2];
    short changed_timestamp[2];
    unsigned int *rect[2];
    void *gputexture[2];
    int icon_id;
    short tag;
    char _pad[2];
};

struct ImageUser2_93_0 {
    Scene2_93_0 *scene;
    int framenr;
    int frames;
    int offset, sfra;
    char _pad0, cycl;
    char ok;
    char multiview_eye;
    short pass;
    char _pad1[2];
    int tile;
    int _pad2;
    short multi_index, view, layer;
    short flag;
};

struct ImageAnim2_93_0 {
    ImageAnim2_93_0 *next, *prev;
    void *anim;
};

struct ImageView2_93_0 {
    ImageView2_93_0 *next, *prev;
    char name[64];
    char filepath[1024];
};

struct ImagePackedFile2_93_0 {
    ImagePackedFile2_93_0 *next, *prev;
    PackedFile2_93_0 *packedfile;
    char filepath[1024];
};

struct RenderSlot2_93_0 {
    RenderSlot2_93_0 *next, *prev;
    char name[64];
    void *render;
};

struct ImageTile_Runtime2_93_0 {
    int tilearray_layer;
    int _pad;
    int tilearray_offset[2];
    int tilearray_size[2];
};

struct IpoDriver2_93_0 {
    Object2_93_0 *ob;
    short blocktype, adrcode;
    short type, flag;
    char name[128];
};

struct KeyBlock2_93_0 {
    KeyBlock2_93_0 *next, *prev;
    float pos;
    float curval;
    short type;
    char _pad1[2];
    short relative;
    short flag;
    int totelem;
    int uid;
    void *data;
    char name[64];
    char vgroup[64];
    float slidermin;
    float slidermax;
};

struct EditLatt2_93_0 {
    Lattice2_93_0 *latt;
    int shapenr;
    char needs_flush_to_id;
};

struct Base2_93_0 {
    Base2_93_0 *next, *prev;
    short flag_from_collection;
    short flag;
    unsigned short local_view_bits;
    short sx, sy;
    char _pad1[6];
    Object2_93_0 *object;
    unsigned int lay;
    int flag_legacy;
    unsigned short local_collections_bits;
    short _pad2[3];
    Base2_93_0 *base_orig;
    void *_pad;
};

struct ViewLayerEEVEE2_93_0 {
    int render_passes;
    int _pad[1];
};

struct ViewLayerAOV2_93_0 {
    ViewLayerAOV2_93_0 *next, *prev;
    char name[64];
    int flag;
    int type;
};

struct LightProbeCache2_93_0 {
    float position[3], parallax_type;
    float attenuation_fac;
    float attenuation_type;
    float _pad3[2];
    float attenuationmat[4][4];
    float parallaxmat[4][4];
};

struct LightGridCache2_93_0 {
    float mat[4][4];
    int resolution[3], offset;
    float corner[3], attenuation_scale;
    float increment_x[3], attenuation_bias;
    float increment_y[3], level_bias;
    float increment_z[3], _pad4;
    float visibility_bias, visibility_bleed, visibility_range, _pad5;
};

struct LightCacheTexture2_93_0 {
    void *tex;
    char *data;
    int tex_size[3];
    char data_type;
    char components;
    char _pad[2];
};

struct LineStyleModifier2_93_0 {
    LineStyleModifier2_93_0 *next, *prev;
    char name[64];
    int type;
    float influence;
    int flags;
    int blend;
};

struct Link2_93_0 {
    Link2_93_0 *next, *prev;
};

struct LinkData2_93_0 {
    LinkData2_93_0 *next, *prev;
    void *data;
};

struct ListBase2_93_0 {
    void *first, *last;
};

struct MaskParent2_93_0 {
    int id_type;
    int type;
    ID2_93_0 *id;
    char parent[64];
    char sub_parent[64];
    float parent_orig[2];
    float parent_corners_orig[4][2];
};

struct MaskSplinePointUW2_93_0 {
    float u, w;
    int flag;
};

struct MaskLayerShape2_93_0 {
    MaskLayerShape2_93_0 *next, *prev;
    float *data;
    int tot_vert;
    int frame;
    char flag;
    char _pad[7];
};

struct MaskLayerShapeElem2_93_0 {
    float value[8];
};

struct TexPaintSlot2_93_0 {
    Image2_93_0 *ima;
    char *uvname;
    int valid;
    int interp;
};

struct MaterialGPencilStyle2_93_0 {
    Image2_93_0 *sima;
    Image2_93_0 *ima;
    float stroke_rgba[4];
    float fill_rgba[4];
    float mix_rgba[4];
    short flag;
    short index;
    short stroke_style;
    short fill_style;
    float mix_factor;
    float gradient_angle;
    float gradient_radius;
    char _pad2[4];
    float gradient_scale[2];
    float gradient_shift[2];
    float texture_angle;
    float texture_scale[2];
    float texture_offset[2];
    float texture_opacity;
    float texture_pixsize;
    int mode;
    int gradient_type;
    float mix_stroke_factor;
    int alignment_mode;
    float alignment_rotation;
};

struct MaterialLineArt2_93_0 {
    int flags;
    unsigned char transparency_mask;
    unsigned char _pad[3];
};

struct MVert2_93_0 {
    float co[3];
    short no[3];
    char flag, bweight;
};

struct MEdge2_93_0 {
    unsigned int v1, v2;
    char crease, bweight;
    short flag;
};

struct MPoly2_93_0 {
    int loopstart;
    int totloop;
    short mat_nr;
    char flag, _pad;
};

struct MLoop2_93_0 {
    unsigned int v;
    unsigned int e;
};

struct MSelect2_93_0 {
    int index;
    int type;
};

struct MLoopTri2_93_0 {
    unsigned int tri[3];
    unsigned int poly;
};

struct MVertTri2_93_0 {
    unsigned int tri[3];
};

struct MFloatProperty2_93_0 {
    float f;
};

struct MIntProperty2_93_0 {
    int i;
};

struct MStringProperty2_93_0 {
    char s[255], s_len;
};

struct MBoolProperty2_93_0 {
    uint8_t b;
};

struct MDeformWeight2_93_0 {
    unsigned int def_nr;
    float weight;
};

struct MDeformVert2_93_0 {
    MDeformWeight2_93_0 *dw;
    int totweight;
    int flag;
};

struct MVertSkin2_93_0 {
    float radius[3];
    int flag;
};

struct MLoopUV2_93_0 {
    float uv[2];
    int flag;
};

struct MLoopCol2_93_0 {
    unsigned char r, g, b, a;
};

struct MPropCol2_93_0 {
    float color[4];
};

struct GridPaintMask2_93_0 {
    float *data;
    unsigned int level;
    char _pad[4];
};

struct OrigSpaceFace2_93_0 {
    float uv[4][2];
};

struct OrigSpaceLoop2_93_0 {
    float uv[2];
};

struct FreestyleEdge2_93_0 {
    char flag;
};

struct FreestyleFace2_93_0 {
    char flag;
};

struct MFace2_93_0 {
    unsigned int v1, v2, v3, v4;
    short mat_nr;
    char edcode, flag;
};

struct MTFace2_93_0 {
    float uv[4][2];
};

struct MCol2_93_0 {
    unsigned char a, r, g, b;
};

struct MRecast2_93_0 {
    int i;
};

struct MLoopTri_Store2_93_0 {
    MLoopTri2_93_0 *array, *array_wip;
    int len;
    int len_alloc;
};

struct TFace2_93_0 {
    void *tpage;
    float uv[4][2];
    unsigned int col[4];
    char flag, transp;
    short mode, tile, unwrap;
};

struct MetaElem2_93_0 {
    MetaElem2_93_0 *next, *prev;
    BoundBox2_93_0 *bb;
    short type, flag;
    char _pad[4];
    float x, y, z;
    float quat[4];
    float expx;
    float expy;
    float expz;
    float rad;
    float rad2;
    float s;
    float len;
    float *mat, *imat;
};

struct MDefInfluence2_93_0 {
    int vertex;
    float weight;
};

struct MDefCell2_93_0 {
    int offset;
    int totinfluence;
};

struct MeshCacheVertexVelocity2_93_0 {
    float vel[3];
};

struct SDefBind2_93_0 {
    unsigned int *vert_inds;
    unsigned int numverts;
    int mode;
    float *vert_weights;
    float normal_dist;
    float influence;
};

struct SDefVert2_93_0 {
    SDefBind2_93_0 *binds;
    unsigned int numbinds;
    char _pad[4];
};

struct NodesModifierSettings2_93_0 {
    IDProperty2_93_0 *properties;
};

struct MovieClipUser2_93_0 {
    int framenr;
    short render_size, render_flag;
};

struct MovieClipProxy2_93_0 {
    char dir[768];
    short tc;
    short quality;
    short build_size_flag;
    short build_tc_flag;
};

struct bActionModifier2_93_0 {
    bActionModifier2_93_0 *next, *prev;
    short type, flag;
    char channel[32];
    float noisesize, turbul;
    short channels;
    short no_rot_axis;
    Object2_93_0 *ob;
};

struct bNodeStack2_93_0 {
    float vec[4];
    float min, max;
    void *data;
    short hasinput;
    short hasoutput;
    short datatype;
    short sockettype;
    short is_copy;
    short external;
    char _pad[4];
};

struct bNodeInstanceKey2_93_0 {
    unsigned int value;
};

struct bNodeLink2_93_0 {
    bNodeLink2_93_0 *next, *prev;
    bNode2_93_0 *fromnode, *tonode;
    bNodeSocket2_93_0 *fromsock, *tosock;
    int flag;
    int multi_input_socket_index;
};

struct bNodeSocketValueInt2_93_0 {
    int subtype;
    int value;
    int min, max;
};

struct bNodeSocketValueFloat2_93_0 {
    int subtype;
    float value;
    float min, max;
};

struct bNodeSocketValueBoolean2_93_0 {
    char value;
};

struct bNodeSocketValueVector2_93_0 {
    int subtype;
    float value[3];
    float min, max;
};

struct bNodeSocketValueRGBA2_93_0 {
    float value[4];
};

struct bNodeSocketValueString2_93_0 {
    int subtype;
    char _pad[4];
    char value[1024];
};

struct bNodeSocketValueObject2_93_0 {
    Object2_93_0 *value;
};

struct bNodeSocketValueImage2_93_0 {
    Image2_93_0 *value;
};

struct bNodeSocketValueCollection2_93_0 {
    Collection2_93_0 *value;
};

struct NodeFrame2_93_0 {
    short flag;
    short label_size;
};

struct NodeImageAnim2_93_0 {
    int frames;
    int sfra;
    int nr;
    char cyclic;
    char movie;
    char _pad[2];
};

struct ColorCorrectionData2_93_0 {
    float saturation;
    float contrast;
    float gamma;
    float gain;
    float lift;
    char _pad[4];
};

struct NodeBokehImage2_93_0 {
    float angle;
    int flaps;
    float rounding;
    float catadioptric;
    float lensshift;
};

struct NodeBoxMask2_93_0 {
    float x;
    float y;
    float rotation;
    float height;
    float width;
    char _pad[4];
};

struct NodeEllipseMask2_93_0 {
    float x;
    float y;
    float rotation;
    float height;
    float width;
    char _pad[4];
};

struct NodeImageLayer2_93_0 {
    int pass_index;
    char pass_name[64];
};

struct NodeBlurData2_93_0 {
    short sizex, sizey;
    short samples, maxspeed, minspeed, relative, aspect;
    short curved;
    float fac, percentx, percenty;
    short filtertype;
    char bokeh, gamma;
    int image_in_width, image_in_height;
};

struct NodeDBlurData2_93_0 {
    float center_x, center_y, distance, angle, spin, zoom;
    short iter;
    char wrap, _pad;
};

struct NodeBilateralBlurData2_93_0 {
    float sigma_color, sigma_space;
    short iter;
    char _pad[2];
};

struct NodeAntiAliasingData2_93_0 {
    float threshold;
    float contrast_limit;
    float corner_rounding;
};

struct NodeHueSat2_93_0 {
    float hue, sat, val;
};

struct NodeChroma2_93_0 {
    float t1, t2, t3;
    float fsize, fstrength, falpha;
    float key[4];
    short algorithm, channel;
};

struct NodeTwoXYs2_93_0 {
    short x1, x2, y1, y2;
    float fac_x1, fac_x2, fac_y1, fac_y2;
};

struct NodeTwoFloats2_93_0 {
    float x, y;
};

struct NodeVertexCol2_93_0 {
    char name[64];
};

struct NodeDefocus2_93_0 {
    char bktype, _pad0, preview, gamco;
    short samples, no_zbuf;
    float fstop, maxblur, bthresh, scale;
    float rotation;
    char _pad1[4];
};

struct NodeScriptDict2_93_0 {
    void *dict;
    void *node;
};

struct NodeGlare2_93_0 {
    char quality, type, iter;
    char angle , _pad0, size, star_45, streaks;
    float colmod, mix, threshold, fade;
    float angle_ofs;
    char _pad1[4];
};

struct NodeTonemap2_93_0 {
    float key, offset, gamma;
    float f, m, a, c;
    int type;
};

struct NodeLensDist2_93_0 {
    short jit, proj, fit;
    char _pad[2];
};

struct NodeColorBalance2_93_0 {
    float slope[3];
    float offset[3];
    float power[3];
    float offset_basis;
    char _pad[4];
    float lift[3];
    float gamma[3];
    float gain[3];
};

struct NodeColorspill2_93_0 {
    short limchan, unspill;
    float limscale;
    float uspillr, uspillg, uspillb;
};

struct NodeDilateErode2_93_0 {
    char falloff;
};

struct NodeMask2_93_0 {
    int size_x, size_y;
};

struct NodeSetAlpha2_93_0 {
    char mode;
};

struct NodeShaderAttribute2_93_0 {
    char name[64];
    int type;
    char _pad[4];
};

struct NodeShaderVectTransform2_93_0 {
    int type;
    int convert_from, convert_to;
    char _pad[4];
};

struct TexNodeOutput2_93_0 {
    char name[64];
};

struct NodeKeyingScreenData2_93_0 {
    char tracking_object[64];
};

struct NodeKeyingData2_93_0 {
    float screen_balance;
    float despill_factor;
    float despill_balance;
    int edge_kernel_radius;
    float edge_kernel_tolerance;
    float clip_black, clip_white;
    int dilate_distance;
    int feather_distance;
    int feather_falloff;
    int blur_pre, blur_post;
};

struct NodeTrackPosData2_93_0 {
    char tracking_object[64];
    char track_name[64];
};

struct NodeTranslateData2_93_0 {
    char wrap_axis;
    char relative;
};

struct NodePlaneTrackDeformData2_93_0 {
    char tracking_object[64];
    char plane_track_name[64];
    char flag;
    char motion_blur_samples;
    char _pad[2];
    float motion_blur_shutter;
};

struct NodeShaderScript2_93_0 {
    int mode;
    int flag;
    char filepath[1024];
    char bytecode_hash[64];
    char *bytecode;
};

struct NodeShaderTangent2_93_0 {
    int direction_type;
    int axis;
    char uv_map[64];
};

struct NodeShaderNormalMap2_93_0 {
    int space;
    char uv_map[64];
};

struct NodeShaderUVMap2_93_0 {
    char uv_map[64];
};

struct NodeShaderVertexColor2_93_0 {
    char layer_name[64];
};

struct NodeShaderTexIES2_93_0 {
    int mode;
    char filepath[1024];
};

struct NodeShaderOutputAOV2_93_0 {
    char name[64];
};

struct NodeSunBeams2_93_0 {
    float source[2];
    float ray_length;
};

struct CryptomatteEntry2_93_0 {
    CryptomatteEntry2_93_0 *next, *prev;
    float encoded_hash;
    char name[64];
    char _pad[4];
};

struct CryptomatteLayer2_93_0 {
    CryptomatteEntry2_93_0 *next, *prev;
    char name[64];
};

struct NodeDenoise2_93_0 {
    char hdr;
};

struct NodeAttributeClamp2_93_0 {
    uint8_t data_type;
    uint8_t operation;
};

struct NodeAttributeCompare2_93_0 {
    uint8_t operation;
    uint8_t input_type_a;
    uint8_t input_type_b;
    char _pad[5];
};

struct NodeAttributeMapRange2_93_0 {
    uint8_t data_type;
    uint8_t interpolation_type;
};

struct NodeAttributeMath2_93_0 {
    uint8_t operation;
    uint8_t input_type_a;
    uint8_t input_type_b;
    uint8_t input_type_c;
};

struct NodeAttributeMix2_93_0 {
    uint8_t blend_type;
    uint8_t input_type_factor;
    uint8_t input_type_a;
    uint8_t input_type_b;
};

struct NodeAttributeRandomize2_93_0 {
    uint8_t data_type;
    uint8_t domain;
    uint8_t operation;
    char _pad[1];
};

struct NodeAttributeVectorMath2_93_0 {
    uint8_t operation;
    uint8_t input_type_a;
    uint8_t input_type_b;
    uint8_t input_type_c;
};

struct NodeInputVector2_93_0 {
    float vector[3];
};

struct NodeInputString2_93_0 {
    char *string;
};

struct NodeGeometryRotatePoints2_93_0 {
    uint8_t type;
    uint8_t space;
    uint8_t input_type_axis;
    uint8_t input_type_angle;
    uint8_t input_type_rotation;
    char _pad[3];
};

struct NodeGeometryAlignRotationToVector2_93_0 {
    uint8_t axis;
    uint8_t pivot_axis;
    uint8_t input_type_factor;
    uint8_t input_type_vector;
};

struct NodeGeometryPointScale2_93_0 {
    uint8_t input_type;
};

struct NodeGeometryPointTranslate2_93_0 {
    uint8_t input_type;
};

struct NodeGeometryObjectInfo2_93_0 {
    uint8_t transform_space;
};

struct NodeGeometryPointInstance2_93_0 {
    uint8_t instance_type;
    uint8_t flag;
};

struct NodeGeometryPointsToVolume2_93_0 {
    uint8_t resolution_mode;
    uint8_t input_type_radius;
};

struct NodeGeometryCollectionInfo2_93_0 {
    uint8_t transform_space;
};

struct NodeGeometryAttributeProximity2_93_0 {
    uint8_t target_geometry_element;
};

struct NodeGeometryVolumeToMesh2_93_0 {
    uint8_t resolution_mode;
};

struct NodeAttributeCombineXYZ2_93_0 {
    uint8_t input_type_x;
    uint8_t input_type_y;
    uint8_t input_type_z;
    char _pad[1];
};

struct NodeAttributeSeparateXYZ2_93_0 {
    uint8_t input_type;
};

struct NodeAttributeConvert2_93_0 {
    uint8_t data_type;
    char _pad[1];
    int16_t domain;
};

struct NodeGeometryMeshCircle2_93_0 {
    uint8_t fill_type;
};

struct NodeGeometryMeshCylinder2_93_0 {
    uint8_t fill_type;
};

struct NodeGeometryMeshCone2_93_0 {
    uint8_t fill_type;
};

struct NodeGeometryMeshLine2_93_0 {
    uint8_t mode;
    uint8_t count_mode;
};

struct FluidVertexVelocity2_93_0 {
    float vel[3];
};

struct FluidsimSettings2_93_0 {
    FluidsimModifierData2_93_0 *fmd;
    int threads;
    char _pad1[4];
    short type;
    short show_advancedoptions;
    short resolutionxyz;
    short previewresxyz;
    float realsize;
    short guiDisplayMode;
    short renderDisplayMode;
    float viscosityValue;
    short viscosityMode;
    short viscosityExponent;
    float grav[3];
    float animStart, animEnd;
    int bakeStart, bakeEnd;
    int frameOffset;
    char _pad2[4];
    float gstar;
    int maxRefine;
    float iniVelx, iniVely, iniVelz;
    char surfdataPath[1024];
    float bbStart[3], bbSize[3];
    Ipo2_93_0 *ipo;
    short typeFlags;
    char domainNovecgen, volumeInitType;
    float partSlipValue;
    int generateTracers;
    float generateParticles;
    float surfaceSmoothing;
    int surfaceSubdivs;
    int flag;
    float particleInfSize, particleInfAlpha;
    float farFieldSize;
    FluidVertexVelocity2_93_0 *meshVelocities;
    int totvert;
    float cpsTimeStart;
    float cpsTimeEnd;
    float cpsQuality;
    float attractforceStrength;
    float attractforceRadius;
    float velocityforceStrength;
    float velocityforceRadius;
    int lastgoodframe;
    float animRate;
};

struct PartDeflect2_93_0 {
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
    void *rng;
    float f_noise;
    int seed;
    float drawvec1[4];
    float drawvec2[4];
    float drawvec_falloff_min[3];
    char _pad1[4];
    float drawvec_falloff_max[3];
    char _pad2[4];
    Object2_93_0 *f_source;
    float pdef_cfrict;
    char _pad[4];
};

struct EffectorWeights2_93_0 {
    Collection2_93_0 *group;
    float weight[14];
    float global_gravity;
    short flag, rt[3];
    char _pad[4];
};

struct SBVertex2_93_0 {
    float vec[4];
};

struct bDeformGroup2_93_0 {
    bDeformGroup2_93_0 *next, *prev;
    char name[64];
    char flag, _pad0[7];
};

struct bFaceMap2_93_0 {
    bFaceMap2_93_0 *next, *prev;
    char name[64];
    char flag;
    char _pad0[7];
};

struct BoundBox2_93_0 {
    float vec[8][3];
    int flag;
    char _pad0[4];
};

struct ObjectLineArt2_93_0 {
    short usage;
    short flags;
    float crease_threshold;
};

struct ObHook2_93_0 {
    ObHook2_93_0 *next, *prev;
    Object2_93_0 *parent;
    float parentinv[4][4];
    float mat[4][4];
    float cent[3];
    float falloff;
    char name[64];
    int *indexar;
    int totindex, curindex;
    short type, active;
    float force;
};

struct TreeStoreElem2_93_0 {
    short type, nr, flag, used;
    ID2_93_0 *id;
};

struct TreeStore2_93_0 {
    int totelem;
    int usedelem;
    TreeStoreElem2_93_0 *data;
};

struct PackedFile2_93_0 {
    int size;
    int seek;
    void *data;
};

struct HairKey2_93_0 {
    float co[3];
    float time;
    float weight;
    short editflag;
    char _pad[2];
    float world_co[3];
};

struct ParticleKey2_93_0 {
    float co[3];
    float vel[3];
    float rot[4];
    float ave[3];
    float time;
};

struct ParticleSpring2_93_0 {
    float rest_length;
    unsigned int particle_index[2], delete_flag;
};

struct ChildParticle2_93_0 {
    int num, parent;
    int pa[4];
    float w[4];
    float fuv[4], foffset;
    float rt;
};

struct ParticleTarget2_93_0 {
    ParticleTarget2_93_0 *next, *prev;
    Object2_93_0 *ob;
    int psys;
    short flag, mode;
    float time, duration;
};

struct ParticleDupliWeight2_93_0 {
    ParticleDupliWeight2_93_0 *next, *prev;
    Object2_93_0 *ob;
    short count;
    short flag;
    short index, rt;
};

struct SPHFluidSettings2_93_0 {
    float radius, spring_k, rest_length;
    float plasticity_constant, yield_ratio;
    float plasticity_balance, yield_balance;
    float viscosity_omega, viscosity_beta;
    float stiffness_k, stiffness_knear, rest_density;
    float buoyancy;
    int flag, spring_frames;
    short solver;
    char _pad[6];
};

struct PTCacheExtra2_93_0 {
    PTCacheExtra2_93_0 *next, *prev;
    unsigned int type, totdata;
    void *data;
};

struct RigidBodyOb_Shared2_93_0 {
    void *physics_object;
    void *physics_shape;
};

struct RigidBodyOb2_93_0 {
    short type;
    short shape;
    int flag;
    int col_groups;
    short mesh_source;
    char _pad[2];
    float mass;
    float friction;
    float restitution;
    float margin;
    float lin_damping;
    float ang_damping;
    float lin_sleep_thresh;
    float ang_sleep_thresh;
    float orn[4];
    float pos[3];
    char _pad1[4];
    RigidBodyOb_Shared2_93_0 *shared;
};

struct RigidBodyCon2_93_0 {
    Object2_93_0 *ob1;
    Object2_93_0 *ob2;
    short type;
    short num_solver_iterations;
    int flag;
    float breaking_threshold;
    char spring_type;
    char _pad[3];
    float limit_lin_x_lower;
    float limit_lin_x_upper;
    float limit_lin_y_lower;
    float limit_lin_y_upper;
    float limit_lin_z_lower;
    float limit_lin_z_upper;
    float limit_ang_x_lower;
    float limit_ang_x_upper;
    float limit_ang_y_lower;
    float limit_ang_y_upper;
    float limit_ang_z_lower;
    float limit_ang_z_upper;
    float spring_stiffness_x;
    float spring_stiffness_y;
    float spring_stiffness_z;
    float spring_stiffness_ang_x;
    float spring_stiffness_ang_y;
    float spring_stiffness_ang_z;
    float spring_damping_x;
    float spring_damping_y;
    float spring_damping_z;
    float spring_damping_ang_x;
    float spring_damping_ang_y;
    float spring_damping_ang_z;
    float motor_lin_target_velocity;
    float motor_ang_target_velocity;
    float motor_lin_max_impulse;
    float motor_ang_max_impulse;
    void *physics_constraint;
};

struct AviCodecData2_93_0 {
    void *lpFormat;
    void *lpParms;
    unsigned int cbFormat;
    unsigned int cbParms;
    unsigned int fccType;
    unsigned int fccHandler;
    unsigned int dwKeyFrameEvery;
    unsigned int dwQuality;
    unsigned int dwBytesPerSecond;
    unsigned int dwFlags;
    unsigned int dwInterleaveEvery;
    char _pad[4];
    char avicodecname[128];
};

struct FFMpegCodecData2_93_0 {
    int type;
    int codec;
    int audio_codec;
    int video_bitrate;
    int audio_bitrate;
    int audio_mixrate;
    int audio_channels;
    char _pad0[4];
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
    char _pad1[4];
    IDProperty2_93_0 *properties;
};

struct AudioData2_93_0 {
    int mixrate;
    float main;
    float speed_of_sound;
    float doppler_factor;
    int distance_model;
    short flag;
    char _pad[2];
    float volume;
    char _pad2[4];
};

struct SceneRenderView2_93_0 {
    SceneRenderView2_93_0 *next, *prev;
    char name[64];
    char suffix[64];
    int viewflag;
    char _pad2[4];
};

struct Stereo3dFormat2_93_0 {
    short flag;
    char display_mode;
    char anaglyph_type;
    char interlace_type;
    char _pad[3];
};

struct RenderProfile2_93_0 {
    RenderProfile2_93_0 *next, *prev;
    char name[32];
    short particle_perc;
    short subsurf_max;
    short shadbufsample_max;
    char _pad1[2];
    float ao_error;
    char _pad2[4];
};

struct TimeMarker2_93_0 {
    TimeMarker2_93_0 *next, *prev;
    int frame;
    char name[64];
    unsigned int flag;
    Object2_93_0 *camera;
    IDProperty2_93_0 *prop;
};

struct Paint_Runtime2_93_0 {
    unsigned int tool_offset;
    unsigned short ob_mode;
    char _pad[2];
};

struct PaintToolSlot2_93_0 {
    Brush2_93_0 *brush;
};

struct ParticleBrushData2_93_0 {
    short size;
    short step, invert, count;
    int flag;
    float strength;
};

struct GP_Sculpt_Guide2_93_0 {
    char use_guide;
    char use_snapping;
    char reference_point;
    char type;
    char _pad2[4];
    float angle;
    float angle_snap;
    float spacing;
    float location[3];
    Object2_93_0 *reference_object;
};

struct GP_Interpolate_Settings2_93_0 {
    CurveMapping2_93_0 *custom_ipo;
};

struct UnifiedPaintSettings2_93_0 {
    int size;
    float unprojected_radius;
    float alpha;
    float weight;
    float rgb[3];
    float secondary_rgb[3];
    int flag;
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
    char _pad[4];
    float size_pressure_value;
    float tex_mouse[2];
    float mask_tex_mouse[2];
    void *colorspace;
};

struct CurvePaintSettings2_93_0 {
    char curve_type;
    char flag;
    char depth_mode;
    char surface_plane;
    char fit_method;
    char _pad;
    short error_threshold;
    float radius_min, radius_max;
    float radius_taper_start, radius_taper_end;
    float surface_offset;
    float corner_angle;
};

struct MeshStatVis2_93_0 {
    char type;
    char _pad1[2];
    char overhang_axis;
    float overhang_min, overhang_max;
    float thickness_min, thickness_max;
    char thickness_samples;
    char _pad2[3];
    float distort_min, distort_max;
    float sharp_min, sharp_max;
};

struct SequencerToolSettings2_93_0 {
    int fit_method;
};

struct UnitSettings2_93_0 {
    float scale_length;
    char system;
    char system_rotation;
    short flag;
    char length_unit;
    char mass_unit;
    char time_unit;
    char temperature_unit;
    char _pad[4];
};

struct PhysicsSettings2_93_0 {
    float gravity[3];
    int flag, quick_cache_step, rt;
};

struct DisplaySafeAreas2_93_0 {
    float title[2];
    float action[2];
    float title_center[2];
    float action_center[2];
};

struct SceneEEVEE2_93_0 {
    int flag;
    int gi_diffuse_bounces;
    int gi_cubemap_resolution;
    int gi_visibility_resolution;
    float gi_irradiance_smoothing;
    float gi_glossy_clamp;
    float gi_filter_quality;
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
    float gtao_distance;
    float gtao_factor;
    float gtao_quality;
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
    LightCache2_93_0 *light_cache;
    LightCache2_93_0 *light_cache_data;
    char light_cache_info[64];
    float overscan;
    float light_threshold;
};

struct SceneGpencil2_93_0 {
    float smaa_threshold;
    char _pad[4];
};

struct TransformOrientationSlot2_93_0 {
    int type;
    int index_custom;
    char flag;
    char _pad0[7];
};

struct ScrEdge2_93_0 {
    ScrEdge2_93_0 *next, *prev;
    ScrVert2_93_0 *v1, *v2;
    short border;
    short flag;
    char _pad[4];
};

struct Panel_Runtime2_93_0 {
    int region_ofsx;
    char _pad[4];
    void *custom_data_ptr;
    void *block;
};

struct PanelCategoryStack2_93_0 {
    PanelCategoryStack2_93_0 *next, *prev;
    char idname[64];
};

struct uiListDyn2_93_0 {
    int height;
    int visual_height;
    int visual_height_min;
    int items_len;
    int items_shown;
    int resize;
    int resize_prev;
    int *items_filter_flags;
    int *items_filter_neworder;
};

struct uiList2_93_0 {
    uiList2_93_0 *next, *prev;
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
    IDProperty2_93_0 *properties;
    uiListDyn2_93_0 *dyn_data;
};

struct TransformOrientation2_93_0 {
    TransformOrientation2_93_0 *next, *prev;
    char name[64];
    float mat[3][3];
    char _pad[4];
};

struct uiPreview2_93_0 {
    uiPreview2_93_0 *next, *prev;
    char preview_id[64];
    short height;
    char _pad1[6];
};

struct ScrGlobalAreaData2_93_0 {
    short cur_fixed_height;
    short size_min, size_max;
    short align;
    short flag;
    char _pad[2];
};

struct ScrArea_Runtime2_93_0 {
    bToolRef2_93_0 *tool;
    char is_tool_set;
    char _pad0[7];
};

struct SDNA_StructMember2_93_0 {
    short type;
    short name;
};

struct BHead2_93_0 {
    int code, len;
    const void *old;
    int SDNAnr, nr;
};

struct BHead82_93_0 {
    int code, len;
    uint64_t old;
    int SDNAnr, nr;
};

struct StripAnim2_93_0 {
    StripAnim2_93_0 *next, *prev;
    void *anim;
};

struct StripElem2_93_0 {
    char name[256];
    int orig_width, orig_height;
};

struct StripCrop2_93_0 {
    int top;
    int bottom;
    int left;
    int right;
};

struct StripTransform2_93_0 {
    int xofs;
    int yofs;
    float scale_x;
    float scale_y;
    float rotation;
};

struct StripColorBalance2_93_0 {
    float lift[3];
    float gamma[3];
    float gain[3];
    int flag;
    char _pad[4];
};

struct StripProxy2_93_0 {
    char dir[768];
    char file[256];
    void *anim;
    short tc;
    short quality;
    short build_size_flags;
    short build_tc_flags;
    short build_flags;
    char storage;
    char _pad[5];
};

struct MetaStack2_93_0 {
    MetaStack2_93_0 *next, *prev;
    ListBase2_93_0 *oldbasep;
    Sequence2_93_0 *parseq;
    int disp_range[2];
};

struct WipeVars2_93_0 {
    float edgeWidth, angle;
    short forward, wipetype;
};

struct GlowVars2_93_0 {
    float fMini;
    float fClamp;
    float fBoost;
    float dDist;
    int dQuality;
    int bNoComp;
};

struct TransformVars2_93_0 {
    float ScalexIni;
    float ScaleyIni;
    float xIni;
    float yIni;
    float rotIni;
    int percent;
    int interpolation;
    int uniform_scale;
};

struct SolidColorVars2_93_0 {
    float col[3];
    char _pad[4];
};

struct SpeedControlVars2_93_0 {
    float *frameMap;
    float globalSpeed;
    int flags;
    int length;
    int lastValidFrame;
};

struct GaussianBlurVars2_93_0 {
    float size_x;
    float size_y;
};

struct TextVars2_93_0 {
    char text[512];
    VFont2_93_0 *text_font;
    int text_blf_id;
    int text_size;
    float color[4], shadow_color[4], box_color[4];
    float loc[2];
    float wrap_width;
    float box_margin;
    char flag;
    char align, align_y;
    char _pad[5];
};

struct ColorMixVars2_93_0 {
    int blend_effect;
    float factor;
};

struct SequenceModifierData2_93_0 {
    SequenceModifierData2_93_0 *next, *prev;
    int type, flag;
    char name[64];
    int mask_input_type;
    int mask_time;
    Sequence2_93_0 *mask_sequence;
    Mask2_93_0 *mask_id;
};

struct SequencerScopes2_93_0 {
    void *reference_ibuf;
    void *zebra_ibuf;
    void *waveform_ibuf;
    void *sep_waveform_ibuf;
    void *vector_ibuf;
    void *histogram_ibuf;
};

struct SessionUUID2_93_0 {
    uint64_t uuid_;
};

struct ShaderFxData2_93_0 {
    ShaderFxData2_93_0 *next, *prev;
    int type, mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
};

struct ShaderFxData_Runtime2_93_0 {
    float loc[3];
    char _pad[4];
    void *fx_sh;
    void *fx_sh_b;
    void *fx_sh_c;
};

struct MaskSpaceInfo2_93_0 {
    Mask2_93_0 *mask;
    char draw_flag;
    char draw_type;
    char overlay_mode;
    char _pad3[5];
};

struct FileSelectAssetLibraryUID2_93_0 {
    short type;
    char _pad[2];
    int custom_library_index;
};

struct FileSelectParams2_93_0 {
    char title[96];
    char dir[1090];
    char file[256];
    char renamefile[256];
    short rename_flag;
    char filter_glob[256];
    char filter_search[64];
    int _pad0;
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
    short f_fp;
    char fp_str[8];
};

struct AssetUUID2_93_0 {
    int uuid_asset[4];
    int uuid_variant[4];
    int uuid_revision[4];
};

struct AssetUUIDList2_93_0 {
    AssetUUID2_93_0 *uuids;
    int nbr_uuids;
    char _pad[4];
};

struct FileDirEntryRevision2_93_0 {
    FileDirEntryRevision2_93_0 *next, *prev;
    char *comment;
    void *_pad;
    int uuid[4];
    uint64_t size;
    int64_t time;
    char size_str[16];
    char datetime_str[16 + 8];
};

struct SpaceImageOverlay2_93_0 {
    int flag;
    char _pad[4];
};

struct ConsoleLine2_93_0 {
    ConsoleLine2_93_0 *next, *prev;
    int len_alloc;
    int len;
    char *line;
    int cursor;
    int type;
};

struct SpreadsheetColumnID2_93_0 {
    char *name;
};

struct SpreadsheetColumn2_93_0 {
    SpreadsheetColumn2_93_0 *next, *prev;
    SpreadsheetColumnID2_93_0 *id;
};

struct SpreadsheetContext2_93_0 {
    SpreadsheetContext2_93_0 *next, *prev;
    int type;
    char _pad[4];
};

struct MTex2_93_0 {
    short texco, mapto, maptoneg, blendtype;
    Object2_93_0 *object;
    Tex2_93_0 *tex;
    char uvname[64];
    char projx, projy, projz, mapping;
    char brush_map_mode, brush_angle_mode;
    char _pad[2];
    float ofs[3], size[3], rot, random_angle;
    char _pad0[2];
    short colormodel, pmapto, pmaptoneg;
    short normapspace, which_output;
    float r, g, b, k;
    float def_var, rt;
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

struct CBData2_93_0 {
    float r, g, b, a, pos;
    int cur;
};

struct PointDensity2_93_0 {
    short flag;
    short falloff_type;
    float falloff_softness;
    float radius;
    short source;
    char _pad0[2];
    short color_source;
    short ob_color_source;
    int totpoints;
    Object2_93_0 *object;
    int psys;
    short psys_cache_space;
    short ob_cache_space;
    char vertex_attribute_name[64];
    void *point_tree;
    float *point_data;
    float noise_size;
    short noise_depth;
    short noise_influence;
    short noise_basis;
    char _pad1[6];
    float noise_fac;
    float speed_scale, falloff_speed_scale;
    char _pad2[4];
    ColorBand2_93_0 *coba;
    CurveMapping2_93_0 *falloff_curve;
};

struct TexMapping2_93_0 {
    float loc[3], rot[3], size[3];
    int flag;
    char projx, projy, projz, mapping;
    int type;
    float mat[4][4];
    float min[3], max[3];
    Object2_93_0 *ob;
};

struct TextLine2_93_0 {
    TextLine2_93_0 *next, *prev;
    char *line;
    char *format;
    int len;
    char _pad0[4];
};

struct MovieReconstructedCamera2_93_0 {
    int framenr;
    float error;
    float mat[4][4];
};

struct MovieTrackingCamera2_93_0 {
    void *intrinsics;
    short distortion_model;
    char _pad[2];
    float sensor_width;
    float pixel_aspect;
    float focal;
    short units;
    char _pad1[2];
    float principal[2];
    float k1, k2, k3;
    float division_k1, division_k2;
    float nuke_k1, nuke_k2;
    float brown_k1, brown_k2, brown_k3, brown_k4;
    float brown_p1, brown_p2;
};

struct MovieTrackingMarker2_93_0 {
    float pos[2];
    float pattern_corners[4][2];
    float search_min[2], search_max[2];
    int framenr;
    int flag;
};

struct MovieTrackingTrack2_93_0 {
    MovieTrackingTrack2_93_0 *next, *prev;
    char name[64];
    float pat_min[2] , pat_max[2];
    float search_min[2] , search_max[2];
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
    bGPdata2_93_0 *gpd;
    float weight;
    float weight_stab;
};

struct MovieTrackingPlaneMarker2_93_0 {
    float corners[4][2];
    int framenr;
    int flag;
};

struct MovieTrackingPlaneTrack2_93_0 {
    MovieTrackingPlaneTrack2_93_0 *next, *prev;
    char name[64];
    MovieTrackingTrack2_93_0 **point_tracks;
    int point_tracksnr;
    char _pad[4];
    MovieTrackingPlaneMarker2_93_0 *markers;
    int markersnr;
    int flag;
    Image2_93_0 *image;
    float image_opacity;
    int last_marker;
};

struct MovieTrackingSettings2_93_0 {
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
    int keyframe1;
    int keyframe2;
    int reconstruction_flag;
    int refine_camera_intrinsics;
    float dist;
    int clean_frames, clean_action;
    float clean_error;
    float object_distance;
};

struct MovieTrackingStabilization2_93_0 {
    int flag;
    int tot_track, act_track;
    int tot_rot_track, act_rot_track;
    float maxscale;
    MovieTrackingTrack2_93_0 *rot_track;
    int anchor_frame;
    float target_pos[2];
    float target_rot;
    float scale;
    float locinf, scaleinf, rotinf;
    int filter;
    int ok;
};

struct MovieTrackingReconstruction2_93_0 {
    int flag;
    float error;
    int last_camera;
    int camnr;
    MovieReconstructedCamera2_93_0 *cameras;
};

struct MovieTrackingStats2_93_0 {
    char message[256];
};

struct MovieTrackingDopesheetChannel2_93_0 {
    MovieTrackingDopesheetChannel2_93_0 *next, *prev;
    MovieTrackingTrack2_93_0 *track;
    char _pad[4];
    char name[64];
    int tot_segment;
    int *segments;
    int max_segment, total_frames;
};

struct MovieTrackingDopesheetCoverageSegment2_93_0 {
    MovieTrackingDopesheetCoverageSegment2_93_0 *next, *prev;
    int coverage;
    int start_frame;
    int end_frame;
    char _pad[4];
};

struct uiFont2_93_0 {
    uiFont2_93_0 *next, *prev;
    char filename[1024];
    short blf_id;
    short uifont_id;
    short r_to_l;
    char _pad0[2];
};

struct uiFontStyle2_93_0 {
    short uifont_id;
    short points;
    short kerning;
    short italic, bold;
    short shadow;
    short shadx, shady;
    float shadowalpha;
    float shadowcolor;
};

struct uiWidgetColors2_93_0 {
    unsigned char outline[4];
    unsigned char inner[4];
    unsigned char inner_sel[4];
    unsigned char item[4];
    unsigned char text[4];
    unsigned char text_sel[4];
    unsigned char shaded;
    char _pad0[7];
    short shadetop, shadedown;
    float roundness;
};

struct uiWidgetStateColors2_93_0 {
    unsigned char inner_anim[4];
    unsigned char inner_anim_sel[4];
    unsigned char inner_key[4];
    unsigned char inner_key_sel[4];
    unsigned char inner_driven[4];
    unsigned char inner_driven_sel[4];
    unsigned char inner_overridden[4];
    unsigned char inner_overridden_sel[4];
    unsigned char inner_changed[4];
    unsigned char inner_changed_sel[4];
    float blend;
    char _pad0[4];
};

struct uiPanelColors2_93_0 {
    unsigned char header[4];
    unsigned char back[4];
    unsigned char sub_back[4];
    char _pad0[4];
};

struct ThemeWireColor2_93_0 {
    unsigned char solid[4];
    unsigned char select[4];
    unsigned char active[4];
    short flag;
    char _pad0[2];
};

struct ThemeCollectionColor2_93_0 {
    unsigned char color[4];
};

struct bAddon2_93_0 {
    bAddon2_93_0 *next, *prev;
    char module[64];
    IDProperty2_93_0 *prop;
};

struct bPathCompare2_93_0 {
    bPathCompare2_93_0 *next, *prev;
    char path[768];
    char flag;
    char _pad0[7];
};

struct bUserMenuItem2_93_0 {
    bUserMenuItem2_93_0 *next, *prev;
    char ui_name[64];
    char type;
    char _pad0[7];
};

struct bUserAssetLibrary2_93_0 {
    bUserAssetLibrary2_93_0 *next, *prev;
    char name[64];
    char path[1024];
};

struct SolidLight2_93_0 {
    int flag;
    float smooth;
    char _pad0[8];
    float col[4], spec[4], vec[4];
};

struct WalkNavigation2_93_0 {
    float mouse_speed;
    float walk_speed;
    float walk_speed_factor;
    float view_height;
    float jump_height;
    float teleport_time;
    short flag;
    char _pad0[6];
};

struct UserDef_Runtime2_93_0 {
    char is_dirty;
    char _pad0[7];
};

struct UserDef_SpaceData2_93_0 {
    char section_active;
    char flag;
    char _pad0[6];
};

struct UserDef_FileSpaceData2_93_0 {
    int display_type;
    int thumbnail_size;
    int sort_type;
    int details_flags;
    int flag;
    int _pad0;
    uint64_t filter_id;
    int temp_win_sizex;
    int temp_win_sizey;
};

struct UserDef_Experimental2_93_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
    char SANITIZE_AFTER_HERE;
    char use_new_hair_type;
    char use_new_point_cloud_type;
    char use_sculpt_vertex_colors;
    char use_sculpt_tools_tilt;
    char use_asset_browser;
    char use_override_templates;
    char _pad[6];
};

struct vec2s2_93_0 {
    short x, y;
};

struct vec2f2_93_0 {
    float x, y;
};

struct vec3f2_93_0 {
    float x, y, z;
};

struct rcti2_93_0 {
    int xmin, xmax;
    int ymin, ymax;
};

struct rctf2_93_0 {
    float xmin, xmax;
    float ymin, ymax;
};

struct DualQuat2_93_0 {
    float quat[4];
    float trans[4];
    float scale[4][4];
    float scale_weight;
};

struct RegionView3D2_93_0 {
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
    RegionView3D2_93_0 *localvd;
    void *render_engine;
    void *depths;
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

struct View3DCursor2_93_0 {
    float location[3];
    float rotation_quaternion[4];
    float rotation_euler[3];
    float rotation_axis[3], rotation_angle;
    short rotation_mode;
    char _pad[6];
};

struct View3DShading2_93_0 {
    char type;
    char prev_type;
    char prev_type_wire;
    char color_type;
    short flag;
    char light;
    char background_type;
    char cavity_type;
    char wire_color_type;
    char _pad[2];
    char studio_light[256];
    char lookdev_light[256];
    char matcap[256];
    float shadow_intensity;
    float single_color[3];
    float studiolight_rot_z;
    float studiolight_background;
    float studiolight_intensity;
    float studiolight_blur;
    float object_outline_color[3];
    float xray_alpha;
    float xray_alpha_wire;
    float cavity_valley_factor;
    float cavity_ridge_factor;
    float background_color[3];
    float curvature_ridge_factor;
    float curvature_valley_factor;
    int render_pass;
    char aov_name[64];
    IDProperty2_93_0 *prop;
    void *_pad2;
};

struct View3DOverlay2_93_0 {
    int flag;
    int edit_flag;
    float normals_length;
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
};

struct View3D_Runtime2_93_0 {
    void *properties_storage;
    int flag;
    char _pad1[4];
};

struct Volume_Runtime2_93_0 {
    void *grids;
    int frame;
    int default_simplify_level;
};

struct VolumeDisplay2_93_0 {
    float density;
    int wireframe_type;
    int wireframe_detail;
    int interpolation_method;
    int axis_slice_method;
    int slice_axis;
    float slice_depth;
    int _pad[1];
};

struct VolumeRender2_93_0 {
    int precision;
    int space;
    float step_size;
    float clipping;
};

struct Report2_93_0 {
    Report2_93_0 *next, *prev;
    short type;
    short flag;
    int len;
    const char *typestr;
    const char *message;
};

struct ReportTimerInfo2_93_0 {
    float col[4];
    float widthfac;
};

struct wmOperatorTypeMacro2_93_0 {
    wmOperatorTypeMacro2_93_0 *next, *prev;
    char idname[64];
    IDProperty2_93_0 *properties;
    void *ptr;
};

struct wmKeyMapItem2_93_0 {
    wmKeyMapItem2_93_0 *next, *prev;
    char idname[64];
    IDProperty2_93_0 *properties;
    char propvalue_str[64];
    short propvalue;
    short type;
    short val;
    short shift, ctrl, alt, oskey;
    short keymodifier;
    short flag;
    short maptype;
    short id;
    char _pad[2];
    void *ptr;
};

struct wmKeyMapDiffItem2_93_0 {
    wmKeyMapDiffItem2_93_0 *next, *prev;
    wmKeyMapItem2_93_0 *remove_item;
    wmKeyMapItem2_93_0 *add_item;
};

struct wmKeyConfigPref2_93_0 {
    wmKeyConfigPref2_93_0 *next, *prev;
    char idname[64];
    IDProperty2_93_0 *prop;
};

struct bToolRef_Runtime2_93_0 {
    int cursor;
    char keymap[64];
    char gizmo_group[64];
    char data_block[64];
    char keymap_fallback[64];
    char op[64];
    int index;
};

struct bToolRef2_93_0 {
    bToolRef2_93_0 *next, *prev;
    char idname[64];
    char idname_fallback[64];
    short tag;
    short space_type;
    int mode;
    IDProperty2_93_0 *properties;
    bToolRef_Runtime2_93_0 *runtime;
};

struct WorkSpaceLayout2_93_0 {
    WorkSpaceLayout2_93_0 *next, *prev;
    bScreen2_93_0 *screen;
    char name[64];
};

struct wmOwnerID2_93_0 {
    wmOwnerID2_93_0 *next, *prev;
    char name[64];
};

struct WorkSpaceDataRelation2_93_0 {
    WorkSpaceDataRelation2_93_0 *next, *prev;
    void *parent;
    void *value;
    int parentid;
    char _pad_0[4];
};

struct WorkSpaceInstanceHook2_93_0 {
    WorkSpace2_93_0 *active;
    WorkSpaceLayout2_93_0 *act_layout;
    WorkSpace2_93_0 *temp_workspace_store;
    WorkSpaceLayout2_93_0 *temp_layout_store;
};

struct bPoseChannel_Runtime2_93_0 {
    SessionUUID2_93_0 session_uuid;
    DualQuat2_93_0 deform_dual_quat;
    int bbone_segments;
    void *bbone_rest_mats;
    void *bbone_pose_mats;
    void *bbone_deform_mats;
    DualQuat2_93_0 *bbone_dual_quats;
};

struct bPose2_93_0 {
    ListBase2_93_0 chanbase;
    void *chanhash;
    bPoseChannel2_93_0 **chan_array;
    short flag;
    char _pad[2];
    unsigned int proxy_layer;
    char _pad1[4];
    float ctime;
    float stride_offset[3];
    float cyclic_offset[3];
    ListBase2_93_0 agroups;
    int active_group;
    int iksolver;
    void *ikdata;
    void *ikparam;
    bAnimVizSettings2_93_0 avs;
    char proxy_act_bone[64];
};

struct bActionGroup2_93_0 {
    bActionGroup2_93_0 *next, *prev;
    ListBase2_93_0 channels;
    int flag;
    int customCol;
    char name[64];
    ThemeWireColor2_93_0 cs;
};

struct bAction2_93_0 {
    ID2_93_0 id;
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

struct bDopeSheet2_93_0 {
    ID2_93_0 *source;
    ListBase2_93_0 chanbase;
    Collection2_93_0 *filter_grp;
    char searchstr[64];
    int filterflag;
    int filterflag2;
    int flag;
    int renameIndex;
};

struct bActionChannel2_93_0 {
    bActionChannel2_93_0 *next, *prev;
    bActionGroup2_93_0 *grp;
    Ipo2_93_0 *ipo;
    ListBase2_93_0 constraintChannels;
    int flag;
    char name[64];
    int temp;
};

struct FMod_Limits2_93_0 {
    rctf2_93_0 rect;
    int flag;
    char _pad[4];
};

struct DriverVar2_93_0 {
    DriverVar2_93_0 *next, *prev;
    char name[64];
    DriverTarget2_93_0 targets[8];
    char num_targets;
    char type;
    short flag;
    float curval;
};

struct ChannelDriver2_93_0 {
    ListBase2_93_0 variables;
    char expression[256];
    void *expr_comp;
    void *expr_simple;
    float curval;
    float influence;
    int type;
    int flag;
};

struct FCurve2_93_0 {
    FCurve2_93_0 *next, *prev;
    bActionGroup2_93_0 *grp;
    ChannelDriver2_93_0 *driver;
    ListBase2_93_0 modifiers;
    BezTriple2_93_0 *bezt;
    FPoint2_93_0 *fpt;
    unsigned int totvert;
    int active_keyframe_index;
    float curval;
    short flag;
    short extend;
    char auto_smoothing;
    char _pad[3];
    int array_index;
    char *rna_path;
    int color_mode;
    float color[3];
    float prev_norm_factor, prev_offset;
};

struct NlaStrip2_93_0 {
    NlaStrip2_93_0 *next, *prev;
    ListBase2_93_0 strips;
    bAction2_93_0 *act;
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
    NlaStrip2_93_0 *orig_strip;
    void *_pad3;
};

struct NlaTrack2_93_0 {
    NlaTrack2_93_0 *next, *prev;
    ListBase2_93_0 strips;
    int flag;
    int index;
    char name[64];
};

struct KeyingSet2_93_0 {
    KeyingSet2_93_0 *next, *prev;
    ListBase2_93_0 paths;
    char idname[64];
    char name[64];
    char description[240];
    char typeinfo[64];
    int active_path;
    short flag;
    short keyingflag;
    short keyingoverride;
    char _pad[6];
};

struct AnimData2_93_0 {
    bAction2_93_0 *action;
    bAction2_93_0 *tmpact;
    ListBase2_93_0 nla_tracks;
    NlaTrack2_93_0 *act_track;
    NlaStrip2_93_0 *actstrip;
    ListBase2_93_0 drivers;
    ListBase2_93_0 overrides;
    FCurve2_93_0 **driver_array;
    int flag;
    char _pad[4];
    short act_blendmode;
    short act_extendmode;
    float act_influence;
};

struct IdAdtTemplate2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
};

struct Bone2_93_0 {
    Bone2_93_0 *next, *prev;
    IDProperty2_93_0 *prop;
    Bone2_93_0 *parent;
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
    float curve_in_x, curve_in_y;
    float curve_out_x, curve_out_y;
    float ease1, ease2;
    float scale_in_x, scale_in_y;
    float scale_out_x, scale_out_y;
    float size[3];
    int layer;
    short segments;
    char bbone_prev_type;
    char bbone_next_type;
    Bone2_93_0 *bbone_prev;
    Bone2_93_0 *bbone_next;
};

struct bArmature2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 bonebase;
    void *bonehash;
    void *_pad1;
    ListBase2_93_0 *edbo;
    Bone2_93_0 *act_bone;
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

struct AssetMetaData2_93_0 {
    IDProperty2_93_0 *properties;
    char *description;
    ListBase2_93_0 tags;
    short active_tag;
    short tot_tags;
    char _pad[4];
};

struct BoidRuleGoalAvoid2_93_0 {
    BoidRule2_93_0 rule;
    Object2_93_0 *ob;
    int options;
    float fear_factor;
    int signal_id, channels;
};

struct BoidRuleAvoidCollision2_93_0 {
    BoidRule2_93_0 rule;
    int options;
    float look_ahead;
};

struct BoidRuleFollowLeader2_93_0 {
    BoidRule2_93_0 rule;
    Object2_93_0 *ob;
    float loc[3], oloc[3];
    float cfra, distance;
    int options, queue_size;
};

struct BoidRuleAverageSpeed2_93_0 {
    BoidRule2_93_0 rule;
    float wander, level, speed, rt;
};

struct BoidRuleFight2_93_0 {
    BoidRule2_93_0 rule;
    float distance, flee_distance;
};

struct BoidState2_93_0 {
    BoidState2_93_0 *next, *prev;
    ListBase2_93_0 rules;
    ListBase2_93_0 conditions;
    ListBase2_93_0 actions;
    char name[32];
    int id, flag;
    int ruleset_type;
    float rule_fuzziness;
    int signal_id, channels;
    float volume, falloff;
};

struct BoidSettings2_93_0 {
    int options, last_state_id;
    float landing_smoothness, height;
    float banking, pitch;
    float health, aggression;
    float strength, accuracy, range;
    float air_min_speed, air_max_speed;
    float air_max_acc, air_max_ave;
    float air_personal_space;
    float land_jump_speed, land_max_speed;
    float land_max_acc, land_max_ave;
    float land_personal_space;
    float land_stick_force;
    ListBase2_93_0 states;
};

struct Brush2_93_0 {
    ID2_93_0 id;
    BrushClone2_93_0 clone;
    CurveMapping2_93_0 *curve;
    MTex2_93_0 mtex;
    MTex2_93_0 mask_mtex;
    Brush2_93_0 *toggle_brush;
    void *icon_imbuf;
    PreviewImage2_93_0 *preview;
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
    BrushGpencilSettings2_93_0 *gpencil_settings;
};

struct Palette2_93_0 {
    ID2_93_0 id;
    ListBase2_93_0 colors;
    int active_color;
    char _pad[4];
};

struct PaintCurvePoint2_93_0 {
    BezTriple2_93_0 bez;
    float pressure;
};

struct PaintCurve2_93_0 {
    ID2_93_0 id;
    PaintCurvePoint2_93_0 *points;
    int tot_points;
    int add_index;
};

struct CacheFile2_93_0 {
    ID2_93_0 id;
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
    short flag;
    short draw_flag;
    char _pad[3];
    char velocity_unit;
    char velocity_name[64];
    void *handle;
    char handle_filepath[1024];
    void *handle_readers;
};

struct CameraBGImage2_93_0 {
    CameraBGImage2_93_0 *next, *prev;
    Image2_93_0 *ima;
    ImageUser2_93_0 iuser;
    MovieClip2_93_0 *clip;
    MovieClipUser2_93_0 cuser;
    float offset[2], scale, rotation;
    float alpha;
    short flag;
    short source;
};

struct Camera2_93_0 {
    ID2_93_0 id;
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
    Object2_93_0 *dof_ob;
    GPUDOFSettings2_93_0 gpu_dof;
    CameraDOFSettings2_93_0 dof;
    ListBase2_93_0 bg_images;
    char sensor_fit;
    char _pad[7];
    CameraStereoSettings2_93_0 stereo;
    Camera_Runtime2_93_0 runtime;
};

struct Collection2_93_0 {
    ID2_93_0 id;
    ListBase2_93_0 gobject;
    ListBase2_93_0 children;
    PreviewImage2_93_0 *preview;
    unsigned int layer;
    float instance_offset[3];
    short flag;
    short tag;
    short lineart_usage;
    int16_t color_tag;
    ListBase2_93_0 object_cache;
    ListBase2_93_0 object_cache_instanced;
    ListBase2_93_0 parents;
    SceneCollection2_93_0 *collection;
    ViewLayer2_93_0 *view_layer;
};

struct CurveMapping2_93_0 {
    int flag, cur;
    int preset;
    int changed_timestamp;
    rctf2_93_0 curr, clipr;
    CurveMap2_93_0 cm[4];
    float black[3], white[3];
    float bwmul[3];
    float sample[3];
    short tone;
    char _pad[6];
};

struct Scopes2_93_0 {
    int ok;
    int sample_full;
    int sample_lines;
    float accuracy;
    int wavefrm_mode;
    float wavefrm_alpha;
    float wavefrm_yfac;
    int wavefrm_height;
    float vecscope_alpha;
    int vecscope_height;
    float minmax[3][2];
    Histogram2_93_0 hist;
    float *waveform_1;
    float *waveform_2;
    float *waveform_3;
    float *vecscope;
    int waveform_tot;
    char _pad[4];
};

struct bPythonConstraint2_93_0 {
    Text2_93_0 *text;
    IDProperty2_93_0 *prop;
    int flag;
    int tarnum;
    ListBase2_93_0 targets;
    Object2_93_0 *tar;
    char subtarget[64];
};

struct bArmatureConstraint2_93_0 {
    int flag;
    char _pad[4];
    ListBase2_93_0 targets;
};

struct CurveProfile2_93_0 {
    short path_len;
    short segments_len;
    int preset;
    CurveProfilePoint2_93_0 *path;
    CurveProfilePoint2_93_0 *table;
    CurveProfilePoint2_93_0 *segments;
    int flag;
    int changed_timestamp;
    rctf2_93_0 view_rect, clip_rect;
};

struct EditNurb2_93_0 {
    ListBase2_93_0 nurbs;
    void *keyindex;
    int shapenr;
    char needs_flush_to_id;
};

struct Curve2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 nurb;
    EditNurb2_93_0 *editnurb;
    Object2_93_0 *bevobj, *taperobj, *textoncurve;
    Ipo2_93_0 *ipo;
    Key2_93_0 *key;
    Material2_93_0 **mat;
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
    void *batch_cache;
};

struct DynamicPaintSurface2_93_0 {
    DynamicPaintSurface2_93_0 *next, *prev;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    void *data;
    Collection2_93_0 *brush_group;
    EffectorWeights2_93_0 *effector_weights;
    void *pointcache;
    ListBase2_93_0 ptcaches;
    int current_frame;
    char name[64];
    short format, type;
    short disp_type, image_fileformat;
    short effect_ui;
    short init_color_type;
    int flags, effect;
    int image_resolution, substeps;
    int start_frame, end_frame;
    float init_color[4];
    Tex2_93_0 *init_texture;
    char init_layername[64];
    int dry_speed, diss_speed;
    float color_dry_threshold;
    float depth_clamp, disp_factor;
    float spread_speed, color_spread_speed, shrink_speed;
    float drip_vel, drip_acc;
    float influence_scale, radius_scale;
    float wave_damping, wave_speed, wave_timescale, wave_spring, wave_smoothness;
    char _pad2[4];
    char uvlayer_name[64];
    char image_output_path[1024];
    char output_name[64];
    char output_name2[64];
};

struct DynamicPaintCanvasSettings2_93_0 {
    DynamicPaintModifierData2_93_0 *pmd;
    ListBase2_93_0 surfaces;
    short active_sur, flags;
    char _pad[4];
    char error[64];
};

struct FluidDomainSettings2_93_0 {
    FluidModifierData2_93_0 *fmd;
    void *fluid;
    void *fluid_old;
    void *fluid_mutex;
    Collection2_93_0 *fluid_group;
    Collection2_93_0 *force_group;
    Collection2_93_0 *effector_group;
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
    Object2_93_0 *guide_parent;
    FluidDomainVertexVelocity2_93_0 *mesh_velocities;
    EffectorWeights2_93_0 *effector_weights;
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
    short noise_type;
    char _pad3[2];
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
    int totvert;
    short mesh_generator;
    char _pad6[6];
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

struct FreestyleConfig2_93_0 {
    ListBase2_93_0 modules;
    int mode;
    int raycasting_algorithm;
    int flags;
    float sphere_radius;
    float dkr_epsilon;
    float crease_angle;
    ListBase2_93_0 linesets;
};

struct NoiseGpencilModifierData2_93_0 {
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
    char _pad[4];
    int step;
    int layer_pass;
    int seed;
    CurveMapping2_93_0 *curve_intensity;
};

struct SubdivGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    int level;
    int layer_pass;
    short type;
    char _pad[6];
};

struct ThickGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float thickness_fac;
    int thickness;
    int layer_pass;
    char _pad[4];
    CurveMapping2_93_0 *curve_thickness;
};

struct TimeGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    char layername[64];
    int layer_pass;
    int flag;
    int offset;
    float frame_scale;
    int mode;
    int sfra, efra;
    char _pad[4];
};

struct ColorGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    float hsv[3];
    char modify_color;
    char _pad[3];
    int layer_pass;
    char _pad1[4];
    CurveMapping2_93_0 *curve_intensity;
};

struct OpacityGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float factor;
    char modify_color;
    char _pad[3];
    int layer_pass;
    float hardeness;
    CurveMapping2_93_0 *curve_intensity;
};

struct ArrayGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Object2_93_0 *object;
    Material2_93_0 *material;
    int count;
    int flag;
    float offset[3];
    float shift[3];
    float rnd_offset[3];
    float rnd_rot[3];
    float rnd_scale[3];
    char _pad[4];
    int seed;
    int pass_index;
    char layername[64];
    char materialname[64];
    int mat_rpl;
    int layer_pass;
};

struct BuildGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    int pass_index;
    char materialname[64];
    int layer_pass;
    float start_frame;
    float end_frame;
    float start_delay;
    float length;
    short flag;
    short mode;
    short transition;
    short time_alignment;
    float percentage_fac;
    char _pad[4];
};

struct LatticeGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Object2_93_0 *object;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float strength;
    int layer_pass;
    void *cache_data;
};

struct MirrorGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Object2_93_0 *object;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    int layer_pass;
    char _pad[4];
};

struct HookGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Object2_93_0 *object;
    Material2_93_0 *material;
    char subtarget[64];
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int layer_pass;
    char _pad[4];
    int flag;
    char falloff_type;
    char _pad1[3];
    float parentinv[4][4];
    float cent[3];
    float falloff;
    float force;
    CurveMapping2_93_0 *curfalloff;
};

struct SimplifyGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    float factor;
    short mode;
    short step;
    int layer_pass;
    float length;
    float distance;
    char _pad[4];
};

struct OffsetGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float loc[3];
    float rot[3];
    float scale[3];
    int layer_pass;
};

struct SmoothGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float factor;
    int step;
    int layer_pass;
    char _pad1[4];
    CurveMapping2_93_0 *curve_intensity;
};

struct MultiplyGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    int layer_pass;
    int flags;
    int duplications;
    float distance;
    float offset;
    float fading_center;
    float fading_thickness;
    float fading_opacity;
};

struct TintGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Object2_93_0 *object;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int layer_pass;
    int flag;
    int mode;
    float factor;
    float radius;
    float rgb[3];
    int type;
    CurveMapping2_93_0 *curve_intensity;
    ColorBand2_93_0 *colorband;
};

struct TextureGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    Material2_93_0 *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
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

struct LineartGpencilModifierData2_93_0 {
    GpencilModifierData2_93_0 modifier;
    short edge_types;
    char source_type;
    char use_multiple_levels;
    short level_start;
    short level_end;
    Object2_93_0 *source_object;
    Collection2_93_0 *source_collection;
    Material2_93_0 *target_material;
    char target_layer[64];
    char source_vertex_group[64];
    char vgname[64];
    float opacity;
    short thickness;
    unsigned char transparency_flags;
    unsigned char transparency_mask;
    float crease_threshold;
    float angle_splitting_threshold;
    float chaining_image_threshold;
    int _pad;
    int calculation_flags;
    int flags;
    void *render_buffer;
};

struct bGPDspoint2_93_0 {
    float x, y, z;
    float pressure;
    float strength;
    float time;
    int flag;
    float uv_fac;
    float uv_rot;
    float uv_fill[2];
    float vert_color[4];
    char _pad2[4];
    bGPDspoint_Runtime2_93_0 runtime;
};

struct bGPDpalette2_93_0 {
    bGPDpalette2_93_0 *next, *prev;
    ListBase2_93_0 colors;
    char info[64];
    short flag;
    char _pad[6];
};

struct bGPDcurve_point2_93_0 {
    BezTriple2_93_0 bezt;
    float pressure;
    float strength;
    int point_index;
    int flag;
    float uv_fac;
    float uv_rot;
    float uv_fill[2];
    float vert_color[4];
    char _pad[4];
};

struct bGPDstroke2_93_0 {
    bGPDstroke2_93_0 *next, *prev;
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
};

struct bGPDframe2_93_0 {
    bGPDframe2_93_0 *next, *prev;
    ListBase2_93_0 strokes;
    int framenum;
    short flag;
    short key_type;
    bGPDframe_Runtime2_93_0 runtime;
};

struct bGPDlayer2_93_0 {
    bGPDlayer2_93_0 *next, *prev;
    ListBase2_93_0 frames;
    bGPDframe2_93_0 *actframe;
    short flag;
    short onion_flag;
    float color[4];
    float fill[4];
    char info[128];
    short thickness;
    short pass_index;
    Object2_93_0 *parent;
    float inverse[4][4];
    char parsubstr[64];
    short partype;
    short line_change;
    float tintcolor[4];
    float opacity;
    char viewlayername[64];
    int blend_mode;
    float vertex_paint_opacity;
    short gstep;
    short gstep_next;
    float gcolor_prev[3];
    float gcolor_next[3];
    char _pad1[4];
    ListBase2_93_0 mask_layers;
    int act_mask;
    char _pad2[4];
    float location[3], rotation[3], scale[3];
    float layer_mat[4][4], layer_invmat[4][4];
    char _pad3[4];
    bGPDlayer_Runtime2_93_0 runtime;
};

struct bGPdata2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 layers;
    int flag;
    int curve_edit_resolution;
    float curve_edit_threshold;
    float curve_edit_corner_angle;
    ListBase2_93_0 palettes;
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
    Material2_93_0 **mat;
    short totcol;
    short totlayer;
    short totframe;
    char _pad2[6];
    int totstroke;
    int totpoint;
    short draw_mode;
    short onion_keytype;
    int select_last_index;
    char _pad3[4];
    bGPgrid2_93_0 grid;
    bGPdata_Runtime2_93_0 runtime;
};

struct IDPropertyData2_93_0 {
    void *pointer;
    ListBase2_93_0 group;
    int val, val2;
};

struct IDOverrideLibraryProperty2_93_0 {
    IDOverrideLibraryProperty2_93_0 *next, *prev;
    char *rna_path;
    ListBase2_93_0 operations;
    short tag;
    char _pad[2];
    unsigned int rna_prop_type;
};

struct IDOverrideLibrary2_93_0 {
    ID2_93_0 *reference;
    ListBase2_93_0 properties;
    ID2_93_0 *storage;
    void *runtime;
};

struct Library2_93_0 {
    ID2_93_0 id;
    void *filedata;
    char filepath[1024];
    char filepath_abs[1024];
    Library2_93_0 *parent;
    PackedFile2_93_0 *packedfile;
    int temp_index;
    short versionfile, subversionfile;
};

struct ImageTile2_93_0 {
    ImageTile2_93_0 *next, *prev;
    ImageTile_Runtime2_93_0 runtime;
    char ok;
    char _pad[3];
    int tile_number;
    char label[64];
};

struct Image2_93_0 {
    ID2_93_0 id;
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

struct IpoCurve2_93_0 {
    IpoCurve2_93_0 *next, *prev;
    BPoint2_93_0 *bp;
    BezTriple2_93_0 *bezt;
    rctf2_93_0 maxrct, totrct;
    short blocktype, adrcode, vartype;
    short totvert;
    short ipo, extrap;
    short flag, rt;
    float ymin, ymax;
    unsigned int bitmask;
    float slide_min, slide_max;
    float curval;
    IpoDriver2_93_0 *driver;
};

struct Ipo2_93_0 {
    ID2_93_0 id;
    ListBase2_93_0 curve;
    rctf2_93_0 cur;
    short blocktype, showkey;
    short muteipo;
    char _pad[2];
};

struct Key2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    KeyBlock2_93_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    ListBase2_93_0 block;
    Ipo2_93_0 *ipo;
    ID2_93_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct Lattice2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    short pntsu, pntsv, pntsw, flag;
    short opntsu, opntsv, opntsw;
    char _pad2[3];
    char typeu, typev, typew;
    int actbp;
    float fu, fv, fw, du, dv, dw;
    BPoint2_93_0 *def;
    Ipo2_93_0 *ipo;
    Key2_93_0 *key;
    MDeformVert2_93_0 *dvert;
    char vgroup[64];
    EditLatt2_93_0 *editlatt;
    void *batch_cache;
};

struct LayerCollection2_93_0 {
    LayerCollection2_93_0 *next, *prev;
    Collection2_93_0 *collection;
    SceneCollection2_93_0 *scene_collection;
    short flag;
    short runtime_flag;
    char _pad[4];
    ListBase2_93_0 layer_collections;
    unsigned short local_collections_bits;
    short _pad2[3];
};

struct SceneCollection2_93_0 {
    SceneCollection2_93_0 *next, *prev;
    char name[64];
    int active_object_index;
    short flag;
    char type;
    char _pad;
    ListBase2_93_0 objects;
    ListBase2_93_0 scene_collections;
};

struct LightProbe2_93_0 {
    ID2_93_0 id;
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
    Object2_93_0 *parallax_ob;
    Image2_93_0 *image;
    Collection2_93_0 *visibility_grp;
    float distfalloff, distgridinf;
    char _pad[8];
};

struct LightCache2_93_0 {
    int flag;
    int version;
    int type;
    int cube_len, grid_len;
    int mips_len;
    int vis_res, ref_res;
    char _pad[4][2];
    LightCacheTexture2_93_0 grid_tx;
    LightCacheTexture2_93_0 cube_tx;
    LightCacheTexture2_93_0 *cube_mips;
    LightProbeCache2_93_0 *cube_data;
    LightGridCache2_93_0 *grid_data;
};

struct Light2_93_0 {
    ID2_93_0 id;
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
    Ipo2_93_0 *ipo;
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

struct LineStyleColorModifier_AlongStroke2_93_0 {
    LineStyleModifier2_93_0 modifier;
    ColorBand2_93_0 *color_ramp;
};

struct LineStyleAlphaModifier_AlongStroke2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    char _pad[4];
};

struct LineStyleThicknessModifier_AlongStroke2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float value_min, value_max;
    char _pad[4];
};

struct LineStyleColorModifier_DistanceFromCamera2_93_0 {
    LineStyleModifier2_93_0 modifier;
    ColorBand2_93_0 *color_ramp;
    float range_min, range_max;
};

struct LineStyleAlphaModifier_DistanceFromCamera2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float range_min, range_max;
    char _pad[4];
};

struct LineStyleThicknessModifier_DistanceFromCamera2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float range_min, range_max;
    float value_min, value_max;
    char _pad[4];
};

struct LineStyleColorModifier_DistanceFromObject2_93_0 {
    LineStyleModifier2_93_0 modifier;
    Object2_93_0 *target;
    ColorBand2_93_0 *color_ramp;
    float range_min, range_max;
};

struct LineStyleAlphaModifier_DistanceFromObject2_93_0 {
    LineStyleModifier2_93_0 modifier;
    Object2_93_0 *target;
    CurveMapping2_93_0 *curve;
    int flags;
    float range_min, range_max;
    char _pad[4];
};

struct LineStyleThicknessModifier_DistanceFromObject2_93_0 {
    LineStyleModifier2_93_0 modifier;
    Object2_93_0 *target;
    CurveMapping2_93_0 *curve;
    int flags;
    float range_min, range_max;
    float value_min, value_max;
    char _pad[4];
};

struct LineStyleColorModifier_Curvature_3D2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float min_curvature, max_curvature;
    ColorBand2_93_0 *color_ramp;
    float range_min, range_max;
};

struct LineStyleAlphaModifier_Curvature_3D2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float min_curvature, max_curvature;
    char _pad[4];
};

struct LineStyleThicknessModifier_Curvature_3D2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    char _pad[4];
    float min_curvature, max_curvature;
    float min_thickness, max_thickness;
};

struct LineStyleColorModifier_Noise2_93_0 {
    LineStyleModifier2_93_0 modifier;
    ColorBand2_93_0 *color_ramp;
    float period, amplitude;
    int seed;
    char _pad[4];
};

struct LineStyleAlphaModifier_Noise2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float period, amplitude;
    int seed;
};

struct LineStyleThicknessModifier_Noise2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float period, amplitude;
    int flags;
    int seed;
};

struct LineStyleColorModifier_CreaseAngle2_93_0 {
    LineStyleModifier2_93_0 modifier;
    ColorBand2_93_0 *color_ramp;
    float min_angle, max_angle;
};

struct LineStyleAlphaModifier_CreaseAngle2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float min_angle, max_angle;
    char _pad[4];
};

struct LineStyleThicknessModifier_CreaseAngle2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    char _pad[4];
    float min_angle, max_angle;
    float min_thickness, max_thickness;
};

struct LineStyleColorModifier_Tangent2_93_0 {
    LineStyleModifier2_93_0 modifier;
    ColorBand2_93_0 *color_ramp;
};

struct LineStyleAlphaModifier_Tangent2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    char _pad[4];
};

struct LineStyleThicknessModifier_Tangent2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float min_thickness, max_thickness;
    char _pad[4];
};

struct LineStyleColorModifier_Material2_93_0 {
    LineStyleModifier2_93_0 modifier;
    ColorBand2_93_0 *color_ramp;
    int flags;
    int mat_attr;
};

struct LineStyleAlphaModifier_Material2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    int mat_attr;
};

struct LineStyleThicknessModifier_Material2_93_0 {
    LineStyleModifier2_93_0 modifier;
    CurveMapping2_93_0 *curve;
    int flags;
    float value_min, value_max;
    int mat_attr;
};

struct LineStyleGeometryModifier_Sampling2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float sampling;
    char _pad[4];
};

struct LineStyleGeometryModifier_BezierCurve2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float error;
    char _pad[4];
};

struct LineStyleGeometryModifier_SinusDisplacement2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float wavelength, amplitude, phase;
    char _pad[4];
};

struct LineStyleGeometryModifier_SpatialNoise2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float amplitude, scale;
    unsigned int octaves;
    int flags;
};

struct LineStyleGeometryModifier_PerlinNoise1D2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float frequency, amplitude;
    float angle;
    unsigned int octaves;
    int seed;
    char _pad1[4];
};

struct LineStyleGeometryModifier_PerlinNoise2D2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float frequency, amplitude;
    float angle;
    unsigned int octaves;
    int seed;
    char _pad1[4];
};

struct LineStyleGeometryModifier_BackboneStretcher2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float backbone_length;
    char _pad[4];
};

struct LineStyleGeometryModifier_TipRemover2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float tip_length;
    char _pad[4];
};

struct LineStyleGeometryModifier_Polygonalization2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float error;
    char _pad[4];
};

struct LineStyleGeometryModifier_GuidingLines2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float offset;
    char _pad[4];
};

struct LineStyleGeometryModifier_Blueprint2_93_0 {
    LineStyleModifier2_93_0 modifier;
    int flags;
    unsigned int rounds;
    float backbone_length;
    unsigned int random_radius;
    unsigned int random_center;
    unsigned int random_backbone;
};

struct LineStyleGeometryModifier_2DOffset2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float start, end;
    float x, y;
};

struct LineStyleGeometryModifier_2DTransform2_93_0 {
    LineStyleModifier2_93_0 modifier;
    int pivot;
    float scale_x, scale_y;
    float angle;
    float pivot_u;
    float pivot_x, pivot_y;
    char _pad[4];
};

struct LineStyleGeometryModifier_Simplification2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float tolerance;
    char _pad[4];
};

struct LineStyleThicknessModifier_Calligraphy2_93_0 {
    LineStyleModifier2_93_0 modifier;
    float min_thickness, max_thickness;
    float orientation;
    char _pad[4];
};

struct FreestyleLineStyle2_93_0 {
    ID2_93_0 id;
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
    MTex2_93_0 *mtex[18];
    void *nodetree;
    ListBase2_93_0 color_modifiers;
    ListBase2_93_0 alpha_modifiers;
    ListBase2_93_0 thickness_modifiers;
    ListBase2_93_0 geometry_modifiers;
};

struct Mask2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 masklayers;
    int masklay_act;
    int masklay_tot;
    int sfra, efra;
    int flag;
    char _pad[4];
};

struct MaskSplinePoint2_93_0 {
    BezTriple2_93_0 bezt;
    char _pad[4];
    int tot_uw;
    MaskSplinePointUW2_93_0 *uw;
    MaskParent2_93_0 parent;
};

struct MaskSpline2_93_0 {
    MaskSpline2_93_0 *next, *prev;
    short flag;
    char offset_mode;
    char weight_interp;
    int tot_point;
    MaskSplinePoint2_93_0 *points;
    MaskParent2_93_0 parent;
    MaskSplinePoint2_93_0 *points_deform;
};

struct MaskLayer2_93_0 {
    MaskLayer2_93_0 *next, *prev;
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
    char restrictflag;
};

struct Material2_93_0 {
    ID2_93_0 id;
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
    Ipo2_93_0 *ipo;
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
    MaterialLineArt2_93_0 lineart;
};

struct Mesh_Runtime2_93_0 {
    Mesh2_93_0 *mesh_eval;
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
};

struct MetaBall2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    ListBase2_93_0 elems;
    ListBase2_93_0 disp;
    ListBase2_93_0 *editelems;
    Ipo2_93_0 *ipo;
    Material2_93_0 **mat;
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

struct ModifierData2_93_0 {
    ModifierData2_93_0 *next, *prev;
    int type, mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
    ModifierData2_93_0 *orig_modifier_data;
    SessionUUID2_93_0 session_uuid;
    void *runtime;
};

struct MovieClip_RuntimeGPUTexture2_93_0 {
    void *next, *prev;
    MovieClipUser2_93_0 user;
    void *gputexture[3];
};

struct MovieClip_Runtime2_93_0 {
    ListBase2_93_0 gputextures;
};

struct MovieClipScopes2_93_0 {
    short ok;
    short use_track_mask;
    int track_preview_height;
    int frame_width, frame_height;
    MovieTrackingMarker2_93_0 undist_marker;
    void *track_search;
    void *track_preview;
    float track_pos[2];
    short track_disabled;
    short track_locked;
    int scene_framenr;
    MovieTrackingTrack2_93_0 *track;
    MovieTrackingMarker2_93_0 *marker;
    float slide_scale[2];
};

struct bActionStrip2_93_0 {
    bActionStrip2_93_0 *next, *prev;
    short flag, mode;
    short stride_axis;
    short curmod;
    Ipo2_93_0 *ipo;
    bAction2_93_0 *act;
    Object2_93_0 *object;
    float start, end;
    float actstart, actend;
    float actoffs;
    float stridelen;
    float repeat;
    float scale;
    float blendin, blendout;
    char stridechannel[32];
    char offs_bone[32];
    ListBase2_93_0 modifiers;
};

struct bNodeSocket2_93_0 {
    bNodeSocket2_93_0 *next, *prev, *new_sock;
    IDProperty2_93_0 *prop;
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
    char _pad[1];
    short total_inputs;
    char label[64];
    char description[64];
    void *cache;
    int own_index;
    int to_index;
    bNodeSocket2_93_0 *groupsock;
    bNodeLink2_93_0 *link;
    bNodeStack2_93_0 ns;
};

struct bNode2_93_0 {
    bNode2_93_0 *next, *prev, *new_node;
    IDProperty2_93_0 *prop;
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
    bNode2_93_0 *parent;
    ID2_93_0 *id;
    void *storage;
    bNode2_93_0 *original;
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
};

struct bNodeInstanceHashEntry2_93_0 {
    bNodeInstanceKey2_93_0 key;
    short tag;
};

struct NodeColorCorrection2_93_0 {
    ColorCorrectionData2_93_0 master;
    ColorCorrectionData2_93_0 shadows;
    ColorCorrectionData2_93_0 midtones;
    ColorCorrectionData2_93_0 highlights;
    float startmidtones;
    float endmidtones;
};

struct NodeCryptomatte_Runtime2_93_0 {
    ListBase2_93_0 layers;
    float add[3];
    float remove[3];
};

struct SoftBody_Shared2_93_0 {
    void *pointcache;
    ListBase2_93_0 ptcaches;
};

struct SoftBody2_93_0 {
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
    int sfra, efra;
    int interval;
    short local, solverflags;
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
    Collection2_93_0 *collision_group;
    EffectorWeights2_93_0 *effector_weights;
    float lcom[3];
    float lrot[3][3];
    float lscale[3][3];
    int last_frame;
};

struct Object_Runtime2_93_0 {
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
    Mesh2_93_0 *mesh_deform_eval;
    bGPdata2_93_0 *gpd_orig;
    bGPdata2_93_0 *gpd_eval;
    Mesh2_93_0 *object_as_temp_mesh;
    Curve2_93_0 *object_as_temp_curve;
    void *curve_cache;
    unsigned short local_collections_bits;
    short _pad2[3];
};

struct BoidParticle2_93_0 {
    Object2_93_0 *ground;
    BoidData2_93_0 data;
    float gravity[3];
    float wander[3];
    float rt;
};

struct ParticleData2_93_0 {
    ParticleKey2_93_0 state;
    ParticleKey2_93_0 prev_state;
    HairKey2_93_0 *hair;
    ParticleKey2_93_0 *keys;
    BoidParticle2_93_0 *boid;
    int totkey;
    float time, lifetime;
    float dietime;
    int num;
    int num_dmcache;
    float fuv[4], foffset;
    float size;
    float sphdensity;
    char _pad[4];
    int hair_index;
    short flag;
    short alive;
};

struct ParticleSettings2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    BoidSettings2_93_0 *boids;
    SPHFluidSettings2_93_0 *fluid;
    EffectorWeights2_93_0 *effector_weights;
    Collection2_93_0 *collision_group;
    int flag, rt;
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
    MTex2_93_0 *mtex[18];
    Collection2_93_0 *instance_collection;
    ListBase2_93_0 instance_weights;
    Collection2_93_0 *force_group;
    Object2_93_0 *instance_object;
    Object2_93_0 *bb_ob;
    Ipo2_93_0 *ipo;
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

struct PTCacheMem2_93_0 {
    PTCacheMem2_93_0 *next, *prev;
    unsigned int frame, totpoint;
    unsigned int data_types, flag;
    void *data[8];
    ListBase2_93_0 extradata;
};

struct RigidBodyWorld_Shared2_93_0 {
    void *pointcache;
    ListBase2_93_0 ptcaches;
    void *physics_world;
};

struct RigidBodyWorld2_93_0 {
    EffectorWeights2_93_0 *effector_weights;
    Collection2_93_0 *group;
    Object2_93_0 **objects;
    Collection2_93_0 *constraints;
    char _pad[4];
    float ltime;
    RigidBodyWorld_Shared2_93_0 *shared;
    void *pointcache;
    ListBase2_93_0 ptcaches;
    int numbodies;
    short substeps_per_frame;
    short num_solver_iterations;
    int flag;
    float time_scale;
};

struct ImageFormatData2_93_0 {
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
    ColorManagedViewSettings2_93_0 view_settings;
    ColorManagedDisplaySettings2_93_0 display_settings;
};

struct Paint2_93_0 {
    Brush2_93_0 *brush;
    PaintToolSlot2_93_0 *tool_slots;
    int tool_slots_len;
    char _pad1[4];
    Palette2_93_0 *palette;
    CurveMapping2_93_0 *cavity_curve;
    void *paint_cursor;
    unsigned char paint_cursor_col[4];
    int flags;
    int num_input_samples;
    int symmetry_flags;
    float tile_offset[3];
    char _pad2[4];
    Paint_Runtime2_93_0 runtime;
};

struct ParticleEditSettings2_93_0 {
    short flag;
    short totrekey;
    short totaddkey;
    short brushtype;
    ParticleBrushData2_93_0 brush[7];
    void *paintcursor;
    float emitterdist, rt;
    int selectmode;
    int edittype;
    int draw_step, fade_frames;
    Scene2_93_0 *scene;
    Object2_93_0 *object;
    Object2_93_0 *shape_object;
};

struct GP_Sculpt_Settings2_93_0 {
    void *paintcursor;
    int flag;
    int lock_axis;
    float isect_threshold;
    char _pad[4];
    CurveMapping2_93_0 *cur_falloff;
    CurveMapping2_93_0 *cur_primitive;
    GP_Sculpt_Guide2_93_0 guide;
};

struct SceneDisplay2_93_0 {
    float light_direction[3];
    float shadow_shift, shadow_focus;
    float matcap_ssao_distance;
    float matcap_ssao_attenuation;
    int matcap_ssao_samples;
    char viewport_aa;
    char render_aa;
    char _pad[6];
    View3DShading2_93_0 shading;
};

struct bScreen2_93_0 {
    ID2_93_0 id;
    ListBase2_93_0 vertbase;
    ListBase2_93_0 edgebase;
    ListBase2_93_0 areabase;
    ListBase2_93_0 regionbase;
    Scene2_93_0 *scene;
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

struct ScrVert2_93_0 {
    ScrVert2_93_0 *next, *prev, *newv;
    vec2s2_93_0 vec;
    short flag, editflag;
};

struct ScrAreaMap2_93_0 {
    ListBase2_93_0 vertbase;
    ListBase2_93_0 edgebase;
    ListBase2_93_0 areabase;
};

struct Panel2_93_0 {
    Panel2_93_0 *next, *prev;
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
    Panel_Runtime2_93_0 runtime;
};

struct PanelCategoryDyn2_93_0 {
    PanelCategoryDyn2_93_0 *next, *prev;
    char idname[64];
    rcti2_93_0 rect;
};

struct ScrArea2_93_0 {
    ScrArea2_93_0 *next, *prev;
    ScrVert2_93_0 *v1, *v2, *v3, *v4;
    bScreen2_93_0 *full;
    rcti2_93_0 totrct;
    char spacetype;
    char butspacetype;
    short butspacetype_subtype;
    short winx, winy;
    char headertype;
    char do_refresh;
    short flag;
    short region_active_win;
    char _pad[2];
    void *type;
    ScrGlobalAreaData2_93_0 *global;
    ListBase2_93_0 spacedata;
    ListBase2_93_0 regionbase;
    ListBase2_93_0 handlers;
    ListBase2_93_0 actionzones;
    ScrArea_Runtime2_93_0 runtime;
};

struct ARegion_Runtime2_93_0 {
    const char *category;
    rcti2_93_0 visible_rect;
    int offset_x, offset_y;
};

struct SDNA_Struct2_93_0 {
    short type;
    short members_len;
    SDNA_StructMember2_93_0 members[];
};

struct Strip2_93_0 {
    Strip2_93_0 *next, *prev;
    int us, done;
    int startstill, endstill;
    StripElem2_93_0 *stripdata;
    char dir[768];
    StripProxy2_93_0 *proxy;
    StripCrop2_93_0 *crop;
    StripTransform2_93_0 *transform;
    StripColorBalance2_93_0 *color_balance;
    ColorManagedColorspaceSettings2_93_0 colorspace_settings;
};

struct SequenceRuntime2_93_0 {
    SessionUUID2_93_0 session_uuid;
};

struct Editing2_93_0 {
    ListBase2_93_0 *seqbasep;
    ListBase2_93_0 seqbase;
    ListBase2_93_0 metastack;
    Sequence2_93_0 *act_seq;
    char act_imagedir[1024];
    char act_sounddir[1024];
    char proxy_dir[1024];
    int over_ofs, over_cfra;
    int over_flag, proxy_storage;
    rctf2_93_0 over_border;
    void *cache;
    float recycle_max_cost;
    int cache_flag;
    void *prefetch_job;
    int64_t disk_cache_timestamp;
};

struct ColorBalanceModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
    StripColorBalance2_93_0 color_balance;
    float color_multiply;
};

struct BrightContrastModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
    float bright;
    float contrast;
};

struct SequencerMaskModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
};

struct WhiteBalanceModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
    float white_value[3];
    char _pad[4];
};

struct SequencerTonemapModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
    float key, offset, gamma;
    float intensity, contrast, adaptation, correction;
    int type;
};

struct BlurShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    float radius[2];
    int flag;
    int samples;
    float rotation;
    char _pad[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct ColorizeShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    int mode;
    float low_color[4];
    float high_color[4];
    float factor;
    int flag;
    char _pad[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct FlipShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    int flag;
    int flipmode;
    ShaderFxData_Runtime2_93_0 runtime;
};

struct GlowShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    float glow_color[4];
    float select_color[3];
    float threshold;
    int flag;
    int mode;
    float blur[2];
    int samples;
    float rotation;
    int blend_mode;
    char _pad[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct PixelShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    int size[3];
    int flag;
    float rgba[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct RimShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    int offset[2];
    int flag;
    float rim_rgb[3];
    float mask_rgb[3];
    int mode;
    int blur[2];
    int samples;
    char _pad[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct ShadowShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    Object2_93_0 *object;
    int offset[2];
    int flag;
    float shadow_rgba[4];
    float amplitude;
    float period;
    float phase;
    int orientation;
    float scale[2];
    float rotation;
    int blur[2];
    int samples;
    char _pad[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct SwirlShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    Object2_93_0 *object;
    int flag;
    int radius;
    float angle;
    int transparent;
    ShaderFxData_Runtime2_93_0 runtime;
};

struct WaveShaderFxData2_93_0 {
    ShaderFxData2_93_0 shaderfx;
    float amplitude;
    float period;
    float phase;
    int orientation;
    int flag;
    char _pad[4];
    ShaderFxData_Runtime2_93_0 runtime;
};

struct Simulation2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    void *nodetree;
    uint32_t flag;
    char _pad[4];
};

struct bSound2_93_0 {
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
    void *cache;
    void *waveform;
    void *playback_handle;
    void *spinlock;
};

struct SpaceLink2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
};

struct SpaceInfo2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char rpt_mask;
    char _pad[7];
};

struct SpaceGraph_Runtime2_93_0 {
    char flag;
    char _pad[7];
    ListBase2_93_0 ghost_curves;
};

struct FileAssetSelectParams2_93_0 {
    FileSelectParams2_93_0 base_params;
    FileSelectAssetLibraryUID2_93_0 asset_library;
};

struct FileFolderHistory2_93_0 {
    void *next, *prev;
    char browse_mode;
    char _pad[7];
    ListBase2_93_0 folders_prev;
    ListBase2_93_0 folders_next;
};

struct SpaceFile2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char browse_mode;
    char _pad1[1];
    short tags;
    int scroll_offset;
    FileSelectParams2_93_0 *params;
    FileAssetSelectParams2_93_0 *asset_params;
    void *_pad2;
    void *files;
    ListBase2_93_0 *folders_prev;
    ListBase2_93_0 *folders_next;
    ListBase2_93_0 folder_histories;
    wmOperator2_93_0 *op;
    void *smoothscroll_timer;
    void *previews_timer;
    void *layout;
    short recentnr, bookmarknr;
    short systemnr, system_bookmarknr;
    void *runtime;
};

struct FileDirEntryVariant2_93_0 {
    FileDirEntryVariant2_93_0 *next, *prev;
    int uuid[4];
    char *name;
    char *description;
    ListBase2_93_0 revisions;
    int nbr_revisions;
    int act_revision;
};

struct FileDirEntry2_93_0 {
    FileDirEntry2_93_0 *next, *prev;
    int uuid[4];
    char *name;
    char *description;
    FileDirEntryRevision2_93_0 *entry;
    int typeflag;
    int blentype;
    char *relpath;
    char *redirection_path;
    ID2_93_0 *id;
    AssetMetaData2_93_0 *asset_data;
    int preview_icon_id;
    char **tags;
    int nbr_tags;
    short status;
    short flags;
    int attributes;
    ListBase2_93_0 variants;
    int nbr_variants;
    int act_variant;
};

struct FileDirEntryArr2_93_0 {
    ListBase2_93_0 entries;
    int nbr_entries;
    int nbr_entries_filtered;
    int entry_idx_start, entry_idx_end;
    char root[1024];
};

struct SpaceText_Runtime2_93_0 {
    int lheight_px;
    int cwidth_px;
    rcti2_93_0 scroll_region_handle;
    rcti2_93_0 scroll_region_select;
    int line_number_display_digits;
    int viewlines;
    float scroll_px_per_line;
    int scroll_ofs_px[2];
    char _pad1[4];
    void *drawcache;
};

struct Script2_93_0 {
    ID2_93_0 id;
    void *py_draw;
    void *py_event;
    void *py_button;
    void *py_browsercallback;
    void *py_globaldict;
    int flags, lastspace;
    char scriptname[1024];
    char scriptarg[256];
};

struct SpaceScript2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Script2_93_0 *script;
    short flags, menunr;
    char _pad1[4];
    void *but_refs;
};

struct bNodeTreePath2_93_0 {
    bNodeTreePath2_93_0 *next, *prev;
    void *nodetree;
    bNodeInstanceKey2_93_0 parent_key;
    char _pad[4];
    float view_center[2];
    char node_name[64];
    char display_name[64];
};

struct SpaceConsole2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    int lheight;
    char _pad[4];
    ListBase2_93_0 scrollback;
    ListBase2_93_0 history;
    char prompt[256];
    char language[32];
    int sel_start;
    int sel_end;
};

struct SpaceUserPref2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char _pad1[7];
    char filter_type;
    char filter[64];
};

struct SpaceTopBar2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
};

struct SpaceStatusBar2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
};

struct SpreadsheetContextObject2_93_0 {
    SpreadsheetContext2_93_0 base;
    Object2_93_0 *object;
};

struct SpreadsheetContextModifier2_93_0 {
    SpreadsheetContext2_93_0 base;
    char *modifier_name;
};

struct SpreadsheetContextNode2_93_0 {
    SpreadsheetContext2_93_0 base;
    char *node_name;
};

struct SpaceSpreadsheet2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    ListBase2_93_0 columns;
    ListBase2_93_0 context_path;
    uint8_t filter_flag;
    uint8_t geometry_component_type;
    uint8_t attribute_domain;
    uint8_t object_eval_state;
    uint32_t flag;
    void *runtime;
};

struct Speaker2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    bSound2_93_0 *sound;
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

struct ColorBand2_93_0 {
    short tot, cur;
    char ipotype, ipotype_hue;
    char color_mode;
    char _pad[1];
    CBData2_93_0 data[32];
};

struct Tex2_93_0 {
    ID2_93_0 id;
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
    ImageUser2_93_0 iuser;
    void *nodetree;
    Ipo2_93_0 *ipo;
    Image2_93_0 *ima;
    ColorBand2_93_0 *coba;
    PreviewImage2_93_0 *preview;
    char use_nodes;
    char _pad[7];
};

struct Text2_93_0 {
    ID2_93_0 id;
    char *filepath;
    void *compiled;
    int flags;
    char _pad0[4];
    ListBase2_93_0 lines;
    TextLine2_93_0 *curl, *sell;
    int curc, selc;
    double mtime;
};

struct MovieTrackingObject2_93_0 {
    MovieTrackingObject2_93_0 *next, *prev;
    char name[64];
    int flag;
    float scale;
    ListBase2_93_0 tracks;
    ListBase2_93_0 plane_tracks;
    MovieTrackingReconstruction2_93_0 reconstruction;
    int keyframe1, keyframe2;
};

struct MovieTrackingDopesheet2_93_0 {
    int ok;
    short sort_method;
    short flag;
    ListBase2_93_0 coverage_segments;
    ListBase2_93_0 channels;
    int tot_channel;
    char _pad[4];
};

struct uiStyle2_93_0 {
    uiStyle2_93_0 *next, *prev;
    char name[64];
    uiFontStyle2_93_0 paneltitle;
    uiFontStyle2_93_0 grouplabel;
    uiFontStyle2_93_0 widgetlabel;
    uiFontStyle2_93_0 widget;
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

struct ThemeUI2_93_0 {
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
};

struct ThemeSpace2_93_0 {
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

struct bUserMenu2_93_0 {
    bUserMenu2_93_0 *next, *prev;
    char space_type;
    char _pad0[7];
    char context[64];
    ListBase2_93_0 items;
};

struct bUserMenuItem_Op2_93_0 {
    bUserMenuItem2_93_0 item;
    char op_idname[64];
    IDProperty2_93_0 *prop;
    char opcontext;
    char _pad0[7];
};

struct bUserMenuItem_Menu2_93_0 {
    bUserMenuItem2_93_0 item;
    char mt_idname[64];
};

struct bUserMenuItem_Prop2_93_0 {
    bUserMenuItem2_93_0 item;
    char context_data_path[256];
    char prop_id[64];
    int prop_index;
    char _pad0[4];
};

struct VFont2_93_0 {
    ID2_93_0 id;
    char filepath[1024];
    void *data;
    PackedFile2_93_0 *packedfile;
    PackedFile2_93_0 *temp_pf;
};

struct View2D2_93_0 {
    rctf2_93_0 tot, cur;
    rcti2_93_0 vert, hor;
    rcti2_93_0 mask;
    float min[2], max[2];
    float minzoom, maxzoom;
    short scroll;
    short scroll_ui;
    short keeptot;
    short keepzoom;
    short keepofs;
    short flag;
    short align;
    short winx, winy;
    short oldwinx, oldwiny;
    short around;
    char alpha_vert, alpha_hor;
    char _pad[6];
    void *sms;
    void *smooth_timer;
};

struct View3D2_93_0 {
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
    Object2_93_0 *camera, *ob_center;
    rctf2_93_0 render_border;
    View3D2_93_0 *localvd;
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
    bGPdata2_93_0 *gpd;
    short stereo3d_flag;
    char stereo3d_camera;
    char _pad4;
    float stereo3d_convergence_factor;
    float stereo3d_volume_alpha;
    float stereo3d_convergence_alpha;
    View3DShading2_93_0 shading;
    View3DOverlay2_93_0 overlay;
    View3D_Runtime2_93_0 runtime;
};

struct Volume2_93_0 {
    ID2_93_0 id;
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
    void *batch_cache;
    Volume_Runtime2_93_0 runtime;
};

struct ReportList2_93_0 {
    ListBase2_93_0 list;
    int printlevel;
    int storelevel;
    int flag;
    char _pad[4];
    void *reporttimer;
};

struct wmKeyConfig2_93_0 {
    wmKeyConfig2_93_0 *next, *prev;
    char idname[64];
    char basename[64];
    ListBase2_93_0 keymaps;
    int actkeymap;
    short flag;
    char _pad0[2];
};

struct wmOperator2_93_0 {
    wmOperator2_93_0 *next, *prev;
    char idname[64];
    IDProperty2_93_0 *properties;
    void *type;
    void *customdata;
    void *py_instance;
    void *ptr;
    ReportList2_93_0 *reports;
    ListBase2_93_0 macro;
    wmOperator2_93_0 *opm;
    void *layout;
    short flag;
    char _pad[6];
};

struct WorkSpace2_93_0 {
    ID2_93_0 id;
    ListBase2_93_0 layouts;
    ListBase2_93_0 hook_layout_relations;
    ListBase2_93_0 owner_ids;
    ListBase2_93_0 tools;
    char _pad[4];
    int object_mode;
    int flags;
    int order;
    char *status_text;
};

struct World2_93_0 {
    ID2_93_0 id;
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
    Ipo2_93_0 *ipo;
    short pr_texture, use_nodes;
    char _pad[4];
    PreviewImage2_93_0 *preview;
    void *nodetree;
    ListBase2_93_0 gpumaterial;
};

struct XrSessionSettings2_93_0 {
    View3DShading2_93_0 shading;
    char _pad[7];
    char base_pose_type;
    Object2_93_0 *base_pose_object;
    float base_pose_location[3];
    float base_pose_angle;
    char draw_flags;
    char _pad2[3];
    float clip_start, clip_end;
    int flag;
};

struct bPoseChannel2_93_0 {
    bPoseChannel2_93_0 *next, *prev;
    IDProperty2_93_0 *prop;
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
    Bone2_93_0 *bone;
    bPoseChannel2_93_0 *parent;
    bPoseChannel2_93_0 *child;
    ListBase2_93_0 iktree;
    ListBase2_93_0 siktree;
    bMotionPath2_93_0 *mpath;
    Object2_93_0 *custom;
    bPoseChannel2_93_0 *custom_tx;
    float custom_scale;
    char _pad1[4];
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
    float curve_in_x, curve_in_y;
    float curve_out_x, curve_out_y;
    float ease1, ease2;
    float scale_in_x, scale_in_y;
    float scale_out_x, scale_out_y;
    bPoseChannel2_93_0 *bbone_prev;
    bPoseChannel2_93_0 *bbone_next;
    void *temp;
    bPoseChannelDrawData2_93_0 *draw_data;
    bPoseChannel2_93_0 *orig_pchan;
    bPoseChannel_Runtime2_93_0 runtime;
};

struct SpaceAction2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
    bAction2_93_0 *action;
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

struct IDProperty2_93_0 {
    IDProperty2_93_0 *next, *prev;
    char type, subtype;
    short flag;
    char name[64];
    int saved;
    IDPropertyData2_93_0 data;
    int len;
    int totallen;
};

struct ViewLayer2_93_0 {
    ViewLayer2_93_0 *next, *prev;
    char name[64];
    short flag;
    char _pad[6];
    ListBase2_93_0 object_bases;
    void *stats;
    Base2_93_0 *basact;
    ListBase2_93_0 layer_collections;
    LayerCollection2_93_0 *active_collection;
    int layflag;
    int passflag;
    float pass_alpha_threshold;
    short cryptomatte_flag;
    short cryptomatte_levels;
    char _pad1[4];
    int samples;
    Material2_93_0 *mat_override;
    IDProperty2_93_0 *id_properties;
    FreestyleConfig2_93_0 freestyle_config;
    ViewLayerEEVEE2_93_0 eevee;
    ListBase2_93_0 aovs;
    ViewLayerAOV2_93_0 *active_aov;
    ListBase2_93_0 drawdata;
    Base2_93_0 **object_bases_array;
    void *object_bases_hash;
};

struct Mesh2_93_0 {
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
    Mesh2_93_0 *texcomesh;
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
    Mesh_Runtime2_93_0 runtime;
};

struct MappingInfoModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Tex2_93_0 *texture;
    Object2_93_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
};

struct SubsurfModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    short subdivType, levels, renderLevels, flags;
    short uv_smooth;
    short quality;
    short boundary_smooth;
    char _pad[2];
    void *emCache, *mCache;
};

struct LatticeModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
};

struct CurveModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
};

struct BuildModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    float start, length;
    short flag;
    short randomize;
    int seed;
};

struct MaskModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
};

struct ArrayModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *start_cap;
    Object2_93_0 *end_cap;
    Object2_93_0 *curve_ob;
    Object2_93_0 *offset_ob;
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

struct MirrorModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    short axis;
    short flag;
    float tolerance;
    float bisect_threshold;
    char _pad[4];
    float uv_offset[2];
    float uv_offset_copy[2];
    Object2_93_0 *mirror_ob;
};

struct EdgeSplitModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    float split_angle;
    int flags;
};

struct BevelModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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
};

struct FluidModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    FluidDomainSettings2_93_0 *domain;
    FluidFlowSettings2_93_0 *flow;
    FluidEffectorSettings2_93_0 *effector;
    float time;
    int type;
};

struct DisplaceModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Tex2_93_0 *texture;
    Object2_93_0 *map_object;
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

struct UVProjectModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *projectors[10];
    char _pad2[4];
    int num_projectors;
    float aspectx, aspecty;
    float scalex, scaley;
    char uvlayer_name[64];
    int uvlayer_tmp;
    char _pad[4];
};

struct DecimateModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct SmoothModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    float fac;
    char defgrp_name[64];
    short flag, repeat;
};

struct CastModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag, type;
};

struct WaveModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Tex2_93_0 *texture;
    Object2_93_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
    Object2_93_0 *objectcenter;
    char defgrp_name[64];
    short flag;
    char _pad[2];
    float startx, starty, height, width;
    float narrow, speed, damp, falloff;
    float timeoffs, lifetime;
    char _pad1[4];
};

struct HookModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
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
};

struct SoftbodyModifierData2_93_0 {
    ModifierData2_93_0 modifier;
};

struct ClothModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct CollisionModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct SurfaceModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    MVert2_93_0 *x;
    MVert2_93_0 *v;
    Mesh2_93_0 *mesh;
    void *bvhtree;
    int cfra, numverts;
};

struct BooleanModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
    Collection2_93_0 *collection;
    float double_threshold;
    char operation;
    char solver;
    char flag;
    char bm_flag;
};

struct ParticleSystemModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    void *psys;
    Mesh2_93_0 *mesh_final;
    Mesh2_93_0 *mesh_original;
    int totdmvert, totdmedge, totdmface;
    short flag;
    char _pad[2];
};

struct ParticleInstanceModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *ob;
    short psys, flag, axis, space;
    float position, random_position;
    float rotation, random_rotation;
    float particle_amount, particle_offset;
    char index_layer_name[64];
    char value_layer_name[64];
};

struct ExplodeModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    int *facepa;
    short flag, vgroup;
    float protect;
    char uvname[64];
};

struct MultiresModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char lvl, sculptlvl, renderlvl, totlvl;
    char simple;
    char flags, _pad[2];
    short quality;
    short uv_smooth;
    short boundary_smooth;
    char _pad2[2];
};

struct FluidsimModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    FluidsimSettings2_93_0 *fss;
};

struct SmokeModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    int type;
    int _pad;
};

struct ShrinkwrapModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *target;
    Object2_93_0 *auxTarget;
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

struct SimpleDeformModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *origin;
    char vgroup_name[64];
    float factor;
    float limit[2];
    char mode;
    char axis;
    char deform_axis;
    char flag;
};

struct ShapeKeyModifierData2_93_0 {
    ModifierData2_93_0 modifier;
};

struct SolidifyModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct ScrewModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *ob_axis;
    unsigned int steps;
    unsigned int render_steps;
    unsigned int iter;
    float screw_ofs;
    float angle;
    float merge_dist;
    short flag;
    char axis;
    char _pad[5];
};

struct OceanModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct WarpModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Tex2_93_0 *texture;
    Object2_93_0 *map_object;
    char map_bone[64];
    char uvlayer_name[64];
    int uvlayer_tmp;
    int texmapping;
    Object2_93_0 *object_from;
    Object2_93_0 *object_to;
    char bone_from[64];
    char bone_to[64];
    CurveMapping2_93_0 *curfalloff;
    char defgrp_name[64];
    float strength;
    float falloff_radius;
    char flag;
    char falloff_type;
    char _pad[6];
};

struct WeightVGEditModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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
    Object2_93_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[64];
    char _pad0[4];
};

struct WeightVGMixModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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
    Object2_93_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[64];
    char flag;
    char _pad1[3];
};

struct WeightVGProximityModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char defgrp_name[64];
    CurveMapping2_93_0 *cmap_curve;
    int proximity_mode;
    int proximity_flags;
    Object2_93_0 *proximity_ob_target;
    float mask_constant;
    char mask_defgrp_name[64];
    int mask_tex_use_channel;
    Tex2_93_0 *mask_texture;
    Object2_93_0 *mask_tex_map_obj;
    char mask_tex_map_bone[64];
    int mask_tex_mapping;
    char mask_tex_uvlayer_name[64];
    float min_dist, max_dist;
    short falloff_type;
    char _pad0[2];
};

struct DynamicPaintModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    DynamicPaintCanvasSettings2_93_0 *canvas;
    DynamicPaintBrushSettings2_93_0 *brush;
    int type;
    char _pad[4];
};

struct RemeshModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct SkinModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct TriangulateModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    float lambda, lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag, repeat;
};

struct UVWarpModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char axis_u, axis_v;
    short flag;
    float center[2];
    float offset[2];
    float scale[2];
    float rotation;
    Object2_93_0 *object_src;
    char bone_src[64];
    Object2_93_0 *object_dst;
    char bone_dst[64];
    char vgroup_name[64];
    char uvlayer_name[64];
};

struct MeshCacheModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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

struct LaplacianDeformModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char anchor_grp_name[64];
    int total_verts, repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag, mat_ofs;
    char _pad[4];
};

struct WeldModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct DataTransferModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *ob_source;
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
};

struct NormalEditModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char defgrp_name[64];
    Object2_93_0 *target;
    short mode;
    short flag;
    short mix_mode;
    char _pad[2];
    float mix_factor;
    float mix_limit;
    float offset[3];
    char _pad0[4];
};

struct MeshSeqCacheModifierData2_93_0 {
    ModifierData2_93_0 modifier;
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
};

struct SurfaceDeformModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    void *depsgraph;
    Object2_93_0 *target;
    SDefVert2_93_0 *verts;
    float falloff;
    unsigned int numverts, numpoly;
    int flags;
    float mat[4][4];
    float strength;
    char _pad[4];
    char defgrp_name[64];
};

struct WeightedNormalModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    char defgrp_name[64];
    char mode, flag;
    short weight;
    float thresh;
};

struct NodesModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    void *node_group;
    NodesModifierSettings2_93_0 settings;
};

struct MeshToVolumeModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char fill_volume;
    char _pad1[3];
    float interior_band_width;
    float exterior_band_width;
    float density;
    char _pad2[4];
};

struct VolumeDisplaceModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Tex2_93_0 *texture;
    Object2_93_0 *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

struct VolumeToMeshModifierData2_93_0 {
    ModifierData2_93_0 modifier;
    Object2_93_0 *object;
    float threshold;
    float adaptivity;
    uint32_t flag;
    int resolution_mode;
    float voxel_size;
    int voxel_amount;
    char grid_name[64];
};

struct MovieTracking2_93_0 {
    MovieTrackingSettings2_93_0 settings;
    MovieTrackingCamera2_93_0 camera;
    ListBase2_93_0 tracks;
    ListBase2_93_0 plane_tracks;
    MovieTrackingReconstruction2_93_0 reconstruction;
    MovieTrackingStabilization2_93_0 stabilization;
    MovieTrackingTrack2_93_0* act_track;
    MovieTrackingPlaneTrack2_93_0* act_plane_track;
    ListBase2_93_0 objects;
    int objectnr, tot_object;
    MovieTrackingStats2_93_0* stats;
    MovieTrackingDopesheet2_93_0 dopesheet;
};

struct MovieClip2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    char filepath[1024];
    int source;
    int lastframe;
    int lastsize[2];
    float aspx, aspy;
    void *anim;
    void *cache;
    bGPdata2_93_0 *gpd;
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

struct bNodePreview2_93_0 {
    bNodeInstanceHashEntry2_93_0 hash_entry;
    unsigned char *rect;
    short xsize, ysize;
};

struct NodeImageFile2_93_0 {
    char name[1024];
    ImageFormatData2_93_0 im_format;
    int sfra, efra;
};

struct NodeImageMultiFile2_93_0 {
    char base_path[1024];
    ImageFormatData2_93_0 format;
    int sfra , efra;
    int active_input;
    char _pad[4];
};

struct NodeImageMultiFileSocket2_93_0 {
    short use_render_format;
    short use_node_format;
    char save_as_render;
    char _pad1[3];
    char path[1024];
    ImageFormatData2_93_0 format;
    char layer[30];
    char _pad2[2];
};

struct ColorMapping2_93_0 {
    ColorBand2_93_0 coba;
    float bright, contrast, saturation;
    int flag;
    float blend_color[3];
    float blend_factor;
    int blend_type;
    char _pad[4];
};

struct NodeTexBase2_93_0 {
    TexMapping2_93_0 tex_mapping;
    ColorMapping2_93_0 color_mapping;
};

struct NodeTexSky2_93_0 {
    NodeTexBase2_93_0 base;
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

struct NodeTexImage2_93_0 {
    NodeTexBase2_93_0 base;
    ImageUser2_93_0 iuser;
    int color_space;
    int projection;
    float projection_blend;
    int interpolation;
    int extension;
    char _pad[4];
};

struct NodeTexChecker2_93_0 {
    NodeTexBase2_93_0 base;
};

struct NodeTexBrick2_93_0 {
    NodeTexBase2_93_0 base;
    int offset_freq, squash_freq;
    float offset, squash;
};

struct NodeTexEnvironment2_93_0 {
    NodeTexBase2_93_0 base;
    ImageUser2_93_0 iuser;
    int color_space;
    int projection;
    int interpolation;
    char _pad[4];
};

struct NodeTexGradient2_93_0 {
    NodeTexBase2_93_0 base;
    int gradient_type;
    char _pad[4];
};

struct NodeTexNoise2_93_0 {
    NodeTexBase2_93_0 base;
    int dimensions;
    char _pad[4];
};

struct NodeTexVoronoi2_93_0 {
    NodeTexBase2_93_0 base;
    int dimensions;
    int feature;
    int distance;
    int coloring;
};

struct NodeTexMusgrave2_93_0 {
    NodeTexBase2_93_0 base;
    int musgrave_type;
    int dimensions;
};

struct NodeTexWave2_93_0 {
    NodeTexBase2_93_0 base;
    int wave_type;
    int bands_direction;
    int rings_direction;
    int wave_profile;
};

struct NodeTexMagic2_93_0 {
    NodeTexBase2_93_0 base;
    int depth;
    char _pad[4];
};

struct NodeShaderTexPointDensity2_93_0 {
    NodeTexBase2_93_0 base;
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

struct NodeCryptomatte2_93_0 {
    ImageUser2_93_0 iuser;
    ListBase2_93_0 entries;
    char layer_name[64];
    char *matte_id;
    int num_inputs;
    char _pad[4];
    NodeCryptomatte_Runtime2_93_0 runtime;
};

struct NodeAttributeColorRamp2_93_0 {
    ColorBand2_93_0 color_ramp;
};

struct Object2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    DrawDataList2_93_0 drawdata;
    void *sculpt;
    short type, partype;
    int par1, par2, par3;
    char parsubstr[64];
    Object2_93_0 *parent, *track;
    Object2_93_0 *proxy, *proxy_group, *proxy_from;
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
    Object_Runtime2_93_0 runtime;
};

struct SceneRenderLayer2_93_0 {
    SceneRenderLayer2_93_0 *next, *prev;
    char name[64];
    Material2_93_0 *mat_override;
    unsigned int lay;
    unsigned int lay_zmask;
    unsigned int lay_exclude;
    int layflag;
    int passflag;
    int pass_xor;
    int samples;
    float pass_alpha_threshold;
    IDProperty2_93_0 *prop;
    FreestyleConfig2_93_0 freestyleConfig;
};

struct BakeData2_93_0 {
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
    char _pad[6];
    Object2_93_0 *cage_object;
};

struct RenderData2_93_0 {
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
    int tilex, tiley;
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
    int preview_start_resolution;
    short preview_pixel_size;
    short debug_pass_type;
    ListBase2_93_0 views;
    short actview;
    short views_format;
    short hair_type, hair_subdiv;
    CurveMapping2_93_0 mblur_shutter_curve;
};

struct ImagePaintSettings2_93_0 {
    Paint2_93_0 paint;
    short flag, missing_data;
    short seam_bleed, normal_angle;
    short screen_grab_size[2];
    int mode;
    Image2_93_0 *stencil;
    Image2_93_0 *clone;
    Image2_93_0 *canvas;
    float stencil_col[3];
    float dither;
    int interp;
    char _pad[4];
};

struct Sculpt2_93_0 {
    Paint2_93_0 paint;
    int flags;
    int automasking_flags;
    int radial_symm[3];
    float detail_size;
    int symmetrize_direction;
    float gravity_factor;
    float constant_detail;
    float detail_percent;
    Object2_93_0 *gravity_object;
};

struct UvSculpt2_93_0 {
    Paint2_93_0 paint;
};

struct GpPaint2_93_0 {
    Paint2_93_0 paint;
    int flag;
    int mode;
};

struct GpVertexPaint2_93_0 {
    Paint2_93_0 paint;
    int flag;
    char _pad[4];
};

struct GpSculptPaint2_93_0 {
    Paint2_93_0 paint;
    int flag;
    char _pad[4];
};

struct GpWeightPaint2_93_0 {
    Paint2_93_0 paint;
    int flag;
    char _pad[4];
};

struct VPaint2_93_0 {
    Paint2_93_0 paint;
    char flag;
    char _pad[3];
    int radial_symm[3];
};

struct ToolSettings2_93_0 {
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
    char gpencil_seq_align;
    char gpencil_ima_align;
    char annotate_v3d_align;
    short annotate_thickness;
    char gpencil_selectmode_edit;
    char gpencil_selectmode_sculpt;
    GP_Sculpt_Settings2_93_0 gp_sculpt;
    GP_Interpolate_Settings2_93_0 gp_interpolate;
    ImagePaintSettings2_93_0 imapaint;
    ParticleEditSettings2_93_0 particle;
    float proportional_size;
    float select_thresh;
    short autokey_flag;
    char autokey_mode;
    char keyframe_type;
    char multires_subdiv_type;
    char edge_mode;
    char edge_mode_live_unwrap;
    char _pad1[1];
    char transform_pivot_point;
    char transform_flag;
    char snap_mode, snap_node_mode;
    char snap_uv_mode;
    char snap_flag;
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
    SequencerToolSettings2_93_0 *sequencer_tool_settings;
};

struct Scene2_93_0 {
    ID2_93_0 id;
    AnimData2_93_0 *adt;
    Object2_93_0 *camera;
    World2_93_0 *world;
    Scene2_93_0 *set;
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
    Editing2_93_0 *ed;
    ToolSettings2_93_0 *toolsettings;
    void *_pad4;
    DisplaySafeAreas2_93_0 safe_areas;
    RenderData2_93_0 r;
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
    bGPdata2_93_0 *gpd;
    MovieClip2_93_0 *clip;
    PhysicsSettings2_93_0 physics_settings;
    void *_pad8;
    CustomData_MeshMasks2_93_0 customdata_mask;
    CustomData_MeshMasks2_93_0 customdata_mask_modal;
    ColorManagedViewSettings2_93_0 view_settings;
    ColorManagedDisplaySettings2_93_0 display_settings;
    ColorManagedColorspaceSettings2_93_0 sequencer_colorspace_settings;
    RigidBodyWorld2_93_0 *rigidbody_world;
    PreviewImage2_93_0 *preview;
    ListBase2_93_0 view_layers;
    Collection2_93_0 *master_collection;
    SceneCollection2_93_0 *collection;
    IDProperty2_93_0 *layer_properties;
    void *_pad9;
    SceneDisplay2_93_0 display;
    SceneEEVEE2_93_0 eevee;
    SceneGpencil2_93_0 grease_pencil_settings;
};

struct ARegion2_93_0 {
    ARegion2_93_0 *next, *prev;
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
    ARegion_Runtime2_93_0 runtime;
};

struct Sequence2_93_0 {
    Sequence2_93_0 *next, *prev;
    void *tmp;
    void *lib;
    char name[64];
    int flag, type;
    int len;
    int start;
    int startofs, endofs;
    int startstill, endstill;
    int machine, depth;
    int startdisp, enddisp;
    float sat;
    float mul, handsize;
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    Strip2_93_0 *strip;
    Ipo2_93_0 *ipo;
    Scene2_93_0 *scene;
    Object2_93_0 *scene_camera;
    MovieClip2_93_0 *clip;
    Mask2_93_0 *mask;
    ListBase2_93_0 anims;
    float effect_fader;
    float speed_fader;
    Sequence2_93_0 *seq1, *seq2, *seq3;
    ListBase2_93_0 seqbase;
    bSound2_93_0 *sound;
    void *scene_sound;
    float volume;
    float pitch, pan;
    float strobe;
    void *effectdata;
    int anim_startofs;
    int anim_endofs;
    int blend_mode;
    float blend_opacity;
    int sfra;
    char alpha_mode;
    char _pad[2];
    char views_format;
    Stereo3dFormat2_93_0 *stereo3d_format;
    IDProperty2_93_0 *prop;
    ListBase2_93_0 modifiers;
    int cache_flag;
    int _pad2[3];
    SequenceRuntime2_93_0 runtime;
};

struct CurvesModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
    CurveMapping2_93_0 curve_mapping;
};

struct HueCorrectModifierData2_93_0 {
    SequenceModifierData2_93_0 modifier;
    CurveMapping2_93_0 curve_mapping;
};

struct SpaceProperties2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
    short space_subtype;
    short mainb, mainbo, mainbuser;
    short preview;
    char _pad[4];
    char flag;
    char outliner_sync;
    void *path;
    int pathflag, dataicon;
    ID2_93_0 *pinid;
    void *texuser;
    void *runtime;
};

struct SpaceOutliner2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
    ListBase2_93_0 tree;
    void *treestore;
    char search_string[64];
    TreeStoreElem2_93_0 search_tse;
    short flag, outlinevis, storeflag;
    char search_flags;
    char sync_select_dirty;
    int filter;
    char filter_state;
    char show_restrict_flags;
    short filter_id_type;
    void *runtime;
};

struct SpaceGraph2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
    bDopeSheet2_93_0 *ads;
    short mode;
    short autosnap;
    int flag;
    float cursorTime;
    float cursorVal;
    int around;
    char _pad[4];
    SpaceGraph_Runtime2_93_0 runtime;
};

struct SpaceNla2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    short autosnap;
    short flag;
    char _pad[4];
    bDopeSheet2_93_0 *ads;
    View2D2_93_0 v2d;
};

struct SpaceSeq2_93_0 {
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
    int view;
    int overlay_type;
    int draw_flag;
    char _pad[4];
    bGPdata2_93_0 *gpd;
    SequencerScopes2_93_0 scopes;
    char multiview_eye;
    char _pad2[7];
};

struct SpaceImage2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Image2_93_0 *image;
    ImageUser2_93_0 iuser;
    Scopes2_93_0 scopes;
    Histogram2_93_0 sample_line_hist;
    bGPdata2_93_0 *gpd;
    float cursor[2];
    float xof, yof;
    float zoom;
    float centx, centy;
    char mode;
    char mode_prev;
    char pin;
    char _pad1;
    short curtile;
    short lock;
    char dt_uv;
    char sticky;
    char dt_uvstretch;
    char around;
    int flag;
    char pixel_snap_mode;
    char _pad2[7];
    float uv_opacity;
    int tile_grid_shape[2];
    MaskSpaceInfo2_93_0 mask_info;
    SpaceImageOverlay2_93_0 overlay;
};

struct SpaceText2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    Text2_93_0 *text;
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
    SpaceText_Runtime2_93_0 runtime;
};

struct SpaceNode2_93_0 {
    SpaceLink2_93_0 *next, *prev;
    ListBase2_93_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    View2D2_93_0 v2d;
    ID2_93_0 *id, *from;
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
    bGPdata2_93_0 *gpd;
    void *runtime;
};

struct SpaceClip2_93_0 {
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
    MaskSpaceInfo2_93_0 mask_info;
};

struct bTheme2_93_0 {
    bTheme2_93_0 *next, *prev;
    char name[32];
    ThemeUI2_93_0 tui;
    ThemeSpace2_93_0 space_properties;
    ThemeSpace2_93_0 space_view3d;
    ThemeSpace2_93_0 space_file;
    ThemeSpace2_93_0 space_graph;
    ThemeSpace2_93_0 space_info;
    ThemeSpace2_93_0 space_action;
    ThemeSpace2_93_0 space_nla;
    ThemeSpace2_93_0 space_sequencer;
    ThemeSpace2_93_0 space_image;
    ThemeSpace2_93_0 space_text;
    ThemeSpace2_93_0 space_outliner;
    ThemeSpace2_93_0 space_node;
    ThemeSpace2_93_0 space_preferences;
    ThemeSpace2_93_0 space_console;
    ThemeSpace2_93_0 space_clip;
    ThemeSpace2_93_0 space_topbar;
    ThemeSpace2_93_0 space_statusbar;
    ThemeSpace2_93_0 space_spreadsheet;
    ThemeWireColor2_93_0 tarm[20];
    ThemeCollectionColor2_93_0 collection_color[8];
    int active_theme_area;
    char _pad0[4];
};

struct UserDef2_93_0 {
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
    char _pad10[3];
    char statusbar_flag;
    WalkNavigation2_93_0 walk_navigation;
    UserDef_SpaceData2_93_0 space_data;
    UserDef_FileSpaceData2_93_0 file_space_data;
    UserDef_Experimental2_93_0 experimental;
    UserDef_Runtime2_93_0 runtime;
};

struct wmXrData2_93_0 {
    void *runtime;
    XrSessionSettings2_93_0 session_settings;
};

struct wmWindowManager2_93_0 {
    ID2_93_0 id;
    wmWindow2_93_0 *windrawable, *winactive;
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
    wmXrData2_93_0 xr;
};

struct wmWindow2_93_0 {
    wmWindow2_93_0 *next, *prev;
    void *ghostwin;
    void *gpuctx;
    wmWindow2_93_0 *parent;
    Scene2_93_0 *scene;
    Scene2_93_0 *new_scene;
    char view_layer_name[64];
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
    void *tweak;
    void *ime_data;
    ListBase2_93_0 event_queue;
    ListBase2_93_0 handlers;
    ListBase2_93_0 modalhandlers;
    ListBase2_93_0 gesture;
    Stereo3dFormat2_93_0 *stereo3d_format;
    ListBase2_93_0 drawcalls;
    void *cursor_keymap_status;
};

#endif
