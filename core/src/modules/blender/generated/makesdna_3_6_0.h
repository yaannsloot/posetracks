/* SPDX-FileCopyrightText: 2025 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef MAKESDNA_3_6_0_H
#define MAKESDNA_3_6_0_H

struct ARegion3_6_0;
struct ARegion_Runtime3_6_0;
struct AnimData3_6_0;
struct AnimOverride3_6_0;
struct ArmatureGpencilModifierData3_6_0;
struct ArmatureModifierData3_6_0;
struct ArrayGpencilModifierData3_6_0;
struct ArrayModifierData3_6_0;
struct AssetFilterSettings3_6_0;
struct AssetHandle3_6_0;
struct AssetLibraryReference3_6_0;
struct AssetMetaData3_6_0;
struct AssetTag3_6_0;
struct AssetWeakReference3_6_0;
struct AudioData3_6_0;
struct AviCodecData3_6_0;
struct BHead3_6_0;
struct BHead43_6_0;
struct BHead83_6_0;
struct BPoint3_6_0;
struct BakeData3_6_0;
struct Base3_6_0;
struct BevList3_6_0;
struct BevPoint3_6_0;
struct BevelModifierData3_6_0;
struct BezTriple3_6_0;
struct BlurShaderFxData3_6_0;
struct BoidData3_6_0;
struct BoidParticle3_6_0;
struct BoidRule3_6_0;
struct BoidRuleAverageSpeed3_6_0;
struct BoidRuleAvoidCollision3_6_0;
struct BoidRuleFight3_6_0;
struct BoidRuleFollowLeader3_6_0;
struct BoidRuleGoalAvoid3_6_0;
struct BoidSettings3_6_0;
struct BoidState3_6_0;
struct Bone3_6_0;
struct BooleanModifierData3_6_0;
struct BoundBox3_6_0;
struct BrightContrastModifierData3_6_0;
struct Brush3_6_0;
struct BrushClone3_6_0;
struct BrushCurvesSculptSettings3_6_0;
struct BrushGpencilSettings3_6_0;
struct BuildEff3_6_0;
struct BuildGpencilModifierData3_6_0;
struct BuildModifierData3_6_0;
struct CBData3_6_0;
struct CacheFile3_6_0;
struct CacheFileLayer3_6_0;
struct CacheObjectPath3_6_0;
struct Camera3_6_0;
struct CameraBGImage3_6_0;
struct CameraDOFSettings3_6_0;
struct CameraStereoSettings3_6_0;
struct Camera_Runtime3_6_0;
struct CastModifierData3_6_0;
struct ChannelDriver3_6_0;
struct CharInfo3_6_0;
struct ChildParticle3_6_0;
struct ClothCollSettings3_6_0;
struct ClothModifierData3_6_0;
struct ClothSimSettings3_6_0;
struct Collection3_6_0;
struct CollectionChild3_6_0;
struct CollectionObject3_6_0;
struct Collection_Runtime3_6_0;
struct CollisionModifierData3_6_0;
struct ColorBalanceModifierData3_6_0;
struct ColorCorrectionData3_6_0;
struct ColorGpencilModifierData3_6_0;
struct ColorManagedColorspaceSettings3_6_0;
struct ColorManagedDisplaySettings3_6_0;
struct ColorManagedViewSettings3_6_0;
struct ColorMixVars3_6_0;
struct ColorizeShaderFxData3_6_0;
struct ConsoleLine3_6_0;
struct CorrectiveSmoothDeltaCache3_6_0;
struct CorrectiveSmoothModifierData3_6_0;
struct CryptomatteEntry3_6_0;
struct CryptomatteLayer3_6_0;
struct Curve3_6_0;
struct CurveMap3_6_0;
struct CurveMapPoint3_6_0;
struct CurveModifierData3_6_0;
struct CurvePaintSettings3_6_0;
struct CurveProfile3_6_0;
struct CurveProfilePoint3_6_0;
struct Curves3_6_0;
struct CurvesGeometry3_6_0;
struct CurvesSculpt3_6_0;
struct CustomData3_6_0;
struct CustomDataExternal3_6_0;
struct CustomDataLayer3_6_0;
struct CustomData_MeshMasks3_6_0;
struct DashGpencilModifierData3_6_0;
struct DashGpencilModifierSegment3_6_0;
struct DataTransferModifierData3_6_0;
struct DecimateModifierData3_6_0;
struct DisplaceModifierData3_6_0;
struct DisplaySafeAreas3_6_0;
struct DrawDataList3_6_0;
struct DriverTarget3_6_0;
struct DualQuat3_6_0;
struct DynamicPaintBrushSettings3_6_0;
struct DynamicPaintCanvasSettings3_6_0;
struct DynamicPaintModifierData3_6_0;
struct DynamicPaintRuntime3_6_0;
struct DynamicPaintSurface3_6_0;
struct EdgeSplitModifierData3_6_0;
struct EditLatt3_6_0;
struct EditNurb3_6_0;
struct Editing3_6_0;
struct EditingRuntime3_6_0;
struct Effect3_6_0;
struct EffectorWeights3_6_0;
struct EnvelopeGpencilModifierData3_6_0;
struct ExplodeModifierData3_6_0;
struct FCM_EnvelopeData3_6_0;
struct FCurve3_6_0;
struct FFMpegCodecData3_6_0;
struct FMod_Cycles3_6_0;
struct FMod_Envelope3_6_0;
struct FMod_FunctionGenerator3_6_0;
struct FMod_Generator3_6_0;
struct FMod_Limits3_6_0;
struct FMod_Noise3_6_0;
struct FMod_Python3_6_0;
struct FMod_Stepped3_6_0;
struct FModifier3_6_0;
struct FPoint3_6_0;
struct FileAssetSelectParams3_6_0;
struct FileDirEntryArr3_6_0;
struct FileFolderHistory3_6_0;
struct FileGlobal3_6_0;
struct FileSelectParams3_6_0;
struct FlipShaderFxData3_6_0;
struct FluidEffectorSettings3_6_0;
struct FluidFlowSettings3_6_0;
struct FluidModifierData3_6_0;
struct FluidVertexVelocity3_6_0;
struct FluidsimModifierData3_6_0;
struct FluidsimSettings3_6_0;
struct FreestyleConfig3_6_0;
struct FreestyleEdge3_6_0;
struct FreestyleFace3_6_0;
struct FreestyleLineSet3_6_0;
struct FreestyleLineStyle3_6_0;
struct FreestyleModuleConfig3_6_0;
struct GPUDOFSettings3_6_0;
struct GP_Interpolate_Settings3_6_0;
struct GP_Sculpt_Guide3_6_0;
struct GP_Sculpt_Settings3_6_0;
struct GaussianBlurVars3_6_0;
struct GlowShaderFxData3_6_0;
struct GlowVars3_6_0;
struct GpPaint3_6_0;
struct GpSculptPaint3_6_0;
struct GpVertexPaint3_6_0;
struct GpWeightPaint3_6_0;
struct GpencilModifierData3_6_0;
struct GridPaintMask3_6_0;
struct HairKey3_6_0;
struct Histogram3_6_0;
struct HookGpencilModifierData3_6_0;
struct HookModifierData3_6_0;
struct ID3_6_0;
struct IDOverrideLibrary3_6_0;
struct IDOverrideLibraryProperty3_6_0;
struct IDOverrideLibraryPropertyOperation3_6_0;
struct IDOverrideLibraryRuntime3_6_0;
struct IDProperty3_6_0;
struct IDPropertyData3_6_0;
struct IDPropertyUIData3_6_0;
struct IDPropertyUIDataBool3_6_0;
struct IDPropertyUIDataFloat3_6_0;
struct IDPropertyUIDataID3_6_0;
struct IDPropertyUIDataInt3_6_0;
struct IDPropertyUIDataString3_6_0;
struct IDViewerPathElem3_6_0;
struct ID_Runtime3_6_0;
struct ID_Runtime_Remap3_6_0;
struct IdAdtTemplate3_6_0;
struct Image3_6_0;
struct ImageAnim3_6_0;
struct ImageFormatData3_6_0;
struct ImagePackedFile3_6_0;
struct ImagePaintSettings3_6_0;
struct ImageTile3_6_0;
struct ImageTile_Runtime3_6_0;
struct ImageUser3_6_0;
struct ImageView3_6_0;
struct Image_Runtime3_6_0;
struct Ipo3_6_0;
struct IpoCurve3_6_0;
struct IpoDriver3_6_0;
struct KS_Path3_6_0;
struct Key3_6_0;
struct KeyBlock3_6_0;
struct KeyingSet3_6_0;
struct LaplacianDeformModifierData3_6_0;
struct LaplacianSmoothModifierData3_6_0;
struct Lattice3_6_0;
struct LatticeGpencilModifierData3_6_0;
struct LatticeModifierData3_6_0;
struct LayerCollection3_6_0;
struct LengthGpencilModifierData3_6_0;
struct Library3_6_0;
struct LibraryWeakReference3_6_0;
struct Library_Runtime3_6_0;
struct Light3_6_0;
struct LightCache3_6_0;
struct LightGridCache3_6_0;
struct LightProbe3_6_0;
struct LightProbeBakingData3_6_0;
struct LightProbeBlockData3_6_0;
struct LightProbeCache3_6_0;
struct LightProbeConnectivityData3_6_0;
struct LightProbeGridCacheFrame3_6_0;
struct LightProbeIrradianceData3_6_0;
struct LightProbeObjectCache3_6_0;
struct LightProbeVisibilityData3_6_0;
struct LightgroupMembership3_6_0;
struct LineStyleAlphaModifier_AlongStroke3_6_0;
struct LineStyleAlphaModifier_CreaseAngle3_6_0;
struct LineStyleAlphaModifier_Curvature_3D3_6_0;
struct LineStyleAlphaModifier_DistanceFromCamera3_6_0;
struct LineStyleAlphaModifier_DistanceFromObject3_6_0;
struct LineStyleAlphaModifier_Material3_6_0;
struct LineStyleAlphaModifier_Noise3_6_0;
struct LineStyleAlphaModifier_Tangent3_6_0;
struct LineStyleColorModifier_AlongStroke3_6_0;
struct LineStyleColorModifier_CreaseAngle3_6_0;
struct LineStyleColorModifier_Curvature_3D3_6_0;
struct LineStyleColorModifier_DistanceFromCamera3_6_0;
struct LineStyleColorModifier_DistanceFromObject3_6_0;
struct LineStyleColorModifier_Material3_6_0;
struct LineStyleColorModifier_Noise3_6_0;
struct LineStyleColorModifier_Tangent3_6_0;
struct LineStyleGeometryModifier_2DOffset3_6_0;
struct LineStyleGeometryModifier_2DTransform3_6_0;
struct LineStyleGeometryModifier_BackboneStretcher3_6_0;
struct LineStyleGeometryModifier_BezierCurve3_6_0;
struct LineStyleGeometryModifier_Blueprint3_6_0;
struct LineStyleGeometryModifier_GuidingLines3_6_0;
struct LineStyleGeometryModifier_PerlinNoise1D3_6_0;
struct LineStyleGeometryModifier_PerlinNoise2D3_6_0;
struct LineStyleGeometryModifier_Polygonalization3_6_0;
struct LineStyleGeometryModifier_Sampling3_6_0;
struct LineStyleGeometryModifier_Simplification3_6_0;
struct LineStyleGeometryModifier_SinusDisplacement3_6_0;
struct LineStyleGeometryModifier_SpatialNoise3_6_0;
struct LineStyleGeometryModifier_TipRemover3_6_0;
struct LineStyleModifier3_6_0;
struct LineStyleThicknessModifier_AlongStroke3_6_0;
struct LineStyleThicknessModifier_Calligraphy3_6_0;
struct LineStyleThicknessModifier_CreaseAngle3_6_0;
struct LineStyleThicknessModifier_Curvature_3D3_6_0;
struct LineStyleThicknessModifier_DistanceFromCamera3_6_0;
struct LineStyleThicknessModifier_DistanceFromObject3_6_0;
struct LineStyleThicknessModifier_Material3_6_0;
struct LineStyleThicknessModifier_Noise3_6_0;
struct LineStyleThicknessModifier_Tangent3_6_0;
struct LineartGpencilModifierData3_6_0;
struct Link3_6_0;
struct LinkData3_6_0;
struct ListBase3_6_0;
struct MBoolProperty3_6_0;
struct MCol3_6_0;
struct MDefCell3_6_0;
struct MDefInfluence3_6_0;
struct MDeformVert3_6_0;
struct MDeformWeight3_6_0;
struct MDisps3_6_0;
struct MFace3_6_0;
struct MFloatProperty3_6_0;
struct MInt8Property3_6_0;
struct MIntProperty3_6_0;
struct MLoopCol3_6_0;
struct MLoopTri3_6_0;
struct MPropCol3_6_0;
struct MRecast3_6_0;
struct MSelect3_6_0;
struct MStringProperty3_6_0;
struct MTFace3_6_0;
struct MTex3_6_0;
struct MVertSkin3_6_0;
struct MVertTri3_6_0;
struct MappingInfoModifierData3_6_0;
struct Mask3_6_0;
struct MaskLayer3_6_0;
struct MaskLayerShape3_6_0;
struct MaskLayerShapeElem3_6_0;
struct MaskModifierData3_6_0;
struct MaskParent3_6_0;
struct MaskSpaceInfo3_6_0;
struct MaskSpline3_6_0;
struct MaskSplinePoint3_6_0;
struct MaskSplinePointUW3_6_0;
struct Material3_6_0;
struct MaterialGPencilStyle3_6_0;
struct MaterialLineArt3_6_0;
struct Mesh3_6_0;
struct MeshCacheModifierData3_6_0;
struct MeshSeqCacheModifierData3_6_0;
struct MeshStatVis3_6_0;
struct MeshToVolumeModifierData3_6_0;
struct MetaBall3_6_0;
struct MetaElem3_6_0;
struct MetaStack3_6_0;
struct MirrorGpencilModifierData3_6_0;
struct MirrorModifierData3_6_0;
struct ModifierData3_6_0;
struct ModifierViewerPathElem3_6_0;
struct MovieClip3_6_0;
struct MovieClipProxy3_6_0;
struct MovieClipScopes3_6_0;
struct MovieClipUser3_6_0;
struct MovieClip_Runtime3_6_0;
struct MovieClip_RuntimeGPUTexture3_6_0;
struct MovieReconstructedCamera3_6_0;
struct MovieTracking3_6_0;
struct MovieTrackingCamera3_6_0;
struct MovieTrackingDopesheet3_6_0;
struct MovieTrackingDopesheetChannel3_6_0;
struct MovieTrackingDopesheetCoverageSegment3_6_0;
struct MovieTrackingMarker3_6_0;
struct MovieTrackingObject3_6_0;
struct MovieTrackingPlaneMarker3_6_0;
struct MovieTrackingPlaneTrack3_6_0;
struct MovieTrackingReconstruction3_6_0;
struct MovieTrackingSettings3_6_0;
struct MovieTrackingStabilization3_6_0;
struct MovieTrackingStats3_6_0;
struct MovieTrackingTrack3_6_0;
struct MultiplyGpencilModifierData3_6_0;
struct MultiresModifierData3_6_0;
struct NlaStrip3_6_0;
struct NlaTrack3_6_0;
struct NodeAccumulateField3_6_0;
struct NodeAntiAliasingData3_6_0;
struct NodeBilateralBlurData3_6_0;
struct NodeBlurData3_6_0;
struct NodeBokehImage3_6_0;
struct NodeBoxMask3_6_0;
struct NodeCMPCombSepColor3_6_0;
struct NodeChroma3_6_0;
struct NodeColorBalance3_6_0;
struct NodeColorCorrection3_6_0;
struct NodeColorspill3_6_0;
struct NodeCombSepColor3_6_0;
struct NodeConvertColorSpace3_6_0;
struct NodeCryptomatte3_6_0;
struct NodeCryptomatte_Runtime3_6_0;
struct NodeDBlurData3_6_0;
struct NodeDefocus3_6_0;
struct NodeDenoise3_6_0;
struct NodeDilateErode3_6_0;
struct NodeEllipseMask3_6_0;
struct NodeFrame3_6_0;
struct NodeFunctionCompare3_6_0;
struct NodeGeometryAttributeCapture3_6_0;
struct NodeGeometryCollectionInfo3_6_0;
struct NodeGeometryCurveFill3_6_0;
struct NodeGeometryCurveFillet3_6_0;
struct NodeGeometryCurvePrimitiveArc3_6_0;
struct NodeGeometryCurvePrimitiveBezierSegment3_6_0;
struct NodeGeometryCurvePrimitiveCircle3_6_0;
struct NodeGeometryCurvePrimitiveLine3_6_0;
struct NodeGeometryCurvePrimitiveQuad3_6_0;
struct NodeGeometryCurveResample3_6_0;
struct NodeGeometryCurveSample3_6_0;
struct NodeGeometryCurveSelectHandles3_6_0;
struct NodeGeometryCurveSetHandles3_6_0;
struct NodeGeometryCurveSplineType3_6_0;
struct NodeGeometryCurveToPoints3_6_0;
struct NodeGeometryCurveTrim3_6_0;
struct NodeGeometryDeleteGeometry3_6_0;
struct NodeGeometryDistributePointsInVolume3_6_0;
struct NodeGeometryDuplicateElements3_6_0;
struct NodeGeometryExtrudeMesh3_6_0;
struct NodeGeometryImageTexture3_6_0;
struct NodeGeometryInputNamedAttribute3_6_0;
struct NodeGeometryMergeByDistance3_6_0;
struct NodeGeometryMeshCircle3_6_0;
struct NodeGeometryMeshCone3_6_0;
struct NodeGeometryMeshCylinder3_6_0;
struct NodeGeometryMeshLine3_6_0;
struct NodeGeometryMeshToPoints3_6_0;
struct NodeGeometryMeshToVolume3_6_0;
struct NodeGeometryObjectInfo3_6_0;
struct NodeGeometryPointsToVolume3_6_0;
struct NodeGeometryProximity3_6_0;
struct NodeGeometryRaycast3_6_0;
struct NodeGeometrySampleIndex3_6_0;
struct NodeGeometrySampleVolume3_6_0;
struct NodeGeometrySeparateGeometry3_6_0;
struct NodeGeometrySetCurveHandlePositions3_6_0;
struct NodeGeometrySimulationInput3_6_0;
struct NodeGeometrySimulationOutput3_6_0;
struct NodeGeometryStoreNamedAttribute3_6_0;
struct NodeGeometryStringToCurves3_6_0;
struct NodeGeometrySubdivisionSurface3_6_0;
struct NodeGeometryTransferAttribute3_6_0;
struct NodeGeometryUVUnwrap3_6_0;
struct NodeGeometryViewer3_6_0;
struct NodeGeometryVolumeToMesh3_6_0;
struct NodeGlare3_6_0;
struct NodeHueSat3_6_0;
struct NodeImageAnim3_6_0;
struct NodeImageFile3_6_0;
struct NodeImageLayer3_6_0;
struct NodeImageMultiFile3_6_0;
struct NodeImageMultiFileSocket3_6_0;
struct NodeInputBool3_6_0;
struct NodeInputColor3_6_0;
struct NodeInputInt3_6_0;
struct NodeInputString3_6_0;
struct NodeInputVector3_6_0;
struct NodeKeyingData3_6_0;
struct NodeKeyingScreenData3_6_0;
struct NodeLensDist3_6_0;
struct NodeMapRange3_6_0;
struct NodeMask3_6_0;
struct NodePlaneTrackDeformData3_6_0;
struct NodeRandomValue3_6_0;
struct NodeScriptDict3_6_0;
struct NodeSetAlpha3_6_0;
struct NodeShaderAttribute3_6_0;
struct NodeShaderMix3_6_0;
struct NodeShaderNormalMap3_6_0;
struct NodeShaderOutputAOV3_6_0;
struct NodeShaderPrincipled3_6_0;
struct NodeShaderScript3_6_0;
struct NodeShaderTangent3_6_0;
struct NodeShaderTexIES3_6_0;
struct NodeShaderUVMap3_6_0;
struct NodeShaderVectTransform3_6_0;
struct NodeShaderVertexColor3_6_0;
struct NodeSimulationItem3_6_0;
struct NodeSunBeams3_6_0;
struct NodeSwitch3_6_0;
struct NodeTonemap3_6_0;
struct NodeTrackPosData3_6_0;
struct NodeTranslateData3_6_0;
struct NodeTwoFloats3_6_0;
struct NodeTwoXYs3_6_0;
struct NodeVertexCol3_6_0;
struct NodeViewerPathElem3_6_0;
struct NodesModifierData3_6_0;
struct NodesModifierSettings3_6_0;
struct NoiseGpencilModifierData3_6_0;
struct NormalEditModifierData3_6_0;
struct Nurb3_6_0;
struct ObHook3_6_0;
struct Object3_6_0;
struct ObjectLineArt3_6_0;
struct Object_Runtime3_6_0;
struct OceanModifierData3_6_0;
struct OffsetGpencilModifierData3_6_0;
struct OpacityGpencilModifierData3_6_0;
struct OrigSpaceFace3_6_0;
struct OrigSpaceLoop3_6_0;
struct OutlineGpencilModifierData3_6_0;
struct PTCacheExtra3_6_0;
struct PTCacheMem3_6_0;
struct PackedFile3_6_0;
struct Paint3_6_0;
struct PaintCurve3_6_0;
struct PaintCurvePoint3_6_0;
struct PaintModeSettings3_6_0;
struct PaintToolSlot3_6_0;
struct Paint_Runtime3_6_0;
struct Palette3_6_0;
struct PaletteColor3_6_0;
struct Panel3_6_0;
struct PanelCategoryDyn3_6_0;
struct PanelCategoryStack3_6_0;
struct Panel_Runtime3_6_0;
struct PartDeflect3_6_0;
struct PartEff3_6_0;
struct Particle3_6_0;
struct ParticleBrushData3_6_0;
struct ParticleData3_6_0;
struct ParticleDupliWeight3_6_0;
struct ParticleInstanceModifierData3_6_0;
struct ParticleKey3_6_0;
struct ParticleSettings3_6_0;
struct ParticleSpring3_6_0;
struct ParticleSystemModifierData3_6_0;
struct ParticleTarget3_6_0;
struct PhysicsSettings3_6_0;
struct PixelShaderFxData3_6_0;
struct PointCloud3_6_0;
struct PointDensity3_6_0;
struct PreviewImage3_6_0;
struct RegionView3D3_6_0;
struct RemeshModifierData3_6_0;
struct RenderProfile3_6_0;
struct RenderSlot3_6_0;
struct Report3_6_0;
struct ReportList3_6_0;
struct ReportTimerInfo3_6_0;
struct RigidBodyCon3_6_0;
struct RigidBodyOb3_6_0;
struct RigidBodyOb_Shared3_6_0;
struct RigidBodyWorld3_6_0;
struct RigidBodyWorld_Shared3_6_0;
struct RimShaderFxData3_6_0;
struct SBVertex3_6_0;
struct SDNA_StructMember3_6_0;
struct SDefBind3_6_0;
struct SDefVert3_6_0;
struct SPHFluidSettings3_6_0;
struct SceneCollection3_6_0;
struct SceneDisplay3_6_0;
struct SceneEEVEE3_6_0;
struct SceneGpencil3_6_0;
struct SceneRenderLayer3_6_0;
struct SceneRenderView3_6_0;
struct Scopes3_6_0;
struct ScrArea3_6_0;
struct ScrAreaMap3_6_0;
struct ScrArea_Runtime3_6_0;
struct ScrEdge3_6_0;
struct ScrGlobalAreaData3_6_0;
struct ScrVert3_6_0;
struct ScrewModifierData3_6_0;
struct Script3_6_0;
struct Sculpt3_6_0;
struct SeqTimelineChannel3_6_0;
struct Sequence3_6_0;
struct SequenceModifierData3_6_0;
struct SequenceRuntime3_6_0;
struct SequencerMaskModifierData3_6_0;
struct SequencerPreviewOverlay3_6_0;
struct SequencerScopes3_6_0;
struct SequencerTimelineOverlay3_6_0;
struct SequencerTonemapModifierData3_6_0;
struct SequencerToolSettings3_6_0;
struct SessionUUID3_6_0;
struct ShaderFxData3_6_0;
struct ShaderFxData_Runtime3_6_0;
struct ShadowShaderFxData3_6_0;
struct ShapeKeyModifierData3_6_0;
struct ShrinkwrapGpencilModifierData3_6_0;
struct ShrinkwrapModifierData3_6_0;
struct SimpleDeformModifierData3_6_0;
struct SimplifyGpencilModifierData3_6_0;
struct Simulation3_6_0;
struct SkinModifierData3_6_0;
struct SmokeModifierData3_6_0;
struct SmoothGpencilModifierData3_6_0;
struct SmoothModifierData3_6_0;
struct SoftBody3_6_0;
struct SoftBody_Shared3_6_0;
struct SoftbodyModifierData3_6_0;
struct SolidColorVars3_6_0;
struct SolidLight3_6_0;
struct SolidifyModifierData3_6_0;
struct SpaceAction3_6_0;
struct SpaceAction_Runtime3_6_0;
struct SpaceClip3_6_0;
struct SpaceConsole3_6_0;
struct SpaceFile3_6_0;
struct SpaceGraph3_6_0;
struct SpaceGraph_Runtime3_6_0;
struct SpaceImage3_6_0;
struct SpaceImageOverlay3_6_0;
struct SpaceInfo3_6_0;
struct SpaceLink3_6_0;
struct SpaceNla3_6_0;
struct SpaceNode3_6_0;
struct SpaceNodeOverlay3_6_0;
struct SpaceOutliner3_6_0;
struct SpaceProperties3_6_0;
struct SpaceScript3_6_0;
struct SpaceSeq3_6_0;
struct SpaceSeqRuntime3_6_0;
struct SpaceSpreadsheet3_6_0;
struct SpaceStatusBar3_6_0;
struct SpaceText3_6_0;
struct SpaceText_Runtime3_6_0;
struct SpaceTopBar3_6_0;
struct SpaceUserPref3_6_0;
struct Speaker3_6_0;
struct SpeedControlVars3_6_0;
struct SpreadsheetColumn3_6_0;
struct SpreadsheetColumnID3_6_0;
struct SpreadsheetRowFilter3_6_0;
struct Stereo3dFormat3_6_0;
struct Strip3_6_0;
struct StripAnim3_6_0;
struct StripColorBalance3_6_0;
struct StripCrop3_6_0;
struct StripElem3_6_0;
struct StripProxy3_6_0;
struct StripTransform3_6_0;
struct SubdivGpencilModifierData3_6_0;
struct SubsurfModifierData3_6_0;
struct SurfaceDeformModifierData3_6_0;
struct SurfaceModifierData3_6_0;
struct SurfaceModifierData_Runtime3_6_0;
struct SwirlShaderFxData3_6_0;
struct Tex3_6_0;
struct TexMapping3_6_0;
struct TexNodeOutput3_6_0;
struct TexPaintSlot3_6_0;
struct Text3_6_0;
struct TextBox3_6_0;
struct TextLine3_6_0;
struct TextVars3_6_0;
struct TextureGpencilModifierData3_6_0;
struct ThemeCollectionColor3_6_0;
struct ThemeSpace3_6_0;
struct ThemeStripColor3_6_0;
struct ThemeUI3_6_0;
struct ThemeWireColor3_6_0;
struct ThickGpencilModifierData3_6_0;
struct TimeGpencilModifierData3_6_0;
struct TimeGpencilModifierSegment3_6_0;
struct TimeMarker3_6_0;
struct TintGpencilModifierData3_6_0;
struct TransformOrientation3_6_0;
struct TransformOrientationSlot3_6_0;
struct TransformVars3_6_0;
struct TreeStore3_6_0;
struct TreeStoreElem3_6_0;
struct TriangulateModifierData3_6_0;
struct UVProjectModifierData3_6_0;
struct UVWarpModifierData3_6_0;
struct UnifiedPaintSettings3_6_0;
struct UnitSettings3_6_0;
struct UserDef_Experimental3_6_0;
struct UserDef_FileSpaceData3_6_0;
struct UserDef_Runtime3_6_0;
struct UserDef_SpaceData3_6_0;
struct UvSculpt3_6_0;
struct VFont3_6_0;
struct VPaint3_6_0;
struct View2D3_6_0;
struct View3D3_6_0;
struct View3DCursor3_6_0;
struct View3DOverlay3_6_0;
struct View3DShading3_6_0;
struct View3D_Runtime3_6_0;
struct ViewLayer3_6_0;
struct ViewLayerAOV3_6_0;
struct ViewLayerEEVEE3_6_0;
struct ViewLayerLightgroup3_6_0;
struct ViewerPath3_6_0;
struct ViewerPathElem3_6_0;
struct Volume3_6_0;
struct VolumeDisplaceModifierData3_6_0;
struct VolumeDisplay3_6_0;
struct VolumeRender3_6_0;
struct VolumeToMeshModifierData3_6_0;
struct Volume_Runtime3_6_0;
struct WalkNavigation3_6_0;
struct WarpModifierData3_6_0;
struct WaveEff3_6_0;
struct WaveModifierData3_6_0;
struct WaveShaderFxData3_6_0;
struct WeightAngleGpencilModifierData3_6_0;
struct WeightProxGpencilModifierData3_6_0;
struct WeightVGEditModifierData3_6_0;
struct WeightVGMixModifierData3_6_0;
struct WeightVGProximityModifierData3_6_0;
struct WeightedNormalModifierData3_6_0;
struct WeldModifierData3_6_0;
struct WhiteBalanceModifierData3_6_0;
struct WipeVars3_6_0;
struct WireframeModifierData3_6_0;
struct WorkSpace3_6_0;
struct WorkSpaceDataRelation3_6_0;
struct WorkSpaceInstanceHook3_6_0;
struct WorkSpaceLayout3_6_0;
struct World3_6_0;
struct XrActionMap3_6_0;
struct XrActionMapBinding3_6_0;
struct XrActionMapItem3_6_0;
struct XrComponentPath3_6_0;
struct XrSessionSettings3_6_0;
struct XrUserPath3_6_0;
struct bAction3_6_0;
struct bActionChannel3_6_0;
struct bActionConstraint3_6_0;
struct bActionGroup3_6_0;
struct bActionModifier3_6_0;
struct bActionStrip3_6_0;
struct bAddon3_6_0;
struct bAnimVizSettings3_6_0;
struct bArmature3_6_0;
struct bArmatureConstraint3_6_0;
struct bCameraSolverConstraint3_6_0;
struct bChildOfConstraint3_6_0;
struct bClampToConstraint3_6_0;
struct bConstraint3_6_0;
struct bConstraintChannel3_6_0;
struct bConstraintTarget3_6_0;
struct bDampTrackConstraint3_6_0;
struct bDeformGroup3_6_0;
struct bDistLimitConstraint3_6_0;
struct bDopeSheet3_6_0;
struct bFaceMap3_6_0;
struct bFollowPathConstraint3_6_0;
struct bFollowTrackConstraint3_6_0;
struct bGPDcontrolpoint3_6_0;
struct bGPDcurve3_6_0;
struct bGPDcurve_point3_6_0;
struct bGPDframe3_6_0;
struct bGPDframe_Runtime3_6_0;
struct bGPDlayer3_6_0;
struct bGPDlayer_Mask3_6_0;
struct bGPDlayer_Runtime3_6_0;
struct bGPDpalette3_6_0;
struct bGPDpalettecolor3_6_0;
struct bGPDspoint3_6_0;
struct bGPDspoint_Runtime3_6_0;
struct bGPDstroke3_6_0;
struct bGPDstroke_Runtime3_6_0;
struct bGPDtriangle3_6_0;
struct bGPdata3_6_0;
struct bGPdata_Runtime3_6_0;
struct bGPgrid3_6_0;
struct bIKParam3_6_0;
struct bItasc3_6_0;
struct bKinematicConstraint3_6_0;
struct bLocLimitConstraint3_6_0;
struct bLocateLikeConstraint3_6_0;
struct bLockTrackConstraint3_6_0;
struct bMinMaxConstraint3_6_0;
struct bMotionPath3_6_0;
struct bMotionPathVert3_6_0;
struct bNode3_6_0;
struct bNodeInstanceHashEntry3_6_0;
struct bNodeInstanceKey3_6_0;
struct bNodeLink3_6_0;
struct bNodePreview3_6_0;
struct bNodeSocket3_6_0;
struct bNodeSocketValueBoolean3_6_0;
struct bNodeSocketValueCollection3_6_0;
struct bNodeSocketValueFloat3_6_0;
struct bNodeSocketValueImage3_6_0;
struct bNodeSocketValueInt3_6_0;
struct bNodeSocketValueMaterial3_6_0;
struct bNodeSocketValueObject3_6_0;
struct bNodeSocketValueRGBA3_6_0;
struct bNodeSocketValueString3_6_0;
struct bNodeSocketValueTexture3_6_0;
struct bNodeSocketValueVector3_6_0;
struct bNodeStack3_6_0;
struct bNodeTree3_6_0;
struct bNodeTreePath3_6_0;
struct bObjectSolverConstraint3_6_0;
struct bPathCompare3_6_0;
struct bPivotConstraint3_6_0;
struct bPose3_6_0;
struct bPoseChannel3_6_0;
struct bPoseChannelDrawData3_6_0;
struct bPoseChannel_Runtime3_6_0;
struct bPythonConstraint3_6_0;
struct bRigidBodyJointConstraint3_6_0;
struct bRotLimitConstraint3_6_0;
struct bRotateLikeConstraint3_6_0;
struct bSameVolumeConstraint3_6_0;
struct bScreen3_6_0;
struct bShrinkwrapConstraint3_6_0;
struct bSizeLikeConstraint3_6_0;
struct bSizeLimitConstraint3_6_0;
struct bSound3_6_0;
struct bSplineIKConstraint3_6_0;
struct bStretchToConstraint3_6_0;
struct bToolRef3_6_0;
struct bToolRef_Runtime3_6_0;
struct bTrackToConstraint3_6_0;
struct bTransLikeConstraint3_6_0;
struct bTransformCacheConstraint3_6_0;
struct bTransformConstraint3_6_0;
struct bUUID3_6_0;
struct bUserAssetLibrary3_6_0;
struct bUserMenu3_6_0;
struct bUserMenuItem3_6_0;
struct bUserMenuItem_Menu3_6_0;
struct bUserMenuItem_Op3_6_0;
struct bUserMenuItem_Prop3_6_0;
struct bUserScriptDirectory3_6_0;
struct rctf3_6_0;
struct rcti3_6_0;
struct tPaletteColorHSV3_6_0;
struct uiFont3_6_0;
struct uiFontStyle3_6_0;
struct uiList3_6_0;
struct uiPanelColors3_6_0;
struct uiPreview3_6_0;
struct uiStyle3_6_0;
struct uiWidgetColors3_6_0;
struct uiWidgetStateColors3_6_0;
struct vec2f3_6_0;
struct vec2i3_6_0;
struct vec2s3_6_0;
struct vec3f3_6_0;
struct wmKeyConfig3_6_0;
struct wmKeyConfigPref3_6_0;
struct wmKeyMapDiffItem3_6_0;
struct wmKeyMapItem3_6_0;
struct wmOperator3_6_0;
struct wmOperatorTypeMacro3_6_0;
struct wmOwnerID3_6_0;
struct wmWindow3_6_0;
struct wmWindowManager3_6_0;
struct wmXrData3_6_0;

struct Link3_6_0 {
    struct Link3_6_0 *next;
    struct Link3_6_0 *prev;
};

struct ListBase3_6_0 {
    void *first;
    void *last;
};

struct Library_Runtime3_6_0 {
    void *name_map;
};

struct vec2s3_6_0 {
    short x;
    short y;
};

struct vec2i3_6_0 {
    int x;
    int y;
};

struct vec2f3_6_0 {
    float x;
    float y;
};

struct vec3f3_6_0 {
    float x;
    float y;
    float z;
};

struct rcti3_6_0 {
    int xmin;
    int xmax;
    int ymin;
    int ymax;
};

struct rctf3_6_0 {
    float xmin;
    float xmax;
    float ymin;
    float ymax;
};

struct bAnimVizSettings3_6_0 {
    short recalc;
    short path_type;
    short path_step;
    short path_range;
    short path_viewflag;
    short path_bakeflag;
    char _pad[4];
    int path_sf;
    int path_ef;
    int path_bc;
    int path_ac;
};

struct SpaceAction_Runtime3_6_0 {
    char flag;
    char _pad0[7];
};

struct GPUDOFSettings3_6_0 {
    float focus_distance;
    float fstop;
    float focal_length;
    float sensor;
    float rotation;
    float ratio;
    int num_blades;
    int high_quality;
};

struct CameraDOFSettings3_6_0 {
    void *focus_object;
    char focus_subtarget[64];
    float focus_distance;
    float aperture_fstop;
    float aperture_rotation;
    float aperture_ratio;
    int aperture_blades;
    short flag;
    char _pad[2];
};

struct Camera_Runtime3_6_0 {
    float drw_corners[2][4][2];
    float drw_tria[2][2];
    float drw_depth[2];
    float drw_focusmat[4][4];
    float drw_normalmat[4][4];
};

struct CameraStereoSettings3_6_0 {
    float interocular_distance;
    float convergence_distance;
    short convergence_mode;
    short pivot;
    short flag;
    char _pad[2];
    float pole_merge_angle_from;
    float pole_merge_angle_to;
};

struct bDeformGroup3_6_0 {
    struct bDeformGroup3_6_0 *next;
    struct bDeformGroup3_6_0 *prev;
    char name[64];
    char flag;
    char _pad0[7];
};

struct BoundBox3_6_0 {
    float vec[3][8];
    int flag;
    char _pad0[4];
};

struct DrawDataList3_6_0 {
    void *first;
    void *last;
};

struct ObjectLineArt3_6_0 {
    short usage;
    short flags;
    float crease_threshold;
    unsigned char intersection_priority;
    char _pad[7];
};

struct bConstraintChannel3_6_0 {
    struct bConstraintChannel3_6_0 *next;
    struct bConstraintChannel3_6_0 *prev;
    void *ipo;
    short flag;
    char name[30];
};

struct bConstraint3_6_0 {
    struct bConstraint3_6_0 *next;
    struct bConstraint3_6_0 *prev;
    void *data;
    short type;
    short flag;
    char ownspace;
    char tarspace;
    short ui_expand_flag;
    void *space_object;
    char space_subtarget[64];
    char name[64];
    float enforce;
    float headtail;
    void *ipo;
    float lin_error;
    float rot_error;
};

struct bKinematicConstraint3_6_0 {
    void *tar;
    short iterations;
    short flag;
    short rootbone;
    short max_rootbone;
    char subtarget[64];
    void *poletar;
    char polesubtarget[64];
    float poleangle;
    float weight;
    float orientweight;
    float grabtarget[3];
    short type;
    short mode;
    float dist;
};

struct bTrackToConstraint3_6_0 {
    void *tar;
    int reserved1;
    int reserved2;
    int flags;
    char _pad[4];
    char subtarget[64];
};

struct bRotateLikeConstraint3_6_0 {
    void *tar;
    int flag;
    char euler_order;
    char mix_mode;
    char _pad[2];
    char subtarget[64];
};

struct bLocateLikeConstraint3_6_0 {
    void *tar;
    int flag;
    int reserved1;
    char subtarget[64];
};

struct bActionConstraint3_6_0 {
    void *tar;
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
    void *act;
    char subtarget[64];
};

struct bFollowPathConstraint3_6_0 {
    void *tar;
    float offset;
    float offset_fac;
    int followflag;
    short trackflag;
    short upflag;
};

struct BevList3_6_0 {
    struct BevList3_6_0 *next;
    struct BevList3_6_0 *prev;
    int nr;
    int dupe_nr;
    int poly;
    int hole;
    int charidx;
    int *segbevcount;
    float *seglen;
    struct BevPoint3_6_0 *bevpoints;
};

struct BevPoint3_6_0 {
    float vec[3];
    float tilt;
    float radius;
    float weight;
    float offset;
    float sina;
    float cosa;
    float dir[3];
    float tan[3];
    float quat[4];
    short dupe_tag;
};

struct BezTriple3_6_0 {
    float vec[3][3];
    float tilt;
    float weight;
    float radius;
    char ipo;
    unsigned char h1;
    unsigned char h2;
    unsigned char f1;
    unsigned char f2;
    unsigned char f3;
    char hide;
    char easing;
    float back;
    float amplitude;
    float period;
    char auto_handle_type;
    char _pad[3];
};

struct BPoint3_6_0 {
    float vec[4];
    float tilt;
    float weight;
    unsigned char f1;
    char _pad1[1];
    short hide;
    float radius;
    char _pad[4];
};

struct Nurb3_6_0 {
    struct Nurb3_6_0 *next;
    struct Nurb3_6_0 *prev;
    short type;
    short mat_nr;
    short hide;
    short flag;
    int pntsu;
    int pntsv;
    char _pad[4];
    short resolu;
    short resolv;
    short orderu;
    short orderv;
    short flagu;
    short flagv;
    float *knotsu;
    float *knotsv;
    struct BPoint3_6_0 *bp;
    struct BezTriple3_6_0 *bezt;
    short tilt_interp;
    short radius_interp;
    int charidx;
};

struct CharInfo3_6_0 {
    float kern;
    short mat_nr;
    char flag;
    char _pad[1];
};

struct Effect3_6_0 {
    struct Effect3_6_0 *next;
    struct Effect3_6_0 *prev;
    short type;
    short flag;
    short buttype;
    char _pad0[2];
};

struct BuildEff3_6_0 {
    struct BuildEff3_6_0 *next;
    struct BuildEff3_6_0 *prev;
    short type;
    short flag;
    short buttype;
    char _pad0[2];
    float len;
    float sfra;
};

struct Particle3_6_0 {
    float co[3];
    float no[3];
    float time;
    float lifetime;
    short mat_nr;
    char _pad0[2];
};

struct PartEff3_6_0 {
    struct PartEff3_6_0 *next;
    struct PartEff3_6_0 *prev;
    short type;
    short flag;
    short buttype;
    short stype;
    short vertgroup;
    short userjit;
    float sta;
    float end;
    float lifetime;
    int totpart;
    int totkey;
    int seed;
    float normfac;
    float obfac;
    float randfac;
    float texfac;
    float randlife;
    float force[3];
    float damp;
    float nabla;
    float vectsize;
    float maxlen;
    float defvec[3];
    char _pad[4];
    float mult[4];
    float life[4];
    short child[4];
    short mat[4];
    short texmap;
    short curmult;
    short staticstep;
    short omat;
    short timetex;
    short speedtex;
    short flag2;
    short flag2neg;
    short disp;
    short vertgroup_v;
    char vgroupname[64];
    char vgroupname_v[64];
    float imat[4][4];
    struct Particle3_6_0 *keys;
    void *group;
};

struct WaveEff3_6_0 {
    struct WaveEff3_6_0 *next;
    struct WaveEff3_6_0 *prev;
    short type;
    short flag;
    short buttype;
    short stype;
    float startx;
    float starty;
    float height;
    float width;
    float narrow;
    float speed;
    float minfac;
    float damp;
    float timeoffs;
    float lifetime;
};

struct FileGlobal3_6_0 {
    char subvstr[4];
    short subversion;
    short minversion;
    short minsubversion;
    char _pad[6];
    void *curscreen;
    void *curscene;
    void *cur_view_layer;
    void *_pad1;
    int fileflags;
    int globalf;
    int build_commit_timestamp;
    char build_hash[16];
    char filepath[1024];
};

struct Image_Runtime3_6_0 {
    void *cache_mutex;
    void *partial_update_register;
    void *partial_update_user;
};

struct ColorManagedColorspaceSettings3_6_0 {
    char name[64];
};

struct KeyBlock3_6_0 {
    struct KeyBlock3_6_0 *next;
    struct KeyBlock3_6_0 *prev;
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

struct MaterialLineArt3_6_0 {
    int flags;
    unsigned char material_mask_bits;
    unsigned char mat_occlusion;
    unsigned char intersection_priority;
    char _pad;
};

struct MFace3_6_0 {
    unsigned int v1;
    unsigned int v2;
    unsigned int v3;
    unsigned int v4;
    short mat_nr;
    char edcode;
    char flag;
};

struct MDeformWeight3_6_0 {
    unsigned int def_nr;
    float weight;
};

struct MDeformVert3_6_0 {
    struct MDeformWeight3_6_0 *dw;
    int totweight;
    int flag;
};

struct MCol3_6_0 {
    unsigned char a;
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

struct CustomData3_6_0 {
    struct CustomDataLayer3_6_0 *layers;
    int typemap[52];
    char _pad[4];
    int totlayer;
    int maxlayer;
    int totsize;
    void *pool;
    struct CustomDataExternal3_6_0 *external;
};

struct MetaElem3_6_0 {
    struct MetaElem3_6_0 *next;
    struct MetaElem3_6_0 *prev;
    void *bb;
    short type;
    short flag;
    char _pad[4];
    float x;
    float y;
    float z;
    float quat[4];
    float expx;
    float expy;
    float expz;
    float rad;
    float rad2;
    float s;
    float len;
    float *mat;
    float *imat;
};

struct PackedFile3_6_0 {
    int size;
    int seek;
    void *data;
};

struct Base3_6_0 {
    struct Base3_6_0 *next;
    struct Base3_6_0 *prev;
    struct Object3_6_0 *object;
    struct Base3_6_0 *base_orig;
    unsigned int lay;
    short flag;
    short flag_from_collection;
    short flag_legacy;
    unsigned short local_view_bits;
    unsigned short local_collections_bits;
    char _pad1[2];
};

struct AviCodecData3_6_0 {
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

struct ScrEdge3_6_0 {
    struct ScrEdge3_6_0 *next;
    struct ScrEdge3_6_0 *prev;
    struct ScrVert3_6_0 *v1;
    struct ScrVert3_6_0 *v2;
    short border;
    short flag;
    char _pad[4];
};

struct ScrArea_Runtime3_6_0 {
    void *tool;
    char is_tool_set;
    char _pad0[7];
};

struct BHead3_6_0 {
    int code;
    int len;
    const  void *old;
    int SDNAnr;
    int nr;
};

struct BHead43_6_0 {
    int code;
    int len;
    int old;
    int SDNAnr;
    int nr;
};

struct BHead83_6_0 {
    int code;
    int len;
    int old;
    int SDNAnr;
    int nr;
};

struct StripElem3_6_0 {
    char filename[256];
    int orig_width;
    int orig_height;
    float orig_fps;
};

struct MetaStack3_6_0 {
    struct MetaStack3_6_0 *next;
    struct MetaStack3_6_0 *prev;
    struct ListBase3_6_0 *oldbasep;
    struct ListBase3_6_0 *old_channels;
    struct Sequence3_6_0 *parseq;
    int disp_range[2];
};

struct EditingRuntime3_6_0 {
    void *sequence_lookup;
};

struct SequencerScopes3_6_0 {
    void *reference_ibuf;
    void *zebra_ibuf;
    void *waveform_ibuf;
    void *sep_waveform_ibuf;
    void *vector_ibuf;
    void *histogram_ibuf;
};

struct SequencerTimelineOverlay3_6_0 {
    int flag;
    char _pad0[4];
};

struct SequencerPreviewOverlay3_6_0 {
    int flag;
    char _pad0[4];
};

struct MaskSpaceInfo3_6_0 {
    void *mask;
    char draw_flag;
    char draw_type;
    char overlay_mode;
    char _pad3[1];
    float blend_factor;
};

struct Histogram3_6_0 {
    int channels;
    int x_resolution;
    float data_luma[256];
    float data_r[256];
    float data_g[256];
    float data_b[256];
    float data_a[256];
    float xmax;
    float ymax;
    short mode;
    short flag;
    int height;
    float co[2][2];
};

struct SpaceImageOverlay3_6_0 {
    int flag;
    char _pad[4];
};

struct ImageUser3_6_0 {
    void *scene;
    int framenr;
    int frames;
    int offset;
    int sfra;
    char cycl;
    char multiview_eye;
    short pass;
    int tile;
    short multi_index;
    short view;
    short layer;
    short flag;
};

struct MTex3_6_0 {
    short texco;
    short mapto;
    short maptoneg;
    short blendtype;
    struct Object3_6_0 *object;
    struct Tex3_6_0 *tex;
    char uvname[68];
    char _pad1[4];
    char projx;
    char projy;
    char projz;
    char mapping;
    char brush_map_mode;
    char brush_angle_mode;
    char _pad[2];
    float ofs[3];
    float size[3];
    float rot;
    float random_angle;
    char _pad0[2];
    short colormodel;
    short normapspace;
    short which_output;
    float r;
    float g;
    float b;
    float k;
    float def_var;
    float colfac;
    float varfac;
    float norfac;
    float dispfac;
    float warpfac;
    float colspecfac;
    float mirrfac;
    float alphafac;
    float difffac;
    float specfac;
    float emitfac;
    float hardfac;
    float raymirrfac;
    float translfac;
    float ambfac;
    float colemitfac;
    float colreflfac;
    float coltransfac;
    float densfac;
    float scatterfac;
    float reflfac;
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
    float shadowfac;
    float zenupfac;
    float zendownfac;
    float blendfac;
};

struct CBData3_6_0 {
    float r;
    float g;
    float b;
    float a;
    float pos;
    int cur;
};

struct TextLine3_6_0 {
    struct TextLine3_6_0 *next;
    struct TextLine3_6_0 *prev;
    char *line;
    char *format;
    int len;
    char _pad0[4];
};

struct View3D_Runtime3_6_0 {
    void *properties_storage;
    int flag;
    char _pad1[4];
    void *local_stats;
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

struct View3DShading3_6_0 {
    char type;
    char prev_type;
    char prev_type_wire;
    char color_type;
    short flag;
    char light;
    char background_type;
    char cavity_type;
    char wire_color_type;
    char use_compositor;
    char _pad;
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
    struct IDProperty3_6_0 *prop;
    void *_pad2;
};

struct AudioData3_6_0 {
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

struct bLockTrackConstraint3_6_0 {
    void *tar;
    int trackflag;
    int lockflag;
    char subtarget[64];
};

struct Panel_Runtime3_6_0 {
    int region_ofsx;
    char _pad[4];
    void *custom_data_ptr;
    void *block;
    void *context;
};

struct uiWidgetStateColors3_6_0 {
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

struct uiWidgetColors3_6_0 {
    unsigned char outline[4];
    unsigned char inner[4];
    unsigned char inner_sel[4];
    unsigned char item[4];
    unsigned char text[4];
    unsigned char text_sel[4];
    unsigned char shaded;
    char _pad0[7];
    short shadetop;
    short shadedown;
    float roundness;
};

struct uiPanelColors3_6_0 {
    unsigned char header[4];
    unsigned char back[4];
    unsigned char sub_back[4];
    char _pad0[4];
};

struct SolidLight3_6_0 {
    int flag;
    float smooth;
    float col[4];
    float spec[4];
    float vec[4];
};

struct PartDeflect3_6_0 {
    int flag;
    short deflect;
    short forcefield;
    short falloff;
    short shape;
    short tex_mode;
    short kink;
    short kink_axis;
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
    float clump_fac;
    float clump_pow;
    float kink_freq;
    float kink_shape;
    float kink_amp;
    float free_end;
    float tex_nabla;
    void *tex;
    void *rng;
    float f_noise;
    int seed;
    float drawvec1[4];
    float drawvec2[4];
    float drawvec_falloff_min[3];
    char _pad1[4];
    float drawvec_falloff_max[3];
    char _pad2[4];
    void *f_source;
    float pdef_cfrict;
    char _pad[4];
};

struct WipeVars3_6_0 {
    float edgeWidth;
    float angle;
    short forward;
    short wipetype;
};

struct GlowVars3_6_0 {
    float fMini;
    float fClamp;
    float fBoost;
    float dDist;
    int dQuality;
    int bNoComp;
};

struct ObHook3_6_0 {
    struct ObHook3_6_0 *next;
    struct ObHook3_6_0 *prev;
    struct Object3_6_0 *parent;
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

struct bStretchToConstraint3_6_0 {
    void *tar;
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

struct TreeStoreElem3_6_0 {
    short type;
    short nr;
    short flag;
    short used;
    struct ID3_6_0 *id;
};

struct TreeStore3_6_0 {
    int totelem;
    int usedelem;
    struct TreeStoreElem3_6_0 *data;
};

struct SBVertex3_6_0 {
    float vec[4];
};

struct TimeMarker3_6_0 {
    struct TimeMarker3_6_0 *next;
    struct TimeMarker3_6_0 *prev;
    int frame;
    char name[64];
    unsigned int flag;
    struct Object3_6_0 *camera;
    struct IDProperty3_6_0 *prop;
};

struct bMinMaxConstraint3_6_0 {
    void *tar;
    int minmaxflag;
    float offset;
    int flag;
    char subtarget[64];
    int _pad;
};

struct TextBox3_6_0 {
    float x;
    float y;
    float w;
    float h;
};

struct IpoDriver3_6_0 {
    void *ob;
    short blocktype;
    short adrcode;
    short type;
    short flag;
    char name[128];
};

struct SessionUUID3_6_0 {
    unsigned long long uuid_;
};

struct FluidsimSettings3_6_0 {
    void *fmd;
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
    float animStart;
    float animEnd;
    int bakeStart;
    int bakeEnd;
    int frameOffset;
    char _pad2[4];
    float gstar;
    int maxRefine;
    float iniVelx;
    float iniVely;
    float iniVelz;
    char surfdataPath[1024];
    float bbStart[3];
    float bbSize[3];
    void *ipo;
    short typeFlags;
    char domainNovecgen;
    char volumeInitType;
    float partSlipValue;
    int generateTracers;
    float generateParticles;
    float surfaceSmoothing;
    int surfaceSubdivs;
    int flag;
    float particleInfSize;
    float particleInfAlpha;
    float farFieldSize;
    struct FluidVertexVelocity3_6_0 *meshVelocities;
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

struct CurveMapPoint3_6_0 {
    float x;
    float y;
    short flag;
    short shorty;
};

struct CurveMap3_6_0 {
    short totpoint;
    short flag;
    float range;
    float mintable;
    float maxtable;
    float ext_in[2];
    float ext_out[2];
    struct CurveMapPoint3_6_0 *curve;
    struct CurveMapPoint3_6_0 *table;
    struct CurveMapPoint3_6_0 *premultable;
    float premul_ext_in[2];
    float premul_ext_out[2];
};

struct bSizeLikeConstraint3_6_0 {
    void *tar;
    int flag;
    float power;
    char subtarget[64];
};

struct PreviewImage3_6_0 {
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

struct MSelect3_6_0 {
    int index;
    int type;
};

struct bNodeStack3_6_0 {
    float vec[4];
    float min;
    float max;
    void *data;
    short hasinput;
    short hasoutput;
    short datatype;
    short sockettype;
    short is_copy;
    short external;
    char _pad[4];
};

struct bNodeLink3_6_0 {
    struct bNodeLink3_6_0 *next;
    struct bNodeLink3_6_0 *prev;
    struct bNode3_6_0 *fromnode;
    struct bNode3_6_0 *tonode;
    struct bNodeSocket3_6_0 *fromsock;
    struct bNodeSocket3_6_0 *tosock;
    int flag;
    int multi_input_socket_index;
};

struct NodeImageAnim3_6_0 {
    int frames;
    int sfra;
    int nr;
    char cyclic;
    char movie;
    char _pad[2];
};

struct NodeBlurData3_6_0 {
    short sizex;
    short sizey;
    short samples;
    short maxspeed;
    short minspeed;
    short relative;
    short aspect;
    short curved;
    float fac;
    float percentx;
    float percenty;
    short filtertype;
    char bokeh;
    char gamma;
    int image_in_width;
    int image_in_height;
};

struct NodeHueSat3_6_0 {
    float hue;
    float sat;
    float val;
};

struct FFMpegCodecData3_6_0 {
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

struct SpaceNodeOverlay3_6_0 {
    int flag;
};

struct TexMapping3_6_0 {
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
    struct Object3_6_0 *ob;
};

struct BrushClone3_6_0 {
    struct Image3_6_0 *image;
    float offset[2];
    float alpha;
    char _pad[4];
};

struct bLocLimitConstraint3_6_0 {
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
    short flag;
    short flag2;
};

struct bRotLimitConstraint3_6_0 {
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
    short flag;
    short flag2;
    char euler_order;
    char _pad[3];
};

struct bSizeLimitConstraint3_6_0 {
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
    short flag;
    short flag2;
};

struct bRigidBodyJointConstraint3_6_0 {
    void *tar;
    void *child;
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
    const  void *anonymous_id;
    const  int *sharing_info;
};

struct MTFace3_6_0 {
    float uv[2][4];
};

struct bActionModifier3_6_0 {
    struct bActionModifier3_6_0 *next;
    struct bActionModifier3_6_0 *prev;
    short type;
    short flag;
    char channel[32];
    float noisesize;
    float turbul;
    short channels;
    short no_rot_axis;
    void *ob;
};

struct NodeChroma3_6_0 {
    float t1;
    float t2;
    float t3;
    float fsize;
    float fstrength;
    float falpha;
    float key[4];
    short algorithm;
    short channel;
};

struct NodeVertexCol3_6_0 {
    char name[64];
};

struct NodeDefocus3_6_0 {
    char bktype;
    char _pad0;
    char preview;
    char gamco;
    short samples;
    short no_zbuf;
    float fstop;
    float maxblur;
    float bthresh;
    float scale;
    float rotation;
    char _pad1[4];
};

struct TransformVars3_6_0 {
    float ScalexIni;
    float ScaleyIni;
    float xIni;
    float yIni;
    float rotIni;
    int percent;
    int interpolation;
    int uniform_scale;
};

struct SolidColorVars3_6_0 {
    float col[3];
    char _pad[4];
};

struct SpeedControlVars3_6_0 {
    float *frameMap;
    float globalSpeed;
    int flags;
    int speed_control_type;
    float speed_fader;
    float speed_fader_length;
    float speed_fader_frame_number;
};

struct bClampToConstraint3_6_0 {
    void *tar;
    int flag;
    int flag2;
};

struct LinkData3_6_0 {
    struct LinkData3_6_0 *next;
    struct LinkData3_6_0 *prev;
    void *data;
};

struct ThemeWireColor3_6_0 {
    unsigned char solid[4];
    unsigned char select[4];
    unsigned char active[4];
    short flag;
    char _pad0[2];
};

struct ClothSimSettings3_6_0 {
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
    void *effector_weights;
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

struct ClothCollSettings3_6_0 {
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
    void *group;
    short vgroup_selfcol;
    short vgroup_objcol;
    char _pad2[4];
    float clamp;
    float self_clamp;
};

struct bConstraintTarget3_6_0 {
    struct bConstraintTarget3_6_0 *next;
    struct bConstraintTarget3_6_0 *prev;
    void *tar;
    char subtarget[64];
    float matrix[4][4];
    short space;
    short flag;
    short type;
    short rotOrder;
    float weight;
    char _pad[4];
};

struct bChildOfConstraint3_6_0 {
    void *tar;
    int flag;
    char _pad[4];
    float invmat[4][4];
    char subtarget[64];
};

struct bTransformConstraint3_6_0 {
    void *tar;
    char subtarget[64];
    short from;
    short to;
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

struct bDistLimitConstraint3_6_0 {
    void *tar;
    char subtarget[64];
    float dist;
    float soft;
    short flag;
    short mode;
    char _pad[4];
};

struct MFloatProperty3_6_0 {
    float f;
};

struct MIntProperty3_6_0 {
    int i;
};

struct MStringProperty3_6_0 {
    char s[255];
    char s_len;
};

struct OrigSpaceFace3_6_0 {
    float uv[2][4];
};

struct MDefInfluence3_6_0 {
    int vertex;
    float weight;
};

struct MDefCell3_6_0 {
    int offset;
    int influences_num;
};

struct NodeDBlurData3_6_0 {
    float center_x;
    float center_y;
    float distance;
    float angle;
    float spin;
    float zoom;
    short iter;
    char _pad[2];
};

struct NodeBilateralBlurData3_6_0 {
    float sigma_color;
    float sigma_space;
    short iter;
    char _pad[2];
};

struct NodeTwoXYs3_6_0 {
    short x1;
    short x2;
    short y1;
    short y2;
    float fac_x1;
    float fac_x2;
    float fac_y1;
    float fac_y2;
};

struct NodeTwoFloats3_6_0 {
    float x;
    float y;
};

struct NodeScriptDict3_6_0 {
    void *dict;
    void *node;
};

struct NodeGlare3_6_0 {
    char quality;
    char type;
    char iter;
    char angle;
    char _pad0;
    char size;
    char star_45;
    char streaks;
    float colmod;
    float mix;
    float threshold;
    float fade;
    float angle_ofs;
    char _pad1[4];
};

struct NodeTonemap3_6_0 {
    float key;
    float offset;
    float gamma;
    float f;
    float m;
    float a;
    float c;
    int type;
};

struct NodeLensDist3_6_0 {
    short jit;
    short proj;
    short fit;
    char _pad[2];
};

struct HairKey3_6_0 {
    float co[3];
    float time;
    float weight;
    short editflag;
    char _pad[2];
    float world_co[3];
};

struct ParticleKey3_6_0 {
    float co[3];
    float vel[3];
    float rot[4];
    float ave[3];
    float time;
};

struct ChildParticle3_6_0 {
    int num;
    int parent;
    int pa[4];
    float w[4];
    float fuv[4];
    float foffset;
    char _pad0[4];
};

struct RenderProfile3_6_0 {
    struct RenderProfile3_6_0 *next;
    struct RenderProfile3_6_0 *prev;
    char name[32];
    short particle_perc;
    short subsurf_max;
    short shadbufsample_max;
    char _pad1[2];
    float ao_error;
    char _pad2[4];
};

struct ParticleBrushData3_6_0 {
    short size;
    short step;
    short invert;
    short count;
    int flag;
    float strength;
};

struct TransformOrientation3_6_0 {
    struct TransformOrientation3_6_0 *next;
    struct TransformOrientation3_6_0 *prev;
    char name[64];
    float mat[3][3];
    char _pad[4];
};

struct StripCrop3_6_0 {
    int top;
    int bottom;
    int left;
    int right;
};

struct StripTransform3_6_0 {
    float xofs;
    float yofs;
    float scale_x;
    float scale_y;
    float rotation;
    float origin[2];
    int filter;
};

struct StripColorBalance3_6_0 {
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

struct bGPDspoint_Runtime3_6_0 {
    struct bGPDspoint3_6_0 *pt_orig;
    int idx_orig;
    char _pad0[4];
};

struct bGPDstroke_Runtime3_6_0 {
    char tmp_layerinfo[128];
    float multi_frame_falloff;
    int stroke_start;
    int fill_start;
    int vertex_start;
    int curve_start;
    int _pad0;
    struct bGPDstroke3_6_0 *gps_orig;
    void *_pad2;
};

struct bGPDframe_Runtime3_6_0 {
    int frameid;
    int onion_id;
    struct bGPDframe3_6_0 *gpf_orig;
};

struct bGPDlayer_Runtime3_6_0 {
    int icon_id;
    char _pad[4];
    struct bGPDlayer3_6_0 *gpl_orig;
};

struct bGPgrid3_6_0 {
    float color[3];
    float scale[2];
    float offset[2];
    char _pad1[4];
    int lines;
    char _pad[4];
};

struct bGPdata_Runtime3_6_0 {
    void *sbuffer;
    void *sbuffer_position_buf;
    void *sbuffer_color_buf;
    void *sbuffer_batch;
    struct bGPDstroke3_6_0 *sbuffer_gps;
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
    struct Brush3_6_0 *sbuffer_brush;
    void *gpencil_cache;
    void *lineart_cache;
    void *update_cache;
};

struct MLoopCol3_6_0 {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
};

struct bShrinkwrapConstraint3_6_0 {
    void *target;
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

struct SurfaceModifierData_Runtime3_6_0 {
    float *(vert_positions_prev[3]);
    float *(vert_velocities[3]);
    void *mesh;
    void *bvhtree;
    int cfra_prev;
    int verts_num;
};

struct TexNodeOutput3_6_0 {
    char name[64];
};

struct PointDensity3_6_0 {
    short flag;
    short falloff_type;
    float falloff_softness;
    float radius;
    short source;
    char _pad0[2];
    short color_source;
    short ob_color_source;
    int totpoints;
    struct Object3_6_0 *object;
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

struct uiFont3_6_0 {
    struct uiFont3_6_0 *next;
    struct uiFont3_6_0 *prev;
    char filepath[1024];
    short blf_id;
    short uifont_id;
};

struct uiFontStyle3_6_0 {
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
    char _pad2[4];
};

struct bMotionPathVert3_6_0 {
    float co[3];
    int flag;
};

struct bMotionPath3_6_0 {
    struct bMotionPathVert3_6_0 *points;
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

struct bIKParam3_6_0 {
    int iksolver;
};

struct bItasc3_6_0 {
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

struct FModifier3_6_0 {
    struct FModifier3_6_0 *next;
    struct FModifier3_6_0 *prev;
    struct FCurve3_6_0 *curve;
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

struct FMod_Generator3_6_0 {
    float *coefficients;
    unsigned int arraysize;
    int poly_order;
    int mode;
    int flag;
};

struct FMod_FunctionGenerator3_6_0 {
    float amplitude;
    float phase_multiplier;
    float phase_offset;
    float value_offset;
    int type;
    int flag;
};

struct FCM_EnvelopeData3_6_0 {
    float min;
    float max;
    float time;
    short f1;
    short f2;
};

struct FMod_Envelope3_6_0 {
    struct FCM_EnvelopeData3_6_0 *data;
    int totvert;
    float midval;
    float min;
    float max;
};

struct FMod_Cycles3_6_0 {
    short before_mode;
    short after_mode;
    short before_cycles;
    short after_cycles;
};

struct FMod_Python3_6_0 {
    void *script;
    struct IDProperty3_6_0 *prop;
};

struct FMod_Noise3_6_0 {
    float size;
    float strength;
    float phase;
    float offset;
    short depth;
    short modification;
};

struct DriverTarget3_6_0 {
    struct ID3_6_0 *id;
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

struct FPoint3_6_0 {
    float vec[2];
    int flag;
    char _pad[4];
};

struct KS_Path3_6_0 {
    struct KS_Path3_6_0 *next;
    struct KS_Path3_6_0 *prev;
    struct ID3_6_0 *id;
    char group[64];
    int idtype;
    short groupmode;
    short flag;
    char *rna_path;
    int array_index;
    short keyingflag;
    short keyingoverride;
};

struct AnimOverride3_6_0 {
    struct AnimOverride3_6_0 *next;
    struct AnimOverride3_6_0 *prev;
    char *rna_path;
    int array_index;
    float value;
};

struct BoidRule3_6_0 {
    struct BoidRule3_6_0 *next;
    struct BoidRule3_6_0 *prev;
    int type;
    int flag;
    char name[32];
};

struct BoidData3_6_0 {
    float health;
    float acc[3];
    short state_id;
    short mode;
};

struct bSplineIKConstraint3_6_0 {
    void *tar;
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

struct bDampTrackConstraint3_6_0 {
    void *tar;
    int trackflag;
    char _pad[4];
    char subtarget[64];
};

struct MDisps3_6_0 {
    int totdisp;
    int level;
    float *(disps[3]);
    unsigned int *hidden;
};

struct EffectorWeights3_6_0 {
    void *group;
    float weight[14];
    float global_gravity;
    short flag;
    char _pad[2];
};

struct ParticleTarget3_6_0 {
    struct ParticleTarget3_6_0 *next;
    struct ParticleTarget3_6_0 *prev;
    void *ob;
    int psys;
    short flag;
    short mode;
    float time;
    float duration;
};

struct ParticleDupliWeight3_6_0 {
    struct ParticleDupliWeight3_6_0 *next;
    struct ParticleDupliWeight3_6_0 *prev;
    void *ob;
    short count;
    short flag;
    short index;
    char _pad0[2];
};

struct Paint_Runtime3_6_0 {
    unsigned int tool_offset;
    unsigned short ob_mode;
    char _pad[2];
};

struct UnitSettings3_6_0 {
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

struct PhysicsSettings3_6_0 {
    float gravity[3];
    int flag;
    int quick_cache_step;
    char _pad0[4];
};

struct FileSelectParams3_6_0 {
    char title[96];
    char dir[1090];
    char file[256];
    char renamefile[256];
    short rename_flag;
    char _pad[4];
    const  struct ID3_6_0 *rename_id;
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

struct ConsoleLine3_6_0 {
    struct ConsoleLine3_6_0 *next;
    struct ConsoleLine3_6_0 *prev;
    int len_alloc;
    int len;
    char *line;
    int cursor;
    int type;
};

struct RegionView3D3_6_0 {
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
    struct BoundBox3_6_0 *clipbb;
    struct RegionView3D3_6_0 *localvd;
    void *render_engine;
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

struct Report3_6_0 {
    struct Report3_6_0 *next;
    struct Report3_6_0 *prev;
    short type;
    short flag;
    int len;
    const  char *typestr;
    const  char *message;
};

struct wmOperatorTypeMacro3_6_0 {
    struct wmOperatorTypeMacro3_6_0 *next;
    struct wmOperatorTypeMacro3_6_0 *prev;
    char idname[64];
    struct IDProperty3_6_0 *properties;
    void *ptr;
};

struct wmKeyMapItem3_6_0 {
    struct wmKeyMapItem3_6_0 *next;
    struct wmKeyMapItem3_6_0 *prev;
    char idname[64];
    struct IDProperty3_6_0 *properties;
    char propvalue_str[64];
    short propvalue;
    short type;
    signed char val;
    signed char direction;
    short shift;
    short ctrl;
    short alt;
    short oskey;
    short keymodifier;
    short flag;
    short maptype;
    short id;
    char _pad[2];
    void *ptr;
};

struct bTransLikeConstraint3_6_0 {
    void *tar;
    int flag;
    char mix_mode;
    char _pad[3];
    char subtarget[64];
};

struct CustomDataExternal3_6_0 {
    char filepath[1024];
};

struct NodeColorBalance3_6_0 {
    float slope[3];
    float offset[3];
    float power[3];
    float offset_basis;
    char _pad[4];
    float lift[3];
    float gamma[3];
    float gain[3];
};

struct bAddon3_6_0 {
    struct bAddon3_6_0 *next;
    struct bAddon3_6_0 *prev;
    char module[64];
    struct IDProperty3_6_0 *prop;
};

struct FMod_Stepped3_6_0 {
    float step_size;
    float offset;
    float start_frame;
    float end_frame;
    int flag;
};

struct bSameVolumeConstraint3_6_0 {
    char free_axis;
    char mode;
    char _pad[2];
    float volume;
};

struct bPivotConstraint3_6_0 {
    void *tar;
    char subtarget[64];
    float offset[3];
    short rotAxis;
    short flag;
};

struct NodeColorspill3_6_0 {
    short limchan;
    short unspill;
    float limscale;
    float uspillr;
    float uspillg;
    float uspillb;
};

struct SPHFluidSettings3_6_0 {
    float radius;
    float spring_k;
    float rest_length;
    float plasticity_constant;
    float yield_ratio;
    float plasticity_balance;
    float yield_balance;
    float viscosity_omega;
    float viscosity_beta;
    float stiffness_k;
    float stiffness_knear;
    float rest_density;
    float buoyancy;
    int flag;
    int spring_frames;
    short solver;
    char _pad[6];
};

struct ReportTimerInfo3_6_0 {
    float col[4];
    float widthfac;
};

struct EditLatt3_6_0 {
    struct Lattice3_6_0 *latt;
    int shapenr;
    char needs_flush_to_id;
};

struct PTCacheExtra3_6_0 {
    struct PTCacheExtra3_6_0 *next;
    struct PTCacheExtra3_6_0 *prev;
    unsigned int type;
    unsigned int totdata;
    void *data;
};

struct ParticleSpring3_6_0 {
    float rest_length;
    unsigned int particle_index[2];
    unsigned int delete_flag;
};

struct FluidVertexVelocity3_6_0 {
    float vel[3];
};

struct wmKeyMapDiffItem3_6_0 {
    struct wmKeyMapDiffItem3_6_0 *next;
    struct wmKeyMapDiffItem3_6_0 *prev;
    struct wmKeyMapItem3_6_0 *remove_item;
    struct wmKeyMapItem3_6_0 *add_item;
};

struct MRecast3_6_0 {
    int i;
};

struct bNodeSocketValueInt3_6_0 {
    int subtype;
    int value;
    int min;
    int max;
};

struct bNodeSocketValueFloat3_6_0 {
    int subtype;
    float value;
    float min;
    float max;
};

struct bNodeSocketValueBoolean3_6_0 {
    char value;
};

struct bNodeSocketValueVector3_6_0 {
    int subtype;
    float value[3];
    float min;
    float max;
};

struct bNodeSocketValueRGBA3_6_0 {
    float value[4];
};

struct bFollowTrackConstraint3_6_0 {
    void *clip;
    char track[64];
    int flag;
    int frame_method;
    char object[64];
    void *camera;
    void *depth_ob;
};

struct bCameraSolverConstraint3_6_0 {
    void *clip;
    int flag;
    char _pad[4];
};

struct DynamicPaintBrushSettings3_6_0 {
    void *pmd;
    void *psys;
    int flags;
    int collision;
    float r;
    float g;
    float b;
    float alpha;
    float wetness;
    float particle_radius;
    float particle_smooth;
    float paint_distance;
    void *paint_ramp;
    void *vel_ramp;
    short proximity_falloff;
    short wave_type;
    short ray_dir;
    char _pad[2];
    float wave_factor;
    float wave_clamp;
    float max_velocity;
    float smudge_strength;
};

struct MovieReconstructedCamera3_6_0 {
    int framenr;
    float error;
    float mat[4][4];
};

struct MovieTrackingCamera3_6_0 {
    void *intrinsics;
    short distortion_model;
    char _pad[2];
    float sensor_width;
    float pixel_aspect;
    float focal;
    short units;
    char _pad1[2];
    float principal_point[2];
    float principal_legacy[2];
    float k1;
    float k2;
    float k3;
    float division_k1;
    float division_k2;
    float nuke_k1;
    float nuke_k2;
    float brown_k1;
    float brown_k2;
    float brown_k3;
    float brown_k4;
    float brown_p1;
    float brown_p2;
};

struct MovieTrackingMarker3_6_0 {
    float pos[2];
    float pattern_corners[2][4];
    float search_min[2];
    float search_max[2];
    int framenr;
    int flag;
};

struct MovieTrackingTrack3_6_0 {
    struct MovieTrackingTrack3_6_0 *next;
    struct MovieTrackingTrack3_6_0 *prev;
    char name[64];
    float pat_min_legacy[2];
    float pat_max_legacy[2];
    float search_min_legacy[2];
    float search_max_legacy[2];
    float offset[2];
    int markersnr;
    int _pad;
    struct MovieTrackingMarker3_6_0 *markers;
    float bundle_pos[3];
    float error;
    int flag;
    int pat_flag;
    int search_flag;
    float color[3];
    short frames_limit;
    short margin;
    short pattern_match;
    short motion_model;
    int algorithm_flag;
    float minimum_correlation;
    void *gpd;
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
    int clean_frames;
    int clean_action;
    float clean_error;
    float object_distance;
};

struct MovieTrackingStabilization3_6_0 {
    int flag;
    int tot_track;
    int act_track;
    int tot_rot_track;
    int act_rot_track;
    float maxscale;
    struct MovieTrackingTrack3_6_0 *rot_track_legacy;
    int anchor_frame;
    float target_pos[2];
    float target_rot;
    float scale;
    float locinf;
    float scaleinf;
    float rotinf;
    int filter;
    int _pad;
};

struct MovieTrackingReconstruction3_6_0 {
    int flag;
    float error;
    int last_camera;
    int camnr;
    struct MovieReconstructedCamera3_6_0 *cameras;
};

struct MovieTrackingStats3_6_0 {
    char message[256];
};

struct MovieClipUser3_6_0 {
    int framenr;
    short render_size;
    short render_flag;
};

struct MovieClipProxy3_6_0 {
    char dir[768];
    short tc;
    short quality;
    short build_size_flag;
    short build_tc_flag;
};

struct Stereo3dFormat3_6_0 {
    short flag;
    char display_mode;
    char anaglyph_type;
    char interlace_type;
    char _pad[3];
};

struct ColorManagedDisplaySettings3_6_0 {
    char display_device[64];
};

struct ColorManagedViewSettings3_6_0 {
    int flag;
    char _pad[4];
    char look[64];
    char view_transform[64];
    float exposure;
    float gamma;
    void *curve_mapping;
    void *_pad2;
};

struct NodeShaderAttribute3_6_0 {
    char name[64];
    int type;
    char _pad[4];
};

struct bObjectSolverConstraint3_6_0 {
    void *clip;
    int flag;
    char _pad[4];
    char object[64];
    float invmat[4][4];
    void *camera;
};

struct OrigSpaceLoop3_6_0 {
    float uv[2];
};

struct UnifiedPaintSettings3_6_0 {
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
    float start_pixel_radius;
    float size_pressure_value;
    float tex_mouse[2];
    float mask_tex_mouse[2];
    void *colorspace;
};

struct MaskParent3_6_0 {
    int id_type;
    int type;
    struct ID3_6_0 *id;
    char parent[64];
    char sub_parent[64];
    float parent_orig[2];
    float parent_corners_orig[2][4];
};

struct MaskSplinePointUW3_6_0 {
    float u;
    float w;
    int flag;
};

struct MaskLayerShape3_6_0 {
    struct MaskLayerShape3_6_0 *next;
    struct MaskLayerShape3_6_0 *prev;
    float *data;
    int tot_vert;
    int frame;
    char flag;
    char _pad[7];
};

struct MaskLayerShapeElem3_6_0 {
    float value[8];
};

struct GridPaintMask3_6_0 {
    float *data;
    unsigned int level;
    char _pad[4];
};

struct MVertSkin3_6_0 {
    float radius[3];
    int flag;
};

struct MovieTrackingDopesheetChannel3_6_0 {
    struct MovieTrackingDopesheetChannel3_6_0 *next;
    struct MovieTrackingDopesheetChannel3_6_0 *prev;
    struct MovieTrackingTrack3_6_0 *track;
    char _pad[4];
    char name[64];
    int tot_segment;
    int *segments;
    int max_segment;
    int total_frames;
    int first_not_disabled_marker_framenr;
    int last_not_disabled_marker_framenr;
};

struct NodeFrame3_6_0 {
    short flag;
    short label_size;
};

struct ColorCorrectionData3_6_0 {
    float saturation;
    float contrast;
    float gamma;
    float gain;
    float lift;
    char _pad[4];
};

struct NodeBokehImage3_6_0 {
    float angle;
    int flaps;
    float rounding;
    float catadioptric;
    float lensshift;
};

struct NodeBoxMask3_6_0 {
    float x;
    float y;
    float rotation;
    float height;
    float width;
    char _pad[4];
};

struct NodeEllipseMask3_6_0 {
    float x;
    float y;
    float rotation;
    float height;
    float width;
    char _pad[4];
};

struct NodeImageLayer3_6_0 {
    int pass_index;
    char pass_name[64];
};

struct NodeDilateErode3_6_0 {
    char falloff;
};

struct NodeMask3_6_0 {
    int size_x;
    int size_y;
};

struct NodeKeyingScreenData3_6_0 {
    char tracking_object[64];
};

struct NodeKeyingData3_6_0 {
    float screen_balance;
    float despill_factor;
    float despill_balance;
    int edge_kernel_radius;
    float edge_kernel_tolerance;
    float clip_black;
    float clip_white;
    int dilate_distance;
    int feather_distance;
    int feather_falloff;
    int blur_pre;
    int blur_post;
};

struct NodeTrackPosData3_6_0 {
    char tracking_object[64];
    char track_name[64];
};

struct SequenceModifierData3_6_0 {
    struct SequenceModifierData3_6_0 *next;
    struct SequenceModifierData3_6_0 *prev;
    int type;
    int flag;
    char name[64];
    int mask_input_type;
    int mask_time;
    struct Sequence3_6_0 *mask_sequence;
    void *mask_id;
};

struct bNodeSocketValueString3_6_0 {
    int subtype;
    char _pad[4];
    char value[1024];
};

struct NodeShaderScript3_6_0 {
    int mode;
    int flag;
    char filepath[1024];
    char bytecode_hash[64];
    char *bytecode;
};

struct NodeShaderTangent3_6_0 {
    int direction_type;
    int axis;
    char uv_map[64];
};

struct NodeShaderNormalMap3_6_0 {
    int space;
    char uv_map[64];
};

struct NodeTranslateData3_6_0 {
    char wrap_axis;
    char relative;
};

struct RigidBodyOb3_6_0 {
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
    struct RigidBodyOb_Shared3_6_0 *shared;
};

struct RigidBodyCon3_6_0 {
    void *ob1;
    void *ob2;
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

struct uiList3_6_0 {
    struct uiList3_6_0 *next;
    struct uiList3_6_0 *prev;
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
    struct IDProperty3_6_0 *properties;
    void *dyn_data;
};

struct FreestyleLineSet3_6_0 {
    struct FreestyleLineSet3_6_0 *next;
    struct FreestyleLineSet3_6_0 *prev;
    char name[64];
    int flags;
    int selection;
    short qi;
    char _pad1[2];
    int qi_start;
    int qi_end;
    int edge_types;
    int exclude_edge_types;
    char _pad2[4];
    void *group;
    void *linestyle;
};

struct FreestyleModuleConfig3_6_0 {
    struct FreestyleModuleConfig3_6_0 *next;
    struct FreestyleModuleConfig3_6_0 *prev;
    void *script;
    short is_displayed;
    char _pad[6];
};

struct LineStyleModifier3_6_0 {
    struct LineStyleModifier3_6_0 *next;
    struct LineStyleModifier3_6_0 *prev;
    char name[64];
    int type;
    float influence;
    int flags;
    int blend;
};

struct FreestyleEdge3_6_0 {
    char flag;
};

struct FreestyleFace3_6_0 {
    char flag;
};

struct MovieTrackingDopesheetCoverageSegment3_6_0 {
    struct MovieTrackingDopesheetCoverageSegment3_6_0 *next;
    struct MovieTrackingDopesheetCoverageSegment3_6_0 *prev;
    int coverage;
    int start_frame;
    int end_frame;
    char _pad[4];
};

struct MeshStatVis3_6_0 {
    char type;
    char _pad1[2];
    char overhang_axis;
    float overhang_min;
    float overhang_max;
    float thickness_min;
    float thickness_max;
    char thickness_samples;
    char _pad2[3];
    float distort_min;
    float distort_max;
    float sharp_min;
    float sharp_max;
};

struct bNodeInstanceKey3_6_0 {
    unsigned int value;
};

struct bPathCompare3_6_0 {
    struct bPathCompare3_6_0 *next;
    struct bPathCompare3_6_0 *prev;
    char path[768];
    char flag;
    char _pad0[7];
};

struct MovieTrackingPlaneMarker3_6_0 {
    float corners[2][4];
    int framenr;
    int flag;
};

struct MovieTrackingPlaneTrack3_6_0 {
    struct MovieTrackingPlaneTrack3_6_0 *next;
    struct MovieTrackingPlaneTrack3_6_0 *prev;
    char name[64];
    struct MovieTrackingTrack3_6_0 **point_tracks;
    int point_tracksnr;
    char _pad[4];
    struct MovieTrackingPlaneMarker3_6_0 *markers;
    int markersnr;
    int flag;
    struct Image3_6_0 *image;
    float image_opacity;
    int last_marker;
};

struct NodeShaderVectTransform3_6_0 {
    int type;
    int convert_from;
    int convert_to;
    char _pad[4];
};

struct NodePlaneTrackDeformData3_6_0 {
    char tracking_object[64];
    char plane_track_name[64];
    char flag;
    char motion_blur_samples;
    char _pad[2];
    float motion_blur_shutter;
};

struct WalkNavigation3_6_0 {
    float mouse_speed;
    float walk_speed;
    float walk_speed_factor;
    float view_height;
    float jump_height;
    float teleport_time;
    short flag;
    char _pad0[6];
};

struct PanelCategoryStack3_6_0 {
    struct PanelCategoryStack3_6_0 *next;
    struct PanelCategoryStack3_6_0 *prev;
    char idname[64];
};

struct __crt_locale_data_public3_6_0 {
    const  unsigned short *_locale_pctype;
    int _locale_mb_cur_max;
    unsigned int _locale_lc_codepage;
};

struct __crt_locale_pointers3_6_0 {
    void *locinfo;
    void *mbcinfo;
};

struct _Mbstatet3_6_0 {
    unsigned long _Wchar;
    unsigned short _Byte;
    unsigned short _State;
};

struct NodeShaderUVMap3_6_0 {
    char uv_map[64];
};

struct uiPreview3_6_0 {
    struct uiPreview3_6_0 *next;
    struct uiPreview3_6_0 *prev;
    char preview_id[64];
    short height;
    char _pad1[6];
};

struct PaletteColor3_6_0 {
    struct PaletteColor3_6_0 *next;
    struct PaletteColor3_6_0 *prev;
    float rgb[3];
    float value;
};

struct TexPaintSlot3_6_0 {
    void *ima;
    void *image_user;
    char *uvname;
    char *attribute_name;
    int valid;
    int interp;
};

struct NodeSunBeams3_6_0 {
    float source[2];
    float ray_length;
};

struct GaussianBlurVars3_6_0 {
    float size_x;
    float size_y;
};

struct RenderSlot3_6_0 {
    struct RenderSlot3_6_0 *next;
    struct RenderSlot3_6_0 *prev;
    char name[64];
    void *render;
};

struct DisplaySafeAreas3_6_0 {
    float title[2];
    float action[2];
    float title_center[2];
    float action_center[2];
};

struct ImageAnim3_6_0 {
    struct ImageAnim3_6_0 *next;
    struct ImageAnim3_6_0 *prev;
    void *anim;
};

struct ImageView3_6_0 {
    struct ImageView3_6_0 *next;
    struct ImageView3_6_0 *prev;
    char name[64];
    char filepath[1024];
};

struct ImagePackedFile3_6_0 {
    struct ImagePackedFile3_6_0 *next;
    struct ImagePackedFile3_6_0 *prev;
    void *packedfile;
    int view;
    int tile_number;
    char filepath[1024];
};

struct CorrectiveSmoothDeltaCache3_6_0 {
    float *(deltas[3]);
    unsigned int deltas_num;
    float lambda;
    float scale;
    short repeat;
    short flag;
    char smooth_type;
    char rest_source;
    char _pad[6];
};

struct SceneRenderView3_6_0 {
    struct SceneRenderView3_6_0 *next;
    struct SceneRenderView3_6_0 *prev;
    char name[64];
    char suffix[64];
    int viewflag;
    char _pad2[4];
};

struct StripAnim3_6_0 {
    struct StripAnim3_6_0 *next;
    struct StripAnim3_6_0 *prev;
    void *anim;
};

struct MLoopTri3_6_0 {
    unsigned int tri[3];
};

struct MVertTri3_6_0 {
    unsigned int tri[3];
};

struct TextVars3_6_0 {
    char text[512];
    void *text_font;
    int text_blf_id;
    float text_size;
    float color[4];
    float shadow_color[4];
    float box_color[4];
    float loc[2];
    float wrap_width;
    float box_margin;
    char flag;
    char align;
    char align_y;
    char _pad[5];
};

struct bTransformCacheConstraint3_6_0 {
    void *cache_file;
    char object_path[1024];
    void *reader;
    char reader_object_path[1024];
};

struct bGPDtriangle3_6_0 {
    unsigned int verts[3];
};

struct bGPDpalettecolor3_6_0 {
    struct bGPDpalettecolor3_6_0 *next;
    struct bGPDpalettecolor3_6_0 *prev;
    char info[64];
    float color[4];
    float fill[4];
    short flag;
    char _pad[6];
};

struct CurvePaintSettings3_6_0 {
    char curve_type;
    char flag;
    char depth_mode;
    char surface_plane;
    char fit_method;
    char _pad;
    short error_threshold;
    float radius_min;
    float radius_max;
    float radius_taper_start;
    float radius_taper_end;
    float surface_offset;
    float corner_angle;
};

struct SDefBind3_6_0 {
    unsigned int *vert_inds;
    unsigned int verts_num;
    int mode;
    float *vert_weights;
    float normal_dist;
    float influence;
};

struct SDefVert3_6_0 {
    struct SDefBind3_6_0 *binds;
    unsigned int binds_num;
    unsigned int vertex_idx;
};

struct GP_Interpolate_Settings3_6_0 {
    void *custom_ipo;
};

struct IDOverrideLibraryPropertyOperation3_6_0 {
    struct IDOverrideLibraryPropertyOperation3_6_0 *next;
    struct IDOverrideLibraryPropertyOperation3_6_0 *prev;
    short operation;
    short flag;
    short tag;
    char _pad0[2];
    char *subitem_reference_name;
    char *subitem_local_name;
    int subitem_reference_index;
    int subitem_local_index;
};

struct DualQuat3_6_0 {
    float quat[4];
    float trans[4];
    float scale[4][4];
    float scale_weight;
};

struct bUserMenuItem3_6_0 {
    struct bUserMenuItem3_6_0 *next;
    struct bUserMenuItem3_6_0 *prev;
    char ui_name[64];
    char type;
    char _pad0[7];
};

struct UserDef_Runtime3_6_0 {
    char is_dirty;
    char _pad0[7];
};

struct bPoseChannelDrawData3_6_0 {
    float solid_color[4];
    float wire_color[4];
    int bbone_matrix_len;
    float bbone_matrix[0][4][4];
};

struct BrushGpencilSettings3_6_0 {
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
    signed char caps_type;
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
    void *curve_sensitivity;
    void *curve_strength;
    void *curve_jitter;
    void *curve_rand_pressure;
    void *curve_rand_strength;
    void *curve_rand_uv;
    void *curve_rand_hue;
    void *curve_rand_saturation;
    void *curve_rand_value;
    float outline_fac;
    char _pad1[4];
    void *material;
    void *material_alt;
};

struct CollectionObject3_6_0 {
    struct CollectionObject3_6_0 *next;
    struct CollectionObject3_6_0 *prev;
    void *ob;
};

struct CollectionChild3_6_0 {
    struct CollectionChild3_6_0 *next;
    struct CollectionChild3_6_0 *prev;
    struct Collection3_6_0 *collection;
};

struct CustomData_MeshMasks3_6_0 {
    unsigned long long vmask;
    unsigned long long emask;
    unsigned long long fmask;
    unsigned long long pmask;
    unsigned long long lmask;
};

struct DynamicPaintRuntime3_6_0 {
    void *canvas_mesh;
    void *brush_mesh;
};

struct GpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 *next;
    struct GpencilModifierData3_6_0 *prev;
    int type;
    int mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
};

struct bGPDcontrolpoint3_6_0 {
    float x;
    float y;
    float z;
    float color[4];
    int size;
};

struct ViewLayerEEVEE3_6_0 {
    int render_passes;
    int _pad[1];
};

struct LightProbeCache3_6_0 {
    float position[3];
    float parallax_type;
    float attenuation_fac;
    float attenuation_type;
    float _pad3[2];
    float attenuationmat[4][4];
    float parallaxmat[4][4];
};

struct LightGridCache3_6_0 {
    float mat[4][4];
    int resolution[3];
    int offset;
    float corner[3];
    float attenuation_scale;
    float increment_x[3];
    float attenuation_bias;
    float increment_y[3];
    float level_bias;
    float increment_z[3];
    float _pad4;
    float visibility_bias;
    float visibility_bleed;
    float visibility_range;
    float _pad5;
};

struct LightCache3_6_0 {
    int flag;
    int version;
    int type;
    int cube_len;
    int grid_len;
    int mips_len;
    int vis_res;
    int ref_res;
    char _pad[2][4];
    int grid_tx;
    int cube_tx;
    int *cube_mips;
    struct LightProbeCache3_6_0 *cube_data;
    struct LightGridCache3_6_0 *grid_data;
};

struct MaterialGPencilStyle3_6_0 {
    void *sima;
    void *ima;
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

struct bFaceMap3_6_0 {
    struct bFaceMap3_6_0 *next;
    struct bFaceMap3_6_0 *prev;
    char name[64];
    char flag;
    char _pad0[7];
};

struct View3DCursor3_6_0 {
    float location[3];
    float rotation_quaternion[4];
    float rotation_euler[3];
    float rotation_axis[3];
    float rotation_angle;
    short rotation_mode;
    char _pad[6];
};

struct PaintToolSlot3_6_0 {
    void *brush;
};

struct GP_Sculpt_Guide3_6_0 {
    char use_guide;
    char use_snapping;
    char reference_point;
    char type;
    char _pad2[4];
    float angle;
    float angle_snap;
    float spacing;
    float location[3];
    struct Object3_6_0 *reference_object;
};

struct SceneEEVEE3_6_0 {
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
    int shadow_pool_size;
    char _pad[4];
    void *light_cache;
    void *light_cache_data;
    char light_cache_info[64];
    float overscan;
    float light_threshold;
};

struct TransformOrientationSlot3_6_0 {
    int type;
    int index_custom;
    char flag;
    char _pad0[7];
};

struct NodeShaderTexIES3_6_0 {
    int mode;
    char filepath[1024];
};

struct RigidBodyOb_Shared3_6_0 {
    void *physics_object;
    void *physics_shape;
};

struct ScrGlobalAreaData3_6_0 {
    short cur_fixed_height;
    short size_min;
    short size_max;
    short align;
    short flag;
    char _pad[2];
};

struct ColorMixVars3_6_0 {
    int blend_effect;
    float factor;
};

struct ShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 *next;
    struct ShaderFxData3_6_0 *prev;
    int type;
    int mode;
    char _pad0[4];
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
};

struct ShaderFxData_Runtime3_6_0 {
    float loc[3];
    char _pad[4];
    void *fx_sh;
    void *fx_sh_b;
    void *fx_sh_c;
};

struct wmKeyConfigPref3_6_0 {
    struct wmKeyConfigPref3_6_0 *next;
    struct wmKeyConfigPref3_6_0 *prev;
    char idname[64];
    struct IDProperty3_6_0 *prop;
};

struct bToolRef_Runtime3_6_0 {
    int cursor;
    char keymap[64];
    char gizmo_group[64];
    char data_block[64];
    char keymap_fallback[64];
    char op[64];
    int index;
    int flag;
};

struct bToolRef3_6_0 {
    struct bToolRef3_6_0 *next;
    struct bToolRef3_6_0 *prev;
    char idname[64];
    char idname_fallback[64];
    short tag;
    short space_type;
    int mode;
    struct IDProperty3_6_0 *properties;
    struct bToolRef_Runtime3_6_0 *runtime;
};

struct WorkSpaceLayout3_6_0 {
    struct WorkSpaceLayout3_6_0 *next;
    struct WorkSpaceLayout3_6_0 *prev;
    void *screen;
    char name[64];
};

struct wmOwnerID3_6_0 {
    struct wmOwnerID3_6_0 *next;
    struct wmOwnerID3_6_0 *prev;
    char name[64];
};

struct AssetLibraryReference3_6_0 {
    short type;
    char _pad1[2];
    int custom_library_index;
};

struct WorkSpaceDataRelation3_6_0 {
    struct WorkSpaceDataRelation3_6_0 *next;
    struct WorkSpaceDataRelation3_6_0 *prev;
    void *parent;
    void *value;
    int parentid;
    char _pad_0[4];
};

struct WorkSpaceInstanceHook3_6_0 {
    struct WorkSpace3_6_0 *active;
    struct WorkSpaceLayout3_6_0 *act_layout;
    struct WorkSpace3_6_0 *temp_workspace_store;
    struct WorkSpaceLayout3_6_0 *temp_layout_store;
};

struct UserDef_SpaceData3_6_0 {
    char section_active;
    char flag;
    char _pad0[6];
};

struct UserDef_FileSpaceData3_6_0 {
    int display_type;
    int thumbnail_size;
    int sort_type;
    int details_flags;
    int flag;
    int _pad0;
    unsigned long long filter_id;
    int temp_win_sizex;
    int temp_win_sizey;
};

struct NodeShaderVertexColor3_6_0 {
    char layer_name[64];
};

struct NodeDenoise3_6_0 {
    char hdr;
    char prefilter;
};

struct ImageTile_Runtime3_6_0 {
    int tilearray_layer;
    int _pad;
    int tilearray_offset[2];
    int tilearray_size[2];
};

struct UserDef_Experimental3_6_0 {
    char use_undo_legacy;
    char no_override_auto_resync;
    char use_cycles_debug;
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
    char use_override_templates;
    char enable_eevee_next;
    char use_sculpt_texture_paint;
    char enable_workbench_next;
    char use_new_volume_nodes;
    char _pad[6];
};

struct CurveProfilePoint3_6_0 {
    float x;
    float y;
    short flag;
    char h1;
    char h2;
    float h1_loc[2];
    float h2_loc[2];
    char _pad[4];
    struct CurveProfile3_6_0 *profile;
};

struct FluidFlowSettings3_6_0 {
    void *fmd;
    void *mesh;
    void *psys;
    void *noise_texture;
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
    char uvlayer_name[68];
    char _pad3[4];
    short vgroup_density;
    short type;
    short behavior;
    short source;
    short texture_type;
    short _pad4[3];
    int flags;
};

struct FluidEffectorSettings3_6_0 {
    void *fmd;
    void *mesh;
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

struct NodeShaderOutputAOV3_6_0 {
    char name[64];
};

struct tPaletteColorHSV3_6_0 {
    float rgb[3];
    float value;
    float h;
    float s;
    float v;
};

struct bGPDlayer_Mask3_6_0 {
    struct bGPDlayer_Mask3_6_0 *next;
    struct bGPDlayer_Mask3_6_0 *prev;
    char name[128];
    short flag;
    short sort_index;
    char _pad[4];
};

struct SceneGpencil3_6_0 {
    float smaa_threshold;
    char _pad[4];
};

struct Volume_Runtime3_6_0 {
    void *grids;
    int frame;
    int default_simplify_level;
    char velocity_x_grid[64];
    char velocity_y_grid[64];
    char velocity_z_grid[64];
};

struct VolumeDisplay3_6_0 {
    float density;
    int wireframe_type;
    int wireframe_detail;
    int interpolation_method;
    int axis_slice_method;
    int slice_axis;
    float slice_depth;
    int _pad[1];
};

struct VolumeRender3_6_0 {
    int precision;
    int space;
    float step_size;
    float clipping;
};

struct IDOverrideLibraryRuntime3_6_0 {
    void *rna_path_to_override_properties;
    unsigned int tag;
};

struct MPropCol3_6_0 {
    float color[4];
};

struct bNodeSocketValueObject3_6_0 {
    struct Object3_6_0 *value;
};

struct bNodeSocketValueImage3_6_0 {
    struct Image3_6_0 *value;
};

struct ThemeCollectionColor3_6_0 {
    unsigned char color[4];
};

struct SDNA_StructMember3_6_0 {
    short type;
    short name;
};

struct bUserAssetLibrary3_6_0 {
    struct bUserAssetLibrary3_6_0 *next;
    struct bUserAssetLibrary3_6_0 *prev;
    char name[64];
    char path[1024];
    short import_method;
    short flag;
    char _pad0[4];
};

struct AssetTag3_6_0 {
    struct AssetTag3_6_0 *next;
    struct AssetTag3_6_0 *prev;
    char name[64];
};

struct bUUID3_6_0 {
    unsigned int time_low;
    unsigned short time_mid;
    unsigned short time_hi_and_version;
    unsigned char clock_seq_hi_and_reserved;
    unsigned char clock_seq_low;
    unsigned char node[6];
};

struct bGPDcurve3_6_0 {
    struct bGPDcurve_point3_6_0 *curve_points;
    int tot_curve_points;
    short flag;
    char _pad[2];
};

struct ViewLayerAOV3_6_0 {
    struct ViewLayerAOV3_6_0 *next;
    struct ViewLayerAOV3_6_0 *prev;
    char name[64];
    int flag;
    int type;
};

struct NodesModifierSettings3_6_0 {
    void *properties;
};

struct SequencerToolSettings3_6_0 {
    int fit_method;
    short snap_mode;
    short snap_flag;
    int overlap_mode;
    int snap_distance;
    int pivot_point;
};

struct bNodeSocketValueCollection3_6_0 {
    void *value;
};

struct NodeSetAlpha3_6_0 {
    char mode;
};

struct CryptomatteEntry3_6_0 {
    struct CryptomatteEntry3_6_0 *next;
    struct CryptomatteEntry3_6_0 *prev;
    float encoded_hash;
    char name[64];
    char _pad[4];
};

struct NodeInputVector3_6_0 {
    float vector[3];
};

struct NodeGeometryObjectInfo3_6_0 {
    unsigned char transform_space;
};

struct MBoolProperty3_6_0 {
    unsigned char b;
};

struct NodeAntiAliasingData3_6_0 {
    float threshold;
    float contrast_limit;
    float corner_rounding;
};

struct CryptomatteLayer3_6_0 {
    struct CryptomatteEntry3_6_0 *next;
    struct CryptomatteEntry3_6_0 *prev;
    char name[64];
};

struct NodeInputString3_6_0 {
    char *string;
};

struct NodeGeometryPointsToVolume3_6_0 {
    unsigned char resolution_mode;
    unsigned char input_type_radius;
};

struct NodeGeometryCollectionInfo3_6_0 {
    unsigned char transform_space;
};

struct NodeGeometryVolumeToMesh3_6_0 {
    unsigned char resolution_mode;
};

struct NodeGeometryMeshCircle3_6_0 {
    unsigned char fill_type;
};

struct NodeGeometryMeshCylinder3_6_0 {
    unsigned char fill_type;
};

struct NodeGeometryMeshCone3_6_0 {
    unsigned char fill_type;
};

struct NodeGeometryMeshLine3_6_0 {
    unsigned char mode;
    unsigned char count_mode;
};

struct SpreadsheetColumnID3_6_0 {
    char *name;
};

struct SpreadsheetColumn3_6_0 {
    struct SpreadsheetColumn3_6_0 *next;
    struct SpreadsheetColumn3_6_0 *prev;
    struct SpreadsheetColumnID3_6_0 *id;
    unsigned char data_type;
    char _pad0[7];
    char *display_name;
};

struct IDPropertyUIData3_6_0 {
    char *description;
    int rna_subtype;
    char _pad[4];
};

struct LibraryWeakReference3_6_0 {
    char library_filepath[1024];
    char library_id_name[66];
    char _pad[2];
};

struct ThemeStripColor3_6_0 {
    unsigned char color[4];
};

struct AssetHandle3_6_0 {
    const  void *file_data;
};

struct CacheObjectPath3_6_0 {
    struct CacheObjectPath3_6_0 *next;
    struct CacheObjectPath3_6_0 *prev;
    char path[4096];
};

struct DashGpencilModifierSegment3_6_0 {
    char name[64];
    struct DashGpencilModifierData3_6_0 *dmd;
    int dash;
    int gap;
    float radius;
    float opacity;
    int mat_nr;
    int flag;
};

struct bNodeSocketValueTexture3_6_0 {
    struct Tex3_6_0 *value;
};

struct bNodeSocketValueMaterial3_6_0 {
    void *value;
};

struct NodeShaderPrincipled3_6_0 {
    char use_subsurface_auto_radius;
    char _pad[3];
};

struct NodeRandomValue3_6_0 {
    unsigned char data_type;
};

struct NodeInputBool3_6_0 {
    unsigned char boolean;
};

struct NodeInputInt3_6_0 {
    int integer;
};

struct NodeInputColor3_6_0 {
    float color[4];
};

struct NodeGeometryProximity3_6_0 {
    unsigned char target_element;
};

struct NodeGeometrySubdivisionSurface3_6_0 {
    unsigned char uv_smooth;
    unsigned char boundary_smooth;
};

struct NodeSwitch3_6_0 {
    unsigned char input_type;
};

struct NodeGeometryCurveSplineType3_6_0 {
    unsigned char spline_type;
};

struct NodeGeometrySetCurveHandlePositions3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurveSetHandles3_6_0 {
    unsigned char handle_type;
    unsigned char mode;
};

struct NodeGeometryCurveSelectHandles3_6_0 {
    unsigned char handle_type;
    unsigned char mode;
};

struct NodeGeometryCurvePrimitiveLine3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurvePrimitiveBezierSegment3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurvePrimitiveCircle3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurvePrimitiveQuad3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurveResample3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurveFillet3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurveTrim3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurveToPoints3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurveSample3_6_0 {
    unsigned char mode;
    signed char use_all_curves;
    signed char data_type;
    char _pad[1];
};

struct NodeGeometryTransferAttribute3_6_0 {
    signed char data_type;
    signed char domain;
    unsigned char mode;
    char _pad[1];
};

struct NodeGeometryRaycast3_6_0 {
    unsigned char mapping;
    signed char data_type;
    unsigned char input_type_ray_direction;
    unsigned char input_type_ray_length;
};

struct NodeGeometryCurveFill3_6_0 {
    unsigned char mode;
};

struct NodeGeometryMeshToPoints3_6_0 {
    unsigned char mode;
};

struct NodeGeometryAttributeCapture3_6_0 {
    signed char data_type;
    signed char domain;
};

struct NodeGeometryStringToCurves3_6_0 {
    unsigned char overflow;
    unsigned char align_x;
    unsigned char align_y;
    unsigned char pivot_mode;
};

struct NodeGeometryDeleteGeometry3_6_0 {
    signed char domain;
    signed char mode;
};

struct NodeGeometrySeparateGeometry3_6_0 {
    signed char domain;
};

struct NodeGeometryImageTexture3_6_0 {
    signed char interpolation;
    signed char extension;
};

struct NodeGeometryViewer3_6_0 {
    signed char data_type;
    signed char domain;
};

struct SpreadsheetRowFilter3_6_0 {
    struct SpreadsheetRowFilter3_6_0 *next;
    struct SpreadsheetRowFilter3_6_0 *prev;
    char column_name[64];
    unsigned char operation;
    unsigned char flag;
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

struct CacheFileLayer3_6_0 {
    struct CacheFileLayer3_6_0 *next;
    struct CacheFileLayer3_6_0 *prev;
    char filepath[1024];
    int flag;
    int _pad;
};

struct NodeConvertColorSpace3_6_0 {
    char from_color_space[64];
    char to_color_space[64];
};

struct NodeMapRange3_6_0 {
    unsigned char data_type;
    unsigned char interpolation_type;
    unsigned char clamp;
    char _pad[5];
};

struct NodeAccumulateField3_6_0 {
    unsigned char data_type;
    unsigned char domain;
};

struct NodeGeometryExtrudeMesh3_6_0 {
    unsigned char mode;
};

struct NodeGeometryCurvePrimitiveArc3_6_0 {
    unsigned char mode;
};

struct NodeFunctionCompare3_6_0 {
    signed char operation;
    signed char data_type;
    signed char mode;
    char _pad[1];
};

struct ID_Runtime_Remap3_6_0 {
    int status;
    int skipped_refcounted;
    int skipped_direct;
    int skipped_indirect;
};

struct BrushCurvesSculptSettings3_6_0 {
    int add_amount;
    int points_per_curve;
    unsigned int flag;
    float minimum_length;
    float curve_length;
    float minimum_distance;
    int density_add_attempts;
    unsigned char density_mode;
    char _pad[3];
    void *curve_parameter_falloff;
};

struct ViewLayerLightgroup3_6_0 {
    struct ViewLayerLightgroup3_6_0 *next;
    struct ViewLayerLightgroup3_6_0 *prev;
    char name[64];
};

struct LightgroupMembership3_6_0 {
    char name[64];
};

struct MInt8Property3_6_0 {
    signed char i;
};

struct NodeGeometryMergeByDistance3_6_0 {
    unsigned char mode;
};

struct NodeGeometryStoreNamedAttribute3_6_0 {
    signed char data_type;
    signed char domain;
};

struct NodeGeometryInputNamedAttribute3_6_0 {
    signed char data_type;
};

struct NodeGeometryDuplicateElements3_6_0 {
    signed char domain;
};

struct SeqTimelineChannel3_6_0 {
    struct SeqTimelineChannel3_6_0 *next;
    struct SeqTimelineChannel3_6_0 *prev;
    char name[64];
    int index;
    int flag;
};

struct XrComponentPath3_6_0 {
    struct XrComponentPath3_6_0 *next;
    struct XrComponentPath3_6_0 *prev;
    char path[192];
};

struct XrUserPath3_6_0 {
    struct XrUserPath3_6_0 *next;
    struct XrUserPath3_6_0 *prev;
    char path[64];
};

struct NodeCMPCombSepColor3_6_0 {
    unsigned char mode;
    unsigned char ycc_mode;
};

struct NodeGeometryMeshToVolume3_6_0 {
    unsigned char resolution_mode;
};

struct NodeGeometryUVUnwrap3_6_0 {
    unsigned char method;
};

struct NodeCombSepColor3_6_0 {
    signed char mode;
};

struct TimeGpencilModifierSegment3_6_0 {
    char name[64];
    struct TimeGpencilModifierData3_6_0 *gpmd;
    int seg_start;
    int seg_end;
    int seg_mode;
    int seg_repeat;
};

struct ViewerPathElem3_6_0 {
    struct ViewerPathElem3_6_0 *next;
    struct ViewerPathElem3_6_0 *prev;
    int type;
    char _pad[4];
};

struct NodeGeometrySampleIndex3_6_0 {
    signed char data_type;
    signed char domain;
    signed char clamp;
    char _pad[1];
};

struct NodeGeometryDistributePointsInVolume3_6_0 {
    unsigned char mode;
};

struct NodeShaderMix3_6_0 {
    signed char data_type;
    signed char factor_mode;
    signed char clamp_factor;
    signed char clamp_result;
    signed char blend_type;
    char _pad[3];
};

struct bUserScriptDirectory3_6_0 {
    struct bUserScriptDirectory3_6_0 *next;
    struct bUserScriptDirectory3_6_0 *prev;
    char name[64];
    char dir_path[768];
};

struct AssetWeakReference3_6_0 {
    char _pad[6];
    short asset_library_type;
    const  char *asset_library_identifier;
    const  char *relative_asset_identifier;
};

struct LightProbeBakingData3_6_0 {
    float *(L0[4]);
    float *(L1_a[4]);
    float *(L1_b[4]);
    float *(L1_c[4]);
};

struct LightProbeIrradianceData3_6_0 {
    float *(L0[3]);
    float *(L1_a[3]);
    float *(L1_b[3]);
    float *(L1_c[3]);
};

struct LightProbeVisibilityData3_6_0 {
    float *L0;
    float *L1_a;
    float *L1_b;
    float *L1_c;
};

struct LightProbeConnectivityData3_6_0 {
    unsigned char *bitmask;
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
    struct LightProbeGridCacheFrame3_6_0 *grid_static_cache;
};

struct NodeSimulationItem3_6_0 {
    char *name;
    short socket_type;
    short attribute_domain;
    int identifier;
};

struct NodeGeometrySimulationInput3_6_0 {
    int output_node_id;
};

struct NodeGeometrySimulationOutput3_6_0 {
    struct NodeSimulationItem3_6_0 *items;
    int items_num;
    int active_index;
    int next_identifier;
    int _pad;
};

struct NodeGeometrySampleVolume3_6_0 {
    signed char grid_type;
    signed char interpolation_mode;
};

struct ID_Runtime3_6_0 {
    struct ID_Runtime_Remap3_6_0 remap;
};

struct View2D3_6_0 {
    struct rctf3_6_0 tot;
    struct rctf3_6_0 cur;
    struct rcti3_6_0 vert;
    struct rcti3_6_0 hor;
    struct rcti3_6_0 mask;
    float min[2];
    float max[2];
    float minzoom;
    float maxzoom;
    short scroll;
    short scroll_ui;
    short keeptot;
    short keepzoom;
    short keepofs;
    short flag;
    short align;
    short winx;
    short winy;
    short oldwinx;
    short oldwiny;
    short around;
    char alpha_vert;
    char alpha_hor;
    char _pad[6];
    void *sms;
    void *smooth_timer;
};

struct bPoseChannel_Runtime3_6_0 {
    struct SessionUUID3_6_0 session_uuid;
    struct DualQuat3_6_0 deform_dual_quat;
    int bbone_segments;
    void *bbone_rest_mats;
    void *bbone_pose_mats;
    void *bbone_deform_mats;
    struct DualQuat3_6_0 *bbone_dual_quats;
};

struct bPose3_6_0 {
    struct ListBase3_6_0 chanbase;
    void *chanhash;
    struct bPoseChannel3_6_0 **chan_array;
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

struct bActionChannel3_6_0 {
    struct bActionChannel3_6_0 *next;
    struct bActionChannel3_6_0 *prev;
    struct bActionGroup3_6_0 *grp;
    void *ipo;
    struct ListBase3_6_0 constraintChannels;
    int flag;
    char name[64];
    int temp;
};

struct bDopeSheet3_6_0 {
    struct ID3_6_0 *source;
    struct ListBase3_6_0 chanbase;
    void *filter_grp;
    char searchstr[64];
    int filterflag;
    int filterflag2;
    int flag;
    int renameIndex;
};

struct Bone3_6_0 {
    struct Bone3_6_0 *next;
    struct Bone3_6_0 *prev;
    struct IDProperty3_6_0 *prop;
    struct Bone3_6_0 *parent;
    struct ListBase3_6_0 childbase;
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
    float dist;
    float weight;
    float xwidth;
    float length;
    float zwidth;
    float rad_head;
    float rad_tail;
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
    float size[3];
    int layer;
    short segments;
    char bbone_prev_type;
    char bbone_next_type;
    int bbone_flag;
    short bbone_prev_flag;
    short bbone_next_flag;
    struct Bone3_6_0 *bbone_prev;
    struct Bone3_6_0 *bbone_next;
};

struct Object_Runtime3_6_0 {
    struct CustomData_MeshMasks3_6_0 last_data_mask;
    char last_need_mapping;
    char _pad0[3];
    float parent_display_origin[3];
    int select_id;
    char _pad1[3];
    char is_data_eval_owned;
    double overlay_mode_transfer_start_time;
    struct BoundBox3_6_0 *bb;
    struct ID3_6_0 *data_orig;
    struct ID3_6_0 *data_eval;
    void *geometry_set_eval;
    void *mesh_deform_eval;
    void *editmesh_eval_cage;
    struct BoundBox3_6_0 *editmesh_bb_cage;
    void *gpd_orig;
    void *gpd_eval;
    void *object_as_temp_mesh;
    void *pose_backup;
    void *object_as_temp_curve;
    void *curve_cache;
    void *_pad4;
    unsigned short local_collections_bits;
    short _pad2[3];
    float *(crazyspace_deform_imats[3][3]);
    float *(crazyspace_deform_cos[3]);
    int crazyspace_verts_num;
    int _pad3[3];
};

struct IpoCurve3_6_0 {
    struct IpoCurve3_6_0 *next;
    struct IpoCurve3_6_0 *prev;
    struct BPoint3_6_0 *bp;
    struct BezTriple3_6_0 *bezt;
    struct rctf3_6_0 maxrct;
    struct rctf3_6_0 totrct;
    short blocktype;
    short adrcode;
    short vartype;
    short totvert;
    short ipo;
    short extrap;
    short flag;
    char _pad0[2];
    float ymin;
    float ymax;
    unsigned int bitmask;
    float slide_min;
    float slide_max;
    float curval;
    struct IpoDriver3_6_0 *driver;
};

struct bActionStrip3_6_0 {
    struct bActionStrip3_6_0 *next;
    struct bActionStrip3_6_0 *prev;
    short flag;
    short mode;
    short stride_axis;
    short curmod;
    void *ipo;
    void *act;
    void *object;
    float start;
    float end;
    float actstart;
    float actend;
    float actoffs;
    float stridelen;
    float repeat;
    float scale;
    float blendin;
    float blendout;
    char stridechannel[32];
    char offs_bone[32];
    struct ListBase3_6_0 modifiers;
};

struct ScrVert3_6_0 {
    struct ScrVert3_6_0 *next;
    struct ScrVert3_6_0 *prev;
    struct ScrVert3_6_0 *newv;
    struct vec2s3_6_0 vec;
    short flag;
    short editflag;
};

struct ScrArea3_6_0 {
    struct ScrArea3_6_0 *next;
    struct ScrArea3_6_0 *prev;
    struct ScrVert3_6_0 *v1;
    struct ScrVert3_6_0 *v2;
    struct ScrVert3_6_0 *v3;
    struct ScrVert3_6_0 *v4;
    struct bScreen3_6_0 *full;
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

struct Strip3_6_0 {
    struct Strip3_6_0 *next;
    struct Strip3_6_0 *prev;
    int us;
    int done;
    int startstill;
    int endstill;
    struct StripElem3_6_0 *stripdata;
    char dirpath[768];
    struct StripProxy3_6_0 *proxy;
    struct StripCrop3_6_0 *crop;
    struct StripTransform3_6_0 *transform;
    struct StripColorBalance3_6_0 *color_balance;
    struct ColorManagedColorspaceSettings3_6_0 colorspace_settings;
};

struct SequenceRuntime3_6_0 {
    struct SessionUUID3_6_0 session_uuid;
};

struct Editing3_6_0 {
    struct ListBase3_6_0 *seqbasep;
    struct ListBase3_6_0 *displayed_channels;
    void *_pad0;
    struct ListBase3_6_0 seqbase;
    struct ListBase3_6_0 metastack;
    struct ListBase3_6_0 channels;
    struct Sequence3_6_0 *act_seq;
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

struct SpaceLink3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
};

struct SpaceInfo3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char rpt_mask;
    char _pad[7];
};

struct SpaceSeqRuntime3_6_0 {
    struct rctf3_6_0 last_thumbnail_area;
    void *last_displayed_thumbnails;
    int rename_channel_index;
    float timeline_clamp_custom_range;
};

struct Scopes3_6_0 {
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
    float minmax[2][3];
    struct Histogram3_6_0 hist;
    float *waveform_1;
    float *waveform_2;
    float *waveform_3;
    float *vecscope;
    int waveform_tot;
    char _pad[4];
};

struct SpaceText_Runtime3_6_0 {
    int lheight_px;
    int cwidth_px;
    struct rcti3_6_0 scroll_region_handle;
    struct rcti3_6_0 scroll_region_select;
    int line_number_display_digits;
    int viewlines;
    float scroll_px_per_line;
    int scroll_ofs_px[2];
    char _pad1[4];
    void *drawcache;
};

struct ViewerPath3_6_0 {
    struct ListBase3_6_0 path;
};

struct Panel3_6_0 {
    struct Panel3_6_0 *next;
    struct Panel3_6_0 *prev;
    void *type;
    void *layout;
    char panelname[64];
    char drawname[64];
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
    struct Panel_Runtime3_6_0 runtime;
};

struct ThemeUI3_6_0 {
    struct uiWidgetColors3_6_0 wcol_regular;
    struct uiWidgetColors3_6_0 wcol_tool;
    struct uiWidgetColors3_6_0 wcol_toolbar_item;
    struct uiWidgetColors3_6_0 wcol_text;
    struct uiWidgetColors3_6_0 wcol_radio;
    struct uiWidgetColors3_6_0 wcol_option;
    struct uiWidgetColors3_6_0 wcol_toggle;
    struct uiWidgetColors3_6_0 wcol_num;
    struct uiWidgetColors3_6_0 wcol_numslider;
    struct uiWidgetColors3_6_0 wcol_tab;
    struct uiWidgetColors3_6_0 wcol_menu;
    struct uiWidgetColors3_6_0 wcol_pulldown;
    struct uiWidgetColors3_6_0 wcol_menu_back;
    struct uiWidgetColors3_6_0 wcol_menu_item;
    struct uiWidgetColors3_6_0 wcol_tooltip;
    struct uiWidgetColors3_6_0 wcol_box;
    struct uiWidgetColors3_6_0 wcol_scroll;
    struct uiWidgetColors3_6_0 wcol_progress;
    struct uiWidgetColors3_6_0 wcol_list_item;
    struct uiWidgetColors3_6_0 wcol_pie_menu;
    struct uiWidgetColors3_6_0 wcol_view_item;
    struct uiWidgetStateColors3_6_0 wcol_state;
    unsigned char widget_emboss[4];
    float menu_shadow_fac;
    short menu_shadow_width;
    unsigned char editor_outline[4];
    unsigned char transparent_checker_primary[4];
    unsigned char transparent_checker_secondary[4];
    unsigned char transparent_checker_size;
    char _pad1[1];
    float icon_alpha;
    float icon_saturation;
    unsigned char widget_text_cursor[4];
    unsigned char xaxis[4];
    unsigned char yaxis[4];
    unsigned char zaxis[4];
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
    struct uiPanelColors3_6_0 panelcolors;
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
    unsigned char edge_seam[4];
    unsigned char edge_sharp[4];
    unsigned char edge_facesel[4];
    unsigned char edge_crease[4];
    unsigned char edge_bevel[4];
    unsigned char face[4];
    unsigned char face_select[4];
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
    unsigned char _pad1[6];
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

struct SpaceScript3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct Script3_6_0 *script;
    short flags;
    short menunr;
    char _pad1[4];
    void *but_refs;
};

struct SoftBody3_6_0 {
    int totpoint;
    int totspring;
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
    char local;
    char solverflags;
    struct SBVertex3_6_0 **keys;
    int totpointkey;
    int totkey;
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
    struct SoftBody_Shared3_6_0 *shared;
    void *pointcache;
    struct ListBase3_6_0 ptcaches;
    void *collision_group;
    struct EffectorWeights3_6_0 *effector_weights;
    float lcom[3];
    float lrot[3][3];
    float lscale[3][3];
    int last_frame;
};

struct ModifierData3_6_0 {
    struct ModifierData3_6_0 *next;
    struct ModifierData3_6_0 *prev;
    int type;
    int mode;
    float execution_time;
    short flag;
    short ui_expand_flag;
    char name[64];
    char *error;
    struct SessionUUID3_6_0 session_uuid;
    void *runtime;
};

struct bNodeSocket3_6_0 {
    struct bNodeSocket3_6_0 *next;
    struct bNodeSocket3_6_0 *prev;
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
    char description[64];
    char *default_attribute_name;
    int own_index;
    int to_index;
    struct bNodeLink3_6_0 *link;
    struct bNodeStack3_6_0 ns;
    void *runtime;
};

struct bNodeInstanceHashEntry3_6_0 {
    struct bNodeInstanceKey3_6_0 key;
    short tag;
};

struct bNode3_6_0 {
    struct bNode3_6_0 *next;
    struct bNode3_6_0 *prev;
    struct ListBase3_6_0 inputs;
    struct ListBase3_6_0 outputs;
    char name[64];
    int identifier;
    int flag;
    char idname[64];
    void *typeinfo;
    short type;
    char _pad1[2];
    short custom1;
    short custom2;
    float custom3;
    float custom4;
    struct ID3_6_0 *id;
    void *storage;
    struct IDProperty3_6_0 *prop;
    struct bNode3_6_0 *parent;
    float locx;
    float locy;
    float width;
    float height;
    float offsetx;
    float offsety;
    char label[64];
    float color[3];
    char _pad2[4];
    void *runtime;
};

struct FreestyleConfig3_6_0 {
    struct ListBase3_6_0 modules;
    int mode;
    int raycasting_algorithm;
    int flags;
    float sphere_radius;
    float dkr_epsilon;
    float crease_angle;
    struct ListBase3_6_0 linesets;
};

struct IDPropertyData3_6_0 {
    void *pointer;
    struct ListBase3_6_0 group;
    int val;
    int val2;
};

struct ImageFormatData3_6_0 {
    char imtype;
    char depth;
    char planes;
    char flag;
    char quality;
    char compress;
    char exr_codec;
    char cineon_flag;
    short cineon_white;
    short cineon_black;
    float cineon_gamma;
    char jp2_flag;
    char jp2_codec;
    char tiff_codec;
    char _pad[4];
    char views_format;
    struct Stereo3dFormat3_6_0 stereo3d_format;
    char color_management;
    char _pad1[7];
    struct ColorManagedViewSettings3_6_0 view_settings;
    struct ColorManagedDisplaySettings3_6_0 display_settings;
    struct ColorManagedColorspaceSettings3_6_0 linear_colorspace_settings;
};

struct Paint3_6_0 {
    void *brush;
    struct PaintToolSlot3_6_0 *tool_slots;
    int tool_slots_len;
    char _pad1[4];
    void *palette;
    void *cavity_curve;
    void *paint_cursor;
    unsigned char paint_cursor_col[4];
    int flags;
    int num_input_samples;
    int symmetry_flags;
    float tile_offset[3];
    char _pad2[4];
    struct Paint_Runtime3_6_0 runtime;
};

struct bActionGroup3_6_0 {
    struct bActionGroup3_6_0 *next;
    struct bActionGroup3_6_0 *prev;
    struct ListBase3_6_0 channels;
    int flag;
    int customCol;
    char name[64];
    struct ThemeWireColor3_6_0 cs;
};

struct bPythonConstraint3_6_0 {
    void *text;
    struct IDProperty3_6_0 *prop;
    int flag;
    int tarnum;
    struct ListBase3_6_0 targets;
    void *tar;
    char subtarget[64];
};

struct ParticleData3_6_0 {
    struct ParticleKey3_6_0 state;
    struct ParticleKey3_6_0 prev_state;
    struct HairKey3_6_0 *hair;
    struct ParticleKey3_6_0 *keys;
    struct BoidParticle3_6_0 *boid;
    int totkey;
    float time;
    float lifetime;
    float dietime;
    int num;
    int num_dmcache;
    float fuv[4];
    float foffset;
    float size;
    float sphdensity;
    char _pad[4];
    int hair_index;
    short flag;
    short alive;
};

struct bGPDspoint3_6_0 {
    float x;
    float y;
    float z;
    float pressure;
    float strength;
    float time;
    int flag;
    float uv_fac;
    float uv_rot;
    float uv_fill[2];
    float vert_color[4];
    char _pad2[4];
    struct bGPDspoint_Runtime3_6_0 runtime;
};

struct bGPDstroke3_6_0 {
    struct bGPDstroke3_6_0 *next;
    struct bGPDstroke3_6_0 *prev;
    struct bGPDspoint3_6_0 *points;
    struct bGPDtriangle3_6_0 *triangles;
    int totpoints;
    int tot_triangles;
    short thickness;
    short flag;
    short _pad[2];
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
    void *dvert;
    void *_pad3;
    float vert_color_fill[4];
    struct bGPDcurve3_6_0 *editcurve;
    struct bGPDstroke_Runtime3_6_0 runtime;
    void *_pad5;
};

struct bGPDframe3_6_0 {
    struct bGPDframe3_6_0 *next;
    struct bGPDframe3_6_0 *prev;
    struct ListBase3_6_0 strokes;
    int framenum;
    short flag;
    short key_type;
    struct bGPDframe_Runtime3_6_0 runtime;
};

struct bGPDlayer3_6_0 {
    struct bGPDlayer3_6_0 *next;
    struct bGPDlayer3_6_0 *prev;
    struct ListBase3_6_0 frames;
    struct bGPDframe3_6_0 *actframe;
    short flag;
    short onion_flag;
    float color[4];
    float fill[4];
    char info[128];
    short thickness;
    short pass_index;
    void *parent;
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
    struct ListBase3_6_0 mask_layers;
    int act_mask;
    char _pad2[4];
    float location[3];
    float rotation[3];
    float scale[3];
    float layer_mat[4][4];
    float layer_invmat[4][4];
    char _pad3[4];
    struct bGPDlayer_Runtime3_6_0 runtime;
};

struct uiStyle3_6_0 {
    struct uiStyle3_6_0 *next;
    struct uiStyle3_6_0 *prev;
    char name[64];
    struct uiFontStyle3_6_0 paneltitle;
    struct uiFontStyle3_6_0 grouplabel;
    struct uiFontStyle3_6_0 widgetlabel;
    struct uiFontStyle3_6_0 widget;
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

struct FMod_Limits3_6_0 {
    struct rctf3_6_0 rect;
    int flag;
    char _pad[4];
};

struct ChannelDriver3_6_0 {
    struct ListBase3_6_0 variables;
    char expression[256];
    void *expr_comp;
    void *expr_simple;
    float curval;
    float influence;
    int type;
    int flag;
};

struct FCurve3_6_0 {
    struct FCurve3_6_0 *next;
    struct FCurve3_6_0 *prev;
    struct bActionGroup3_6_0 *grp;
    struct ChannelDriver3_6_0 *driver;
    struct ListBase3_6_0 modifiers;
    struct BezTriple3_6_0 *bezt;
    struct FPoint3_6_0 *fpt;
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
    float prev_norm_factor;
    float prev_offset;
};

struct NlaStrip3_6_0 {
    struct NlaStrip3_6_0 *next;
    struct NlaStrip3_6_0 *prev;
    struct ListBase3_6_0 strips;
    struct bAction3_6_0 *act;
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
    struct NlaStrip3_6_0 *orig_strip;
    void *_pad3;
};

struct NlaTrack3_6_0 {
    struct NlaTrack3_6_0 *next;
    struct NlaTrack3_6_0 *prev;
    struct ListBase3_6_0 strips;
    int flag;
    int index;
    char name[64];
};

struct KeyingSet3_6_0 {
    struct KeyingSet3_6_0 *next;
    struct KeyingSet3_6_0 *prev;
    struct ListBase3_6_0 paths;
    char idname[64];
    char name[64];
    char description[1024];
    char typeinfo[64];
    int active_path;
    short flag;
    short keyingflag;
    short keyingoverride;
    char _pad[6];
};

struct AnimData3_6_0 {
    struct bAction3_6_0 *action;
    struct bAction3_6_0 *tmpact;
    struct ListBase3_6_0 nla_tracks;
    struct NlaTrack3_6_0 *act_track;
    struct NlaStrip3_6_0 *actstrip;
    struct ListBase3_6_0 drivers;
    struct ListBase3_6_0 overrides;
    struct FCurve3_6_0 **driver_array;
    int flag;
    char _pad[4];
    short act_blendmode;
    short act_extendmode;
    float act_influence;
};

struct BoidRuleGoalAvoid3_6_0 {
    struct BoidRule3_6_0 rule;
    void *ob;
    int options;
    float fear_factor;
    int signal_id;
    int channels;
};

struct BoidRuleAvoidCollision3_6_0 {
    struct BoidRule3_6_0 rule;
    int options;
    float look_ahead;
};

struct BoidRuleFollowLeader3_6_0 {
    struct BoidRule3_6_0 rule;
    void *ob;
    float loc[3];
    float oloc[3];
    float cfra;
    float distance;
    int options;
    int queue_size;
};

struct BoidRuleAverageSpeed3_6_0 {
    struct BoidRule3_6_0 rule;
    float wander;
    float level;
    float speed;
    char _pad0[4];
};

struct BoidRuleFight3_6_0 {
    struct BoidRule3_6_0 rule;
    float distance;
    float flee_distance;
};

struct BoidState3_6_0 {
    struct BoidState3_6_0 *next;
    struct BoidState3_6_0 *prev;
    struct ListBase3_6_0 rules;
    struct ListBase3_6_0 conditions;
    struct ListBase3_6_0 actions;
    char name[32];
    int id;
    int flag;
    int ruleset_type;
    float rule_fuzziness;
    int signal_id;
    int channels;
    float volume;
    float falloff;
};

struct BoidSettings3_6_0 {
    int options;
    int last_state_id;
    float landing_smoothness;
    float height;
    float banking;
    float pitch;
    float health;
    float aggression;
    float strength;
    float accuracy;
    float range;
    float air_min_speed;
    float air_max_speed;
    float air_max_acc;
    float air_max_ave;
    float air_personal_space;
    float land_jump_speed;
    float land_max_speed;
    float land_max_acc;
    float land_max_ave;
    float land_personal_space;
    float land_stick_force;
    struct ListBase3_6_0 states;
};

struct PTCacheMem3_6_0 {
    struct PTCacheMem3_6_0 *next;
    struct PTCacheMem3_6_0 *prev;
    unsigned int frame;
    unsigned int totpoint;
    unsigned int data_types;
    unsigned int flag;
    void *data[8];
    struct ListBase3_6_0 extradata;
};

struct BoidParticle3_6_0 {
    void *ground;
    struct BoidData3_6_0 data;
    float gravity[3];
    float wander[3];
    char _pad0[4];
};

struct ARegion_Runtime3_6_0 {
    const  char *category;
    struct rcti3_6_0 visible_rect;
    int offset_x;
    int offset_y;
    void *block_name_map;
};

struct SpaceFile3_6_0 {
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
    struct FileSelectParams3_6_0 *params;
    struct FileAssetSelectParams3_6_0 *asset_params;
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

struct SpaceConsole3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    int lheight;
    char _pad[4];
    struct ListBase3_6_0 scrollback;
    struct ListBase3_6_0 history;
    char prompt[256];
    char language[32];
    int sel_start;
    int sel_end;
};

struct SpaceUserPref3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    char _pad1[7];
    char filter_type;
    char filter[64];
};

struct ReportList3_6_0 {
    struct ListBase3_6_0 list;
    int printlevel;
    int storelevel;
    int flag;
    char _pad[4];
    void *reporttimer;
};

struct ScrAreaMap3_6_0 {
    struct ListBase3_6_0 vertbase;
    struct ListBase3_6_0 edgebase;
    struct ListBase3_6_0 areabase;
};

struct wmKeyConfig3_6_0 {
    struct wmKeyConfig3_6_0 *next;
    struct wmKeyConfig3_6_0 *prev;
    char idname[64];
    char basename[64];
    struct ListBase3_6_0 keymaps;
    int actkeymap;
    short flag;
    char _pad0[2];
};

struct wmOperator3_6_0 {
    struct wmOperator3_6_0 *next;
    struct wmOperator3_6_0 *prev;
    char idname[64];
    struct IDProperty3_6_0 *properties;
    void *type;
    void *customdata;
    void *py_instance;
    void *ptr;
    struct ReportList3_6_0 *reports;
    struct ListBase3_6_0 macro;
    struct wmOperator3_6_0 *opm;
    void *layout;
    short flag;
    char _pad[6];
};

struct EditNurb3_6_0 {
    struct ListBase3_6_0 nurbs;
    void *keyindex;
    int shapenr;
    char needs_flush_to_id;
};

struct DynamicPaintSurface3_6_0 {
    struct DynamicPaintSurface3_6_0 *next;
    struct DynamicPaintSurface3_6_0 *prev;
    struct DynamicPaintCanvasSettings3_6_0 *canvas;
    void *data;
    void *brush_group;
    void *effector_weights;
    void *pointcache;
    struct ListBase3_6_0 ptcaches;
    int current_frame;
    char name[64];
    short format;
    short type;
    short disp_type;
    short image_fileformat;
    short effect_ui;
    short init_color_type;
    int flags;
    int effect;
    int image_resolution;
    int substeps;
    int start_frame;
    int end_frame;
    float init_color[4];
    void *init_texture;
    char init_layername[68];
    int dry_speed;
    int diss_speed;
    float color_dry_threshold;
    float depth_clamp;
    float disp_factor;
    float spread_speed;
    float color_spread_speed;
    float shrink_speed;
    float drip_vel;
    float drip_acc;
    float influence_scale;
    float radius_scale;
    float wave_damping;
    float wave_speed;
    float wave_timescale;
    float wave_spring;
    float wave_smoothness;
    char _pad2[4];
    char uvlayer_name[68];
    char image_output_path[1024];
    char output_name[68];
    char output_name2[68];
};

struct DynamicPaintCanvasSettings3_6_0 {
    void *pmd;
    struct ListBase3_6_0 surfaces;
    short active_sur;
    short flags;
    char _pad[4];
    char error[64];
};

struct MovieTrackingDopesheet3_6_0 {
    int ok;
    short sort_method;
    short flag;
    struct ListBase3_6_0 coverage_segments;
    struct ListBase3_6_0 channels;
    int tot_channel;
    char _pad[4];
};

struct MovieClip_Runtime3_6_0 {
    struct ListBase3_6_0 gputextures;
};

struct MovieClipScopes3_6_0 {
    short ok;
    short use_track_mask;
    int track_preview_height;
    int frame_width;
    int frame_height;
    struct MovieTrackingMarker3_6_0 undist_marker;
    void *track_search;
    void *track_preview;
    float track_pos[2];
    short track_disabled;
    short track_locked;
    int scene_framenr;
    struct MovieTrackingTrack3_6_0 *track;
    struct MovieTrackingMarker3_6_0 *marker;
    float slide_scale[2];
};

struct MovieTrackingObject3_6_0 {
    struct MovieTrackingObject3_6_0 *next;
    struct MovieTrackingObject3_6_0 *prev;
    char name[64];
    int flag;
    float scale;
    struct ListBase3_6_0 tracks;
    struct ListBase3_6_0 plane_tracks;
    struct MovieTrackingTrack3_6_0 *active_track;
    struct MovieTrackingPlaneTrack3_6_0 *active_plane_track;
    struct MovieTrackingReconstruction3_6_0 reconstruction;
    int keyframe1;
    int keyframe2;
};

struct MaskSplinePoint3_6_0 {
    struct BezTriple3_6_0 bezt;
    char _pad[4];
    int tot_uw;
    struct MaskSplinePointUW3_6_0 *uw;
    struct MaskParent3_6_0 parent;
};

struct MaskSpline3_6_0 {
    struct MaskSpline3_6_0 *next;
    struct MaskSpline3_6_0 *prev;
    short flag;
    char offset_mode;
    char weight_interp;
    int tot_point;
    struct MaskSplinePoint3_6_0 *points;
    struct MaskParent3_6_0 parent;
    struct MaskSplinePoint3_6_0 *points_deform;
};

struct MaskLayer3_6_0 {
    struct MaskLayer3_6_0 *next;
    struct MaskLayer3_6_0 *prev;
    char name[64];
    struct ListBase3_6_0 splines;
    struct ListBase3_6_0 splines_shapes;
    struct MaskSpline3_6_0 *act_spline;
    struct MaskSplinePoint3_6_0 *act_point;
    float alpha;
    char blend;
    char blend_flag;
    char falloff;
    char _pad[7];
    char flag;
    char visibility_flag;
};

struct NodeColorCorrection3_6_0 {
    struct ColorCorrectionData3_6_0 master;
    struct ColorCorrectionData3_6_0 shadows;
    struct ColorCorrectionData3_6_0 midtones;
    struct ColorCorrectionData3_6_0 highlights;
    float startmidtones;
    float endmidtones;
};

struct ColorBalanceModifierData3_6_0 {
    struct SequenceModifierData3_6_0 modifier;
    struct StripColorBalance3_6_0 color_balance;
    float color_multiply;
};

struct BrightContrastModifierData3_6_0 {
    struct SequenceModifierData3_6_0 modifier;
    float bright;
    float contrast;
};

struct RigidBodyWorld3_6_0 {
    struct EffectorWeights3_6_0 *effector_weights;
    void *group;
    void *objects;
    void *constraints;
    char _pad[4];
    float ltime;
    struct RigidBodyWorld_Shared3_6_0 *shared;
    void *pointcache;
    struct ListBase3_6_0 ptcaches;
    int numbodies;
    short substeps_per_frame;
    short num_solver_iterations;
    int flag;
    float time_scale;
};

struct LineStyleColorModifier_AlongStroke3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *color_ramp;
};

struct LineStyleAlphaModifier_AlongStroke3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    char _pad[4];
};

struct LineStyleThicknessModifier_AlongStroke3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float value_min;
    float value_max;
    char _pad[4];
};

struct LineStyleColorModifier_DistanceFromCamera3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *color_ramp;
    float range_min;
    float range_max;
};

struct LineStyleAlphaModifier_DistanceFromCamera3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float range_min;
    float range_max;
    char _pad[4];
};

struct LineStyleThicknessModifier_DistanceFromCamera3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float range_min;
    float range_max;
    float value_min;
    float value_max;
    char _pad[4];
};

struct LineStyleColorModifier_DistanceFromObject3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *target;
    void *color_ramp;
    float range_min;
    float range_max;
};

struct LineStyleAlphaModifier_DistanceFromObject3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *target;
    void *curve;
    int flags;
    float range_min;
    float range_max;
    char _pad[4];
};

struct LineStyleThicknessModifier_DistanceFromObject3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *target;
    void *curve;
    int flags;
    float range_min;
    float range_max;
    float value_min;
    float value_max;
    char _pad[4];
};

struct LineStyleColorModifier_Material3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *color_ramp;
    int flags;
    int mat_attr;
};

struct LineStyleAlphaModifier_Material3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    int mat_attr;
};

struct LineStyleThicknessModifier_Material3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float value_min;
    float value_max;
    int mat_attr;
};

struct LineStyleGeometryModifier_Sampling3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float sampling;
    char _pad[4];
};

struct LineStyleGeometryModifier_BezierCurve3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float error;
    char _pad[4];
};

struct LineStyleGeometryModifier_SinusDisplacement3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float wavelength;
    float amplitude;
    float phase;
    char _pad[4];
};

struct LineStyleGeometryModifier_SpatialNoise3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float amplitude;
    float scale;
    unsigned int octaves;
    int flags;
};

struct LineStyleGeometryModifier_PerlinNoise1D3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float frequency;
    float amplitude;
    float angle;
    unsigned int octaves;
    int seed;
    char _pad1[4];
};

struct LineStyleGeometryModifier_PerlinNoise2D3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float frequency;
    float amplitude;
    float angle;
    unsigned int octaves;
    int seed;
    char _pad1[4];
};

struct LineStyleGeometryModifier_BackboneStretcher3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float backbone_length;
    char _pad[4];
};

struct LineStyleGeometryModifier_TipRemover3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float tip_length;
    char _pad[4];
};

struct LineStyleGeometryModifier_Polygonalization3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float error;
    char _pad[4];
};

struct LineStyleGeometryModifier_GuidingLines3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float offset;
    char _pad[4];
};

struct LineStyleGeometryModifier_Blueprint3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    int flags;
    unsigned int rounds;
    float backbone_length;
    unsigned int random_radius;
    unsigned int random_center;
    unsigned int random_backbone;
};

struct LineStyleGeometryModifier_2DOffset3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float start;
    float end;
    float x;
    float y;
};

struct LineStyleGeometryModifier_2DTransform3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    int pivot;
    float scale_x;
    float scale_y;
    float angle;
    float pivot_u;
    float pivot_x;
    float pivot_y;
    char _pad[4];
};

struct LineStyleThicknessModifier_Calligraphy3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float min_thickness;
    float max_thickness;
    float orientation;
    char _pad[4];
};

struct bNodeTreePath3_6_0 {
    struct bNodeTreePath3_6_0 *next;
    struct bNodeTreePath3_6_0 *prev;
    struct bNodeTree3_6_0 *nodetree;
    struct bNodeInstanceKey3_6_0 parent_key;
    char _pad[4];
    float view_center[2];
    char node_name[64];
    char display_name[64];
};

struct SequencerMaskModifierData3_6_0 {
    struct SequenceModifierData3_6_0 modifier;
};

struct PanelCategoryDyn3_6_0 {
    struct PanelCategoryDyn3_6_0 *next;
    struct PanelCategoryDyn3_6_0 *prev;
    char idname[64];
    struct rcti3_6_0 rect;
};

struct PaintCurvePoint3_6_0 {
    struct BezTriple3_6_0 bez;
    float pressure;
};

struct LineStyleColorModifier_Curvature_3D3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float min_curvature;
    float max_curvature;
    void *color_ramp;
    float range_min;
    float range_max;
};

struct LineStyleAlphaModifier_Curvature_3D3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float min_curvature;
    float max_curvature;
    char _pad[4];
};

struct LineStyleThicknessModifier_Curvature_3D3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    char _pad[4];
    float min_curvature;
    float max_curvature;
    float min_thickness;
    float max_thickness;
};

struct LineStyleColorModifier_Noise3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *color_ramp;
    float period;
    float amplitude;
    int seed;
    char _pad[4];
};

struct LineStyleAlphaModifier_Noise3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float period;
    float amplitude;
    int seed;
};

struct LineStyleThicknessModifier_Noise3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float period;
    float amplitude;
    int flags;
    int seed;
};

struct LineStyleColorModifier_CreaseAngle3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *color_ramp;
    float min_angle;
    float max_angle;
};

struct LineStyleAlphaModifier_CreaseAngle3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float min_angle;
    float max_angle;
    char _pad[4];
};

struct LineStyleThicknessModifier_CreaseAngle3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    char _pad[4];
    float min_angle;
    float max_angle;
    float min_thickness;
    float max_thickness;
};

struct LineStyleColorModifier_Tangent3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *color_ramp;
};

struct LineStyleAlphaModifier_Tangent3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    char _pad[4];
};

struct LineStyleThicknessModifier_Tangent3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    void *curve;
    int flags;
    float min_thickness;
    float max_thickness;
    char _pad[4];
};

struct LineStyleGeometryModifier_Simplification3_6_0 {
    struct LineStyleModifier3_6_0 modifier;
    float tolerance;
    char _pad[4];
};

struct FileDirEntryArr3_6_0 {
    struct ListBase3_6_0 entries;
    int entries_num;
    int entries_filtered_num;
    char root[1024];
};

struct WhiteBalanceModifierData3_6_0 {
    struct SequenceModifierData3_6_0 modifier;
    float white_value[3];
    char _pad[4];
};

struct SequencerTonemapModifierData3_6_0 {
    struct SequenceModifierData3_6_0 modifier;
    float key;
    float offset;
    float gamma;
    float intensity;
    float contrast;
    float adaptation;
    float correction;
    int type;
};

struct bGPDpalette3_6_0 {
    struct bGPDpalette3_6_0 *next;
    struct bGPDpalette3_6_0 *prev;
    struct ListBase3_6_0 colors;
    char info[64];
    short flag;
    char _pad[6];
};

struct IDOverrideLibraryProperty3_6_0 {
    struct IDOverrideLibraryProperty3_6_0 *next;
    struct IDOverrideLibraryProperty3_6_0 *prev;
    char *rna_path;
    struct ListBase3_6_0 operations;
    short tag;
    char _pad[2];
    unsigned int rna_prop_type;
};

struct IDOverrideLibrary3_6_0 {
    struct ID3_6_0 *reference;
    struct ListBase3_6_0 properties;
    struct ID3_6_0 *hierarchy_root;
    struct ID3_6_0 *storage;
    struct IDOverrideLibraryRuntime3_6_0 *runtime;
    unsigned int flag;
    char _pad_1[4];
};

struct bUserMenu3_6_0 {
    struct bUserMenu3_6_0 *next;
    struct bUserMenu3_6_0 *prev;
    char space_type;
    char _pad0[7];
    char context[64];
    struct ListBase3_6_0 items;
};

struct bUserMenuItem_Op3_6_0 {
    struct bUserMenuItem3_6_0 item;
    char op_idname[64];
    struct IDProperty3_6_0 *prop;
    char op_prop_enum[64];
    char opcontext;
    char _pad0[7];
};

struct bUserMenuItem_Menu3_6_0 {
    struct bUserMenuItem3_6_0 item;
    char mt_idname[64];
};

struct bUserMenuItem_Prop3_6_0 {
    struct bUserMenuItem3_6_0 item;
    char context_data_path[256];
    char prop_id[64];
    int prop_index;
    char _pad0[4];
};

struct MovieClip_RuntimeGPUTexture3_6_0 {
    void *next;
    void *prev;
    struct MovieClipUser3_6_0 user;
    void *gputexture[3];
};

struct CameraBGImage3_6_0 {
    struct CameraBGImage3_6_0 *next;
    struct CameraBGImage3_6_0 *prev;
    struct Image3_6_0 *ima;
    struct ImageUser3_6_0 iuser;
    struct MovieClip3_6_0 *clip;
    struct MovieClipUser3_6_0 cuser;
    float offset[2];
    float scale;
    float rotation;
    float alpha;
    short flag;
    short source;
};

struct Collection_Runtime3_6_0 {
    struct ID3_6_0 *owner_id;
    struct ListBase3_6_0 object_cache;
    struct ListBase3_6_0 object_cache_instanced;
    struct ListBase3_6_0 parents;
    void *gobject_hash;
    unsigned char tag;
    char _pad0[7];
};

struct bArmatureConstraint3_6_0 {
    int flag;
    char _pad[4];
    struct ListBase3_6_0 targets;
};

struct NoiseGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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
    void *curve_intensity;
};

struct SubdivGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    int level;
    int layer_pass;
    short type;
    char _pad[6];
};

struct ThickGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float thickness_fac;
    int thickness;
    int layer_pass;
    char _pad[4];
    void *curve_thickness;
};

struct TimeGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    int layer_pass;
    int flag;
    int offset;
    float frame_scale;
    int mode;
    int sfra;
    int efra;
    char _pad[4];
    struct TimeGpencilModifierSegment3_6_0 *segments;
    int segments_len;
    int segment_active_index;
};

struct TintGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *object;
    void *material;
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
    void *curve_intensity;
    void *colorband;
};

struct ColorGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    float hsv[3];
    char modify_color;
    char _pad[3];
    int layer_pass;
    char _pad1[4];
    void *curve_intensity;
};

struct OpacityGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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
    void *curve_intensity;
};

struct ArrayGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *object;
    void *material;
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

struct BuildGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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
    float speed_fac;
    float speed_maxgap;
    short time_mode;
    char _pad[6];
    void *object;
    float percentage_fac;
    float fade_fac;
    char target_vgname[64];
    float fade_opacity_strength;
    float fade_thickness_strength;
};

struct LatticeGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *object;
    void *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float strength;
    int layer_pass;
    void *cache_data;
};

struct MirrorGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *object;
    void *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    int layer_pass;
    char _pad[4];
};

struct HookGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *object;
    void *material;
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
    void *curfalloff;
};

struct SimplifyGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    char materialname[64];
    int pass_index;
    int flag;
    float factor;
    short mode;
    short step;
    int layer_pass;
    float length;
    float sharp_threshold;
    float distance;
};

struct OffsetGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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
    int mode;
    int stroke_step;
    int stroke_start_offset;
    int layer_pass;
    char _pad[4];
};

struct SmoothGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    char materialname[64];
    char vgname[64];
    int pass_index;
    int flag;
    float factor;
    int step;
    int layer_pass;
    char _pad1[4];
    void *curve_intensity;
};

struct ArmatureGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    short deformflag;
    short multi;
    int _pad;
    void *object;
    float *(vert_coords_prev[3]);
    char vgname[64];
};

struct LayerCollection3_6_0 {
    struct LayerCollection3_6_0 *next;
    struct LayerCollection3_6_0 *prev;
    void *collection;
    struct SceneCollection3_6_0 *scene_collection;
    short flag;
    short runtime_flag;
    char _pad[4];
    struct ListBase3_6_0 layer_collections;
    unsigned short local_collections_bits;
    short _pad2[3];
};

struct SceneCollection3_6_0 {
    struct SceneCollection3_6_0 *next;
    struct SceneCollection3_6_0 *prev;
    char name[64];
    int active_object_index;
    short flag;
    char type;
    char _pad;
    struct ListBase3_6_0 objects;
    struct ListBase3_6_0 scene_collections;
};

struct SceneDisplay3_6_0 {
    float light_direction[3];
    float shadow_shift;
    float shadow_focus;
    float matcap_ssao_distance;
    float matcap_ssao_attenuation;
    int matcap_ssao_samples;
    char viewport_aa;
    char render_aa;
    char _pad[6];
    struct View3DShading3_6_0 shading;
};

struct NodeCryptomatte_Runtime3_6_0 {
    struct ListBase3_6_0 layers;
    float add[3];
    float remove[3];
};

struct SoftBody_Shared3_6_0 {
    void *pointcache;
    struct ListBase3_6_0 ptcaches;
};

struct RigidBodyWorld_Shared3_6_0 {
    void *pointcache;
    struct ListBase3_6_0 ptcaches;
    void *physics_world;
};

struct BlurShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    float radius[2];
    int flag;
    int samples;
    float rotation;
    char _pad[4];
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct ColorizeShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    int mode;
    float low_color[4];
    float high_color[4];
    float factor;
    int flag;
    char _pad[4];
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct FlipShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    int flag;
    int flipmode;
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct GlowShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
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
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct PixelShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    int size[3];
    int flag;
    float rgba[4];
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct RimShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    int offset[2];
    int flag;
    float rim_rgb[3];
    float mask_rgb[3];
    int mode;
    int blur[2];
    int samples;
    char _pad[4];
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct ShadowShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    void *object;
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
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct SwirlShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    void *object;
    int flag;
    int radius;
    float angle;
    int transparent;
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct WaveShaderFxData3_6_0 {
    struct ShaderFxData3_6_0 shaderfx;
    float amplitude;
    float period;
    float phase;
    int orientation;
    int flag;
    char _pad[4];
    struct ShaderFxData_Runtime3_6_0 runtime;
};

struct SpaceGraph_Runtime3_6_0 {
    char flag;
    char _pad[7];
    struct ListBase3_6_0 ghost_curves;
};

struct SpaceTopBar3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
};

struct SpaceStatusBar3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
};

struct ImageTile3_6_0 {
    struct ImageTile3_6_0 *next;
    struct ImageTile3_6_0 *prev;
    struct ImageTile_Runtime3_6_0 runtime;
    int tile_number;
    int gen_x;
    int gen_y;
    char gen_type;
    char gen_flag;
    short gen_depth;
    float gen_color[4];
    char label[64];
};

struct CurveProfile3_6_0 {
    short path_len;
    short segments_len;
    int preset;
    struct CurveProfilePoint3_6_0 *path;
    struct CurveProfilePoint3_6_0 *table;
    struct CurveProfilePoint3_6_0 *segments;
    int flag;
    int changed_timestamp;
    struct rctf3_6_0 view_rect;
    struct rctf3_6_0 clip_rect;
};

struct MultiplyGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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

struct GP_Sculpt_Settings3_6_0 {
    void *paintcursor;
    int flag;
    int lock_axis;
    float isect_threshold;
    char _pad[4];
    void *cur_falloff;
    void *cur_primitive;
    struct GP_Sculpt_Guide3_6_0 guide;
};

struct XrSessionSettings3_6_0 {
    struct View3DShading3_6_0 shading;
    float base_scale;
    char _pad[3];
    char base_pose_type;
    struct Object3_6_0 *base_pose_object;
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

struct TextureGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
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

struct AssetMetaData3_6_0 {
    void *local_type_info;
    struct IDProperty3_6_0 *properties;
    struct bUUID3_6_0 catalog_id;
    char catalog_simple_name[64];
    char *author;
    char *description;
    char *copyright;
    char *license;
    struct ListBase3_6_0 tags;
    short active_tag;
    short tot_tags;
    char _pad[4];
};

struct bGPDcurve_point3_6_0 {
    struct BezTriple3_6_0 bezt;
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

struct FileAssetSelectParams3_6_0 {
    struct FileSelectParams3_6_0 base_params;
    struct AssetLibraryReference3_6_0 asset_library_ref;
    short asset_catalog_visibility;
    char _pad[6];
    struct bUUID3_6_0 catalog_id;
    short import_type;
    char _pad2[6];
};

struct FileFolderHistory3_6_0 {
    void *next;
    void *prev;
    char browse_mode;
    char _pad[7];
    struct ListBase3_6_0 folders_prev;
    struct ListBase3_6_0 folders_next;
};

struct LineartGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    unsigned short edge_types;
    char source_type;
    char use_multiple_levels;
    short level_start;
    short level_end;
    void *source_camera;
    void *light_contour_object;
    void *source_object;
    void *source_collection;
    void *target_material;
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

struct IDPropertyUIDataInt3_6_0 {
    struct IDPropertyUIData3_6_0 base;
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

struct IDPropertyUIDataFloat3_6_0 {
    struct IDPropertyUIData3_6_0 base;
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

struct IDPropertyUIDataString3_6_0 {
    struct IDPropertyUIData3_6_0 base;
    char *default_value;
};

struct IDPropertyUIDataID3_6_0 {
    struct IDPropertyUIData3_6_0 base;
    short id_type;
    char _pad[6];
};

struct AssetFilterSettings3_6_0 {
    struct ListBase3_6_0 tags;
    unsigned long long id_types;
};

struct LengthGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    int pass_index;
    int flag;
    int layer_pass;
    float start_fac;
    float end_fac;
    float rand_start_fac;
    float rand_end_fac;
    float rand_offset;
    float overshoot_fac;
    int seed;
    int step;
    int mode;
    char _pad[4];
    float point_density;
    float segment_influence;
    float max_angle;
};

struct DashGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    int pass_index;
    int flag;
    int layer_pass;
    int dash_offset;
    struct DashGpencilModifierSegment3_6_0 *segments;
    int segments_len;
    int segment_active_index;
};

struct WeightProxGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    char target_vgname[64];
    void *material;
    char layername[64];
    char vgname[64];
    int pass_index;
    int flag;
    float min_weight;
    int layer_pass;
    float dist_start;
    float dist_end;
    void *object;
};

struct WeightAngleGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    char target_vgname[64];
    void *material;
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

struct XrActionMapBinding3_6_0 {
    struct XrActionMapBinding3_6_0 *next;
    struct XrActionMapBinding3_6_0 *prev;
    char name[64];
    char profile[256];
    struct ListBase3_6_0 component_paths;
    float float_threshold;
    short axis_flag;
    char _pad[2];
    float pose_location[3];
    float pose_rotation[3];
};

struct XrActionMapItem3_6_0 {
    struct XrActionMapItem3_6_0 *next;
    struct XrActionMapItem3_6_0 *prev;
    char name[64];
    char type;
    char _pad[7];
    struct ListBase3_6_0 user_paths;
    char op[64];
    struct IDProperty3_6_0 *op_properties;
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
    struct ListBase3_6_0 bindings;
};

struct XrActionMap3_6_0 {
    struct XrActionMap3_6_0 *next;
    struct XrActionMap3_6_0 *prev;
    char name[64];
    struct ListBase3_6_0 items;
    short selitem;
    char _pad[6];
};

struct ShrinkwrapGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *target;
    void *aux_target;
    void *material;
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

struct CurvesGeometry3_6_0 {
    int *curve_offsets;
    struct CustomData3_6_0 point_data;
    struct CustomData3_6_0 curve_data;
    int point_num;
    int curve_num;
    void *runtime;
};

struct EnvelopeGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *material;
    char layername[64];
    char vgname[64];
    int pass_index;
    int flag;
    int mode;
    int mat_nr;
    float thickness;
    float strength;
    int skip;
    int layer_pass;
    int spread;
    char _pad[4];
};

struct PaintModeSettings3_6_0 {
    char canvas_source;
    char _pad[7];
    struct Image3_6_0 *canvas_image;
    struct ImageUser3_6_0 image_user;
};

struct OutlineGpencilModifierData3_6_0 {
    struct GpencilModifierData3_6_0 modifier;
    void *object;
    void *material;
    char layername[64];
    int pass_index;
    int flag;
    int thickness;
    float sample_length;
    int subdiv;
    int layer_pass;
    void *outline_material;
};

struct IDViewerPathElem3_6_0 {
    struct ViewerPathElem3_6_0 base;
    struct ID3_6_0 *id;
};

struct ModifierViewerPathElem3_6_0 {
    struct ViewerPathElem3_6_0 base;
    char *modifier_name;
};

struct NodeViewerPathElem3_6_0 {
    struct ViewerPathElem3_6_0 base;
    int node_id;
    char _pad1[4];
    char *node_name;
};

struct IDPropertyUIDataBool3_6_0 {
    struct IDPropertyUIData3_6_0 base;
    signed char *default_array;
    int default_array_len;
    char _pad[3];
    signed char default_value;
};

struct LightProbeGridCacheFrame3_6_0 {
    int size[3];
    int data_layout;
    int block_len;
    int block_size;
    struct LightProbeBlockData3_6_0 *block_infos;
    struct LightProbeBakingData3_6_0 baking;
    struct LightProbeIrradianceData3_6_0 irradiance;
    struct LightProbeVisibilityData3_6_0 visibility;
    struct LightProbeConnectivityData3_6_0 connectivity;
    char _pad[4];
    int surfels_len;
    void *surfels;
};

struct ID3_6_0 {
    void *next;
    void *prev;
    struct ID3_6_0 *newid;
    struct Library3_6_0 *lib;
    void *asset_data;
    char name[66];
    short flag;
    int tag;
    int us;
    int icon_id;
    unsigned int recalc;
    unsigned int recalc_up_to_undo_push;
    unsigned int recalc_after_undo_push;
    unsigned int session_uuid;
    struct IDProperty3_6_0 *properties;
    struct IDOverrideLibrary3_6_0 *override_library;
    struct ID3_6_0 *orig_id;
    void *py_instance;
    struct LibraryWeakReference3_6_0 *library_weak_reference;
    struct ID_Runtime3_6_0 runtime;
};

struct bPoseChannel3_6_0 {
    struct bPoseChannel3_6_0 *next;
    struct bPoseChannel3_6_0 *prev;
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
    void *bone;
    struct bPoseChannel3_6_0 *parent;
    struct bPoseChannel3_6_0 *child;
    struct ListBase3_6_0 iktree;
    struct ListBase3_6_0 siktree;
    struct bMotionPath3_6_0 *mpath;
    struct Object3_6_0 *custom;
    struct bPoseChannel3_6_0 *custom_tx;
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
    struct bPoseChannel3_6_0 *bbone_prev;
    struct bPoseChannel3_6_0 *bbone_next;
    void *temp;
    struct bPoseChannelDrawData3_6_0 *draw_data;
    struct bPoseChannel3_6_0 *orig_pchan;
    struct bPoseChannel_Runtime3_6_0 runtime;
};

struct SpaceAction3_6_0 {
    void *next;
    void *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D3_6_0 v2d;
    struct bAction3_6_0 *action;
    struct bDopeSheet3_6_0 ads;
    float timeslide;
    short flag;
    char mode;
    char mode_prev;
    char autosnap;
    char cache_display;
    char _pad1[6];
    struct SpaceAction_Runtime3_6_0 runtime;
};

struct Sequence3_6_0 {
    struct Sequence3_6_0 *next;
    struct Sequence3_6_0 *prev;
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
    int _pad3;
    int startdisp;
    int enddisp;
    float sat;
    float mul;
    float _pad;
    short anim_preseek;
    short streamindex;
    int multicam_source;
    int clip_flag;
    struct Strip3_6_0 *strip;
    void *ipo;
    void *scene;
    struct Object3_6_0 *scene_camera;
    struct MovieClip3_6_0 *clip;
    void *mask;
    struct ListBase3_6_0 anims;
    float effect_fader;
    float speed_fader;
    struct Sequence3_6_0 *seq1;
    struct Sequence3_6_0 *seq2;
    struct Sequence3_6_0 *seq3;
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
    char _pad4[2];
    int cache_flag;
    int sfra;
    char views_format;
    char _pad1[3];
    struct Stereo3dFormat3_6_0 *stereo3d_format;
    struct IDProperty3_6_0 *prop;
    struct ListBase3_6_0 modifiers;
    float media_playback_rate;
    float speed_factor;
    struct SequenceRuntime3_6_0 runtime;
};

struct SpaceSeq3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D3_6_0 v2d;
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
    struct SequencerScopes3_6_0 scopes;
    struct SequencerPreviewOverlay3_6_0 preview_overlay;
    struct SequencerTimelineOverlay3_6_0 timeline_overlay;
    char multiview_eye;
    char _pad2[7];
    struct SpaceSeqRuntime3_6_0 runtime;
};

struct SpaceImage3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct Image3_6_0 *image;
    struct ImageUser3_6_0 iuser;
    struct Scopes3_6_0 scopes;
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

struct SpaceNla3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    short autosnap;
    short flag;
    char _pad[4];
    struct bDopeSheet3_6_0 *ads;
    struct View2D3_6_0 v2d;
};

struct SpaceText3_6_0 {
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
    struct SpaceText_Runtime3_6_0 runtime;
};

struct View3D3_6_0 {
    void *next;
    void *prev;
    struct ListBase3_6_0 regionbase;
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
    struct Object3_6_0 *camera;
    struct Object3_6_0 *ob_center;
    struct rctf3_6_0 render_border;
    struct View3D3_6_0 *localvd;
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
    float lens;
    float grid;
    float clip_start;
    float clip_end;
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
    void *gpd;
    short stereo3d_flag;
    char stereo3d_camera;
    char _pad4;
    float stereo3d_convergence_factor;
    float stereo3d_volume_alpha;
    float stereo3d_convergence_alpha;
    struct View3DShading3_6_0 shading;
    struct View3DOverlay3_6_0 overlay;
    struct ViewerPath3_6_0 viewer_path;
    struct View3D_Runtime3_6_0 runtime;
};

struct SubsurfModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct LatticeModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *object;
    char name[64];
    float strength;
    short flag;
    char _pad[2];
    void *_pad1;
};

struct CurveModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *object;
    char name[64];
    short defaxis;
    short flag;
    char _pad[4];
    void *_pad1;
};

struct BuildModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float start;
    float length;
    short flag;
    short randomize;
    int seed;
};

struct MirrorModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct DecimateModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct WaveModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ArmatureModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    short deformflag;
    short multi;
    char _pad2[4];
    void *object;
    float *(vert_coords_prev[3]);
    char defgrp_name[64];
};

struct HookModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct SoftbodyModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
};

struct BooleanModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ArrayModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct bNodePreview3_6_0 {
    struct bNodeInstanceHashEntry3_6_0 hash_entry;
    unsigned char *rect;
    short xsize;
    short ysize;
};

struct SceneRenderLayer3_6_0 {
    struct SceneRenderLayer3_6_0 *next;
    struct SceneRenderLayer3_6_0 *prev;
    char name[64];
    void *mat_override;
    unsigned int lay;
    unsigned int lay_zmask;
    unsigned int lay_exclude;
    int layflag;
    int passflag;
    int pass_xor;
    int samples;
    float pass_alpha_threshold;
    struct IDProperty3_6_0 *prop;
    struct FreestyleConfig3_6_0 freestyleConfig;
};

struct SpaceNode3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D3_6_0 v2d;
    struct ID3_6_0 *id;
    struct ID3_6_0 *from;
    short flag;
    char insert_ofs_dir;
    char _pad1;
    float xof;
    float yof;
    float zoom;
    struct ListBase3_6_0 treepath;
    struct bNodeTree3_6_0 *edittree;
    struct bNodeTree3_6_0 *nodetree;
    char tree_idname[64];
    int treetype;
    short texfrom;
    short shaderfrom;
    void *gpd;
    struct SpaceNodeOverlay3_6_0 overlay;
    char _pad2[4];
    void *runtime;
};

struct IDProperty3_6_0 {
    struct IDProperty3_6_0 *next;
    struct IDProperty3_6_0 *prev;
    char type;
    char subtype;
    short flag;
    char name[64];
    char _pad0[4];
    struct IDPropertyData3_6_0 data;
    int len;
    int totallen;
    struct IDPropertyUIData3_6_0 *ui_data;
};

struct EdgeSplitModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float split_angle;
    int flags;
};

struct DisplaceModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct UVProjectModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct NodeImageFile3_6_0 {
    char name[1024];
    struct ImageFormatData3_6_0 im_format;
    int sfra;
    int efra;
};

struct ImagePaintSettings3_6_0 {
    struct Paint3_6_0 paint;
    short flag;
    short missing_data;
    short seam_bleed;
    short normal_angle;
    short screen_grab_size[2];
    int mode;
    struct Image3_6_0 *stencil;
    struct Image3_6_0 *clone;
    struct Image3_6_0 *canvas;
    float stencil_col[3];
    float dither;
    int interp;
    char _pad[4];
};

struct SmoothModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float fac;
    char defgrp_name[64];
    short flag;
    short repeat;
};

struct CastModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *object;
    float fac;
    float radius;
    float size;
    char defgrp_name[64];
    short flag;
    short type;
    void *_pad1;
};

struct BevelModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ClothModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct CollisionModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float *(x[3]);
    float *(xnew[3]);
    float *(xold[3]);
    float *(current_xnew[3]);
    float *(current_x[3]);
    float *(current_v[3]);
    void *tri;
    unsigned int mvert_num;
    unsigned int tri_num;
    float time_x;
    float time_xnew;
    char is_static;
    char _pad[7];
    void *bvhtree;
};

struct ParticleSystemModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ParticleInstanceModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ExplodeModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    int *facepa;
    short flag;
    short vgroup;
    float protect;
    char uvname[68];
    char _pad1[4];
    void *_pad2;
};

struct MaskModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *ob_arm;
    char vgroup[64];
    short mode;
    short flag;
    float threshold;
    void *_pad1;
};

struct FluidsimModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *fss;
    void *_pad1;
};

struct ShrinkwrapModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct SimpleDeformModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct SurfaceModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    struct SurfaceModifierData_Runtime3_6_0 runtime;
};

struct SmokeModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    int type;
    int _pad;
};

struct MultiresModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ShapeKeyModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
};

struct Sculpt3_6_0 {
    struct Paint3_6_0 paint;
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
    float automasking_start_normal_limit;
    float automasking_start_normal_falloff;
    float automasking_view_normal_limit;
    float automasking_view_normal_falloff;
    void *automasking_cavity_curve;
    void *automasking_cavity_curve_op;
    struct Object3_6_0 *gravity_object;
};

struct VPaint3_6_0 {
    struct Paint3_6_0 paint;
    char flag;
    char _pad[3];
    int radial_symm[3];
};

struct ARegion3_6_0 {
    struct ARegion3_6_0 *next;
    struct ARegion3_6_0 *prev;
    struct View2D3_6_0 v2d;
    struct rcti3_6_0 winrct;
    struct rcti3_6_0 drawrct;
    short winx;
    short winy;
    int category_scroll;
    char _pad0[4];
    short visible;
    short regiontype;
    short alignment;
    short flag;
    short sizex;
    short sizey;
    short do_draw;
    short do_draw_paintcursor;
    short overlap;
    short flagfullscreen;
    void *type;
    struct ListBase3_6_0 uiblocks;
    struct ListBase3_6_0 panels;
    struct ListBase3_6_0 panels_category_active;
    struct ListBase3_6_0 ui_lists;
    struct ListBase3_6_0 ui_previews;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 panels_category;
    void *gizmo_map;
    void *regiontimer;
    void *draw_buffer;
    char *headerstr;
    void *regiondata;
    struct ARegion_Runtime3_6_0 runtime;
};

struct wmXrData3_6_0 {
    void *runtime;
    struct XrSessionSettings3_6_0 session_settings;
};

struct wmWindow3_6_0 {
    struct wmWindow3_6_0 *next;
    struct wmWindow3_6_0 *prev;
    void *ghostwin;
    void *gpuctx;
    struct wmWindow3_6_0 *parent;
    void *scene;
    void *new_scene;
    char view_layer_name[64];
    void *unpinned_scene;
    void *workspace_hook;
    struct ScrAreaMap3_6_0 global_areas;
    struct bScreen3_6_0 *screen;
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
    void *ime_data;
    struct ListBase3_6_0 event_queue;
    struct ListBase3_6_0 handlers;
    struct ListBase3_6_0 modalhandlers;
    struct ListBase3_6_0 gesture;
    void *stereo3d_format;
    struct ListBase3_6_0 drawcalls;
    void *cursor_keymap_status;
};

struct SolidifyModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ScrewModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct MappingInfoModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *texture;
    void *map_object;
    char map_bone[64];
    char uvlayer_name[68];
    char _pad1[4];
    int uvlayer_tmp;
    int texmapping;
};

struct WarpModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct WeightVGEditModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct WeightVGMixModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct WeightVGProximityModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct OceanModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct DynamicPaintModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *canvas;
    void *brush;
    int type;
    char _pad[4];
};

struct MovieTracking3_6_0 {
    struct MovieTrackingSettings3_6_0 settings;
    struct MovieTrackingCamera3_6_0 camera;
    struct ListBase3_6_0 tracks_legacy;
    struct ListBase3_6_0 plane_tracks_legacy;
    struct MovieTrackingReconstruction3_6_0 reconstruction_legacy;
    struct MovieTrackingStabilization3_6_0 stabilization;
    struct MovieTrackingTrack3_6_0 *act_track_legacy;
    struct MovieTrackingPlaneTrack3_6_0 *act_plane_track_legacy;
    struct ListBase3_6_0 objects;
    int objectnr;
    int tot_object;
    struct MovieTrackingStats3_6_0 *stats;
    struct MovieTrackingDopesheet3_6_0 dopesheet;
};

struct SpaceClip3_6_0 {
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
    struct MovieClip3_6_0 *clip;
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

struct RemeshModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct UvSculpt3_6_0 {
    struct Paint3_6_0 paint;
};

struct NodeImageMultiFile3_6_0 {
    char base_path[1024];
    struct ImageFormatData3_6_0 format;
    int sfra;
    int efra;
    int active_input;
    char _pad[4];
};

struct NodeImageMultiFileSocket3_6_0 {
    short use_render_format;
    short use_node_format;
    char save_as_render;
    char _pad1[3];
    char path[1024];
    struct ImageFormatData3_6_0 format;
    char layer[30];
    char _pad2[2];
};

struct SkinModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float branch_smoothing;
    char flag;
    char symmetry_axes;
    char _pad[2];
};

struct TriangulateModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    int flag;
    int quad_method;
    int ngon_method;
    int min_vertices;
};

struct LaplacianSmoothModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float lambda;
    float lambda_border;
    char _pad1[4];
    char defgrp_name[64];
    short flag;
    short repeat;
};

struct UVWarpModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct MeshCacheModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct LaplacianDeformModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    char anchor_grp_name[64];
    int verts_num;
    int repeat;
    float *vertexco;
    void *cache_system;
    short flag;
    char _pad[6];
};

struct WireframeModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    char defgrp_name[64];
    float offset;
    float offset_fac;
    float offset_fac_vg;
    float crease_weight;
    short flag;
    short mat_ofs;
    char _pad[4];
};

struct BakeData3_6_0 {
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
    struct Object3_6_0 *cage_object;
};

struct DataTransferModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct NormalEditModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct CorrectiveSmoothModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct MeshSeqCacheModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *cache_file;
    char object_path[1024];
    char read_flag;
    char _pad[3];
    float velocity_scale;
    void *reader;
    char reader_object_path[1024];
};

struct SurfaceDeformModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct ViewLayer3_6_0 {
    struct ViewLayer3_6_0 *next;
    struct ViewLayer3_6_0 *prev;
    char name[64];
    short flag;
    char _pad[6];
    struct ListBase3_6_0 object_bases;
    void *stats;
    struct Base3_6_0 *basact;
    struct ListBase3_6_0 layer_collections;
    struct LayerCollection3_6_0 *active_collection;
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
    struct Base3_6_0 **object_bases_array;
    void *object_bases_hash;
};

struct WeightedNormalModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    char defgrp_name[64];
    char mode;
    char flag;
    short weight;
    float thresh;
};

struct GpPaint3_6_0 {
    struct Paint3_6_0 paint;
    int flag;
    int mode;
};

struct NodeCryptomatte3_6_0 {
    struct ImageUser3_6_0 iuser;
    struct ListBase3_6_0 entries;
    char layer_name[64];
    char *matte_id;
    int inputs_num;
    char _pad[4];
    struct NodeCryptomatte_Runtime3_6_0 runtime;
};

struct SpaceProperties3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D3_6_0 v2d;
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
    struct ID3_6_0 *pinid;
    void *texuser;
    void *runtime;
};

struct SpaceOutliner3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D3_6_0 v2d;
    struct ListBase3_6_0 tree;
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

struct SpaceGraph3_6_0 {
    struct SpaceLink3_6_0 *next;
    struct SpaceLink3_6_0 *prev;
    struct ListBase3_6_0 regionbase;
    char spacetype;
    char link_flag;
    char _pad0[6];
    struct View2D3_6_0 v2d;
    struct bDopeSheet3_6_0 *ads;
    short mode;
    short autosnap;
    int flag;
    float cursorTime;
    float cursorVal;
    int around;
    char _pad[4];
    struct SpaceGraph_Runtime3_6_0 runtime;
};

struct FluidModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *domain;
    void *flow;
    void *effector;
    float time;
    int type;
    void *_pad1;
};

struct WeldModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    float merge_dist;
    char defgrp_name[64];
    char mode;
    char flag;
    char _pad[2];
};

struct GpVertexPaint3_6_0 {
    struct Paint3_6_0 paint;
    int flag;
    char _pad[4];
};

struct GpSculptPaint3_6_0 {
    struct Paint3_6_0 paint;
    int flag;
    char _pad[4];
};

struct GpWeightPaint3_6_0 {
    struct Paint3_6_0 paint;
    int flag;
    char _pad[4];
};

struct MeshToVolumeModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *object;
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

struct VolumeDisplaceModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *texture;
    void *texture_map_object;
    int texture_map_mode;
    float strength;
    float texture_mid_level[3];
    float texture_sample_radius;
};

struct VolumeToMeshModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
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

struct NodesModifierData3_6_0 {
    struct ModifierData3_6_0 modifier;
    void *node_group;
    struct NodesModifierSettings3_6_0 settings;
    char *simulation_bake_directory;
    void *_pad;
    void *runtime_eval_log;
    void *simulation_cache;
};

struct SpaceSpreadsheet3_6_0 {
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
    unsigned int flag;
    void *runtime;
};

struct CurvesSculpt3_6_0 {
    struct Paint3_6_0 paint;
};

struct Library3_6_0 {
    struct ID3_6_0 id;
    void *filedata;
    char filepath[1024];
    char filepath_abs[1024];
    struct Library3_6_0 *parent;
    void *packedfile;
    unsigned short tag;
    char _pad_0[6];
    int temp_index;
    short versionfile;
    short subversionfile;
    struct Library_Runtime3_6_0 runtime;
};

struct bAction3_6_0 {
    struct ID3_6_0 id;
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

struct bArmature3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    struct ListBase3_6_0 bonebase;
    void *bonehash;
    void *_pad1;
    struct ListBase3_6_0 *edbo;
    struct Bone3_6_0 *act_bone;
    void *act_edbone;
    char needs_flush_to_id;
    char _pad0[3];
    int flag;
    int drawtype;
    short deformflag;
    short pathflag;
    unsigned int layer_used;
    unsigned int layer;
    unsigned int layer_protected;
    float axes_position;
};

struct Camera3_6_0 {
    struct ID3_6_0 id;
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
    void *ipo;
    void *dof_ob;
    struct GPUDOFSettings3_6_0 gpu_dof;
    struct CameraDOFSettings3_6_0 dof;
    struct ListBase3_6_0 bg_images;
    char sensor_fit;
    char _pad[7];
    struct CameraStereoSettings3_6_0 stereo;
    struct Camera_Runtime3_6_0 runtime;
};

struct Ipo3_6_0 {
    struct ID3_6_0 id;
    struct ListBase3_6_0 curve;
    struct rctf3_6_0 cur;
    short blocktype;
    short showkey;
    short muteipo;
    char _pad[2];
};

struct Object3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    struct DrawDataList3_6_0 drawdata;
    void *sculpt;
    short type;
    short partype;
    int par1;
    int par2;
    int par3;
    char parsubstr[64];
    struct Object3_6_0 *parent;
    struct Object3_6_0 *track;
    struct Object3_6_0 *proxy;
    struct Object3_6_0 *proxy_group;
    struct Object3_6_0 *proxy_from;
    void *ipo;
    struct bAction3_6_0 *action;
    struct bAction3_6_0 *poselib;
    struct bPose3_6_0 *pose;
    void *data;
    void *gpd;
    struct bAnimVizSettings3_6_0 avs;
    struct bMotionPath3_6_0 *mpath;
    void *_pad0;
    struct ListBase3_6_0 constraintChannels;
    struct ListBase3_6_0 effect;
    struct ListBase3_6_0 defbase;
    struct ListBase3_6_0 modifiers;
    struct ListBase3_6_0 greasepencil_modifiers;
    struct ListBase3_6_0 fmaps;
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
    unsigned short actfmap;
    char _pad2[2];
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
    void *lightprobe_cache;
    void *_pad9;
    struct Object_Runtime3_6_0 runtime;
};

struct Curve3_6_0 {
    struct ID3_6_0 id;
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

struct Image3_6_0 {
    struct ID3_6_0 id;
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

struct Key3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    struct KeyBlock3_6_0 *refkey;
    char elemstr[32];
    int elemsize;
    char _pad[4];
    struct ListBase3_6_0 block;
    void *ipo;
    struct ID3_6_0 *from;
    int totkey;
    short flag;
    char type;
    char _pad2;
    float ctime;
    int uidgen;
};

struct Lattice3_6_0 {
    struct ID3_6_0 id;
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
    struct EditLatt3_6_0 *editlatt;
    void *batch_cache;
};

struct Material3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    short flag;
    char _pad1[2];
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
    char _pad2[2];
    float alpha_threshold;
    float refract_depth;
    char blend_method;
    char blend_shadow;
    char blend_flag;
    char _pad3[1];
    struct TexPaintSlot3_6_0 *texpaintslot;
    struct ListBase3_6_0 gpumaterial;
    struct MaterialGPencilStyle3_6_0 *gp_style;
    struct MaterialLineArt3_6_0 lineart;
};

struct Mesh3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    void *ipo;
    void *key;
    void *mat;
    int totvert;
    int totedge;
    int totpoly;
    int totloop;
    int *poly_offset_indices;
    struct CustomData3_6_0 vdata;
    struct CustomData3_6_0 edata;
    struct CustomData3_6_0 pdata;
    struct CustomData3_6_0 ldata;
    struct ListBase3_6_0 vertex_group_names;
    int vertex_group_active_index;
    int attributes_active_index;
    void *edit_mesh;
    struct MSelect3_6_0 *mselect;
    int totselect;
    int act_face;
    struct Mesh3_6_0 *texcomesh;
    float texspace_location[3];
    float texspace_size[3];
    char texspace_flag;
    char editflag;
    unsigned short flag;
    float smoothresh;
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
    struct MDeformVert3_6_0 *dvert;
    struct MTFace3_6_0 *mtface;
    void *tface;
    struct MCol3_6_0 *mcol;
    struct MFace3_6_0 *mface;
    struct CustomData3_6_0 fdata;
    int totface;
    char _pad1[4];
    void *runtime;
};

struct MetaBall3_6_0 {
    struct ID3_6_0 id;
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

struct bScreen3_6_0 {
    struct ID3_6_0 id;
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
    struct ARegion3_6_0 *active_region;
    void *animtimer;
    void *context;
    void *tool_tip;
    struct PreviewImage3_6_0 *preview;
};

struct bSound3_6_0 {
    struct ID3_6_0 id;
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

struct Tex3_6_0 {
    struct ID3_6_0 id;
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
    struct Image3_6_0 *ima;
    void *coba;
    struct PreviewImage3_6_0 *preview;
    char use_nodes;
    char _pad[7];
};

struct Text3_6_0 {
    struct ID3_6_0 id;
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

struct VFont3_6_0 {
    struct ID3_6_0 id;
    char filepath[1024];
    void *data;
    void *packedfile;
    void *temp_pf;
};

struct World3_6_0 {
    struct ID3_6_0 id;
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
    char _pad3[6];
    void *ipo;
    short pr_texture;
    short use_nodes;
    char _pad[4];
    struct PreviewImage3_6_0 *preview;
    void *nodetree;
    void *lightgroup;
    struct ListBase3_6_0 gpumaterial;
};

struct Brush3_6_0 {
    struct ID3_6_0 id;
    struct BrushClone3_6_0 clone;
    void *curve;
    struct MTex3_6_0 mtex;
    struct MTex3_6_0 mask_mtex;
    struct Brush3_6_0 *toggle_brush;
    void *icon_imbuf;
    struct PreviewImage3_6_0 *preview;
    void *gradient;
    struct PaintCurve3_6_0 *paint_curve;
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
    struct BrushGpencilSettings3_6_0 *gpencil_settings;
    struct BrushCurvesSculptSettings3_6_0 *curves_sculpt_settings;
    int automasking_cavity_blur_steps;
    float automasking_cavity_factor;
    void *automasking_cavity_curve;
};

struct ParticleSettings3_6_0 {
    struct ID3_6_0 id;
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

struct Script3_6_0 {
    struct ID3_6_0 id;
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

struct bGPdata3_6_0 {
    struct ID3_6_0 id;
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
    struct bGPdata_Runtime3_6_0 runtime;
};

struct IdAdtTemplate3_6_0 {
    struct ID3_6_0 id;
    struct AnimData3_6_0 *adt;
};

struct wmWindowManager3_6_0 {
    struct ID3_6_0 id;
    struct wmWindow3_6_0 *windrawable;
    struct wmWindow3_6_0 *winactive;
    struct ListBase3_6_0 windows;
    short initialized;
    short file_saved;
    short op_undo_depth;
    short outliner_sync_select_dirty;
    struct ListBase3_6_0 operators;
    struct ListBase3_6_0 notifier_queue;
    void *notifier_queue_set;
    struct ReportList3_6_0 reports;
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
    struct wmXrData3_6_0 xr;
};

struct Speaker3_6_0 {
    struct ID3_6_0 id;
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

struct MovieClip3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    char filepath[1024];
    int source;
    int _pad;
    int lastsize[2];
    float aspx;
    float aspy;
    void *anim;
    void *cache;
    void *gpd;
    struct MovieTracking3_6_0 tracking;
    void *tracking_context;
    struct MovieClipProxy3_6_0 proxy;
    int flag;
    int len;
    int start_frame;
    int frame_offset;
    struct ColorManagedColorspaceSettings3_6_0 colorspace_settings;
    struct MovieClip_Runtime3_6_0 runtime;
};

struct Mask3_6_0 {
    struct ID3_6_0 id;
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

struct FreestyleLineStyle3_6_0 {
    struct ID3_6_0 id;
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

struct Palette3_6_0 {
    struct ID3_6_0 id;
    struct ListBase3_6_0 colors;
    int active_color;
    char _pad[4];
};

struct PaintCurve3_6_0 {
    struct ID3_6_0 id;
    struct PaintCurvePoint3_6_0 *points;
    int tot_points;
    int add_index;
};

struct CacheFile3_6_0 {
    struct ID3_6_0 id;
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

struct Collection3_6_0 {
    struct ID3_6_0 id;
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
    void *collection;
    void *view_layer;
    struct Collection_Runtime3_6_0 runtime;
};

struct LightProbe3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    char type;
    char flag;
    char attenuation_type;
    char parallax_type;
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
    char _pad1[4];
    void *parallax_ob;
    void *image;
    void *visibility_grp;
};

struct Light3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    short type;
    short flag;
    int mode;
    float r;
    float g;
    float b;
    float k;
    float shdwr;
    float shdwg;
    float shdwb;
    float shdwpad;
    float energy;
    float dist;
    float spotsize;
    float spotblend;
    float att1;
    float att2;
    float coeff_const;
    float coeff_lin;
    float coeff_quad;
    char _pad0[4];
    void *curfalloff;
    short falloff_type;
    char _pad2[2];
    float clipsta;
    float clipend;
    float bias;
    float radius;
    short bufsize;
    short samp;
    short buffers;
    short filtertype;
    char bufflag;
    char buftype;
    short area_shape;
    float area_size;
    float area_sizey;
    float area_sizez;
    float area_spread;
    float sun_angle;
    short texact;
    short shadhalostep;
    void *ipo;
    short pr_texture;
    short use_nodes;
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
    struct PreviewImage3_6_0 *preview;
    void *nodetree;
};

struct WorkSpace3_6_0 {
    struct ID3_6_0 id;
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

struct PointCloud3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    int flag;
    int totpoint;
    struct CustomData3_6_0 pdata;
    int attributes_active_index;
    int _pad4;
    void *mat;
    short totcol;
    short _pad3[3];
    void *runtime;
    void *batch_cache;
};

struct Volume3_6_0 {
    struct ID3_6_0 id;
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
    struct Volume_Runtime3_6_0 runtime;
};

struct Simulation3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    void *nodetree;
    unsigned int flag;
    char _pad[4];
};

struct Curves3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    struct CurvesGeometry3_6_0 geometry;
    int flag;
    int attributes_active_index;
    void *mat;
    short totcol;
    char symmetry;
    char selection_domain;
    char _pad[4];
    void *surface;
    char *surface_uv_map;
    void *batch_cache;
};

struct bNodeTree3_6_0 {
    struct ID3_6_0 id;
    void *adt;
    struct ID3_6_0 *owner_id;
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
    struct rctf3_6_0 viewer_border;
    struct ListBase3_6_0 inputs;
    struct ListBase3_6_0 outputs;
    void *previews;
    struct bNodeInstanceKey3_6_0 active_viewer_key;
    char _pad[4];
    struct PreviewImage3_6_0 *preview;
    void *runtime;
};

#endif