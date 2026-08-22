/* Copyright (C) 2025 Ian Sloat
* Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. 
*
* Generated automatically with gen_headers.py */

#ifndef MAKESDNA_MACROS_H
#define MAKESDNA_MACROS_H

#define ANIMDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<AnimData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<AnimData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<AnimData4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<AnimData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<AnimData4_4_0*>(data_ptr)-> C); 
#define ANIMDATA_RETURN_REF(T, M)    ANIMDATA_BASE_RETURN_BODY(T, &, M)
#define ANIMDATA_RETURN_AS(T, M)     ANIMDATA_BASE_RETURN_BODY(T,, M)
#define ANIMDATA_RETURN(M)           ANIMDATA_BASE_RETURN_BODY(,, M)

#define ANIMOVERRIDE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AnimOverride3_6_0*>(data_ptr)-> C); 
#define ANIMOVERRIDE_RETURN_REF(T, M)    ANIMOVERRIDE_BASE_RETURN_BODY(T, &, M)
#define ANIMOVERRIDE_RETURN_AS(T, M)     ANIMOVERRIDE_BASE_RETURN_BODY(T,, M)
#define ANIMOVERRIDE_RETURN(M)           ANIMOVERRIDE_BASE_RETURN_BODY(,, M)

#define AREGION_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ARegion3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ARegion4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ARegion4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ARegion4_4_0*>(data_ptr)-> C); 
#define AREGION_RETURN_REF(T, M)    AREGION_BASE_RETURN_BODY(T, &, M)
#define AREGION_RETURN_AS(T, M)     AREGION_BASE_RETURN_BODY(T,, M)
#define AREGION_RETURN(M)           AREGION_BASE_RETURN_BODY(,, M)

#define AREGION_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ARegion_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ARegion_Runtime4_2_0*>(data_ptr)-> C); 
#define AREGION_RUNTIME_RETURN_REF(T, M)    AREGION_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define AREGION_RUNTIME_RETURN_AS(T, M)     AREGION_RUNTIME_BASE_RETURN_BODY(T,, M)
#define AREGION_RUNTIME_RETURN(M)           AREGION_RUNTIME_BASE_RETURN_BODY(,, M)

#define ARMATUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ArmatureGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define ARMATUREGPENCILMODIFIERDATA_RETURN_REF(T, M)    ARMATUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define ARMATUREGPENCILMODIFIERDATA_RETURN_AS(T, M)     ARMATUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define ARMATUREGPENCILMODIFIERDATA_RETURN(M)           ARMATUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define ARMATUREMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ArmatureModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ArmatureModifierData4_1_0*>(data_ptr)-> C); 
#define ARMATUREMODIFIERDATA_RETURN_REF(T, M)    ARMATUREMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define ARMATUREMODIFIERDATA_RETURN_AS(T, M)     ARMATUREMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define ARMATUREMODIFIERDATA_RETURN(M)           ARMATUREMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define ARRAYGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ArrayGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define ARRAYGPENCILMODIFIERDATA_RETURN_REF(T, M)    ARRAYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define ARRAYGPENCILMODIFIERDATA_RETURN_AS(T, M)     ARRAYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define ARRAYGPENCILMODIFIERDATA_RETURN(M)           ARRAYGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define ARRAYMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ArrayModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ArrayModifierData4_1_0*>(data_ptr)-> C); 
#define ARRAYMODIFIERDATA_RETURN_REF(T, M)    ARRAYMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define ARRAYMODIFIERDATA_RETURN_AS(T, M)     ARRAYMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define ARRAYMODIFIERDATA_RETURN(M)           ARRAYMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define ASSETFILTERSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AssetFilterSettings3_6_0*>(data_ptr)-> C); 
#define ASSETFILTERSETTINGS_RETURN_REF(T, M)    ASSETFILTERSETTINGS_BASE_RETURN_BODY(T, &, M)
#define ASSETFILTERSETTINGS_RETURN_AS(T, M)     ASSETFILTERSETTINGS_BASE_RETURN_BODY(T,, M)
#define ASSETFILTERSETTINGS_RETURN(M)           ASSETFILTERSETTINGS_BASE_RETURN_BODY(,, M)

#define ASSETHANDLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AssetHandle3_6_0*>(data_ptr)-> C); 
#define ASSETHANDLE_RETURN_REF(T, M)    ASSETHANDLE_BASE_RETURN_BODY(T, &, M)
#define ASSETHANDLE_RETURN_AS(T, M)     ASSETHANDLE_BASE_RETURN_BODY(T,, M)
#define ASSETHANDLE_RETURN(M)           ASSETHANDLE_BASE_RETURN_BODY(,, M)

#define ASSETLIBRARYREFERENCE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AssetLibraryReference3_6_0*>(data_ptr)-> C); 
#define ASSETLIBRARYREFERENCE_RETURN_REF(T, M)    ASSETLIBRARYREFERENCE_BASE_RETURN_BODY(T, &, M)
#define ASSETLIBRARYREFERENCE_RETURN_AS(T, M)     ASSETLIBRARYREFERENCE_BASE_RETURN_BODY(T,, M)
#define ASSETLIBRARYREFERENCE_RETURN(M)           ASSETLIBRARYREFERENCE_BASE_RETURN_BODY(,, M)

#define ASSETMETADATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AssetMetaData3_6_0*>(data_ptr)-> C); 
#define ASSETMETADATA_RETURN_REF(T, M)    ASSETMETADATA_BASE_RETURN_BODY(T, &, M)
#define ASSETMETADATA_RETURN_AS(T, M)     ASSETMETADATA_BASE_RETURN_BODY(T,, M)
#define ASSETMETADATA_RETURN(M)           ASSETMETADATA_BASE_RETURN_BODY(,, M)

#define ASSETTAG_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AssetTag3_6_0*>(data_ptr)-> C); 
#define ASSETTAG_RETURN_REF(T, M)    ASSETTAG_BASE_RETURN_BODY(T, &, M)
#define ASSETTAG_RETURN_AS(T, M)     ASSETTAG_BASE_RETURN_BODY(T,, M)
#define ASSETTAG_RETURN(M)           ASSETTAG_BASE_RETURN_BODY(,, M)

#define ASSETWEAKREFERENCE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AssetWeakReference3_6_0*>(data_ptr)-> C); 
#define ASSETWEAKREFERENCE_RETURN_REF(T, M)    ASSETWEAKREFERENCE_BASE_RETURN_BODY(T, &, M)
#define ASSETWEAKREFERENCE_RETURN_AS(T, M)     ASSETWEAKREFERENCE_BASE_RETURN_BODY(T,, M)
#define ASSETWEAKREFERENCE_RETURN(M)           ASSETWEAKREFERENCE_BASE_RETURN_BODY(,, M)

#define AUDIODATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AudioData3_6_0*>(data_ptr)-> C); 
#define AUDIODATA_RETURN_REF(T, M)    AUDIODATA_BASE_RETURN_BODY(T, &, M)
#define AUDIODATA_RETURN_AS(T, M)     AUDIODATA_BASE_RETURN_BODY(T,, M)
#define AUDIODATA_RETURN(M)           AUDIODATA_BASE_RETURN_BODY(,, M)

#define AVICODECDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<AviCodecData3_6_0*>(data_ptr)-> C); 
#define AVICODECDATA_RETURN_REF(T, M)    AVICODECDATA_BASE_RETURN_BODY(T, &, M)
#define AVICODECDATA_RETURN_AS(T, M)     AVICODECDATA_BASE_RETURN_BODY(T,, M)
#define AVICODECDATA_RETURN(M)           AVICODECDATA_BASE_RETURN_BODY(,, M)

#define BACTIONCHANNEL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bActionChannel3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bActionChannel4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bActionChannel4_4_0*>(data_ptr)-> C); 
#define BACTIONCHANNEL_RETURN_REF(T, M)    BACTIONCHANNEL_BASE_RETURN_BODY(T, &, M)
#define BACTIONCHANNEL_RETURN_AS(T, M)     BACTIONCHANNEL_BASE_RETURN_BODY(T,, M)
#define BACTIONCHANNEL_RETURN(M)           BACTIONCHANNEL_BASE_RETURN_BODY(,, M)

#define BACTIONCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bActionConstraint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bActionConstraint4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bActionConstraint4_4_0*>(data_ptr)-> C); 
#define BACTIONCONSTRAINT_RETURN_REF(T, M)    BACTIONCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BACTIONCONSTRAINT_RETURN_AS(T, M)     BACTIONCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BACTIONCONSTRAINT_RETURN(M)           BACTIONCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BACTIONGROUP_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bActionGroup3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bActionGroup4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bActionGroup4_4_0*>(data_ptr)-> C); 
#define BACTIONGROUP_RETURN_REF(T, M)    BACTIONGROUP_BASE_RETURN_BODY(T, &, M)
#define BACTIONGROUP_RETURN_AS(T, M)     BACTIONGROUP_BASE_RETURN_BODY(T,, M)
#define BACTIONGROUP_RETURN(M)           BACTIONGROUP_BASE_RETURN_BODY(,, M)

#define BACTIONMODIFIER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bActionModifier3_6_0*>(data_ptr)-> C); 
#define BACTIONMODIFIER_RETURN_REF(T, M)    BACTIONMODIFIER_BASE_RETURN_BODY(T, &, M)
#define BACTIONMODIFIER_RETURN_AS(T, M)     BACTIONMODIFIER_BASE_RETURN_BODY(T,, M)
#define BACTIONMODIFIER_RETURN(M)           BACTIONMODIFIER_BASE_RETURN_BODY(,, M)

#define BACTIONSTRIP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bActionStrip3_6_0*>(data_ptr)-> C); 
#define BACTIONSTRIP_RETURN_REF(T, M)    BACTIONSTRIP_BASE_RETURN_BODY(T, &, M)
#define BACTIONSTRIP_RETURN_AS(T, M)     BACTIONSTRIP_BASE_RETURN_BODY(T,, M)
#define BACTIONSTRIP_RETURN(M)           BACTIONSTRIP_BASE_RETURN_BODY(,, M)

#define BACTION_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bAction3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bAction4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bAction4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bAction4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bAction4_4_0*>(data_ptr)-> C); 
#define BACTION_RETURN_REF(T, M)    BACTION_BASE_RETURN_BODY(T, &, M)
#define BACTION_RETURN_AS(T, M)     BACTION_BASE_RETURN_BODY(T,, M)
#define BACTION_RETURN(M)           BACTION_BASE_RETURN_BODY(,, M)

#define BADDON_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bAddon3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bAddon4_0_0*>(data_ptr)-> C); 
#define BADDON_RETURN_REF(T, M)    BADDON_BASE_RETURN_BODY(T, &, M)
#define BADDON_RETURN_AS(T, M)     BADDON_BASE_RETURN_BODY(T,, M)
#define BADDON_RETURN(M)           BADDON_BASE_RETURN_BODY(,, M)

#define BAKEDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<BakeData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BakeData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<BakeData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<BakeData4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<BakeData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BakeData4_4_0*>(data_ptr)-> C); 
#define BAKEDATA_RETURN_REF(T, M)    BAKEDATA_BASE_RETURN_BODY(T, &, M)
#define BAKEDATA_RETURN_AS(T, M)     BAKEDATA_BASE_RETURN_BODY(T,, M)
#define BAKEDATA_RETURN(M)           BAKEDATA_BASE_RETURN_BODY(,, M)

#define BANIMVIZSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bAnimVizSettings3_6_0*>(data_ptr)-> C); 
#define BANIMVIZSETTINGS_RETURN_REF(T, M)    BANIMVIZSETTINGS_BASE_RETURN_BODY(T, &, M)
#define BANIMVIZSETTINGS_RETURN_AS(T, M)     BANIMVIZSETTINGS_BASE_RETURN_BODY(T,, M)
#define BANIMVIZSETTINGS_RETURN(M)           BANIMVIZSETTINGS_BASE_RETURN_BODY(,, M)

#define BARMATURECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bArmatureConstraint3_6_0*>(data_ptr)-> C); 
#define BARMATURECONSTRAINT_RETURN_REF(T, M)    BARMATURECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BARMATURECONSTRAINT_RETURN_AS(T, M)     BARMATURECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BARMATURECONSTRAINT_RETURN(M)           BARMATURECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BARMATURE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bArmature3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bArmature4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bArmature4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bArmature4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bArmature4_4_0*>(data_ptr)-> C); 
#define BARMATURE_RETURN_REF(T, M)    BARMATURE_BASE_RETURN_BODY(T, &, M)
#define BARMATURE_RETURN_AS(T, M)     BARMATURE_BASE_RETURN_BODY(T,, M)
#define BARMATURE_RETURN(M)           BARMATURE_BASE_RETURN_BODY(,, M)

#define BASE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Base3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Base4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Base4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Base4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Base4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Base4_4_0*>(data_ptr)-> C); 
#define BASE_RETURN_REF(T, M)    BASE_BASE_RETURN_BODY(T, &, M)
#define BASE_RETURN_AS(T, M)     BASE_BASE_RETURN_BODY(T,, M)
#define BASE_RETURN(M)           BASE_BASE_RETURN_BODY(,, M)

#define BCAMERASOLVERCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bCameraSolverConstraint3_6_0*>(data_ptr)-> C); 
#define BCAMERASOLVERCONSTRAINT_RETURN_REF(T, M)    BCAMERASOLVERCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BCAMERASOLVERCONSTRAINT_RETURN_AS(T, M)     BCAMERASOLVERCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BCAMERASOLVERCONSTRAINT_RETURN(M)           BCAMERASOLVERCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BCHILDOFCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bChildOfConstraint3_6_0*>(data_ptr)-> C); 
#define BCHILDOFCONSTRAINT_RETURN_REF(T, M)    BCHILDOFCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BCHILDOFCONSTRAINT_RETURN_AS(T, M)     BCHILDOFCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BCHILDOFCONSTRAINT_RETURN(M)           BCHILDOFCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BCLAMPTOCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bClampToConstraint3_6_0*>(data_ptr)-> C); 
#define BCLAMPTOCONSTRAINT_RETURN_REF(T, M)    BCLAMPTOCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BCLAMPTOCONSTRAINT_RETURN_AS(T, M)     BCLAMPTOCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BCLAMPTOCONSTRAINT_RETURN(M)           BCLAMPTOCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BCONSTRAINTCHANNEL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bConstraintChannel3_6_0*>(data_ptr)-> C); 
#define BCONSTRAINTCHANNEL_RETURN_REF(T, M)    BCONSTRAINTCHANNEL_BASE_RETURN_BODY(T, &, M)
#define BCONSTRAINTCHANNEL_RETURN_AS(T, M)     BCONSTRAINTCHANNEL_BASE_RETURN_BODY(T,, M)
#define BCONSTRAINTCHANNEL_RETURN(M)           BCONSTRAINTCHANNEL_BASE_RETURN_BODY(,, M)

#define BCONSTRAINTTARGET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bConstraintTarget3_6_0*>(data_ptr)-> C); 
#define BCONSTRAINTTARGET_RETURN_REF(T, M)    BCONSTRAINTTARGET_BASE_RETURN_BODY(T, &, M)
#define BCONSTRAINTTARGET_RETURN_AS(T, M)     BCONSTRAINTTARGET_BASE_RETURN_BODY(T,, M)
#define BCONSTRAINTTARGET_RETURN(M)           BCONSTRAINTTARGET_BASE_RETURN_BODY(,, M)

#define BCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bConstraint3_6_0*>(data_ptr)-> C); 
#define BCONSTRAINT_RETURN_REF(T, M)    BCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BCONSTRAINT_RETURN_AS(T, M)     BCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BCONSTRAINT_RETURN(M)           BCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BDAMPTRACKCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bDampTrackConstraint3_6_0*>(data_ptr)-> C); 
#define BDAMPTRACKCONSTRAINT_RETURN_REF(T, M)    BDAMPTRACKCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BDAMPTRACKCONSTRAINT_RETURN_AS(T, M)     BDAMPTRACKCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BDAMPTRACKCONSTRAINT_RETURN(M)           BDAMPTRACKCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BDEFORMGROUP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bDeformGroup3_6_0*>(data_ptr)-> C); 
#define BDEFORMGROUP_RETURN_REF(T, M)    BDEFORMGROUP_BASE_RETURN_BODY(T, &, M)
#define BDEFORMGROUP_RETURN_AS(T, M)     BDEFORMGROUP_BASE_RETURN_BODY(T,, M)
#define BDEFORMGROUP_RETURN(M)           BDEFORMGROUP_BASE_RETURN_BODY(,, M)

#define BDISTLIMITCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bDistLimitConstraint3_6_0*>(data_ptr)-> C); 
#define BDISTLIMITCONSTRAINT_RETURN_REF(T, M)    BDISTLIMITCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BDISTLIMITCONSTRAINT_RETURN_AS(T, M)     BDISTLIMITCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BDISTLIMITCONSTRAINT_RETURN(M)           BDISTLIMITCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BDOPESHEET_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bDopeSheet3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bDopeSheet4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bDopeSheet4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bDopeSheet4_4_0*>(data_ptr)-> C); 
#define BDOPESHEET_RETURN_REF(T, M)    BDOPESHEET_BASE_RETURN_BODY(T, &, M)
#define BDOPESHEET_RETURN_AS(T, M)     BDOPESHEET_BASE_RETURN_BODY(T,, M)
#define BDOPESHEET_RETURN(M)           BDOPESHEET_BASE_RETURN_BODY(,, M)

#define BEVELMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BevelModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<BevelModifierData4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BevelModifierData4_3_0*>(data_ptr)-> C); 
#define BEVELMODIFIERDATA_RETURN_REF(T, M)    BEVELMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define BEVELMODIFIERDATA_RETURN_AS(T, M)     BEVELMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define BEVELMODIFIERDATA_RETURN(M)           BEVELMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define BEVLIST_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BevList3_6_0*>(data_ptr)-> C); 
#define BEVLIST_RETURN_REF(T, M)    BEVLIST_BASE_RETURN_BODY(T, &, M)
#define BEVLIST_RETURN_AS(T, M)     BEVLIST_BASE_RETURN_BODY(T,, M)
#define BEVLIST_RETURN(M)           BEVLIST_BASE_RETURN_BODY(,, M)

#define BEVPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BevPoint3_6_0*>(data_ptr)-> C); 
#define BEVPOINT_RETURN_REF(T, M)    BEVPOINT_BASE_RETURN_BODY(T, &, M)
#define BEVPOINT_RETURN_AS(T, M)     BEVPOINT_BASE_RETURN_BODY(T,, M)
#define BEVPOINT_RETURN(M)           BEVPOINT_BASE_RETURN_BODY(,, M)

#define BEZTRIPLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BezTriple3_6_0*>(data_ptr)-> C); 
#define BEZTRIPLE_RETURN_REF(T, M)    BEZTRIPLE_BASE_RETURN_BODY(T, &, M)
#define BEZTRIPLE_RETURN_AS(T, M)     BEZTRIPLE_BASE_RETURN_BODY(T,, M)
#define BEZTRIPLE_RETURN(M)           BEZTRIPLE_BASE_RETURN_BODY(,, M)

#define BFACEMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bFaceMap3_6_0*>(data_ptr)-> C); 
#define BFACEMAP_RETURN_REF(T, M)    BFACEMAP_BASE_RETURN_BODY(T, &, M)
#define BFACEMAP_RETURN_AS(T, M)     BFACEMAP_BASE_RETURN_BODY(T,, M)
#define BFACEMAP_RETURN(M)           BFACEMAP_BASE_RETURN_BODY(,, M)

#define BFOLLOWPATHCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bFollowPathConstraint3_6_0*>(data_ptr)-> C); 
#define BFOLLOWPATHCONSTRAINT_RETURN_REF(T, M)    BFOLLOWPATHCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BFOLLOWPATHCONSTRAINT_RETURN_AS(T, M)     BFOLLOWPATHCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BFOLLOWPATHCONSTRAINT_RETURN(M)           BFOLLOWPATHCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BFOLLOWTRACKCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bFollowTrackConstraint3_6_0*>(data_ptr)-> C); 
#define BFOLLOWTRACKCONSTRAINT_RETURN_REF(T, M)    BFOLLOWTRACKCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BFOLLOWTRACKCONSTRAINT_RETURN_AS(T, M)     BFOLLOWTRACKCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BFOLLOWTRACKCONSTRAINT_RETURN(M)           BFOLLOWTRACKCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BGPDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bGPdata3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bGPdata4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bGPdata4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bGPdata4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPdata4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPdata4_4_0*>(data_ptr)-> C); 
#define BGPDATA_RETURN_REF(T, M)    BGPDATA_BASE_RETURN_BODY(T, &, M)
#define BGPDATA_RETURN_AS(T, M)     BGPDATA_BASE_RETURN_BODY(T,, M)
#define BGPDATA_RETURN(M)           BGPDATA_BASE_RETURN_BODY(,, M)

#define BGPDATA_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bGPdata_Runtime3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bGPdata_Runtime4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bGPdata_Runtime4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bGPdata_Runtime4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPdata_Runtime4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPdata_Runtime4_4_0*>(data_ptr)-> C); 
#define BGPDATA_RUNTIME_RETURN_REF(T, M)    BGPDATA_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BGPDATA_RUNTIME_RETURN_AS(T, M)     BGPDATA_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BGPDATA_RUNTIME_RETURN(M)           BGPDATA_RUNTIME_BASE_RETURN_BODY(,, M)

#define BGPDCONTROLPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDcontrolpoint3_6_0*>(data_ptr)-> C); 
#define BGPDCONTROLPOINT_RETURN_REF(T, M)    BGPDCONTROLPOINT_BASE_RETURN_BODY(T, &, M)
#define BGPDCONTROLPOINT_RETURN_AS(T, M)     BGPDCONTROLPOINT_BASE_RETURN_BODY(T,, M)
#define BGPDCONTROLPOINT_RETURN(M)           BGPDCONTROLPOINT_BASE_RETURN_BODY(,, M)

#define BGPDCURVE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDcurve3_6_0*>(data_ptr)-> C); 
#define BGPDCURVE_RETURN_REF(T, M)    BGPDCURVE_BASE_RETURN_BODY(T, &, M)
#define BGPDCURVE_RETURN_AS(T, M)     BGPDCURVE_BASE_RETURN_BODY(T,, M)
#define BGPDCURVE_RETURN(M)           BGPDCURVE_BASE_RETURN_BODY(,, M)

#define BGPDCURVE_POINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDcurve_point3_6_0*>(data_ptr)-> C); 
#define BGPDCURVE_POINT_RETURN_REF(T, M)    BGPDCURVE_POINT_BASE_RETURN_BODY(T, &, M)
#define BGPDCURVE_POINT_RETURN_AS(T, M)     BGPDCURVE_POINT_BASE_RETURN_BODY(T,, M)
#define BGPDCURVE_POINT_RETURN(M)           BGPDCURVE_POINT_BASE_RETURN_BODY(,, M)

#define BGPDFRAME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDframe3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDframe4_4_0*>(data_ptr)-> C); 
#define BGPDFRAME_RETURN_REF(T, M)    BGPDFRAME_BASE_RETURN_BODY(T, &, M)
#define BGPDFRAME_RETURN_AS(T, M)     BGPDFRAME_BASE_RETURN_BODY(T,, M)
#define BGPDFRAME_RETURN(M)           BGPDFRAME_BASE_RETURN_BODY(,, M)

#define BGPDFRAME_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDframe_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDframe_Runtime4_4_0*>(data_ptr)-> C); 
#define BGPDFRAME_RUNTIME_RETURN_REF(T, M)    BGPDFRAME_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BGPDFRAME_RUNTIME_RETURN_AS(T, M)     BGPDFRAME_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BGPDFRAME_RUNTIME_RETURN(M)           BGPDFRAME_RUNTIME_BASE_RETURN_BODY(,, M)

#define BGPDLAYER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDlayer3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDlayer4_4_0*>(data_ptr)-> C); 
#define BGPDLAYER_RETURN_REF(T, M)    BGPDLAYER_BASE_RETURN_BODY(T, &, M)
#define BGPDLAYER_RETURN_AS(T, M)     BGPDLAYER_BASE_RETURN_BODY(T,, M)
#define BGPDLAYER_RETURN(M)           BGPDLAYER_BASE_RETURN_BODY(,, M)

#define BGPDLAYER_MASK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDlayer_Mask3_6_0*>(data_ptr)-> C); 
#define BGPDLAYER_MASK_RETURN_REF(T, M)    BGPDLAYER_MASK_BASE_RETURN_BODY(T, &, M)
#define BGPDLAYER_MASK_RETURN_AS(T, M)     BGPDLAYER_MASK_BASE_RETURN_BODY(T,, M)
#define BGPDLAYER_MASK_RETURN(M)           BGPDLAYER_MASK_BASE_RETURN_BODY(,, M)

#define BGPDLAYER_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDlayer_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDlayer_Runtime4_4_0*>(data_ptr)-> C); 
#define BGPDLAYER_RUNTIME_RETURN_REF(T, M)    BGPDLAYER_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BGPDLAYER_RUNTIME_RETURN_AS(T, M)     BGPDLAYER_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BGPDLAYER_RUNTIME_RETURN(M)           BGPDLAYER_RUNTIME_BASE_RETURN_BODY(,, M)

#define BGPDPALETTECOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDpalettecolor3_6_0*>(data_ptr)-> C); 
#define BGPDPALETTECOLOR_RETURN_REF(T, M)    BGPDPALETTECOLOR_BASE_RETURN_BODY(T, &, M)
#define BGPDPALETTECOLOR_RETURN_AS(T, M)     BGPDPALETTECOLOR_BASE_RETURN_BODY(T,, M)
#define BGPDPALETTECOLOR_RETURN(M)           BGPDPALETTECOLOR_BASE_RETURN_BODY(,, M)

#define BGPDPALETTE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDpalette3_6_0*>(data_ptr)-> C); 
#define BGPDPALETTE_RETURN_REF(T, M)    BGPDPALETTE_BASE_RETURN_BODY(T, &, M)
#define BGPDPALETTE_RETURN_AS(T, M)     BGPDPALETTE_BASE_RETURN_BODY(T,, M)
#define BGPDPALETTE_RETURN(M)           BGPDPALETTE_BASE_RETURN_BODY(,, M)

#define BGPDSPOINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDspoint3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDspoint4_4_0*>(data_ptr)-> C); 
#define BGPDSPOINT_RETURN_REF(T, M)    BGPDSPOINT_BASE_RETURN_BODY(T, &, M)
#define BGPDSPOINT_RETURN_AS(T, M)     BGPDSPOINT_BASE_RETURN_BODY(T,, M)
#define BGPDSPOINT_RETURN(M)           BGPDSPOINT_BASE_RETURN_BODY(,, M)

#define BGPDSPOINT_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDspoint_Runtime3_6_0*>(data_ptr)-> C); 
#define BGPDSPOINT_RUNTIME_RETURN_REF(T, M)    BGPDSPOINT_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BGPDSPOINT_RUNTIME_RETURN_AS(T, M)     BGPDSPOINT_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BGPDSPOINT_RUNTIME_RETURN(M)           BGPDSPOINT_RUNTIME_BASE_RETURN_BODY(,, M)

#define BGPDSTROKE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bGPDstroke3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDstroke4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDstroke4_4_0*>(data_ptr)-> C); 
#define BGPDSTROKE_RETURN_REF(T, M)    BGPDSTROKE_BASE_RETURN_BODY(T, &, M)
#define BGPDSTROKE_RETURN_AS(T, M)     BGPDSTROKE_BASE_RETURN_BODY(T,, M)
#define BGPDSTROKE_RETURN(M)           BGPDSTROKE_BASE_RETURN_BODY(,, M)

#define BGPDSTROKE_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bGPDstroke_Runtime3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bGPDstroke_Runtime4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bGPDstroke_Runtime4_4_0*>(data_ptr)-> C); 
#define BGPDSTROKE_RUNTIME_RETURN_REF(T, M)    BGPDSTROKE_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BGPDSTROKE_RUNTIME_RETURN_AS(T, M)     BGPDSTROKE_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BGPDSTROKE_RUNTIME_RETURN(M)           BGPDSTROKE_RUNTIME_BASE_RETURN_BODY(,, M)

#define BGPDTRIANGLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPDtriangle3_6_0*>(data_ptr)-> C); 
#define BGPDTRIANGLE_RETURN_REF(T, M)    BGPDTRIANGLE_BASE_RETURN_BODY(T, &, M)
#define BGPDTRIANGLE_RETURN_AS(T, M)     BGPDTRIANGLE_BASE_RETURN_BODY(T,, M)
#define BGPDTRIANGLE_RETURN(M)           BGPDTRIANGLE_BASE_RETURN_BODY(,, M)

#define BGPGRID_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bGPgrid3_6_0*>(data_ptr)-> C); 
#define BGPGRID_RETURN_REF(T, M)    BGPGRID_BASE_RETURN_BODY(T, &, M)
#define BGPGRID_RETURN_AS(T, M)     BGPGRID_BASE_RETURN_BODY(T,, M)
#define BGPGRID_RETURN(M)           BGPGRID_BASE_RETURN_BODY(,, M)

#define BHEAD4_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BHead43_6_0*>(data_ptr)-> C); 
#define BHEAD4_RETURN_REF(T, M)    BHEAD4_BASE_RETURN_BODY(T, &, M)
#define BHEAD4_RETURN_AS(T, M)     BHEAD4_BASE_RETURN_BODY(T,, M)
#define BHEAD4_RETURN(M)           BHEAD4_BASE_RETURN_BODY(,, M)

#define BHEAD8_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BHead83_6_0*>(data_ptr)-> C); 
#define BHEAD8_RETURN_REF(T, M)    BHEAD8_BASE_RETURN_BODY(T, &, M)
#define BHEAD8_RETURN_AS(T, M)     BHEAD8_BASE_RETURN_BODY(T,, M)
#define BHEAD8_RETURN(M)           BHEAD8_BASE_RETURN_BODY(,, M)

#define BHEAD_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BHead3_6_0*>(data_ptr)-> C); 
#define BHEAD_RETURN_REF(T, M)    BHEAD_BASE_RETURN_BODY(T, &, M)
#define BHEAD_RETURN_AS(T, M)     BHEAD_BASE_RETURN_BODY(T,, M)
#define BHEAD_RETURN(M)           BHEAD_BASE_RETURN_BODY(,, M)

#define BIKPARAM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bIKParam3_6_0*>(data_ptr)-> C); 
#define BIKPARAM_RETURN_REF(T, M)    BIKPARAM_BASE_RETURN_BODY(T, &, M)
#define BIKPARAM_RETURN_AS(T, M)     BIKPARAM_BASE_RETURN_BODY(T,, M)
#define BIKPARAM_RETURN(M)           BIKPARAM_BASE_RETURN_BODY(,, M)

#define BITASC_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bItasc3_6_0*>(data_ptr)-> C); 
#define BITASC_RETURN_REF(T, M)    BITASC_BASE_RETURN_BODY(T, &, M)
#define BITASC_RETURN_AS(T, M)     BITASC_BASE_RETURN_BODY(T,, M)
#define BITASC_RETURN(M)           BITASC_BASE_RETURN_BODY(,, M)

#define BKINEMATICCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bKinematicConstraint3_6_0*>(data_ptr)-> C); 
#define BKINEMATICCONSTRAINT_RETURN_REF(T, M)    BKINEMATICCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BKINEMATICCONSTRAINT_RETURN_AS(T, M)     BKINEMATICCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BKINEMATICCONSTRAINT_RETURN(M)           BKINEMATICCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BLOCATELIKECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bLocateLikeConstraint3_6_0*>(data_ptr)-> C); 
#define BLOCATELIKECONSTRAINT_RETURN_REF(T, M)    BLOCATELIKECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BLOCATELIKECONSTRAINT_RETURN_AS(T, M)     BLOCATELIKECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BLOCATELIKECONSTRAINT_RETURN(M)           BLOCATELIKECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BLOCKTRACKCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bLockTrackConstraint3_6_0*>(data_ptr)-> C); 
#define BLOCKTRACKCONSTRAINT_RETURN_REF(T, M)    BLOCKTRACKCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BLOCKTRACKCONSTRAINT_RETURN_AS(T, M)     BLOCKTRACKCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BLOCKTRACKCONSTRAINT_RETURN(M)           BLOCKTRACKCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BLOCLIMITCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bLocLimitConstraint3_6_0*>(data_ptr)-> C); 
#define BLOCLIMITCONSTRAINT_RETURN_REF(T, M)    BLOCLIMITCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BLOCLIMITCONSTRAINT_RETURN_AS(T, M)     BLOCLIMITCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BLOCLIMITCONSTRAINT_RETURN(M)           BLOCLIMITCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BLURSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BlurShaderFxData3_6_0*>(data_ptr)-> C); 
#define BLURSHADERFXDATA_RETURN_REF(T, M)    BLURSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define BLURSHADERFXDATA_RETURN_AS(T, M)     BLURSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define BLURSHADERFXDATA_RETURN(M)           BLURSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define BMINMAXCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bMinMaxConstraint3_6_0*>(data_ptr)-> C); 
#define BMINMAXCONSTRAINT_RETURN_REF(T, M)    BMINMAXCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BMINMAXCONSTRAINT_RETURN_AS(T, M)     BMINMAXCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BMINMAXCONSTRAINT_RETURN(M)           BMINMAXCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BMOTIONPATHVERT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bMotionPathVert3_6_0*>(data_ptr)-> C); 
#define BMOTIONPATHVERT_RETURN_REF(T, M)    BMOTIONPATHVERT_BASE_RETURN_BODY(T, &, M)
#define BMOTIONPATHVERT_RETURN_AS(T, M)     BMOTIONPATHVERT_BASE_RETURN_BODY(T,, M)
#define BMOTIONPATHVERT_RETURN(M)           BMOTIONPATHVERT_BASE_RETURN_BODY(,, M)

#define BMOTIONPATH_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bMotionPath3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bMotionPath4_2_0*>(data_ptr)-> C); 
#define BMOTIONPATH_RETURN_REF(T, M)    BMOTIONPATH_BASE_RETURN_BODY(T, &, M)
#define BMOTIONPATH_RETURN_AS(T, M)     BMOTIONPATH_BASE_RETURN_BODY(T,, M)
#define BMOTIONPATH_RETURN(M)           BMOTIONPATH_BASE_RETURN_BODY(,, M)

#define BNODEINSTANCEHASHENTRY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeInstanceHashEntry3_6_0*>(data_ptr)-> C); 
#define BNODEINSTANCEHASHENTRY_RETURN_REF(T, M)    BNODEINSTANCEHASHENTRY_BASE_RETURN_BODY(T, &, M)
#define BNODEINSTANCEHASHENTRY_RETURN_AS(T, M)     BNODEINSTANCEHASHENTRY_BASE_RETURN_BODY(T,, M)
#define BNODEINSTANCEHASHENTRY_RETURN(M)           BNODEINSTANCEHASHENTRY_BASE_RETURN_BODY(,, M)

#define BNODEINSTANCEKEY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeInstanceKey3_6_0*>(data_ptr)-> C); 
#define BNODEINSTANCEKEY_RETURN_REF(T, M)    BNODEINSTANCEKEY_BASE_RETURN_BODY(T, &, M)
#define BNODEINSTANCEKEY_RETURN_AS(T, M)     BNODEINSTANCEKEY_BASE_RETURN_BODY(T,, M)
#define BNODEINSTANCEKEY_RETURN(M)           BNODEINSTANCEKEY_BASE_RETURN_BODY(,, M)

#define BNODELINK_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNodeLink3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeLink4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeLink4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bNodeLink4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeLink4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeLink4_4_0*>(data_ptr)-> C); 
#define BNODELINK_RETURN_REF(T, M)    BNODELINK_BASE_RETURN_BODY(T, &, M)
#define BNODELINK_RETURN_AS(T, M)     BNODELINK_BASE_RETURN_BODY(T,, M)
#define BNODELINK_RETURN(M)           BNODELINK_BASE_RETURN_BODY(,, M)

#define BNODEPREVIEW_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNodePreview3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodePreview4_0_0*>(data_ptr)-> C); 
#define BNODEPREVIEW_RETURN_REF(T, M)    BNODEPREVIEW_BASE_RETURN_BODY(T, &, M)
#define BNODEPREVIEW_RETURN_AS(T, M)     BNODEPREVIEW_BASE_RETURN_BODY(T,, M)
#define BNODEPREVIEW_RETURN(M)           BNODEPREVIEW_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEBOOLEAN_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueBoolean3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEBOOLEAN_RETURN_REF(T, M)    BNODESOCKETVALUEBOOLEAN_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEBOOLEAN_RETURN_AS(T, M)     BNODESOCKETVALUEBOOLEAN_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEBOOLEAN_RETURN(M)           BNODESOCKETVALUEBOOLEAN_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUECOLLECTION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueCollection3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUECOLLECTION_RETURN_REF(T, M)    BNODESOCKETVALUECOLLECTION_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUECOLLECTION_RETURN_AS(T, M)     BNODESOCKETVALUECOLLECTION_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUECOLLECTION_RETURN(M)           BNODESOCKETVALUECOLLECTION_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEFLOAT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueFloat3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEFLOAT_RETURN_REF(T, M)    BNODESOCKETVALUEFLOAT_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEFLOAT_RETURN_AS(T, M)     BNODESOCKETVALUEFLOAT_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEFLOAT_RETURN(M)           BNODESOCKETVALUEFLOAT_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEIMAGE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeSocketValueImage3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeSocketValueImage4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeSocketValueImage4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeSocketValueImage4_4_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEIMAGE_RETURN_REF(T, M)    BNODESOCKETVALUEIMAGE_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEIMAGE_RETURN_AS(T, M)     BNODESOCKETVALUEIMAGE_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEIMAGE_RETURN(M)           BNODESOCKETVALUEIMAGE_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueInt3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEINT_RETURN_REF(T, M)    BNODESOCKETVALUEINT_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEINT_RETURN_AS(T, M)     BNODESOCKETVALUEINT_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEINT_RETURN(M)           BNODESOCKETVALUEINT_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEMATERIAL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueMaterial3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEMATERIAL_RETURN_REF(T, M)    BNODESOCKETVALUEMATERIAL_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEMATERIAL_RETURN_AS(T, M)     BNODESOCKETVALUEMATERIAL_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEMATERIAL_RETURN(M)           BNODESOCKETVALUEMATERIAL_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEOBJECT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNodeSocketValueObject3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeSocketValueObject4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeSocketValueObject4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bNodeSocketValueObject4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeSocketValueObject4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeSocketValueObject4_4_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEOBJECT_RETURN_REF(T, M)    BNODESOCKETVALUEOBJECT_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEOBJECT_RETURN_AS(T, M)     BNODESOCKETVALUEOBJECT_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEOBJECT_RETURN(M)           BNODESOCKETVALUEOBJECT_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUERGBA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueRGBA3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUERGBA_RETURN_REF(T, M)    BNODESOCKETVALUERGBA_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUERGBA_RETURN_AS(T, M)     BNODESOCKETVALUERGBA_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUERGBA_RETURN(M)           BNODESOCKETVALUERGBA_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUESTRING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueString3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUESTRING_RETURN_REF(T, M)    BNODESOCKETVALUESTRING_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUESTRING_RETURN_AS(T, M)     BNODESOCKETVALUESTRING_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUESTRING_RETURN(M)           BNODESOCKETVALUESTRING_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUETEXTURE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeSocketValueTexture3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeSocketValueTexture4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeSocketValueTexture4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeSocketValueTexture4_4_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUETEXTURE_RETURN_REF(T, M)    BNODESOCKETVALUETEXTURE_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUETEXTURE_RETURN_AS(T, M)     BNODESOCKETVALUETEXTURE_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUETEXTURE_RETURN(M)           BNODESOCKETVALUETEXTURE_BASE_RETURN_BODY(,, M)

#define BNODESOCKETVALUEVECTOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeSocketValueVector3_6_0*>(data_ptr)-> C); 
#define BNODESOCKETVALUEVECTOR_RETURN_REF(T, M)    BNODESOCKETVALUEVECTOR_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKETVALUEVECTOR_RETURN_AS(T, M)     BNODESOCKETVALUEVECTOR_BASE_RETURN_BODY(T,, M)
#define BNODESOCKETVALUEVECTOR_RETURN(M)           BNODESOCKETVALUEVECTOR_BASE_RETURN_BODY(,, M)

#define BNODESOCKET_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNodeSocket3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeSocket4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeSocket4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bNodeSocket4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeSocket4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeSocket4_4_0*>(data_ptr)-> C); 
#define BNODESOCKET_RETURN_REF(T, M)    BNODESOCKET_BASE_RETURN_BODY(T, &, M)
#define BNODESOCKET_RETURN_AS(T, M)     BNODESOCKET_BASE_RETURN_BODY(T,, M)
#define BNODESOCKET_RETURN(M)           BNODESOCKET_BASE_RETURN_BODY(,, M)

#define BNODESTACK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bNodeStack3_6_0*>(data_ptr)-> C); 
#define BNODESTACK_RETURN_REF(T, M)    BNODESTACK_BASE_RETURN_BODY(T, &, M)
#define BNODESTACK_RETURN_AS(T, M)     BNODESTACK_BASE_RETURN_BODY(T,, M)
#define BNODESTACK_RETURN(M)           BNODESTACK_BASE_RETURN_BODY(,, M)

#define BNODETREEPATH_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNodeTreePath3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeTreePath4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeTreePath4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bNodeTreePath4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeTreePath4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeTreePath4_4_0*>(data_ptr)-> C); 
#define BNODETREEPATH_RETURN_REF(T, M)    BNODETREEPATH_BASE_RETURN_BODY(T, &, M)
#define BNODETREEPATH_RETURN_AS(T, M)     BNODETREEPATH_BASE_RETURN_BODY(T,, M)
#define BNODETREEPATH_RETURN(M)           BNODETREEPATH_BASE_RETURN_BODY(,, M)

#define BNODETREE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNodeTree3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNodeTree4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNodeTree4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bNodeTree4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNodeTree4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNodeTree4_4_0*>(data_ptr)-> C); 
#define BNODETREE_RETURN_REF(T, M)    BNODETREE_BASE_RETURN_BODY(T, &, M)
#define BNODETREE_RETURN_AS(T, M)     BNODETREE_BASE_RETURN_BODY(T,, M)
#define BNODETREE_RETURN(M)           BNODETREE_BASE_RETURN_BODY(,, M)

#define BNODE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bNode3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bNode4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bNode4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bNode4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bNode4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bNode4_4_0*>(data_ptr)-> C); 
#define BNODE_RETURN_REF(T, M)    BNODE_BASE_RETURN_BODY(T, &, M)
#define BNODE_RETURN_AS(T, M)     BNODE_BASE_RETURN_BODY(T,, M)
#define BNODE_RETURN(M)           BNODE_BASE_RETURN_BODY(,, M)

#define BOBJECTSOLVERCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bObjectSolverConstraint3_6_0*>(data_ptr)-> C); 
#define BOBJECTSOLVERCONSTRAINT_RETURN_REF(T, M)    BOBJECTSOLVERCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BOBJECTSOLVERCONSTRAINT_RETURN_AS(T, M)     BOBJECTSOLVERCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BOBJECTSOLVERCONSTRAINT_RETURN(M)           BOBJECTSOLVERCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BOIDDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidData3_6_0*>(data_ptr)-> C); 
#define BOIDDATA_RETURN_REF(T, M)    BOIDDATA_BASE_RETURN_BODY(T, &, M)
#define BOIDDATA_RETURN_AS(T, M)     BOIDDATA_BASE_RETURN_BODY(T,, M)
#define BOIDDATA_RETURN(M)           BOIDDATA_BASE_RETURN_BODY(,, M)

#define BOIDPARTICLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidParticle3_6_0*>(data_ptr)-> C); 
#define BOIDPARTICLE_RETURN_REF(T, M)    BOIDPARTICLE_BASE_RETURN_BODY(T, &, M)
#define BOIDPARTICLE_RETURN_AS(T, M)     BOIDPARTICLE_BASE_RETURN_BODY(T,, M)
#define BOIDPARTICLE_RETURN(M)           BOIDPARTICLE_BASE_RETURN_BODY(,, M)

#define BOIDRULEAVERAGESPEED_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidRuleAverageSpeed3_6_0*>(data_ptr)-> C); 
#define BOIDRULEAVERAGESPEED_RETURN_REF(T, M)    BOIDRULEAVERAGESPEED_BASE_RETURN_BODY(T, &, M)
#define BOIDRULEAVERAGESPEED_RETURN_AS(T, M)     BOIDRULEAVERAGESPEED_BASE_RETURN_BODY(T,, M)
#define BOIDRULEAVERAGESPEED_RETURN(M)           BOIDRULEAVERAGESPEED_BASE_RETURN_BODY(,, M)

#define BOIDRULEAVOIDCOLLISION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidRuleAvoidCollision3_6_0*>(data_ptr)-> C); 
#define BOIDRULEAVOIDCOLLISION_RETURN_REF(T, M)    BOIDRULEAVOIDCOLLISION_BASE_RETURN_BODY(T, &, M)
#define BOIDRULEAVOIDCOLLISION_RETURN_AS(T, M)     BOIDRULEAVOIDCOLLISION_BASE_RETURN_BODY(T,, M)
#define BOIDRULEAVOIDCOLLISION_RETURN(M)           BOIDRULEAVOIDCOLLISION_BASE_RETURN_BODY(,, M)

#define BOIDRULEFIGHT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidRuleFight3_6_0*>(data_ptr)-> C); 
#define BOIDRULEFIGHT_RETURN_REF(T, M)    BOIDRULEFIGHT_BASE_RETURN_BODY(T, &, M)
#define BOIDRULEFIGHT_RETURN_AS(T, M)     BOIDRULEFIGHT_BASE_RETURN_BODY(T,, M)
#define BOIDRULEFIGHT_RETURN(M)           BOIDRULEFIGHT_BASE_RETURN_BODY(,, M)

#define BOIDRULEFOLLOWLEADER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidRuleFollowLeader3_6_0*>(data_ptr)-> C); 
#define BOIDRULEFOLLOWLEADER_RETURN_REF(T, M)    BOIDRULEFOLLOWLEADER_BASE_RETURN_BODY(T, &, M)
#define BOIDRULEFOLLOWLEADER_RETURN_AS(T, M)     BOIDRULEFOLLOWLEADER_BASE_RETURN_BODY(T,, M)
#define BOIDRULEFOLLOWLEADER_RETURN(M)           BOIDRULEFOLLOWLEADER_BASE_RETURN_BODY(,, M)

#define BOIDRULEGOALAVOID_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidRuleGoalAvoid3_6_0*>(data_ptr)-> C); 
#define BOIDRULEGOALAVOID_RETURN_REF(T, M)    BOIDRULEGOALAVOID_BASE_RETURN_BODY(T, &, M)
#define BOIDRULEGOALAVOID_RETURN_AS(T, M)     BOIDRULEGOALAVOID_BASE_RETURN_BODY(T,, M)
#define BOIDRULEGOALAVOID_RETURN(M)           BOIDRULEGOALAVOID_BASE_RETURN_BODY(,, M)

#define BOIDRULE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidRule3_6_0*>(data_ptr)-> C); 
#define BOIDRULE_RETURN_REF(T, M)    BOIDRULE_BASE_RETURN_BODY(T, &, M)
#define BOIDRULE_RETURN_AS(T, M)     BOIDRULE_BASE_RETURN_BODY(T,, M)
#define BOIDRULE_RETURN(M)           BOIDRULE_BASE_RETURN_BODY(,, M)

#define BOIDSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidSettings3_6_0*>(data_ptr)-> C); 
#define BOIDSETTINGS_RETURN_REF(T, M)    BOIDSETTINGS_BASE_RETURN_BODY(T, &, M)
#define BOIDSETTINGS_RETURN_AS(T, M)     BOIDSETTINGS_BASE_RETURN_BODY(T,, M)
#define BOIDSETTINGS_RETURN(M)           BOIDSETTINGS_BASE_RETURN_BODY(,, M)

#define BOIDSTATE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BoidState3_6_0*>(data_ptr)-> C); 
#define BOIDSTATE_RETURN_REF(T, M)    BOIDSTATE_BASE_RETURN_BODY(T, &, M)
#define BOIDSTATE_RETURN_AS(T, M)     BOIDSTATE_BASE_RETURN_BODY(T,, M)
#define BOIDSTATE_RETURN(M)           BOIDSTATE_BASE_RETURN_BODY(,, M)

#define BONE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Bone3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Bone4_0_0*>(data_ptr)-> C); 
#define BONE_RETURN_REF(T, M)    BONE_BASE_RETURN_BODY(T, &, M)
#define BONE_RETURN_AS(T, M)     BONE_BASE_RETURN_BODY(T,, M)
#define BONE_RETURN(M)           BONE_BASE_RETURN_BODY(,, M)

#define BOOLEANMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BooleanModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BooleanModifierData4_1_0*>(data_ptr)-> C); 
#define BOOLEANMODIFIERDATA_RETURN_REF(T, M)    BOOLEANMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define BOOLEANMODIFIERDATA_RETURN_AS(T, M)     BOOLEANMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define BOOLEANMODIFIERDATA_RETURN(M)           BOOLEANMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define BOUNDBOX_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BoundBox3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BoundBox4_1_0*>(data_ptr)-> C); 
#define BOUNDBOX_RETURN_REF(T, M)    BOUNDBOX_BASE_RETURN_BODY(T, &, M)
#define BOUNDBOX_RETURN_AS(T, M)     BOUNDBOX_BASE_RETURN_BODY(T,, M)
#define BOUNDBOX_RETURN(M)           BOUNDBOX_BASE_RETURN_BODY(,, M)

#define BPATHCOMPARE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bPathCompare3_6_0*>(data_ptr)-> C); 
#define BPATHCOMPARE_RETURN_REF(T, M)    BPATHCOMPARE_BASE_RETURN_BODY(T, &, M)
#define BPATHCOMPARE_RETURN_AS(T, M)     BPATHCOMPARE_BASE_RETURN_BODY(T,, M)
#define BPATHCOMPARE_RETURN(M)           BPATHCOMPARE_BASE_RETURN_BODY(,, M)

#define BPIVOTCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bPivotConstraint3_6_0*>(data_ptr)-> C); 
#define BPIVOTCONSTRAINT_RETURN_REF(T, M)    BPIVOTCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BPIVOTCONSTRAINT_RETURN_AS(T, M)     BPIVOTCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BPIVOTCONSTRAINT_RETURN(M)           BPIVOTCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BPoint3_6_0*>(data_ptr)-> C); 
#define BPOINT_RETURN_REF(T, M)    BPOINT_BASE_RETURN_BODY(T, &, M)
#define BPOINT_RETURN_AS(T, M)     BPOINT_BASE_RETURN_BODY(T,, M)
#define BPOINT_RETURN(M)           BPOINT_BASE_RETURN_BODY(,, M)

#define BPOSECHANNELDRAWDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bPoseChannelDrawData3_6_0*>(data_ptr)-> C); 
#define BPOSECHANNELDRAWDATA_RETURN_REF(T, M)    BPOSECHANNELDRAWDATA_BASE_RETURN_BODY(T, &, M)
#define BPOSECHANNELDRAWDATA_RETURN_AS(T, M)     BPOSECHANNELDRAWDATA_BASE_RETURN_BODY(T,, M)
#define BPOSECHANNELDRAWDATA_RETURN(M)           BPOSECHANNELDRAWDATA_BASE_RETURN_BODY(,, M)

#define BPOSECHANNEL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bPoseChannel3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bPoseChannel4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bPoseChannel4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bPoseChannel4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bPoseChannel4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bPoseChannel4_4_0*>(data_ptr)-> C); 
#define BPOSECHANNEL_RETURN_REF(T, M)    BPOSECHANNEL_BASE_RETURN_BODY(T, &, M)
#define BPOSECHANNEL_RETURN_AS(T, M)     BPOSECHANNEL_BASE_RETURN_BODY(T,, M)
#define BPOSECHANNEL_RETURN(M)           BPOSECHANNEL_BASE_RETURN_BODY(,, M)

#define BPOSECHANNEL_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bPoseChannel_Runtime3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bPoseChannel_Runtime4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bPoseChannel_Runtime4_1_0*>(data_ptr)-> C); 
#define BPOSECHANNEL_RUNTIME_RETURN_REF(T, M)    BPOSECHANNEL_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BPOSECHANNEL_RUNTIME_RETURN_AS(T, M)     BPOSECHANNEL_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BPOSECHANNEL_RUNTIME_RETURN(M)           BPOSECHANNEL_RUNTIME_BASE_RETURN_BODY(,, M)

#define BPOSE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bPose3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bPose4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bPose4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bPose4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bPose4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bPose4_4_0*>(data_ptr)-> C); 
#define BPOSE_RETURN_REF(T, M)    BPOSE_BASE_RETURN_BODY(T, &, M)
#define BPOSE_RETURN_AS(T, M)     BPOSE_BASE_RETURN_BODY(T,, M)
#define BPOSE_RETURN(M)           BPOSE_BASE_RETURN_BODY(,, M)

#define BPYTHONCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bPythonConstraint3_6_0*>(data_ptr)-> C); 
#define BPYTHONCONSTRAINT_RETURN_REF(T, M)    BPYTHONCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BPYTHONCONSTRAINT_RETURN_AS(T, M)     BPYTHONCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BPYTHONCONSTRAINT_RETURN(M)           BPYTHONCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BRIGHTCONTRASTMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<BrightContrastModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BrightContrastModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<BrightContrastModifierData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<BrightContrastModifierData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BrightContrastModifierData4_4_0*>(data_ptr)-> C); 
#define BRIGHTCONTRASTMODIFIERDATA_RETURN_REF(T, M)    BRIGHTCONTRASTMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define BRIGHTCONTRASTMODIFIERDATA_RETURN_AS(T, M)     BRIGHTCONTRASTMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define BRIGHTCONTRASTMODIFIERDATA_RETURN(M)           BRIGHTCONTRASTMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define BRIGIDBODYJOINTCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bRigidBodyJointConstraint3_6_0*>(data_ptr)-> C); 
#define BRIGIDBODYJOINTCONSTRAINT_RETURN_REF(T, M)    BRIGIDBODYJOINTCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BRIGIDBODYJOINTCONSTRAINT_RETURN_AS(T, M)     BRIGIDBODYJOINTCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BRIGIDBODYJOINTCONSTRAINT_RETURN(M)           BRIGIDBODYJOINTCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BROTATELIKECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bRotateLikeConstraint3_6_0*>(data_ptr)-> C); 
#define BROTATELIKECONSTRAINT_RETURN_REF(T, M)    BROTATELIKECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BROTATELIKECONSTRAINT_RETURN_AS(T, M)     BROTATELIKECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BROTATELIKECONSTRAINT_RETURN(M)           BROTATELIKECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BROTLIMITCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bRotLimitConstraint3_6_0*>(data_ptr)-> C); 
#define BROTLIMITCONSTRAINT_RETURN_REF(T, M)    BROTLIMITCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BROTLIMITCONSTRAINT_RETURN_AS(T, M)     BROTLIMITCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BROTLIMITCONSTRAINT_RETURN(M)           BROTLIMITCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BRUSHCLONE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BrushClone3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<BrushClone4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BrushClone4_2_0*>(data_ptr)-> C); 
#define BRUSHCLONE_RETURN_REF(T, M)    BRUSHCLONE_BASE_RETURN_BODY(T, &, M)
#define BRUSHCLONE_RETURN_AS(T, M)     BRUSHCLONE_BASE_RETURN_BODY(T,, M)
#define BRUSHCLONE_RETURN(M)           BRUSHCLONE_BASE_RETURN_BODY(,, M)

#define BRUSHCURVESSCULPTSETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<BrushCurvesSculptSettings3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BrushCurvesSculptSettings4_2_0*>(data_ptr)-> C); 
#define BRUSHCURVESSCULPTSETTINGS_RETURN_REF(T, M)    BRUSHCURVESSCULPTSETTINGS_BASE_RETURN_BODY(T, &, M)
#define BRUSHCURVESSCULPTSETTINGS_RETURN_AS(T, M)     BRUSHCURVESSCULPTSETTINGS_BASE_RETURN_BODY(T,, M)
#define BRUSHCURVESSCULPTSETTINGS_RETURN(M)           BRUSHCURVESSCULPTSETTINGS_BASE_RETURN_BODY(,, M)

#define BRUSHGPENCILSETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<BrushGpencilSettings3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<BrushGpencilSettings4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BrushGpencilSettings4_3_0*>(data_ptr)-> C); 
#define BRUSHGPENCILSETTINGS_RETURN_REF(T, M)    BRUSHGPENCILSETTINGS_BASE_RETURN_BODY(T, &, M)
#define BRUSHGPENCILSETTINGS_RETURN_AS(T, M)     BRUSHGPENCILSETTINGS_BASE_RETURN_BODY(T,, M)
#define BRUSHGPENCILSETTINGS_RETURN(M)           BRUSHGPENCILSETTINGS_BASE_RETURN_BODY(,, M)

#define BRUSH_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Brush3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Brush4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Brush4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Brush4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Brush4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Brush4_4_0*>(data_ptr)-> C); 
#define BRUSH_RETURN_REF(T, M)    BRUSH_BASE_RETURN_BODY(T, &, M)
#define BRUSH_RETURN_AS(T, M)     BRUSH_BASE_RETURN_BODY(T,, M)
#define BRUSH_RETURN(M)           BRUSH_BASE_RETURN_BODY(,, M)

#define BSAMEVOLUMECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bSameVolumeConstraint3_6_0*>(data_ptr)-> C); 
#define BSAMEVOLUMECONSTRAINT_RETURN_REF(T, M)    BSAMEVOLUMECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BSAMEVOLUMECONSTRAINT_RETURN_AS(T, M)     BSAMEVOLUMECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BSAMEVOLUMECONSTRAINT_RETURN(M)           BSAMEVOLUMECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BSCREEN_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bScreen3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bScreen4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bScreen4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bScreen4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bScreen4_4_0*>(data_ptr)-> C); 
#define BSCREEN_RETURN_REF(T, M)    BSCREEN_BASE_RETURN_BODY(T, &, M)
#define BSCREEN_RETURN_AS(T, M)     BSCREEN_BASE_RETURN_BODY(T,, M)
#define BSCREEN_RETURN(M)           BSCREEN_BASE_RETURN_BODY(,, M)

#define BSHRINKWRAPCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bShrinkwrapConstraint3_6_0*>(data_ptr)-> C); 
#define BSHRINKWRAPCONSTRAINT_RETURN_REF(T, M)    BSHRINKWRAPCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BSHRINKWRAPCONSTRAINT_RETURN_AS(T, M)     BSHRINKWRAPCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BSHRINKWRAPCONSTRAINT_RETURN(M)           BSHRINKWRAPCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BSIZELIKECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bSizeLikeConstraint3_6_0*>(data_ptr)-> C); 
#define BSIZELIKECONSTRAINT_RETURN_REF(T, M)    BSIZELIKECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BSIZELIKECONSTRAINT_RETURN_AS(T, M)     BSIZELIKECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BSIZELIKECONSTRAINT_RETURN(M)           BSIZELIKECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BSIZELIMITCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bSizeLimitConstraint3_6_0*>(data_ptr)-> C); 
#define BSIZELIMITCONSTRAINT_RETURN_REF(T, M)    BSIZELIMITCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BSIZELIMITCONSTRAINT_RETURN_AS(T, M)     BSIZELIMITCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BSIZELIMITCONSTRAINT_RETURN(M)           BSIZELIMITCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BSOUND_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<bSound3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<bSound4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bSound4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bSound4_4_0*>(data_ptr)-> C); 
#define BSOUND_RETURN_REF(T, M)    BSOUND_BASE_RETURN_BODY(T, &, M)
#define BSOUND_RETURN_AS(T, M)     BSOUND_BASE_RETURN_BODY(T,, M)
#define BSOUND_RETURN(M)           BSOUND_BASE_RETURN_BODY(,, M)

#define BSPLINEIKCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bSplineIKConstraint3_6_0*>(data_ptr)-> C); 
#define BSPLINEIKCONSTRAINT_RETURN_REF(T, M)    BSPLINEIKCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BSPLINEIKCONSTRAINT_RETURN_AS(T, M)     BSPLINEIKCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BSPLINEIKCONSTRAINT_RETURN(M)           BSPLINEIKCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BSTRETCHTOCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bStretchToConstraint3_6_0*>(data_ptr)-> C); 
#define BSTRETCHTOCONSTRAINT_RETURN_REF(T, M)    BSTRETCHTOCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BSTRETCHTOCONSTRAINT_RETURN_AS(T, M)     BSTRETCHTOCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BSTRETCHTOCONSTRAINT_RETURN(M)           BSTRETCHTOCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BTOOLREF_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bToolRef3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<bToolRef4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bToolRef4_4_0*>(data_ptr)-> C); 
#define BTOOLREF_RETURN_REF(T, M)    BTOOLREF_BASE_RETURN_BODY(T, &, M)
#define BTOOLREF_RETURN_AS(T, M)     BTOOLREF_BASE_RETURN_BODY(T,, M)
#define BTOOLREF_RETURN(M)           BTOOLREF_BASE_RETURN_BODY(,, M)

#define BTOOLREF_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<bToolRef_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bToolRef_Runtime4_3_0*>(data_ptr)-> C); 
#define BTOOLREF_RUNTIME_RETURN_REF(T, M)    BTOOLREF_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define BTOOLREF_RUNTIME_RETURN_AS(T, M)     BTOOLREF_RUNTIME_BASE_RETURN_BODY(T,, M)
#define BTOOLREF_RUNTIME_RETURN(M)           BTOOLREF_RUNTIME_BASE_RETURN_BODY(,, M)

#define BTRACKTOCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bTrackToConstraint3_6_0*>(data_ptr)-> C); 
#define BTRACKTOCONSTRAINT_RETURN_REF(T, M)    BTRACKTOCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BTRACKTOCONSTRAINT_RETURN_AS(T, M)     BTRACKTOCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BTRACKTOCONSTRAINT_RETURN(M)           BTRACKTOCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BTRANSFORMCACHECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bTransformCacheConstraint3_6_0*>(data_ptr)-> C); 
#define BTRANSFORMCACHECONSTRAINT_RETURN_REF(T, M)    BTRANSFORMCACHECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BTRANSFORMCACHECONSTRAINT_RETURN_AS(T, M)     BTRANSFORMCACHECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BTRANSFORMCACHECONSTRAINT_RETURN(M)           BTRANSFORMCACHECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BTRANSFORMCONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bTransformConstraint3_6_0*>(data_ptr)-> C); 
#define BTRANSFORMCONSTRAINT_RETURN_REF(T, M)    BTRANSFORMCONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BTRANSFORMCONSTRAINT_RETURN_AS(T, M)     BTRANSFORMCONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BTRANSFORMCONSTRAINT_RETURN(M)           BTRANSFORMCONSTRAINT_BASE_RETURN_BODY(,, M)

#define BTRANSLIKECONSTRAINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bTransLikeConstraint3_6_0*>(data_ptr)-> C); 
#define BTRANSLIKECONSTRAINT_RETURN_REF(T, M)    BTRANSLIKECONSTRAINT_BASE_RETURN_BODY(T, &, M)
#define BTRANSLIKECONSTRAINT_RETURN_AS(T, M)     BTRANSLIKECONSTRAINT_BASE_RETURN_BODY(T,, M)
#define BTRANSLIKECONSTRAINT_RETURN(M)           BTRANSLIKECONSTRAINT_BASE_RETURN_BODY(,, M)

#define BUILDEFF_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BuildEff3_6_0*>(data_ptr)-> C); 
#define BUILDEFF_RETURN_REF(T, M)    BUILDEFF_BASE_RETURN_BODY(T, &, M)
#define BUILDEFF_RETURN_AS(T, M)     BUILDEFF_BASE_RETURN_BODY(T,, M)
#define BUILDEFF_RETURN(M)           BUILDEFF_BASE_RETURN_BODY(,, M)

#define BUILDGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<BuildGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define BUILDGPENCILMODIFIERDATA_RETURN_REF(T, M)    BUILDGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define BUILDGPENCILMODIFIERDATA_RETURN_AS(T, M)     BUILDGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define BUILDGPENCILMODIFIERDATA_RETURN(M)           BUILDGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define BUILDMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<BuildModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<BuildModifierData4_1_0*>(data_ptr)-> C); 
#define BUILDMODIFIERDATA_RETURN_REF(T, M)    BUILDMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define BUILDMODIFIERDATA_RETURN_AS(T, M)     BUILDMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define BUILDMODIFIERDATA_RETURN(M)           BUILDMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define BUSERASSETLIBRARY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<bUserAssetLibrary3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<bUserAssetLibrary4_0_0*>(data_ptr)-> C); 
#define BUSERASSETLIBRARY_RETURN_REF(T, M)    BUSERASSETLIBRARY_BASE_RETURN_BODY(T, &, M)
#define BUSERASSETLIBRARY_RETURN_AS(T, M)     BUSERASSETLIBRARY_BASE_RETURN_BODY(T,, M)
#define BUSERASSETLIBRARY_RETURN(M)           BUSERASSETLIBRARY_BASE_RETURN_BODY(,, M)

#define BUSERMENUITEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUserMenuItem3_6_0*>(data_ptr)-> C); 
#define BUSERMENUITEM_RETURN_REF(T, M)    BUSERMENUITEM_BASE_RETURN_BODY(T, &, M)
#define BUSERMENUITEM_RETURN_AS(T, M)     BUSERMENUITEM_BASE_RETURN_BODY(T,, M)
#define BUSERMENUITEM_RETURN(M)           BUSERMENUITEM_BASE_RETURN_BODY(,, M)

#define BUSERMENUITEM_MENU_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUserMenuItem_Menu3_6_0*>(data_ptr)-> C); 
#define BUSERMENUITEM_MENU_RETURN_REF(T, M)    BUSERMENUITEM_MENU_BASE_RETURN_BODY(T, &, M)
#define BUSERMENUITEM_MENU_RETURN_AS(T, M)     BUSERMENUITEM_MENU_BASE_RETURN_BODY(T,, M)
#define BUSERMENUITEM_MENU_RETURN(M)           BUSERMENUITEM_MENU_BASE_RETURN_BODY(,, M)

#define BUSERMENUITEM_OP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUserMenuItem_Op3_6_0*>(data_ptr)-> C); 
#define BUSERMENUITEM_OP_RETURN_REF(T, M)    BUSERMENUITEM_OP_BASE_RETURN_BODY(T, &, M)
#define BUSERMENUITEM_OP_RETURN_AS(T, M)     BUSERMENUITEM_OP_BASE_RETURN_BODY(T,, M)
#define BUSERMENUITEM_OP_RETURN(M)           BUSERMENUITEM_OP_BASE_RETURN_BODY(,, M)

#define BUSERMENUITEM_PROP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUserMenuItem_Prop3_6_0*>(data_ptr)-> C); 
#define BUSERMENUITEM_PROP_RETURN_REF(T, M)    BUSERMENUITEM_PROP_BASE_RETURN_BODY(T, &, M)
#define BUSERMENUITEM_PROP_RETURN_AS(T, M)     BUSERMENUITEM_PROP_BASE_RETURN_BODY(T,, M)
#define BUSERMENUITEM_PROP_RETURN(M)           BUSERMENUITEM_PROP_BASE_RETURN_BODY(,, M)

#define BUSERMENU_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUserMenu3_6_0*>(data_ptr)-> C); 
#define BUSERMENU_RETURN_REF(T, M)    BUSERMENU_BASE_RETURN_BODY(T, &, M)
#define BUSERMENU_RETURN_AS(T, M)     BUSERMENU_BASE_RETURN_BODY(T,, M)
#define BUSERMENU_RETURN(M)           BUSERMENU_BASE_RETURN_BODY(,, M)

#define BUSERSCRIPTDIRECTORY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUserScriptDirectory3_6_0*>(data_ptr)-> C); 
#define BUSERSCRIPTDIRECTORY_RETURN_REF(T, M)    BUSERSCRIPTDIRECTORY_BASE_RETURN_BODY(T, &, M)
#define BUSERSCRIPTDIRECTORY_RETURN_AS(T, M)     BUSERSCRIPTDIRECTORY_BASE_RETURN_BODY(T,, M)
#define BUSERSCRIPTDIRECTORY_RETURN(M)           BUSERSCRIPTDIRECTORY_BASE_RETURN_BODY(,, M)

#define BUUID_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<bUUID3_6_0*>(data_ptr)-> C); 
#define BUUID_RETURN_REF(T, M)    BUUID_BASE_RETURN_BODY(T, &, M)
#define BUUID_RETURN_AS(T, M)     BUUID_BASE_RETURN_BODY(T,, M)
#define BUUID_RETURN(M)           BUUID_BASE_RETURN_BODY(,, M)

#define CACHEFILELAYER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CacheFileLayer3_6_0*>(data_ptr)-> C); 
#define CACHEFILELAYER_RETURN_REF(T, M)    CACHEFILELAYER_BASE_RETURN_BODY(T, &, M)
#define CACHEFILELAYER_RETURN_AS(T, M)     CACHEFILELAYER_BASE_RETURN_BODY(T,, M)
#define CACHEFILELAYER_RETURN(M)           CACHEFILELAYER_BASE_RETURN_BODY(,, M)

#define CACHEFILE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CacheFile3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<CacheFile4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<CacheFile4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CacheFile4_4_0*>(data_ptr)-> C); 
#define CACHEFILE_RETURN_REF(T, M)    CACHEFILE_BASE_RETURN_BODY(T, &, M)
#define CACHEFILE_RETURN_AS(T, M)     CACHEFILE_BASE_RETURN_BODY(T,, M)
#define CACHEFILE_RETURN(M)           CACHEFILE_BASE_RETURN_BODY(,, M)

#define CACHEOBJECTPATH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CacheObjectPath3_6_0*>(data_ptr)-> C); 
#define CACHEOBJECTPATH_RETURN_REF(T, M)    CACHEOBJECTPATH_BASE_RETURN_BODY(T, &, M)
#define CACHEOBJECTPATH_RETURN_AS(T, M)     CACHEOBJECTPATH_BASE_RETURN_BODY(T,, M)
#define CACHEOBJECTPATH_RETURN(M)           CACHEOBJECTPATH_BASE_RETURN_BODY(,, M)

#define CAMERABGIMAGE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CameraBGImage3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<CameraBGImage4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<CameraBGImage4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CameraBGImage4_4_0*>(data_ptr)-> C); 
#define CAMERABGIMAGE_RETURN_REF(T, M)    CAMERABGIMAGE_BASE_RETURN_BODY(T, &, M)
#define CAMERABGIMAGE_RETURN_AS(T, M)     CAMERABGIMAGE_BASE_RETURN_BODY(T,, M)
#define CAMERABGIMAGE_RETURN(M)           CAMERABGIMAGE_BASE_RETURN_BODY(,, M)

#define CAMERADOFSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CameraDOFSettings3_6_0*>(data_ptr)-> C); 
#define CAMERADOFSETTINGS_RETURN_REF(T, M)    CAMERADOFSETTINGS_BASE_RETURN_BODY(T, &, M)
#define CAMERADOFSETTINGS_RETURN_AS(T, M)     CAMERADOFSETTINGS_BASE_RETURN_BODY(T,, M)
#define CAMERADOFSETTINGS_RETURN(M)           CAMERADOFSETTINGS_BASE_RETURN_BODY(,, M)

#define CAMERASTEREOSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CameraStereoSettings3_6_0*>(data_ptr)-> C); 
#define CAMERASTEREOSETTINGS_RETURN_REF(T, M)    CAMERASTEREOSETTINGS_BASE_RETURN_BODY(T, &, M)
#define CAMERASTEREOSETTINGS_RETURN_AS(T, M)     CAMERASTEREOSETTINGS_BASE_RETURN_BODY(T,, M)
#define CAMERASTEREOSETTINGS_RETURN(M)           CAMERASTEREOSETTINGS_BASE_RETURN_BODY(,, M)

#define CAMERA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Camera3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Camera4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Camera4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Camera4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Camera4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Camera4_4_0*>(data_ptr)-> C); 
#define CAMERA_RETURN_REF(T, M)    CAMERA_BASE_RETURN_BODY(T, &, M)
#define CAMERA_RETURN_AS(T, M)     CAMERA_BASE_RETURN_BODY(T,, M)
#define CAMERA_RETURN(M)           CAMERA_BASE_RETURN_BODY(,, M)

#define CAMERA_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Camera_Runtime3_6_0*>(data_ptr)-> C); 
#define CAMERA_RUNTIME_RETURN_REF(T, M)    CAMERA_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define CAMERA_RUNTIME_RETURN_AS(T, M)     CAMERA_RUNTIME_BASE_RETURN_BODY(T,, M)
#define CAMERA_RUNTIME_RETURN(M)           CAMERA_RUNTIME_BASE_RETURN_BODY(,, M)

#define CASTMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CastModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CastModifierData4_1_0*>(data_ptr)-> C); 
#define CASTMODIFIERDATA_RETURN_REF(T, M)    CASTMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define CASTMODIFIERDATA_RETURN_AS(T, M)     CASTMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define CASTMODIFIERDATA_RETURN(M)           CASTMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define CBDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CBData3_6_0*>(data_ptr)-> C); 
#define CBDATA_RETURN_REF(T, M)    CBDATA_BASE_RETURN_BODY(T, &, M)
#define CBDATA_RETURN_AS(T, M)     CBDATA_BASE_RETURN_BODY(T,, M)
#define CBDATA_RETURN(M)           CBDATA_BASE_RETURN_BODY(,, M)

#define CHANNELDRIVER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ChannelDriver3_6_0*>(data_ptr)-> C); 
#define CHANNELDRIVER_RETURN_REF(T, M)    CHANNELDRIVER_BASE_RETURN_BODY(T, &, M)
#define CHANNELDRIVER_RETURN_AS(T, M)     CHANNELDRIVER_BASE_RETURN_BODY(T,, M)
#define CHANNELDRIVER_RETURN(M)           CHANNELDRIVER_BASE_RETURN_BODY(,, M)

#define CHARINFO_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CharInfo3_6_0*>(data_ptr)-> C); 
#define CHARINFO_RETURN_REF(T, M)    CHARINFO_BASE_RETURN_BODY(T, &, M)
#define CHARINFO_RETURN_AS(T, M)     CHARINFO_BASE_RETURN_BODY(T,, M)
#define CHARINFO_RETURN(M)           CHARINFO_BASE_RETURN_BODY(,, M)

#define CHILDPARTICLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ChildParticle3_6_0*>(data_ptr)-> C); 
#define CHILDPARTICLE_RETURN_REF(T, M)    CHILDPARTICLE_BASE_RETURN_BODY(T, &, M)
#define CHILDPARTICLE_RETURN_AS(T, M)     CHILDPARTICLE_BASE_RETURN_BODY(T,, M)
#define CHILDPARTICLE_RETURN(M)           CHILDPARTICLE_BASE_RETURN_BODY(,, M)

#define CLOTHCOLLSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ClothCollSettings3_6_0*>(data_ptr)-> C); 
#define CLOTHCOLLSETTINGS_RETURN_REF(T, M)    CLOTHCOLLSETTINGS_BASE_RETURN_BODY(T, &, M)
#define CLOTHCOLLSETTINGS_RETURN_AS(T, M)     CLOTHCOLLSETTINGS_BASE_RETURN_BODY(T,, M)
#define CLOTHCOLLSETTINGS_RETURN(M)           CLOTHCOLLSETTINGS_BASE_RETURN_BODY(,, M)

#define CLOTHMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ClothModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ClothModifierData4_1_0*>(data_ptr)-> C); 
#define CLOTHMODIFIERDATA_RETURN_REF(T, M)    CLOTHMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define CLOTHMODIFIERDATA_RETURN_AS(T, M)     CLOTHMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define CLOTHMODIFIERDATA_RETURN(M)           CLOTHMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define CLOTHSIMSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ClothSimSettings3_6_0*>(data_ptr)-> C); 
#define CLOTHSIMSETTINGS_RETURN_REF(T, M)    CLOTHSIMSETTINGS_BASE_RETURN_BODY(T, &, M)
#define CLOTHSIMSETTINGS_RETURN_AS(T, M)     CLOTHSIMSETTINGS_BASE_RETURN_BODY(T,, M)
#define CLOTHSIMSETTINGS_RETURN(M)           CLOTHSIMSETTINGS_BASE_RETURN_BODY(,, M)

#define COLLECTIONCHILD_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<CollectionChild3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CollectionChild4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<CollectionChild4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<CollectionChild4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CollectionChild4_4_0*>(data_ptr)-> C); 
#define COLLECTIONCHILD_RETURN_REF(T, M)    COLLECTIONCHILD_BASE_RETURN_BODY(T, &, M)
#define COLLECTIONCHILD_RETURN_AS(T, M)     COLLECTIONCHILD_BASE_RETURN_BODY(T,, M)
#define COLLECTIONCHILD_RETURN(M)           COLLECTIONCHILD_BASE_RETURN_BODY(,, M)

#define COLLECTIONOBJECT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<CollectionObject3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CollectionObject4_0_0*>(data_ptr)-> C); 
#define COLLECTIONOBJECT_RETURN_REF(T, M)    COLLECTIONOBJECT_BASE_RETURN_BODY(T, &, M)
#define COLLECTIONOBJECT_RETURN_AS(T, M)     COLLECTIONOBJECT_BASE_RETURN_BODY(T,, M)
#define COLLECTIONOBJECT_RETURN(M)           COLLECTIONOBJECT_BASE_RETURN_BODY(,, M)

#define COLLECTION_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Collection3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Collection4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Collection4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Collection4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Collection4_4_0*>(data_ptr)-> C); 
#define COLLECTION_RETURN_REF(T, M)    COLLECTION_BASE_RETURN_BODY(T, &, M)
#define COLLECTION_RETURN_AS(T, M)     COLLECTION_BASE_RETURN_BODY(T,, M)
#define COLLECTION_RETURN(M)           COLLECTION_BASE_RETURN_BODY(,, M)

#define COLLECTION_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Collection_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Collection_Runtime4_1_0*>(data_ptr)-> C); 
#define COLLECTION_RUNTIME_RETURN_REF(T, M)    COLLECTION_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define COLLECTION_RUNTIME_RETURN_AS(T, M)     COLLECTION_RUNTIME_BASE_RETURN_BODY(T,, M)
#define COLLECTION_RUNTIME_RETURN(M)           COLLECTION_RUNTIME_BASE_RETURN_BODY(,, M)

#define COLLISIONMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CollisionModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CollisionModifierData4_1_0*>(data_ptr)-> C); 
#define COLLISIONMODIFIERDATA_RETURN_REF(T, M)    COLLISIONMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define COLLISIONMODIFIERDATA_RETURN_AS(T, M)     COLLISIONMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define COLLISIONMODIFIERDATA_RETURN(M)           COLLISIONMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define COLORBALANCEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ColorBalanceModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ColorBalanceModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ColorBalanceModifierData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ColorBalanceModifierData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ColorBalanceModifierData4_4_0*>(data_ptr)-> C); 
#define COLORBALANCEMODIFIERDATA_RETURN_REF(T, M)    COLORBALANCEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define COLORBALANCEMODIFIERDATA_RETURN_AS(T, M)     COLORBALANCEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define COLORBALANCEMODIFIERDATA_RETURN(M)           COLORBALANCEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define COLORCORRECTIONDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ColorCorrectionData3_6_0*>(data_ptr)-> C); 
#define COLORCORRECTIONDATA_RETURN_REF(T, M)    COLORCORRECTIONDATA_BASE_RETURN_BODY(T, &, M)
#define COLORCORRECTIONDATA_RETURN_AS(T, M)     COLORCORRECTIONDATA_BASE_RETURN_BODY(T,, M)
#define COLORCORRECTIONDATA_RETURN(M)           COLORCORRECTIONDATA_BASE_RETURN_BODY(,, M)

#define COLORGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ColorGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define COLORGPENCILMODIFIERDATA_RETURN_REF(T, M)    COLORGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define COLORGPENCILMODIFIERDATA_RETURN_AS(T, M)     COLORGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define COLORGPENCILMODIFIERDATA_RETURN(M)           COLORGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define COLORIZESHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ColorizeShaderFxData3_6_0*>(data_ptr)-> C); 
#define COLORIZESHADERFXDATA_RETURN_REF(T, M)    COLORIZESHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define COLORIZESHADERFXDATA_RETURN_AS(T, M)     COLORIZESHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define COLORIZESHADERFXDATA_RETURN(M)           COLORIZESHADERFXDATA_BASE_RETURN_BODY(,, M)

#define COLORMANAGEDCOLORSPACESETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ColorManagedColorspaceSettings3_6_0*>(data_ptr)-> C); 
#define COLORMANAGEDCOLORSPACESETTINGS_RETURN_REF(T, M)    COLORMANAGEDCOLORSPACESETTINGS_BASE_RETURN_BODY(T, &, M)
#define COLORMANAGEDCOLORSPACESETTINGS_RETURN_AS(T, M)     COLORMANAGEDCOLORSPACESETTINGS_BASE_RETURN_BODY(T,, M)
#define COLORMANAGEDCOLORSPACESETTINGS_RETURN(M)           COLORMANAGEDCOLORSPACESETTINGS_BASE_RETURN_BODY(,, M)

#define COLORMANAGEDDISPLAYSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ColorManagedDisplaySettings3_6_0*>(data_ptr)-> C); 
#define COLORMANAGEDDISPLAYSETTINGS_RETURN_REF(T, M)    COLORMANAGEDDISPLAYSETTINGS_BASE_RETURN_BODY(T, &, M)
#define COLORMANAGEDDISPLAYSETTINGS_RETURN_AS(T, M)     COLORMANAGEDDISPLAYSETTINGS_BASE_RETURN_BODY(T,, M)
#define COLORMANAGEDDISPLAYSETTINGS_RETURN(M)           COLORMANAGEDDISPLAYSETTINGS_BASE_RETURN_BODY(,, M)

#define COLORMANAGEDVIEWSETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ColorManagedViewSettings3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ColorManagedViewSettings4_3_0*>(data_ptr)-> C); 
#define COLORMANAGEDVIEWSETTINGS_RETURN_REF(T, M)    COLORMANAGEDVIEWSETTINGS_BASE_RETURN_BODY(T, &, M)
#define COLORMANAGEDVIEWSETTINGS_RETURN_AS(T, M)     COLORMANAGEDVIEWSETTINGS_BASE_RETURN_BODY(T,, M)
#define COLORMANAGEDVIEWSETTINGS_RETURN(M)           COLORMANAGEDVIEWSETTINGS_BASE_RETURN_BODY(,, M)

#define COLORMIXVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ColorMixVars3_6_0*>(data_ptr)-> C); 
#define COLORMIXVARS_RETURN_REF(T, M)    COLORMIXVARS_BASE_RETURN_BODY(T, &, M)
#define COLORMIXVARS_RETURN_AS(T, M)     COLORMIXVARS_BASE_RETURN_BODY(T,, M)
#define COLORMIXVARS_RETURN(M)           COLORMIXVARS_BASE_RETURN_BODY(,, M)

#define CONSOLELINE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ConsoleLine3_6_0*>(data_ptr)-> C); 
#define CONSOLELINE_RETURN_REF(T, M)    CONSOLELINE_BASE_RETURN_BODY(T, &, M)
#define CONSOLELINE_RETURN_AS(T, M)     CONSOLELINE_BASE_RETURN_BODY(T,, M)
#define CONSOLELINE_RETURN(M)           CONSOLELINE_BASE_RETURN_BODY(,, M)

#define CORRECTIVESMOOTHDELTACACHE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CorrectiveSmoothDeltaCache3_6_0*>(data_ptr)-> C); 
#define CORRECTIVESMOOTHDELTACACHE_RETURN_REF(T, M)    CORRECTIVESMOOTHDELTACACHE_BASE_RETURN_BODY(T, &, M)
#define CORRECTIVESMOOTHDELTACACHE_RETURN_AS(T, M)     CORRECTIVESMOOTHDELTACACHE_BASE_RETURN_BODY(T,, M)
#define CORRECTIVESMOOTHDELTACACHE_RETURN(M)           CORRECTIVESMOOTHDELTACACHE_BASE_RETURN_BODY(,, M)

#define CORRECTIVESMOOTHMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CorrectiveSmoothModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CorrectiveSmoothModifierData4_1_0*>(data_ptr)-> C); 
#define CORRECTIVESMOOTHMODIFIERDATA_RETURN_REF(T, M)    CORRECTIVESMOOTHMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define CORRECTIVESMOOTHMODIFIERDATA_RETURN_AS(T, M)     CORRECTIVESMOOTHMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define CORRECTIVESMOOTHMODIFIERDATA_RETURN(M)           CORRECTIVESMOOTHMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define CRYPTOMATTEENTRY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CryptomatteEntry3_6_0*>(data_ptr)-> C); 
#define CRYPTOMATTEENTRY_RETURN_REF(T, M)    CRYPTOMATTEENTRY_BASE_RETURN_BODY(T, &, M)
#define CRYPTOMATTEENTRY_RETURN_AS(T, M)     CRYPTOMATTEENTRY_BASE_RETURN_BODY(T,, M)
#define CRYPTOMATTEENTRY_RETURN(M)           CRYPTOMATTEENTRY_BASE_RETURN_BODY(,, M)

#define CRYPTOMATTELAYER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CryptomatteLayer3_6_0*>(data_ptr)-> C); 
#define CRYPTOMATTELAYER_RETURN_REF(T, M)    CRYPTOMATTELAYER_BASE_RETURN_BODY(T, &, M)
#define CRYPTOMATTELAYER_RETURN_AS(T, M)     CRYPTOMATTELAYER_BASE_RETURN_BODY(T,, M)
#define CRYPTOMATTELAYER_RETURN(M)           CRYPTOMATTELAYER_BASE_RETURN_BODY(,, M)

#define CURVEMAPPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CurveMapPoint3_6_0*>(data_ptr)-> C); 
#define CURVEMAPPOINT_RETURN_REF(T, M)    CURVEMAPPOINT_BASE_RETURN_BODY(T, &, M)
#define CURVEMAPPOINT_RETURN_AS(T, M)     CURVEMAPPOINT_BASE_RETURN_BODY(T,, M)
#define CURVEMAPPOINT_RETURN(M)           CURVEMAPPOINT_BASE_RETURN_BODY(,, M)

#define CURVEMAP_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<CurveMap3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CurveMap4_0_0*>(data_ptr)-> C); 
#define CURVEMAP_RETURN_REF(T, M)    CURVEMAP_BASE_RETURN_BODY(T, &, M)
#define CURVEMAP_RETURN_AS(T, M)     CURVEMAP_BASE_RETURN_BODY(T,, M)
#define CURVEMAP_RETURN(M)           CURVEMAP_BASE_RETURN_BODY(,, M)

#define CURVEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CurveModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CurveModifierData4_1_0*>(data_ptr)-> C); 
#define CURVEMODIFIERDATA_RETURN_REF(T, M)    CURVEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define CURVEMODIFIERDATA_RETURN_AS(T, M)     CURVEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define CURVEMODIFIERDATA_RETURN(M)           CURVEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define CURVEPAINTSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CurvePaintSettings3_6_0*>(data_ptr)-> C); 
#define CURVEPAINTSETTINGS_RETURN_REF(T, M)    CURVEPAINTSETTINGS_BASE_RETURN_BODY(T, &, M)
#define CURVEPAINTSETTINGS_RETURN_AS(T, M)     CURVEPAINTSETTINGS_BASE_RETURN_BODY(T,, M)
#define CURVEPAINTSETTINGS_RETURN(M)           CURVEPAINTSETTINGS_BASE_RETURN_BODY(,, M)

#define CURVEPROFILEPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CurveProfilePoint3_6_0*>(data_ptr)-> C); 
#define CURVEPROFILEPOINT_RETURN_REF(T, M)    CURVEPROFILEPOINT_BASE_RETURN_BODY(T, &, M)
#define CURVEPROFILEPOINT_RETURN_AS(T, M)     CURVEPROFILEPOINT_BASE_RETURN_BODY(T,, M)
#define CURVEPROFILEPOINT_RETURN(M)           CURVEPROFILEPOINT_BASE_RETURN_BODY(,, M)

#define CURVEPROFILE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CurveProfile3_6_0*>(data_ptr)-> C); 
#define CURVEPROFILE_RETURN_REF(T, M)    CURVEPROFILE_BASE_RETURN_BODY(T, &, M)
#define CURVEPROFILE_RETURN_AS(T, M)     CURVEPROFILE_BASE_RETURN_BODY(T,, M)
#define CURVEPROFILE_RETURN(M)           CURVEPROFILE_BASE_RETURN_BODY(,, M)

#define CURVESGEOMETRY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<CurvesGeometry3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<CurvesGeometry4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CurvesGeometry4_3_0*>(data_ptr)-> C); 
#define CURVESGEOMETRY_RETURN_REF(T, M)    CURVESGEOMETRY_BASE_RETURN_BODY(T, &, M)
#define CURVESGEOMETRY_RETURN_AS(T, M)     CURVESGEOMETRY_BASE_RETURN_BODY(T,, M)
#define CURVESGEOMETRY_RETURN(M)           CURVESGEOMETRY_BASE_RETURN_BODY(,, M)

#define CURVESSCULPT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<CurvesSculpt3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<CurvesSculpt4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CurvesSculpt4_3_0*>(data_ptr)-> C); 
#define CURVESSCULPT_RETURN_REF(T, M)    CURVESSCULPT_BASE_RETURN_BODY(T, &, M)
#define CURVESSCULPT_RETURN_AS(T, M)     CURVESSCULPT_BASE_RETURN_BODY(T,, M)
#define CURVESSCULPT_RETURN(M)           CURVESSCULPT_BASE_RETURN_BODY(,, M)

#define CURVES_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Curves3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Curves4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Curves4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Curves4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Curves4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Curves4_4_0*>(data_ptr)-> C); 
#define CURVES_RETURN_REF(T, M)    CURVES_BASE_RETURN_BODY(T, &, M)
#define CURVES_RETURN_AS(T, M)     CURVES_BASE_RETURN_BODY(T,, M)
#define CURVES_RETURN(M)           CURVES_BASE_RETURN_BODY(,, M)

#define CURVE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Curve3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Curve4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Curve4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Curve4_4_0*>(data_ptr)-> C); 
#define CURVE_RETURN_REF(T, M)    CURVE_BASE_RETURN_BODY(T, &, M)
#define CURVE_RETURN_AS(T, M)     CURVE_BASE_RETURN_BODY(T,, M)
#define CURVE_RETURN(M)           CURVE_BASE_RETURN_BODY(,, M)

#define CUSTOMDATAEXTERNAL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CustomDataExternal3_6_0*>(data_ptr)-> C); 
#define CUSTOMDATAEXTERNAL_RETURN_REF(T, M)    CUSTOMDATAEXTERNAL_BASE_RETURN_BODY(T, &, M)
#define CUSTOMDATAEXTERNAL_RETURN_AS(T, M)     CUSTOMDATAEXTERNAL_BASE_RETURN_BODY(T,, M)
#define CUSTOMDATAEXTERNAL_RETURN(M)           CUSTOMDATAEXTERNAL_BASE_RETURN_BODY(,, M)

#define CUSTOMDATALAYER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<CustomDataLayer3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CustomDataLayer4_3_0*>(data_ptr)-> C); 
#define CUSTOMDATALAYER_RETURN_REF(T, M)    CUSTOMDATALAYER_BASE_RETURN_BODY(T, &, M)
#define CUSTOMDATALAYER_RETURN_AS(T, M)     CUSTOMDATALAYER_BASE_RETURN_BODY(T,, M)
#define CUSTOMDATALAYER_RETURN(M)           CUSTOMDATALAYER_BASE_RETURN_BODY(,, M)

#define CUSTOMDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<CustomData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<CustomData4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<CustomData4_3_0*>(data_ptr)-> C); 
#define CUSTOMDATA_RETURN_REF(T, M)    CUSTOMDATA_BASE_RETURN_BODY(T, &, M)
#define CUSTOMDATA_RETURN_AS(T, M)     CUSTOMDATA_BASE_RETURN_BODY(T,, M)
#define CUSTOMDATA_RETURN(M)           CUSTOMDATA_BASE_RETURN_BODY(,, M)

#define CUSTOMDATA_MESHMASKS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<CustomData_MeshMasks3_6_0*>(data_ptr)-> C); 
#define CUSTOMDATA_MESHMASKS_RETURN_REF(T, M)    CUSTOMDATA_MESHMASKS_BASE_RETURN_BODY(T, &, M)
#define CUSTOMDATA_MESHMASKS_RETURN_AS(T, M)     CUSTOMDATA_MESHMASKS_BASE_RETURN_BODY(T,, M)
#define CUSTOMDATA_MESHMASKS_RETURN(M)           CUSTOMDATA_MESHMASKS_BASE_RETURN_BODY(,, M)

#define DASHGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DashGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define DASHGPENCILMODIFIERDATA_RETURN_REF(T, M)    DASHGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define DASHGPENCILMODIFIERDATA_RETURN_AS(T, M)     DASHGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define DASHGPENCILMODIFIERDATA_RETURN(M)           DASHGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define DASHGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DashGpencilModifierSegment3_6_0*>(data_ptr)-> C); 
#define DASHGPENCILMODIFIERSEGMENT_RETURN_REF(T, M)    DASHGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(T, &, M)
#define DASHGPENCILMODIFIERSEGMENT_RETURN_AS(T, M)     DASHGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(T,, M)
#define DASHGPENCILMODIFIERSEGMENT_RETURN(M)           DASHGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(,, M)

#define DATATRANSFERMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<DataTransferModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<DataTransferModifierData4_1_0*>(data_ptr)-> C); 
#define DATATRANSFERMODIFIERDATA_RETURN_REF(T, M)    DATATRANSFERMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define DATATRANSFERMODIFIERDATA_RETURN_AS(T, M)     DATATRANSFERMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define DATATRANSFERMODIFIERDATA_RETURN(M)           DATATRANSFERMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define DECIMATEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<DecimateModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<DecimateModifierData4_1_0*>(data_ptr)-> C); 
#define DECIMATEMODIFIERDATA_RETURN_REF(T, M)    DECIMATEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define DECIMATEMODIFIERDATA_RETURN_AS(T, M)     DECIMATEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define DECIMATEMODIFIERDATA_RETURN(M)           DECIMATEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define DISPLACEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<DisplaceModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<DisplaceModifierData4_1_0*>(data_ptr)-> C); 
#define DISPLACEMODIFIERDATA_RETURN_REF(T, M)    DISPLACEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define DISPLACEMODIFIERDATA_RETURN_AS(T, M)     DISPLACEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define DISPLACEMODIFIERDATA_RETURN(M)           DISPLACEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define DISPLAYSAFEAREAS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DisplaySafeAreas3_6_0*>(data_ptr)-> C); 
#define DISPLAYSAFEAREAS_RETURN_REF(T, M)    DISPLAYSAFEAREAS_BASE_RETURN_BODY(T, &, M)
#define DISPLAYSAFEAREAS_RETURN_AS(T, M)     DISPLAYSAFEAREAS_BASE_RETURN_BODY(T,, M)
#define DISPLAYSAFEAREAS_RETURN(M)           DISPLAYSAFEAREAS_BASE_RETURN_BODY(,, M)

#define DRAWDATALIST_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DrawDataList3_6_0*>(data_ptr)-> C); 
#define DRAWDATALIST_RETURN_REF(T, M)    DRAWDATALIST_BASE_RETURN_BODY(T, &, M)
#define DRAWDATALIST_RETURN_AS(T, M)     DRAWDATALIST_BASE_RETURN_BODY(T,, M)
#define DRAWDATALIST_RETURN(M)           DRAWDATALIST_BASE_RETURN_BODY(,, M)

#define DRIVERTARGET_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<DriverTarget3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<DriverTarget4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<DriverTarget4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<DriverTarget4_4_0*>(data_ptr)-> C); 
#define DRIVERTARGET_RETURN_REF(T, M)    DRIVERTARGET_BASE_RETURN_BODY(T, &, M)
#define DRIVERTARGET_RETURN_AS(T, M)     DRIVERTARGET_BASE_RETURN_BODY(T,, M)
#define DRIVERTARGET_RETURN(M)           DRIVERTARGET_BASE_RETURN_BODY(,, M)

#define DUALQUAT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DualQuat3_6_0*>(data_ptr)-> C); 
#define DUALQUAT_RETURN_REF(T, M)    DUALQUAT_BASE_RETURN_BODY(T, &, M)
#define DUALQUAT_RETURN_AS(T, M)     DUALQUAT_BASE_RETURN_BODY(T,, M)
#define DUALQUAT_RETURN(M)           DUALQUAT_BASE_RETURN_BODY(,, M)

#define DYNAMICPAINTBRUSHSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DynamicPaintBrushSettings3_6_0*>(data_ptr)-> C); 
#define DYNAMICPAINTBRUSHSETTINGS_RETURN_REF(T, M)    DYNAMICPAINTBRUSHSETTINGS_BASE_RETURN_BODY(T, &, M)
#define DYNAMICPAINTBRUSHSETTINGS_RETURN_AS(T, M)     DYNAMICPAINTBRUSHSETTINGS_BASE_RETURN_BODY(T,, M)
#define DYNAMICPAINTBRUSHSETTINGS_RETURN(M)           DYNAMICPAINTBRUSHSETTINGS_BASE_RETURN_BODY(,, M)

#define DYNAMICPAINTCANVASSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DynamicPaintCanvasSettings3_6_0*>(data_ptr)-> C); 
#define DYNAMICPAINTCANVASSETTINGS_RETURN_REF(T, M)    DYNAMICPAINTCANVASSETTINGS_BASE_RETURN_BODY(T, &, M)
#define DYNAMICPAINTCANVASSETTINGS_RETURN_AS(T, M)     DYNAMICPAINTCANVASSETTINGS_BASE_RETURN_BODY(T,, M)
#define DYNAMICPAINTCANVASSETTINGS_RETURN(M)           DYNAMICPAINTCANVASSETTINGS_BASE_RETURN_BODY(,, M)

#define DYNAMICPAINTMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<DynamicPaintModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<DynamicPaintModifierData4_1_0*>(data_ptr)-> C); 
#define DYNAMICPAINTMODIFIERDATA_RETURN_REF(T, M)    DYNAMICPAINTMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define DYNAMICPAINTMODIFIERDATA_RETURN_AS(T, M)     DYNAMICPAINTMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define DYNAMICPAINTMODIFIERDATA_RETURN(M)           DYNAMICPAINTMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define DYNAMICPAINTRUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DynamicPaintRuntime3_6_0*>(data_ptr)-> C); 
#define DYNAMICPAINTRUNTIME_RETURN_REF(T, M)    DYNAMICPAINTRUNTIME_BASE_RETURN_BODY(T, &, M)
#define DYNAMICPAINTRUNTIME_RETURN_AS(T, M)     DYNAMICPAINTRUNTIME_BASE_RETURN_BODY(T,, M)
#define DYNAMICPAINTRUNTIME_RETURN(M)           DYNAMICPAINTRUNTIME_BASE_RETURN_BODY(,, M)

#define DYNAMICPAINTSURFACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<DynamicPaintSurface3_6_0*>(data_ptr)-> C); 
#define DYNAMICPAINTSURFACE_RETURN_REF(T, M)    DYNAMICPAINTSURFACE_BASE_RETURN_BODY(T, &, M)
#define DYNAMICPAINTSURFACE_RETURN_AS(T, M)     DYNAMICPAINTSURFACE_BASE_RETURN_BODY(T,, M)
#define DYNAMICPAINTSURFACE_RETURN(M)           DYNAMICPAINTSURFACE_BASE_RETURN_BODY(,, M)

#define EDGESPLITMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<EdgeSplitModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<EdgeSplitModifierData4_1_0*>(data_ptr)-> C); 
#define EDGESPLITMODIFIERDATA_RETURN_REF(T, M)    EDGESPLITMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define EDGESPLITMODIFIERDATA_RETURN_AS(T, M)     EDGESPLITMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define EDGESPLITMODIFIERDATA_RETURN(M)           EDGESPLITMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define EDITINGRUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<EditingRuntime3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<EditingRuntime4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<EditingRuntime4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<EditingRuntime4_4_0*>(data_ptr)-> C); 
#define EDITINGRUNTIME_RETURN_REF(T, M)    EDITINGRUNTIME_BASE_RETURN_BODY(T, &, M)
#define EDITINGRUNTIME_RETURN_AS(T, M)     EDITINGRUNTIME_BASE_RETURN_BODY(T,, M)
#define EDITINGRUNTIME_RETURN(M)           EDITINGRUNTIME_BASE_RETURN_BODY(,, M)

#define EDITING_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Editing3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Editing4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Editing4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Editing4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Editing4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Editing4_4_0*>(data_ptr)-> C); 
#define EDITING_RETURN_REF(T, M)    EDITING_BASE_RETURN_BODY(T, &, M)
#define EDITING_RETURN_AS(T, M)     EDITING_BASE_RETURN_BODY(T,, M)
#define EDITING_RETURN(M)           EDITING_BASE_RETURN_BODY(,, M)

#define EDITLATT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<EditLatt3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<EditLatt4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<EditLatt4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<EditLatt4_4_0*>(data_ptr)-> C); 
#define EDITLATT_RETURN_REF(T, M)    EDITLATT_BASE_RETURN_BODY(T, &, M)
#define EDITLATT_RETURN_AS(T, M)     EDITLATT_BASE_RETURN_BODY(T,, M)
#define EDITLATT_RETURN(M)           EDITLATT_BASE_RETURN_BODY(,, M)

#define EDITNURB_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<EditNurb3_6_0*>(data_ptr)-> C); 
#define EDITNURB_RETURN_REF(T, M)    EDITNURB_BASE_RETURN_BODY(T, &, M)
#define EDITNURB_RETURN_AS(T, M)     EDITNURB_BASE_RETURN_BODY(T,, M)
#define EDITNURB_RETURN(M)           EDITNURB_BASE_RETURN_BODY(,, M)

#define EFFECTORWEIGHTS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<EffectorWeights3_6_0*>(data_ptr)-> C); 
#define EFFECTORWEIGHTS_RETURN_REF(T, M)    EFFECTORWEIGHTS_BASE_RETURN_BODY(T, &, M)
#define EFFECTORWEIGHTS_RETURN_AS(T, M)     EFFECTORWEIGHTS_BASE_RETURN_BODY(T,, M)
#define EFFECTORWEIGHTS_RETURN(M)           EFFECTORWEIGHTS_BASE_RETURN_BODY(,, M)

#define EFFECT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Effect3_6_0*>(data_ptr)-> C); 
#define EFFECT_RETURN_REF(T, M)    EFFECT_BASE_RETURN_BODY(T, &, M)
#define EFFECT_RETURN_AS(T, M)     EFFECT_BASE_RETURN_BODY(T,, M)
#define EFFECT_RETURN(M)           EFFECT_BASE_RETURN_BODY(,, M)

#define ENVELOPEGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<EnvelopeGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define ENVELOPEGPENCILMODIFIERDATA_RETURN_REF(T, M)    ENVELOPEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define ENVELOPEGPENCILMODIFIERDATA_RETURN_AS(T, M)     ENVELOPEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define ENVELOPEGPENCILMODIFIERDATA_RETURN(M)           ENVELOPEGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define EXPLODEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ExplodeModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ExplodeModifierData4_1_0*>(data_ptr)-> C); 
#define EXPLODEMODIFIERDATA_RETURN_REF(T, M)    EXPLODEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define EXPLODEMODIFIERDATA_RETURN_AS(T, M)     EXPLODEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define EXPLODEMODIFIERDATA_RETURN(M)           EXPLODEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define FCM_ENVELOPEDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FCM_EnvelopeData3_6_0*>(data_ptr)-> C); 
#define FCM_ENVELOPEDATA_RETURN_REF(T, M)    FCM_ENVELOPEDATA_BASE_RETURN_BODY(T, &, M)
#define FCM_ENVELOPEDATA_RETURN_AS(T, M)     FCM_ENVELOPEDATA_BASE_RETURN_BODY(T,, M)
#define FCM_ENVELOPEDATA_RETURN(M)           FCM_ENVELOPEDATA_BASE_RETURN_BODY(,, M)

#define FCURVE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<FCurve3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FCurve4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FCurve4_4_0*>(data_ptr)-> C); 
#define FCURVE_RETURN_REF(T, M)    FCURVE_BASE_RETURN_BODY(T, &, M)
#define FCURVE_RETURN_AS(T, M)     FCURVE_BASE_RETURN_BODY(T,, M)
#define FCURVE_RETURN(M)           FCURVE_BASE_RETURN_BODY(,, M)

#define FFMPEGCODECDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FFMpegCodecData3_6_0*>(data_ptr)-> C); 
#define FFMPEGCODECDATA_RETURN_REF(T, M)    FFMPEGCODECDATA_BASE_RETURN_BODY(T, &, M)
#define FFMPEGCODECDATA_RETURN_AS(T, M)     FFMPEGCODECDATA_BASE_RETURN_BODY(T,, M)
#define FFMPEGCODECDATA_RETURN(M)           FFMPEGCODECDATA_BASE_RETURN_BODY(,, M)

#define FILEASSETSELECTPARAMS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<FileAssetSelectParams3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<FileAssetSelectParams4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<FileAssetSelectParams4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FileAssetSelectParams4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FileAssetSelectParams4_4_0*>(data_ptr)-> C); 
#define FILEASSETSELECTPARAMS_RETURN_REF(T, M)    FILEASSETSELECTPARAMS_BASE_RETURN_BODY(T, &, M)
#define FILEASSETSELECTPARAMS_RETURN_AS(T, M)     FILEASSETSELECTPARAMS_BASE_RETURN_BODY(T,, M)
#define FILEASSETSELECTPARAMS_RETURN(M)           FILEASSETSELECTPARAMS_BASE_RETURN_BODY(,, M)

#define FILEDIRENTRYARR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FileDirEntryArr3_6_0*>(data_ptr)-> C); 
#define FILEDIRENTRYARR_RETURN_REF(T, M)    FILEDIRENTRYARR_BASE_RETURN_BODY(T, &, M)
#define FILEDIRENTRYARR_RETURN_AS(T, M)     FILEDIRENTRYARR_BASE_RETURN_BODY(T,, M)
#define FILEDIRENTRYARR_RETURN(M)           FILEDIRENTRYARR_BASE_RETURN_BODY(,, M)

#define FILEDIRENTRY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FileDirEntry4_2_8*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FileDirEntry4_4_0*>(data_ptr)-> C); 
#define FILEDIRENTRY_RETURN_REF(T, M)    FILEDIRENTRY_BASE_RETURN_BODY(T, &, M)
#define FILEDIRENTRY_RETURN_AS(T, M)     FILEDIRENTRY_BASE_RETURN_BODY(T,, M)
#define FILEDIRENTRY_RETURN(M)           FILEDIRENTRY_BASE_RETURN_BODY(,, M)

#define FILEFOLDERHISTORY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FileFolderHistory3_6_0*>(data_ptr)-> C); 
#define FILEFOLDERHISTORY_RETURN_REF(T, M)    FILEFOLDERHISTORY_BASE_RETURN_BODY(T, &, M)
#define FILEFOLDERHISTORY_RETURN_AS(T, M)     FILEFOLDERHISTORY_BASE_RETURN_BODY(T,, M)
#define FILEFOLDERHISTORY_RETURN(M)           FILEFOLDERHISTORY_BASE_RETURN_BODY(,, M)

#define FILEGLOBAL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FileGlobal3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FileGlobal4_4_0*>(data_ptr)-> C); 
#define FILEGLOBAL_RETURN_REF(T, M)    FILEGLOBAL_BASE_RETURN_BODY(T, &, M)
#define FILEGLOBAL_RETURN_AS(T, M)     FILEGLOBAL_BASE_RETURN_BODY(T,, M)
#define FILEGLOBAL_RETURN(M)           FILEGLOBAL_BASE_RETURN_BODY(,, M)

#define FILESELECTPARAMS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<FileSelectParams3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<FileSelectParams4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FileSelectParams4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FileSelectParams4_4_0*>(data_ptr)-> C); 
#define FILESELECTPARAMS_RETURN_REF(T, M)    FILESELECTPARAMS_BASE_RETURN_BODY(T, &, M)
#define FILESELECTPARAMS_RETURN_AS(T, M)     FILESELECTPARAMS_BASE_RETURN_BODY(T,, M)
#define FILESELECTPARAMS_RETURN(M)           FILESELECTPARAMS_BASE_RETURN_BODY(,, M)

#define FLIPSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FlipShaderFxData3_6_0*>(data_ptr)-> C); 
#define FLIPSHADERFXDATA_RETURN_REF(T, M)    FLIPSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define FLIPSHADERFXDATA_RETURN_AS(T, M)     FLIPSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define FLIPSHADERFXDATA_RETURN(M)           FLIPSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define FLUIDEFFECTORSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FluidEffectorSettings3_6_0*>(data_ptr)-> C); 
#define FLUIDEFFECTORSETTINGS_RETURN_REF(T, M)    FLUIDEFFECTORSETTINGS_BASE_RETURN_BODY(T, &, M)
#define FLUIDEFFECTORSETTINGS_RETURN_AS(T, M)     FLUIDEFFECTORSETTINGS_BASE_RETURN_BODY(T,, M)
#define FLUIDEFFECTORSETTINGS_RETURN(M)           FLUIDEFFECTORSETTINGS_BASE_RETURN_BODY(,, M)

#define FLUIDFLOWSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FluidFlowSettings3_6_0*>(data_ptr)-> C); 
#define FLUIDFLOWSETTINGS_RETURN_REF(T, M)    FLUIDFLOWSETTINGS_BASE_RETURN_BODY(T, &, M)
#define FLUIDFLOWSETTINGS_RETURN_AS(T, M)     FLUIDFLOWSETTINGS_BASE_RETURN_BODY(T,, M)
#define FLUIDFLOWSETTINGS_RETURN(M)           FLUIDFLOWSETTINGS_BASE_RETURN_BODY(,, M)

#define FLUIDMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<FluidModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FluidModifierData4_1_0*>(data_ptr)-> C); 
#define FLUIDMODIFIERDATA_RETURN_REF(T, M)    FLUIDMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define FLUIDMODIFIERDATA_RETURN_AS(T, M)     FLUIDMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define FLUIDMODIFIERDATA_RETURN(M)           FLUIDMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define FLUIDSIMMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<FluidsimModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FluidsimModifierData4_1_0*>(data_ptr)-> C); 
#define FLUIDSIMMODIFIERDATA_RETURN_REF(T, M)    FLUIDSIMMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define FLUIDSIMMODIFIERDATA_RETURN_AS(T, M)     FLUIDSIMMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define FLUIDSIMMODIFIERDATA_RETURN(M)           FLUIDSIMMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define FLUIDSIMSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FluidsimSettings3_6_0*>(data_ptr)-> C); 
#define FLUIDSIMSETTINGS_RETURN_REF(T, M)    FLUIDSIMSETTINGS_BASE_RETURN_BODY(T, &, M)
#define FLUIDSIMSETTINGS_RETURN_AS(T, M)     FLUIDSIMSETTINGS_BASE_RETURN_BODY(T,, M)
#define FLUIDSIMSETTINGS_RETURN(M)           FLUIDSIMSETTINGS_BASE_RETURN_BODY(,, M)

#define FLUIDVERTEXVELOCITY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FluidVertexVelocity3_6_0*>(data_ptr)-> C); 
#define FLUIDVERTEXVELOCITY_RETURN_REF(T, M)    FLUIDVERTEXVELOCITY_BASE_RETURN_BODY(T, &, M)
#define FLUIDVERTEXVELOCITY_RETURN_AS(T, M)     FLUIDVERTEXVELOCITY_BASE_RETURN_BODY(T,, M)
#define FLUIDVERTEXVELOCITY_RETURN(M)           FLUIDVERTEXVELOCITY_BASE_RETURN_BODY(,, M)

#define FMODIFIER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<FModifier3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FModifier4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FModifier4_4_0*>(data_ptr)-> C); 
#define FMODIFIER_RETURN_REF(T, M)    FMODIFIER_BASE_RETURN_BODY(T, &, M)
#define FMODIFIER_RETURN_AS(T, M)     FMODIFIER_BASE_RETURN_BODY(T,, M)
#define FMODIFIER_RETURN(M)           FMODIFIER_BASE_RETURN_BODY(,, M)

#define FMOD_CYCLES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_Cycles3_6_0*>(data_ptr)-> C); 
#define FMOD_CYCLES_RETURN_REF(T, M)    FMOD_CYCLES_BASE_RETURN_BODY(T, &, M)
#define FMOD_CYCLES_RETURN_AS(T, M)     FMOD_CYCLES_BASE_RETURN_BODY(T,, M)
#define FMOD_CYCLES_RETURN(M)           FMOD_CYCLES_BASE_RETURN_BODY(,, M)

#define FMOD_ENVELOPE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_Envelope3_6_0*>(data_ptr)-> C); 
#define FMOD_ENVELOPE_RETURN_REF(T, M)    FMOD_ENVELOPE_BASE_RETURN_BODY(T, &, M)
#define FMOD_ENVELOPE_RETURN_AS(T, M)     FMOD_ENVELOPE_BASE_RETURN_BODY(T,, M)
#define FMOD_ENVELOPE_RETURN(M)           FMOD_ENVELOPE_BASE_RETURN_BODY(,, M)

#define FMOD_FUNCTIONGENERATOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_FunctionGenerator3_6_0*>(data_ptr)-> C); 
#define FMOD_FUNCTIONGENERATOR_RETURN_REF(T, M)    FMOD_FUNCTIONGENERATOR_BASE_RETURN_BODY(T, &, M)
#define FMOD_FUNCTIONGENERATOR_RETURN_AS(T, M)     FMOD_FUNCTIONGENERATOR_BASE_RETURN_BODY(T,, M)
#define FMOD_FUNCTIONGENERATOR_RETURN(M)           FMOD_FUNCTIONGENERATOR_BASE_RETURN_BODY(,, M)

#define FMOD_GENERATOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_Generator3_6_0*>(data_ptr)-> C); 
#define FMOD_GENERATOR_RETURN_REF(T, M)    FMOD_GENERATOR_BASE_RETURN_BODY(T, &, M)
#define FMOD_GENERATOR_RETURN_AS(T, M)     FMOD_GENERATOR_BASE_RETURN_BODY(T,, M)
#define FMOD_GENERATOR_RETURN(M)           FMOD_GENERATOR_BASE_RETURN_BODY(,, M)

#define FMOD_LIMITS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_Limits3_6_0*>(data_ptr)-> C); 
#define FMOD_LIMITS_RETURN_REF(T, M)    FMOD_LIMITS_BASE_RETURN_BODY(T, &, M)
#define FMOD_LIMITS_RETURN_AS(T, M)     FMOD_LIMITS_BASE_RETURN_BODY(T,, M)
#define FMOD_LIMITS_RETURN(M)           FMOD_LIMITS_BASE_RETURN_BODY(,, M)

#define FMOD_NOISE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FMod_Noise3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FMod_Noise4_4_0*>(data_ptr)-> C); 
#define FMOD_NOISE_RETURN_REF(T, M)    FMOD_NOISE_BASE_RETURN_BODY(T, &, M)
#define FMOD_NOISE_RETURN_AS(T, M)     FMOD_NOISE_BASE_RETURN_BODY(T,, M)
#define FMOD_NOISE_RETURN(M)           FMOD_NOISE_BASE_RETURN_BODY(,, M)

#define FMOD_PYTHON_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_Python3_6_0*>(data_ptr)-> C); 
#define FMOD_PYTHON_RETURN_REF(T, M)    FMOD_PYTHON_BASE_RETURN_BODY(T, &, M)
#define FMOD_PYTHON_RETURN_AS(T, M)     FMOD_PYTHON_BASE_RETURN_BODY(T,, M)
#define FMOD_PYTHON_RETURN(M)           FMOD_PYTHON_BASE_RETURN_BODY(,, M)

#define FMOD_STEPPED_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FMod_Stepped3_6_0*>(data_ptr)-> C); 
#define FMOD_STEPPED_RETURN_REF(T, M)    FMOD_STEPPED_BASE_RETURN_BODY(T, &, M)
#define FMOD_STEPPED_RETURN_AS(T, M)     FMOD_STEPPED_BASE_RETURN_BODY(T,, M)
#define FMOD_STEPPED_RETURN(M)           FMOD_STEPPED_BASE_RETURN_BODY(,, M)

#define FPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FPoint3_6_0*>(data_ptr)-> C); 
#define FPOINT_RETURN_REF(T, M)    FPOINT_BASE_RETURN_BODY(T, &, M)
#define FPOINT_RETURN_AS(T, M)     FPOINT_BASE_RETURN_BODY(T,, M)
#define FPOINT_RETURN(M)           FPOINT_BASE_RETURN_BODY(,, M)

#define FREESTYLECONFIG_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FreestyleConfig3_6_0*>(data_ptr)-> C); 
#define FREESTYLECONFIG_RETURN_REF(T, M)    FREESTYLECONFIG_BASE_RETURN_BODY(T, &, M)
#define FREESTYLECONFIG_RETURN_AS(T, M)     FREESTYLECONFIG_BASE_RETURN_BODY(T,, M)
#define FREESTYLECONFIG_RETURN(M)           FREESTYLECONFIG_BASE_RETURN_BODY(,, M)

#define FREESTYLEEDGE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FreestyleEdge3_6_0*>(data_ptr)-> C); 
#define FREESTYLEEDGE_RETURN_REF(T, M)    FREESTYLEEDGE_BASE_RETURN_BODY(T, &, M)
#define FREESTYLEEDGE_RETURN_AS(T, M)     FREESTYLEEDGE_BASE_RETURN_BODY(T,, M)
#define FREESTYLEEDGE_RETURN(M)           FREESTYLEEDGE_BASE_RETURN_BODY(,, M)

#define FREESTYLEFACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FreestyleFace3_6_0*>(data_ptr)-> C); 
#define FREESTYLEFACE_RETURN_REF(T, M)    FREESTYLEFACE_BASE_RETURN_BODY(T, &, M)
#define FREESTYLEFACE_RETURN_AS(T, M)     FREESTYLEFACE_BASE_RETURN_BODY(T,, M)
#define FREESTYLEFACE_RETURN(M)           FREESTYLEFACE_BASE_RETURN_BODY(,, M)

#define FREESTYLELINESET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FreestyleLineSet3_6_0*>(data_ptr)-> C); 
#define FREESTYLELINESET_RETURN_REF(T, M)    FREESTYLELINESET_BASE_RETURN_BODY(T, &, M)
#define FREESTYLELINESET_RETURN_AS(T, M)     FREESTYLELINESET_BASE_RETURN_BODY(T,, M)
#define FREESTYLELINESET_RETURN(M)           FREESTYLELINESET_BASE_RETURN_BODY(,, M)

#define FREESTYLELINESTYLE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<FreestyleLineStyle3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<FreestyleLineStyle4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<FreestyleLineStyle4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<FreestyleLineStyle4_4_0*>(data_ptr)-> C); 
#define FREESTYLELINESTYLE_RETURN_REF(T, M)    FREESTYLELINESTYLE_BASE_RETURN_BODY(T, &, M)
#define FREESTYLELINESTYLE_RETURN_AS(T, M)     FREESTYLELINESTYLE_BASE_RETURN_BODY(T,, M)
#define FREESTYLELINESTYLE_RETURN(M)           FREESTYLELINESTYLE_BASE_RETURN_BODY(,, M)

#define FREESTYLEMODULECONFIG_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<FreestyleModuleConfig3_6_0*>(data_ptr)-> C); 
#define FREESTYLEMODULECONFIG_RETURN_REF(T, M)    FREESTYLEMODULECONFIG_BASE_RETURN_BODY(T, &, M)
#define FREESTYLEMODULECONFIG_RETURN_AS(T, M)     FREESTYLEMODULECONFIG_BASE_RETURN_BODY(T,, M)
#define FREESTYLEMODULECONFIG_RETURN(M)           FREESTYLEMODULECONFIG_BASE_RETURN_BODY(,, M)

#define GAUSSIANBLURVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GaussianBlurVars3_6_0*>(data_ptr)-> C); 
#define GAUSSIANBLURVARS_RETURN_REF(T, M)    GAUSSIANBLURVARS_BASE_RETURN_BODY(T, &, M)
#define GAUSSIANBLURVARS_RETURN_AS(T, M)     GAUSSIANBLURVARS_BASE_RETURN_BODY(T,, M)
#define GAUSSIANBLURVARS_RETURN(M)           GAUSSIANBLURVARS_BASE_RETURN_BODY(,, M)

#define GLOWSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GlowShaderFxData3_6_0*>(data_ptr)-> C); 
#define GLOWSHADERFXDATA_RETURN_REF(T, M)    GLOWSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define GLOWSHADERFXDATA_RETURN_AS(T, M)     GLOWSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define GLOWSHADERFXDATA_RETURN(M)           GLOWSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define GLOWVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GlowVars3_6_0*>(data_ptr)-> C); 
#define GLOWVARS_RETURN_REF(T, M)    GLOWVARS_BASE_RETURN_BODY(T, &, M)
#define GLOWVARS_RETURN_AS(T, M)     GLOWVARS_BASE_RETURN_BODY(T,, M)
#define GLOWVARS_RETURN(M)           GLOWVARS_BASE_RETURN_BODY(,, M)

#define GPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GpencilModifierData3_6_0*>(data_ptr)-> C); 
#define GPENCILMODIFIERDATA_RETURN_REF(T, M)    GPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define GPENCILMODIFIERDATA_RETURN_AS(T, M)     GPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define GPENCILMODIFIERDATA_RETURN(M)           GPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define GPPAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<GpPaint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<GpPaint4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<GpPaint4_3_0*>(data_ptr)-> C); 
#define GPPAINT_RETURN_REF(T, M)    GPPAINT_BASE_RETURN_BODY(T, &, M)
#define GPPAINT_RETURN_AS(T, M)     GPPAINT_BASE_RETURN_BODY(T,, M)
#define GPPAINT_RETURN(M)           GPPAINT_BASE_RETURN_BODY(,, M)

#define GPSCULPTPAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<GpSculptPaint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<GpSculptPaint4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<GpSculptPaint4_3_0*>(data_ptr)-> C); 
#define GPSCULPTPAINT_RETURN_REF(T, M)    GPSCULPTPAINT_BASE_RETURN_BODY(T, &, M)
#define GPSCULPTPAINT_RETURN_AS(T, M)     GPSCULPTPAINT_BASE_RETURN_BODY(T,, M)
#define GPSCULPTPAINT_RETURN(M)           GPSCULPTPAINT_BASE_RETURN_BODY(,, M)

#define GPUDOFSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GPUDOFSettings3_6_0*>(data_ptr)-> C); 
#define GPUDOFSETTINGS_RETURN_REF(T, M)    GPUDOFSETTINGS_BASE_RETURN_BODY(T, &, M)
#define GPUDOFSETTINGS_RETURN_AS(T, M)     GPUDOFSETTINGS_BASE_RETURN_BODY(T,, M)
#define GPUDOFSETTINGS_RETURN(M)           GPUDOFSETTINGS_BASE_RETURN_BODY(,, M)

#define GPVERTEXPAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<GpVertexPaint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<GpVertexPaint4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<GpVertexPaint4_3_0*>(data_ptr)-> C); 
#define GPVERTEXPAINT_RETURN_REF(T, M)    GPVERTEXPAINT_BASE_RETURN_BODY(T, &, M)
#define GPVERTEXPAINT_RETURN_AS(T, M)     GPVERTEXPAINT_BASE_RETURN_BODY(T,, M)
#define GPVERTEXPAINT_RETURN(M)           GPVERTEXPAINT_BASE_RETURN_BODY(,, M)

#define GPWEIGHTPAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<GpWeightPaint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<GpWeightPaint4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<GpWeightPaint4_3_0*>(data_ptr)-> C); 
#define GPWEIGHTPAINT_RETURN_REF(T, M)    GPWEIGHTPAINT_BASE_RETURN_BODY(T, &, M)
#define GPWEIGHTPAINT_RETURN_AS(T, M)     GPWEIGHTPAINT_BASE_RETURN_BODY(T,, M)
#define GPWEIGHTPAINT_RETURN(M)           GPWEIGHTPAINT_BASE_RETURN_BODY(,, M)

#define GP_INTERPOLATE_SETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GP_Interpolate_Settings3_6_0*>(data_ptr)-> C); 
#define GP_INTERPOLATE_SETTINGS_RETURN_REF(T, M)    GP_INTERPOLATE_SETTINGS_BASE_RETURN_BODY(T, &, M)
#define GP_INTERPOLATE_SETTINGS_RETURN_AS(T, M)     GP_INTERPOLATE_SETTINGS_BASE_RETURN_BODY(T,, M)
#define GP_INTERPOLATE_SETTINGS_RETURN(M)           GP_INTERPOLATE_SETTINGS_BASE_RETURN_BODY(,, M)

#define GP_SCULPT_GUIDE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Guide3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Guide4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Guide4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Guide4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Guide4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<GP_Sculpt_Guide4_4_0*>(data_ptr)-> C); 
#define GP_SCULPT_GUIDE_RETURN_REF(T, M)    GP_SCULPT_GUIDE_BASE_RETURN_BODY(T, &, M)
#define GP_SCULPT_GUIDE_RETURN_AS(T, M)     GP_SCULPT_GUIDE_BASE_RETURN_BODY(T,, M)
#define GP_SCULPT_GUIDE_RETURN(M)           GP_SCULPT_GUIDE_BASE_RETURN_BODY(,, M)

#define GP_SCULPT_SETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Settings3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Settings4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Settings4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Settings4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<GP_Sculpt_Settings4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<GP_Sculpt_Settings4_4_0*>(data_ptr)-> C); 
#define GP_SCULPT_SETTINGS_RETURN_REF(T, M)    GP_SCULPT_SETTINGS_BASE_RETURN_BODY(T, &, M)
#define GP_SCULPT_SETTINGS_RETURN_AS(T, M)     GP_SCULPT_SETTINGS_BASE_RETURN_BODY(T,, M)
#define GP_SCULPT_SETTINGS_RETURN(M)           GP_SCULPT_SETTINGS_BASE_RETURN_BODY(,, M)

#define GRIDPAINTMASK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<GridPaintMask3_6_0*>(data_ptr)-> C); 
#define GRIDPAINTMASK_RETURN_REF(T, M)    GRIDPAINTMASK_BASE_RETURN_BODY(T, &, M)
#define GRIDPAINTMASK_RETURN_AS(T, M)     GRIDPAINTMASK_BASE_RETURN_BODY(T,, M)
#define GRIDPAINTMASK_RETURN(M)           GRIDPAINTMASK_BASE_RETURN_BODY(,, M)

#define HAIRKEY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<HairKey3_6_0*>(data_ptr)-> C); 
#define HAIRKEY_RETURN_REF(T, M)    HAIRKEY_BASE_RETURN_BODY(T, &, M)
#define HAIRKEY_RETURN_AS(T, M)     HAIRKEY_BASE_RETURN_BODY(T,, M)
#define HAIRKEY_RETURN(M)           HAIRKEY_BASE_RETURN_BODY(,, M)

#define HISTOGRAM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Histogram3_6_0*>(data_ptr)-> C); 
#define HISTOGRAM_RETURN_REF(T, M)    HISTOGRAM_BASE_RETURN_BODY(T, &, M)
#define HISTOGRAM_RETURN_AS(T, M)     HISTOGRAM_BASE_RETURN_BODY(T,, M)
#define HISTOGRAM_RETURN(M)           HISTOGRAM_BASE_RETURN_BODY(,, M)

#define HOOKGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<HookGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define HOOKGPENCILMODIFIERDATA_RETURN_REF(T, M)    HOOKGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define HOOKGPENCILMODIFIERDATA_RETURN_AS(T, M)     HOOKGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define HOOKGPENCILMODIFIERDATA_RETURN(M)           HOOKGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define HOOKMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<HookModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<HookModifierData4_1_0*>(data_ptr)-> C); 
#define HOOKMODIFIERDATA_RETURN_REF(T, M)    HOOKMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define HOOKMODIFIERDATA_RETURN_AS(T, M)     HOOKMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define HOOKMODIFIERDATA_RETURN(M)           HOOKMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define IDADTTEMPLATE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<IdAdtTemplate3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<IdAdtTemplate4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<IdAdtTemplate4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<IdAdtTemplate4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<IdAdtTemplate4_4_0*>(data_ptr)-> C); 
#define IDADTTEMPLATE_RETURN_REF(T, M)    IDADTTEMPLATE_BASE_RETURN_BODY(T, &, M)
#define IDADTTEMPLATE_RETURN_AS(T, M)     IDADTTEMPLATE_BASE_RETURN_BODY(T,, M)
#define IDADTTEMPLATE_RETURN(M)           IDADTTEMPLATE_BASE_RETURN_BODY(,, M)

#define IDOVERRIDELIBRARYPROPERTYOPERATION_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<IDOverrideLibraryPropertyOperation3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<IDOverrideLibraryPropertyOperation4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<IDOverrideLibraryPropertyOperation4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<IDOverrideLibraryPropertyOperation4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<IDOverrideLibraryPropertyOperation4_4_0*>(data_ptr)-> C); 
#define IDOVERRIDELIBRARYPROPERTYOPERATION_RETURN_REF(T, M)    IDOVERRIDELIBRARYPROPERTYOPERATION_BASE_RETURN_BODY(T, &, M)
#define IDOVERRIDELIBRARYPROPERTYOPERATION_RETURN_AS(T, M)     IDOVERRIDELIBRARYPROPERTYOPERATION_BASE_RETURN_BODY(T,, M)
#define IDOVERRIDELIBRARYPROPERTYOPERATION_RETURN(M)           IDOVERRIDELIBRARYPROPERTYOPERATION_BASE_RETURN_BODY(,, M)

#define IDOVERRIDELIBRARYPROPERTY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDOverrideLibraryProperty3_6_0*>(data_ptr)-> C); 
#define IDOVERRIDELIBRARYPROPERTY_RETURN_REF(T, M)    IDOVERRIDELIBRARYPROPERTY_BASE_RETURN_BODY(T, &, M)
#define IDOVERRIDELIBRARYPROPERTY_RETURN_AS(T, M)     IDOVERRIDELIBRARYPROPERTY_BASE_RETURN_BODY(T,, M)
#define IDOVERRIDELIBRARYPROPERTY_RETURN(M)           IDOVERRIDELIBRARYPROPERTY_BASE_RETURN_BODY(,, M)

#define IDOVERRIDELIBRARYRUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDOverrideLibraryRuntime3_6_0*>(data_ptr)-> C); 
#define IDOVERRIDELIBRARYRUNTIME_RETURN_REF(T, M)    IDOVERRIDELIBRARYRUNTIME_BASE_RETURN_BODY(T, &, M)
#define IDOVERRIDELIBRARYRUNTIME_RETURN_AS(T, M)     IDOVERRIDELIBRARYRUNTIME_BASE_RETURN_BODY(T,, M)
#define IDOVERRIDELIBRARYRUNTIME_RETURN(M)           IDOVERRIDELIBRARYRUNTIME_BASE_RETURN_BODY(,, M)

#define IDOVERRIDELIBRARY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<IDOverrideLibrary3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<IDOverrideLibrary4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<IDOverrideLibrary4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<IDOverrideLibrary4_4_0*>(data_ptr)-> C); 
#define IDOVERRIDELIBRARY_RETURN_REF(T, M)    IDOVERRIDELIBRARY_BASE_RETURN_BODY(T, &, M)
#define IDOVERRIDELIBRARY_RETURN_AS(T, M)     IDOVERRIDELIBRARY_BASE_RETURN_BODY(T,, M)
#define IDOVERRIDELIBRARY_RETURN(M)           IDOVERRIDELIBRARY_BASE_RETURN_BODY(,, M)

#define IDPROPERTYDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDPropertyData3_6_0*>(data_ptr)-> C); 
#define IDPROPERTYDATA_RETURN_REF(T, M)    IDPROPERTYDATA_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYDATA_RETURN_AS(T, M)     IDPROPERTYDATA_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYDATA_RETURN(M)           IDPROPERTYDATA_BASE_RETURN_BODY(,, M)

#define IDPROPERTYUIDATABOOL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDPropertyUIDataBool3_6_0*>(data_ptr)-> C); 
#define IDPROPERTYUIDATABOOL_RETURN_REF(T, M)    IDPROPERTYUIDATABOOL_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYUIDATABOOL_RETURN_AS(T, M)     IDPROPERTYUIDATABOOL_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYUIDATABOOL_RETURN(M)           IDPROPERTYUIDATABOOL_BASE_RETURN_BODY(,, M)

#define IDPROPERTYUIDATAFLOAT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDPropertyUIDataFloat3_6_0*>(data_ptr)-> C); 
#define IDPROPERTYUIDATAFLOAT_RETURN_REF(T, M)    IDPROPERTYUIDATAFLOAT_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYUIDATAFLOAT_RETURN_AS(T, M)     IDPROPERTYUIDATAFLOAT_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYUIDATAFLOAT_RETURN(M)           IDPROPERTYUIDATAFLOAT_BASE_RETURN_BODY(,, M)

#define IDPROPERTYUIDATAID_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDPropertyUIDataID3_6_0*>(data_ptr)-> C); 
#define IDPROPERTYUIDATAID_RETURN_REF(T, M)    IDPROPERTYUIDATAID_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYUIDATAID_RETURN_AS(T, M)     IDPROPERTYUIDATAID_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYUIDATAID_RETURN(M)           IDPROPERTYUIDATAID_BASE_RETURN_BODY(,, M)

#define IDPROPERTYUIDATAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<IDPropertyUIDataInt3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<IDPropertyUIDataInt4_1_0*>(data_ptr)-> C); 
#define IDPROPERTYUIDATAINT_RETURN_REF(T, M)    IDPROPERTYUIDATAINT_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYUIDATAINT_RETURN_AS(T, M)     IDPROPERTYUIDATAINT_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYUIDATAINT_RETURN(M)           IDPROPERTYUIDATAINT_BASE_RETURN_BODY(,, M)

#define IDPROPERTYUIDATASTRING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDPropertyUIDataString3_6_0*>(data_ptr)-> C); 
#define IDPROPERTYUIDATASTRING_RETURN_REF(T, M)    IDPROPERTYUIDATASTRING_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYUIDATASTRING_RETURN_AS(T, M)     IDPROPERTYUIDATASTRING_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYUIDATASTRING_RETURN(M)           IDPROPERTYUIDATASTRING_BASE_RETURN_BODY(,, M)

#define IDPROPERTYUIDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDPropertyUIData3_6_0*>(data_ptr)-> C); 
#define IDPROPERTYUIDATA_RETURN_REF(T, M)    IDPROPERTYUIDATA_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTYUIDATA_RETURN_AS(T, M)     IDPROPERTYUIDATA_BASE_RETURN_BODY(T,, M)
#define IDPROPERTYUIDATA_RETURN(M)           IDPROPERTYUIDATA_BASE_RETURN_BODY(,, M)

#define IDPROPERTY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IDProperty3_6_0*>(data_ptr)-> C); 
#define IDPROPERTY_RETURN_REF(T, M)    IDPROPERTY_BASE_RETURN_BODY(T, &, M)
#define IDPROPERTY_RETURN_AS(T, M)     IDPROPERTY_BASE_RETURN_BODY(T,, M)
#define IDPROPERTY_RETURN(M)           IDPROPERTY_BASE_RETURN_BODY(,, M)

#define IDVIEWERPATHELEM_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<IDViewerPathElem3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<IDViewerPathElem4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<IDViewerPathElem4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<IDViewerPathElem4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<IDViewerPathElem4_4_0*>(data_ptr)-> C); 
#define IDVIEWERPATHELEM_RETURN_REF(T, M)    IDVIEWERPATHELEM_BASE_RETURN_BODY(T, &, M)
#define IDVIEWERPATHELEM_RETURN_AS(T, M)     IDVIEWERPATHELEM_BASE_RETURN_BODY(T,, M)
#define IDVIEWERPATHELEM_RETURN(M)           IDVIEWERPATHELEM_BASE_RETURN_BODY(,, M)

#define ID_BASE_RETURN_BODY(A, B, C) \
    if (get_compatability_mode() < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ID3_6_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ID4_1_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ID4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ID4_4_0*>(data_ptr)-> C); 
#define ID_RETURN_REF(T, M)    ID_BASE_RETURN_BODY(T, &, M)
#define ID_RETURN_AS(T, M)     ID_BASE_RETURN_BODY(T,, M)
#define ID_RETURN(M)           ID_BASE_RETURN_BODY(,, M)

#define ID_BASE_RETURN_BODY_LEGACY(A, B, C) \
    return A ( B reinterpret_cast<ID3_6_0*>(data_ptr)-> C);
#define ID_RETURN_REF_LEGACY(T, M)    ID_BASE_RETURN_BODY_LEGACY(T, &, M)
#define ID_RETURN_AS_LEGACY(T, M)     ID_BASE_RETURN_BODY_LEGACY(T,, M)
#define ID_RETURN_LEGACY(M)           ID_BASE_RETURN_BODY_LEGACY(,, M)

#define ID_BASE_RETURN_BODY_NEW(A, B, C) \
    if (get_compatability_mode() < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ID4_1_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ID4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ID4_4_0*>(data_ptr)-> C);
#define ID_RETURN_REF_NEW(T, M)    ID_BASE_RETURN_BODY_NEW(T, &, M)
#define ID_RETURN_AS_NEW(T, M)     ID_BASE_RETURN_BODY_NEW(T,, M)
#define ID_RETURN_NEW(M)           ID_BASE_RETURN_BODY_NEW(,, M)

#define ID_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ID_Runtime3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ID_Runtime4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ID_Runtime4_4_0*>(data_ptr)-> C); 
#define ID_RUNTIME_RETURN_REF(T, M)    ID_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define ID_RUNTIME_RETURN_AS(T, M)     ID_RUNTIME_BASE_RETURN_BODY(T,, M)
#define ID_RUNTIME_RETURN(M)           ID_RUNTIME_BASE_RETURN_BODY(,, M)

#define ID_RUNTIME_REMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ID_Runtime_Remap3_6_0*>(data_ptr)-> C); 
#define ID_RUNTIME_REMAP_RETURN_REF(T, M)    ID_RUNTIME_REMAP_BASE_RETURN_BODY(T, &, M)
#define ID_RUNTIME_REMAP_RETURN_AS(T, M)     ID_RUNTIME_REMAP_BASE_RETURN_BODY(T,, M)
#define ID_RUNTIME_REMAP_RETURN(M)           ID_RUNTIME_REMAP_BASE_RETURN_BODY(,, M)

#define IMAGEANIM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ImageAnim3_6_0*>(data_ptr)-> C); 
#define IMAGEANIM_RETURN_REF(T, M)    IMAGEANIM_BASE_RETURN_BODY(T, &, M)
#define IMAGEANIM_RETURN_AS(T, M)     IMAGEANIM_BASE_RETURN_BODY(T,, M)
#define IMAGEANIM_RETURN(M)           IMAGEANIM_BASE_RETURN_BODY(,, M)

#define IMAGEFORMATDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ImageFormatData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ImageFormatData4_3_0*>(data_ptr)-> C); 
#define IMAGEFORMATDATA_RETURN_REF(T, M)    IMAGEFORMATDATA_BASE_RETURN_BODY(T, &, M)
#define IMAGEFORMATDATA_RETURN_AS(T, M)     IMAGEFORMATDATA_BASE_RETURN_BODY(T,, M)
#define IMAGEFORMATDATA_RETURN(M)           IMAGEFORMATDATA_BASE_RETURN_BODY(,, M)

#define IMAGEPACKEDFILE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ImagePackedFile3_6_0*>(data_ptr)-> C); 
#define IMAGEPACKEDFILE_RETURN_REF(T, M)    IMAGEPACKEDFILE_BASE_RETURN_BODY(T, &, M)
#define IMAGEPACKEDFILE_RETURN_AS(T, M)     IMAGEPACKEDFILE_BASE_RETURN_BODY(T,, M)
#define IMAGEPACKEDFILE_RETURN(M)           IMAGEPACKEDFILE_BASE_RETURN_BODY(,, M)

#define IMAGEPAINTSETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ImagePaintSettings3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ImagePaintSettings4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ImagePaintSettings4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ImagePaintSettings4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ImagePaintSettings4_4_0*>(data_ptr)-> C); 
#define IMAGEPAINTSETTINGS_RETURN_REF(T, M)    IMAGEPAINTSETTINGS_BASE_RETURN_BODY(T, &, M)
#define IMAGEPAINTSETTINGS_RETURN_AS(T, M)     IMAGEPAINTSETTINGS_BASE_RETURN_BODY(T,, M)
#define IMAGEPAINTSETTINGS_RETURN(M)           IMAGEPAINTSETTINGS_BASE_RETURN_BODY(,, M)

#define IMAGETILE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ImageTile3_6_0*>(data_ptr)-> C); 
#define IMAGETILE_RETURN_REF(T, M)    IMAGETILE_BASE_RETURN_BODY(T, &, M)
#define IMAGETILE_RETURN_AS(T, M)     IMAGETILE_BASE_RETURN_BODY(T,, M)
#define IMAGETILE_RETURN(M)           IMAGETILE_BASE_RETURN_BODY(,, M)

#define IMAGETILE_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ImageTile_Runtime3_6_0*>(data_ptr)-> C); 
#define IMAGETILE_RUNTIME_RETURN_REF(T, M)    IMAGETILE_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define IMAGETILE_RUNTIME_RETURN_AS(T, M)     IMAGETILE_RUNTIME_BASE_RETURN_BODY(T,, M)
#define IMAGETILE_RUNTIME_RETURN(M)           IMAGETILE_RUNTIME_BASE_RETURN_BODY(,, M)

#define IMAGEUSER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ImageUser3_6_0*>(data_ptr)-> C); 
#define IMAGEUSER_RETURN_REF(T, M)    IMAGEUSER_BASE_RETURN_BODY(T, &, M)
#define IMAGEUSER_RETURN_AS(T, M)     IMAGEUSER_BASE_RETURN_BODY(T,, M)
#define IMAGEUSER_RETURN(M)           IMAGEUSER_BASE_RETURN_BODY(,, M)

#define IMAGEVIEW_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ImageView3_6_0*>(data_ptr)-> C); 
#define IMAGEVIEW_RETURN_REF(T, M)    IMAGEVIEW_BASE_RETURN_BODY(T, &, M)
#define IMAGEVIEW_RETURN_AS(T, M)     IMAGEVIEW_BASE_RETURN_BODY(T,, M)
#define IMAGEVIEW_RETURN(M)           IMAGEVIEW_BASE_RETURN_BODY(,, M)

#define IMAGE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Image3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Image4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Image4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Image4_4_0*>(data_ptr)-> C); 
#define IMAGE_RETURN_REF(T, M)    IMAGE_BASE_RETURN_BODY(T, &, M)
#define IMAGE_RETURN_AS(T, M)     IMAGE_BASE_RETURN_BODY(T,, M)
#define IMAGE_RETURN(M)           IMAGE_BASE_RETURN_BODY(,, M)

#define IMAGE_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Image_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Image_Runtime4_2_0*>(data_ptr)-> C); 
#define IMAGE_RUNTIME_RETURN_REF(T, M)    IMAGE_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define IMAGE_RUNTIME_RETURN_AS(T, M)     IMAGE_RUNTIME_BASE_RETURN_BODY(T,, M)
#define IMAGE_RUNTIME_RETURN(M)           IMAGE_RUNTIME_BASE_RETURN_BODY(,, M)

#define IPOCURVE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IpoCurve3_6_0*>(data_ptr)-> C); 
#define IPOCURVE_RETURN_REF(T, M)    IPOCURVE_BASE_RETURN_BODY(T, &, M)
#define IPOCURVE_RETURN_AS(T, M)     IPOCURVE_BASE_RETURN_BODY(T,, M)
#define IPOCURVE_RETURN(M)           IPOCURVE_BASE_RETURN_BODY(,, M)

#define IPODRIVER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<IpoDriver3_6_0*>(data_ptr)-> C); 
#define IPODRIVER_RETURN_REF(T, M)    IPODRIVER_BASE_RETURN_BODY(T, &, M)
#define IPODRIVER_RETURN_AS(T, M)     IPODRIVER_BASE_RETURN_BODY(T,, M)
#define IPODRIVER_RETURN(M)           IPODRIVER_BASE_RETURN_BODY(,, M)

#define IPO_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Ipo3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Ipo4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Ipo4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Ipo4_4_0*>(data_ptr)-> C); 
#define IPO_RETURN_REF(T, M)    IPO_BASE_RETURN_BODY(T, &, M)
#define IPO_RETURN_AS(T, M)     IPO_BASE_RETURN_BODY(T,, M)
#define IPO_RETURN(M)           IPO_BASE_RETURN_BODY(,, M)

#define KEYBLOCK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<KeyBlock3_6_0*>(data_ptr)-> C); 
#define KEYBLOCK_RETURN_REF(T, M)    KEYBLOCK_BASE_RETURN_BODY(T, &, M)
#define KEYBLOCK_RETURN_AS(T, M)     KEYBLOCK_BASE_RETURN_BODY(T,, M)
#define KEYBLOCK_RETURN(M)           KEYBLOCK_BASE_RETURN_BODY(,, M)

#define KEYINGSET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<KeyingSet3_6_0*>(data_ptr)-> C); 
#define KEYINGSET_RETURN_REF(T, M)    KEYINGSET_BASE_RETURN_BODY(T, &, M)
#define KEYINGSET_RETURN_AS(T, M)     KEYINGSET_BASE_RETURN_BODY(T,, M)
#define KEYINGSET_RETURN(M)           KEYINGSET_BASE_RETURN_BODY(,, M)

#define KEY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Key3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Key4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Key4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Key4_4_0*>(data_ptr)-> C); 
#define KEY_RETURN_REF(T, M)    KEY_BASE_RETURN_BODY(T, &, M)
#define KEY_RETURN_AS(T, M)     KEY_BASE_RETURN_BODY(T,, M)
#define KEY_RETURN(M)           KEY_BASE_RETURN_BODY(,, M)

#define KS_PATH_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<KS_Path3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<KS_Path4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<KS_Path4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<KS_Path4_4_0*>(data_ptr)-> C); 
#define KS_PATH_RETURN_REF(T, M)    KS_PATH_BASE_RETURN_BODY(T, &, M)
#define KS_PATH_RETURN_AS(T, M)     KS_PATH_BASE_RETURN_BODY(T,, M)
#define KS_PATH_RETURN(M)           KS_PATH_BASE_RETURN_BODY(,, M)

#define LAPLACIANDEFORMMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<LaplacianDeformModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LaplacianDeformModifierData4_1_0*>(data_ptr)-> C); 
#define LAPLACIANDEFORMMODIFIERDATA_RETURN_REF(T, M)    LAPLACIANDEFORMMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define LAPLACIANDEFORMMODIFIERDATA_RETURN_AS(T, M)     LAPLACIANDEFORMMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define LAPLACIANDEFORMMODIFIERDATA_RETURN(M)           LAPLACIANDEFORMMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define LAPLACIANSMOOTHMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<LaplacianSmoothModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LaplacianSmoothModifierData4_1_0*>(data_ptr)-> C); 
#define LAPLACIANSMOOTHMODIFIERDATA_RETURN_REF(T, M)    LAPLACIANSMOOTHMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define LAPLACIANSMOOTHMODIFIERDATA_RETURN_AS(T, M)     LAPLACIANSMOOTHMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define LAPLACIANSMOOTHMODIFIERDATA_RETURN(M)           LAPLACIANSMOOTHMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define LATTICEGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LatticeGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define LATTICEGPENCILMODIFIERDATA_RETURN_REF(T, M)    LATTICEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define LATTICEGPENCILMODIFIERDATA_RETURN_AS(T, M)     LATTICEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define LATTICEGPENCILMODIFIERDATA_RETURN(M)           LATTICEGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define LATTICEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<LatticeModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LatticeModifierData4_1_0*>(data_ptr)-> C); 
#define LATTICEMODIFIERDATA_RETURN_REF(T, M)    LATTICEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define LATTICEMODIFIERDATA_RETURN_AS(T, M)     LATTICEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define LATTICEMODIFIERDATA_RETURN(M)           LATTICEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define LATTICE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Lattice3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Lattice4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Lattice4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Lattice4_4_0*>(data_ptr)-> C); 
#define LATTICE_RETURN_REF(T, M)    LATTICE_BASE_RETURN_BODY(T, &, M)
#define LATTICE_RETURN_AS(T, M)     LATTICE_BASE_RETURN_BODY(T,, M)
#define LATTICE_RETURN(M)           LATTICE_BASE_RETURN_BODY(,, M)

#define LAYERCOLLECTION_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<LayerCollection3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LayerCollection4_0_0*>(data_ptr)-> C); 
#define LAYERCOLLECTION_RETURN_REF(T, M)    LAYERCOLLECTION_BASE_RETURN_BODY(T, &, M)
#define LAYERCOLLECTION_RETURN_AS(T, M)     LAYERCOLLECTION_BASE_RETURN_BODY(T,, M)
#define LAYERCOLLECTION_RETURN(M)           LAYERCOLLECTION_BASE_RETURN_BODY(,, M)

#define LENGTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LengthGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define LENGTHGPENCILMODIFIERDATA_RETURN_REF(T, M)    LENGTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define LENGTHGPENCILMODIFIERDATA_RETURN_AS(T, M)     LENGTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define LENGTHGPENCILMODIFIERDATA_RETURN(M)           LENGTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define LIBRARYWEAKREFERENCE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LibraryWeakReference3_6_0*>(data_ptr)-> C); 
#define LIBRARYWEAKREFERENCE_RETURN_REF(T, M)    LIBRARYWEAKREFERENCE_BASE_RETURN_BODY(T, &, M)
#define LIBRARYWEAKREFERENCE_RETURN_AS(T, M)     LIBRARYWEAKREFERENCE_BASE_RETURN_BODY(T,, M)
#define LIBRARYWEAKREFERENCE_RETURN(M)           LIBRARYWEAKREFERENCE_BASE_RETURN_BODY(,, M)

#define LIBRARY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Library3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Library4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Library4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Library4_4_0*>(data_ptr)-> C); 
#define LIBRARY_RETURN_REF(T, M)    LIBRARY_BASE_RETURN_BODY(T, &, M)
#define LIBRARY_RETURN_AS(T, M)     LIBRARY_BASE_RETURN_BODY(T,, M)
#define LIBRARY_RETURN(M)           LIBRARY_BASE_RETURN_BODY(,, M)

#define LIBRARY_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Library_Runtime3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Library_Runtime4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Library_Runtime4_4_0*>(data_ptr)-> C); 
#define LIBRARY_RUNTIME_RETURN_REF(T, M)    LIBRARY_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define LIBRARY_RUNTIME_RETURN_AS(T, M)     LIBRARY_RUNTIME_BASE_RETURN_BODY(T,, M)
#define LIBRARY_RUNTIME_RETURN(M)           LIBRARY_RUNTIME_BASE_RETURN_BODY(,, M)

#define LIGHTCACHE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightCache3_6_0*>(data_ptr)-> C); 
#define LIGHTCACHE_RETURN_REF(T, M)    LIGHTCACHE_BASE_RETURN_BODY(T, &, M)
#define LIGHTCACHE_RETURN_AS(T, M)     LIGHTCACHE_BASE_RETURN_BODY(T,, M)
#define LIGHTCACHE_RETURN(M)           LIGHTCACHE_BASE_RETURN_BODY(,, M)

#define LIGHTGRIDCACHE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightGridCache3_6_0*>(data_ptr)-> C); 
#define LIGHTGRIDCACHE_RETURN_REF(T, M)    LIGHTGRIDCACHE_BASE_RETURN_BODY(T, &, M)
#define LIGHTGRIDCACHE_RETURN_AS(T, M)     LIGHTGRIDCACHE_BASE_RETURN_BODY(T,, M)
#define LIGHTGRIDCACHE_RETURN(M)           LIGHTGRIDCACHE_BASE_RETURN_BODY(,, M)

#define LIGHTGROUPMEMBERSHIP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightgroupMembership3_6_0*>(data_ptr)-> C); 
#define LIGHTGROUPMEMBERSHIP_RETURN_REF(T, M)    LIGHTGROUPMEMBERSHIP_BASE_RETURN_BODY(T, &, M)
#define LIGHTGROUPMEMBERSHIP_RETURN_AS(T, M)     LIGHTGROUPMEMBERSHIP_BASE_RETURN_BODY(T,, M)
#define LIGHTGROUPMEMBERSHIP_RETURN(M)           LIGHTGROUPMEMBERSHIP_BASE_RETURN_BODY(,, M)

#define LIGHTPROBEBAKINGDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<LightProbeBakingData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LightProbeBakingData4_0_0*>(data_ptr)-> C); 
#define LIGHTPROBEBAKINGDATA_RETURN_REF(T, M)    LIGHTPROBEBAKINGDATA_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBEBAKINGDATA_RETURN_AS(T, M)     LIGHTPROBEBAKINGDATA_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBEBAKINGDATA_RETURN(M)           LIGHTPROBEBAKINGDATA_BASE_RETURN_BODY(,, M)

#define LIGHTPROBEBLOCKDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightProbeBlockData3_6_0*>(data_ptr)-> C); 
#define LIGHTPROBEBLOCKDATA_RETURN_REF(T, M)    LIGHTPROBEBLOCKDATA_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBEBLOCKDATA_RETURN_AS(T, M)     LIGHTPROBEBLOCKDATA_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBEBLOCKDATA_RETURN(M)           LIGHTPROBEBLOCKDATA_BASE_RETURN_BODY(,, M)

#define LIGHTPROBECACHE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightProbeCache3_6_0*>(data_ptr)-> C); 
#define LIGHTPROBECACHE_RETURN_REF(T, M)    LIGHTPROBECACHE_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBECACHE_RETURN_AS(T, M)     LIGHTPROBECACHE_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBECACHE_RETURN(M)           LIGHTPROBECACHE_BASE_RETURN_BODY(,, M)

#define LIGHTPROBECONNECTIVITYDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<LightProbeConnectivityData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LightProbeConnectivityData4_0_0*>(data_ptr)-> C); 
#define LIGHTPROBECONNECTIVITYDATA_RETURN_REF(T, M)    LIGHTPROBECONNECTIVITYDATA_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBECONNECTIVITYDATA_RETURN_AS(T, M)     LIGHTPROBECONNECTIVITYDATA_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBECONNECTIVITYDATA_RETURN(M)           LIGHTPROBECONNECTIVITYDATA_BASE_RETURN_BODY(,, M)

#define LIGHTPROBEGRIDCACHEFRAME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<LightProbeGridCacheFrame3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LightProbeGridCacheFrame4_0_0*>(data_ptr)-> C); 
#define LIGHTPROBEGRIDCACHEFRAME_RETURN_REF(T, M)    LIGHTPROBEGRIDCACHEFRAME_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBEGRIDCACHEFRAME_RETURN_AS(T, M)     LIGHTPROBEGRIDCACHEFRAME_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBEGRIDCACHEFRAME_RETURN(M)           LIGHTPROBEGRIDCACHEFRAME_BASE_RETURN_BODY(,, M)

#define LIGHTPROBEIRRADIANCEDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightProbeIrradianceData3_6_0*>(data_ptr)-> C); 
#define LIGHTPROBEIRRADIANCEDATA_RETURN_REF(T, M)    LIGHTPROBEIRRADIANCEDATA_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBEIRRADIANCEDATA_RETURN_AS(T, M)     LIGHTPROBEIRRADIANCEDATA_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBEIRRADIANCEDATA_RETURN(M)           LIGHTPROBEIRRADIANCEDATA_BASE_RETURN_BODY(,, M)

#define LIGHTPROBEOBJECTCACHE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<LightProbeObjectCache3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LightProbeObjectCache4_0_0*>(data_ptr)-> C); 
#define LIGHTPROBEOBJECTCACHE_RETURN_REF(T, M)    LIGHTPROBEOBJECTCACHE_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBEOBJECTCACHE_RETURN_AS(T, M)     LIGHTPROBEOBJECTCACHE_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBEOBJECTCACHE_RETURN(M)           LIGHTPROBEOBJECTCACHE_BASE_RETURN_BODY(,, M)

#define LIGHTPROBEVISIBILITYDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LightProbeVisibilityData3_6_0*>(data_ptr)-> C); 
#define LIGHTPROBEVISIBILITYDATA_RETURN_REF(T, M)    LIGHTPROBEVISIBILITYDATA_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBEVISIBILITYDATA_RETURN_AS(T, M)     LIGHTPROBEVISIBILITYDATA_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBEVISIBILITYDATA_RETURN(M)           LIGHTPROBEVISIBILITYDATA_BASE_RETURN_BODY(,, M)

#define LIGHTPROBE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<LightProbe3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<LightProbe4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<LightProbe4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<LightProbe4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<LightProbe4_4_0*>(data_ptr)-> C); 
#define LIGHTPROBE_RETURN_REF(T, M)    LIGHTPROBE_BASE_RETURN_BODY(T, &, M)
#define LIGHTPROBE_RETURN_AS(T, M)     LIGHTPROBE_BASE_RETURN_BODY(T,, M)
#define LIGHTPROBE_RETURN(M)           LIGHTPROBE_BASE_RETURN_BODY(,, M)

#define LIGHT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Light3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Light4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Light4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Light4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Light4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Light4_4_0*>(data_ptr)-> C); 
#define LIGHT_RETURN_REF(T, M)    LIGHT_BASE_RETURN_BODY(T, &, M)
#define LIGHT_RETURN_AS(T, M)     LIGHT_BASE_RETURN_BODY(T,, M)
#define LIGHT_RETURN(M)           LIGHT_BASE_RETURN_BODY(,, M)

#define LINEARTGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineartGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define LINEARTGPENCILMODIFIERDATA_RETURN_REF(T, M)    LINEARTGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define LINEARTGPENCILMODIFIERDATA_RETURN_AS(T, M)     LINEARTGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define LINEARTGPENCILMODIFIERDATA_RETURN(M)           LINEARTGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_AlongStroke3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_ALONGSTROKE_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_ALONGSTROKE_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_ALONGSTROKE_RETURN(M)           LINESTYLEALPHAMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_CREASEANGLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_CreaseAngle3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_CREASEANGLE_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_CREASEANGLE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_CREASEANGLE_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_CREASEANGLE_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_CREASEANGLE_RETURN(M)           LINESTYLEALPHAMODIFIER_CREASEANGLE_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_Curvature_3D3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_CURVATURE_3D_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_CURVATURE_3D_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_CURVATURE_3D_RETURN(M)           LINESTYLEALPHAMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_DistanceFromCamera3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_RETURN(M)           LINESTYLEALPHAMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_DistanceFromObject3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_RETURN(M)           LINESTYLEALPHAMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_MATERIAL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_Material3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_MATERIAL_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_MATERIAL_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_MATERIAL_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_MATERIAL_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_MATERIAL_RETURN(M)           LINESTYLEALPHAMODIFIER_MATERIAL_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_NOISE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_Noise3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_NOISE_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_NOISE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_NOISE_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_NOISE_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_NOISE_RETURN(M)           LINESTYLEALPHAMODIFIER_NOISE_BASE_RETURN_BODY(,, M)

#define LINESTYLEALPHAMODIFIER_TANGENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleAlphaModifier_Tangent3_6_0*>(data_ptr)-> C); 
#define LINESTYLEALPHAMODIFIER_TANGENT_RETURN_REF(T, M)    LINESTYLEALPHAMODIFIER_TANGENT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEALPHAMODIFIER_TANGENT_RETURN_AS(T, M)     LINESTYLEALPHAMODIFIER_TANGENT_BASE_RETURN_BODY(T,, M)
#define LINESTYLEALPHAMODIFIER_TANGENT_RETURN(M)           LINESTYLEALPHAMODIFIER_TANGENT_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_AlongStroke3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_ALONGSTROKE_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_ALONGSTROKE_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_ALONGSTROKE_RETURN(M)           LINESTYLECOLORMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_CREASEANGLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_CreaseAngle3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_CREASEANGLE_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_CREASEANGLE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_CREASEANGLE_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_CREASEANGLE_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_CREASEANGLE_RETURN(M)           LINESTYLECOLORMODIFIER_CREASEANGLE_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_Curvature_3D3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_CURVATURE_3D_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_CURVATURE_3D_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_CURVATURE_3D_RETURN(M)           LINESTYLECOLORMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_DistanceFromCamera3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_RETURN(M)           LINESTYLECOLORMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_DistanceFromObject3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_RETURN(M)           LINESTYLECOLORMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_MATERIAL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_Material3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_MATERIAL_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_MATERIAL_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_MATERIAL_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_MATERIAL_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_MATERIAL_RETURN(M)           LINESTYLECOLORMODIFIER_MATERIAL_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_NOISE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_Noise3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_NOISE_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_NOISE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_NOISE_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_NOISE_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_NOISE_RETURN(M)           LINESTYLECOLORMODIFIER_NOISE_BASE_RETURN_BODY(,, M)

#define LINESTYLECOLORMODIFIER_TANGENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleColorModifier_Tangent3_6_0*>(data_ptr)-> C); 
#define LINESTYLECOLORMODIFIER_TANGENT_RETURN_REF(T, M)    LINESTYLECOLORMODIFIER_TANGENT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLECOLORMODIFIER_TANGENT_RETURN_AS(T, M)     LINESTYLECOLORMODIFIER_TANGENT_BASE_RETURN_BODY(T,, M)
#define LINESTYLECOLORMODIFIER_TANGENT_RETURN(M)           LINESTYLECOLORMODIFIER_TANGENT_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_2DOFFSET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_2DOffset3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_2DOFFSET_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_2DOFFSET_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_2DOFFSET_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_2DOFFSET_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_2DOFFSET_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_2DOFFSET_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_2DTransform3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_2DTRANSFORM_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_BackboneStretcher3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_BACKBONESTRETCHER_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_BezierCurve3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_BEZIERCURVE_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_Blueprint3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_BLUEPRINT_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_GuidingLines3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_GUIDINGLINES_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_PerlinNoise1D3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_PERLINNOISE1D_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_PerlinNoise2D3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_PERLINNOISE2D_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_Polygonalization3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_POLYGONALIZATION_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_SAMPLING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_Sampling3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_SAMPLING_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_SAMPLING_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_SAMPLING_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_SAMPLING_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_SAMPLING_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_SAMPLING_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_Simplification3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_SIMPLIFICATION_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_SinusDisplacement3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_SINUSDISPLACEMENT_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_SpatialNoise3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_SPATIALNOISE_BASE_RETURN_BODY(,, M)

#define LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleGeometryModifier_TipRemover3_6_0*>(data_ptr)-> C); 
#define LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_RETURN_REF(T, M)    LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_RETURN_AS(T, M)     LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_BASE_RETURN_BODY(T,, M)
#define LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_RETURN(M)           LINESTYLEGEOMETRYMODIFIER_TIPREMOVER_BASE_RETURN_BODY(,, M)

#define LINESTYLEMODIFIER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleModifier3_6_0*>(data_ptr)-> C); 
#define LINESTYLEMODIFIER_RETURN_REF(T, M)    LINESTYLEMODIFIER_BASE_RETURN_BODY(T, &, M)
#define LINESTYLEMODIFIER_RETURN_AS(T, M)     LINESTYLEMODIFIER_BASE_RETURN_BODY(T,, M)
#define LINESTYLEMODIFIER_RETURN(M)           LINESTYLEMODIFIER_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_AlongStroke3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_RETURN(M)           LINESTYLETHICKNESSMODIFIER_ALONGSTROKE_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_Calligraphy3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_RETURN(M)           LINESTYLETHICKNESSMODIFIER_CALLIGRAPHY_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_CREASEANGLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_CreaseAngle3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_CREASEANGLE_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_CREASEANGLE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_CREASEANGLE_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_CREASEANGLE_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_CREASEANGLE_RETURN(M)           LINESTYLETHICKNESSMODIFIER_CREASEANGLE_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_Curvature_3D3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_RETURN(M)           LINESTYLETHICKNESSMODIFIER_CURVATURE_3D_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_DistanceFromCamera3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_RETURN(M)           LINESTYLETHICKNESSMODIFIER_DISTANCEFROMCAMERA_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_DistanceFromObject3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_RETURN(M)           LINESTYLETHICKNESSMODIFIER_DISTANCEFROMOBJECT_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_MATERIAL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_Material3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_MATERIAL_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_MATERIAL_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_MATERIAL_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_MATERIAL_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_MATERIAL_RETURN(M)           LINESTYLETHICKNESSMODIFIER_MATERIAL_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_NOISE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_Noise3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_NOISE_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_NOISE_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_NOISE_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_NOISE_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_NOISE_RETURN(M)           LINESTYLETHICKNESSMODIFIER_NOISE_BASE_RETURN_BODY(,, M)

#define LINESTYLETHICKNESSMODIFIER_TANGENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LineStyleThicknessModifier_Tangent3_6_0*>(data_ptr)-> C); 
#define LINESTYLETHICKNESSMODIFIER_TANGENT_RETURN_REF(T, M)    LINESTYLETHICKNESSMODIFIER_TANGENT_BASE_RETURN_BODY(T, &, M)
#define LINESTYLETHICKNESSMODIFIER_TANGENT_RETURN_AS(T, M)     LINESTYLETHICKNESSMODIFIER_TANGENT_BASE_RETURN_BODY(T,, M)
#define LINESTYLETHICKNESSMODIFIER_TANGENT_RETURN(M)           LINESTYLETHICKNESSMODIFIER_TANGENT_BASE_RETURN_BODY(,, M)

#define LINKDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<LinkData3_6_0*>(data_ptr)-> C); 
#define LINKDATA_RETURN_REF(T, M)    LINKDATA_BASE_RETURN_BODY(T, &, M)
#define LINKDATA_RETURN_AS(T, M)     LINKDATA_BASE_RETURN_BODY(T,, M)
#define LINKDATA_RETURN(M)           LINKDATA_BASE_RETURN_BODY(,, M)

#define LINK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Link3_6_0*>(data_ptr)-> C); 
#define LINK_RETURN_REF(T, M)    LINK_BASE_RETURN_BODY(T, &, M)
#define LINK_RETURN_AS(T, M)     LINK_BASE_RETURN_BODY(T,, M)
#define LINK_RETURN(M)           LINK_BASE_RETURN_BODY(,, M)

#define LISTBASE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ListBase3_6_0*>(data_ptr)-> C); 
#define LISTBASE_RETURN_REF(T, M)    LISTBASE_BASE_RETURN_BODY(T, &, M)
#define LISTBASE_RETURN_AS(T, M)     LISTBASE_BASE_RETURN_BODY(T,, M)
#define LISTBASE_RETURN(M)           LISTBASE_BASE_RETURN_BODY(,, M)

#define MAPPINGINFOMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MappingInfoModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MappingInfoModifierData4_1_0*>(data_ptr)-> C); 
#define MAPPINGINFOMODIFIERDATA_RETURN_REF(T, M)    MAPPINGINFOMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MAPPINGINFOMODIFIERDATA_RETURN_AS(T, M)     MAPPINGINFOMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MAPPINGINFOMODIFIERDATA_RETURN(M)           MAPPINGINFOMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MASKLAYERSHAPEELEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MaskLayerShapeElem3_6_0*>(data_ptr)-> C); 
#define MASKLAYERSHAPEELEM_RETURN_REF(T, M)    MASKLAYERSHAPEELEM_BASE_RETURN_BODY(T, &, M)
#define MASKLAYERSHAPEELEM_RETURN_AS(T, M)     MASKLAYERSHAPEELEM_BASE_RETURN_BODY(T,, M)
#define MASKLAYERSHAPEELEM_RETURN(M)           MASKLAYERSHAPEELEM_BASE_RETURN_BODY(,, M)

#define MASKLAYERSHAPE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MaskLayerShape3_6_0*>(data_ptr)-> C); 
#define MASKLAYERSHAPE_RETURN_REF(T, M)    MASKLAYERSHAPE_BASE_RETURN_BODY(T, &, M)
#define MASKLAYERSHAPE_RETURN_AS(T, M)     MASKLAYERSHAPE_BASE_RETURN_BODY(T,, M)
#define MASKLAYERSHAPE_RETURN(M)           MASKLAYERSHAPE_BASE_RETURN_BODY(,, M)

#define MASKLAYER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MaskLayer3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MaskLayer4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MaskLayer4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MaskLayer4_4_0*>(data_ptr)-> C); 
#define MASKLAYER_RETURN_REF(T, M)    MASKLAYER_BASE_RETURN_BODY(T, &, M)
#define MASKLAYER_RETURN_AS(T, M)     MASKLAYER_BASE_RETURN_BODY(T,, M)
#define MASKLAYER_RETURN(M)           MASKLAYER_BASE_RETURN_BODY(,, M)

#define MASKMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MaskModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MaskModifierData4_1_0*>(data_ptr)-> C); 
#define MASKMODIFIERDATA_RETURN_REF(T, M)    MASKMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MASKMODIFIERDATA_RETURN_AS(T, M)     MASKMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MASKMODIFIERDATA_RETURN(M)           MASKMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MASKPARENT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MaskParent3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MaskParent4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MaskParent4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MaskParent4_4_0*>(data_ptr)-> C); 
#define MASKPARENT_RETURN_REF(T, M)    MASKPARENT_BASE_RETURN_BODY(T, &, M)
#define MASKPARENT_RETURN_AS(T, M)     MASKPARENT_BASE_RETURN_BODY(T,, M)
#define MASKPARENT_RETURN(M)           MASKPARENT_BASE_RETURN_BODY(,, M)

#define MASKSPACEINFO_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MaskSpaceInfo3_6_0*>(data_ptr)-> C); 
#define MASKSPACEINFO_RETURN_REF(T, M)    MASKSPACEINFO_BASE_RETURN_BODY(T, &, M)
#define MASKSPACEINFO_RETURN_AS(T, M)     MASKSPACEINFO_BASE_RETURN_BODY(T,, M)
#define MASKSPACEINFO_RETURN(M)           MASKSPACEINFO_BASE_RETURN_BODY(,, M)

#define MASKSPLINEPOINTUW_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MaskSplinePointUW3_6_0*>(data_ptr)-> C); 
#define MASKSPLINEPOINTUW_RETURN_REF(T, M)    MASKSPLINEPOINTUW_BASE_RETURN_BODY(T, &, M)
#define MASKSPLINEPOINTUW_RETURN_AS(T, M)     MASKSPLINEPOINTUW_BASE_RETURN_BODY(T,, M)
#define MASKSPLINEPOINTUW_RETURN(M)           MASKSPLINEPOINTUW_BASE_RETURN_BODY(,, M)

#define MASKSPLINEPOINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MaskSplinePoint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MaskSplinePoint4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MaskSplinePoint4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MaskSplinePoint4_4_0*>(data_ptr)-> C); 
#define MASKSPLINEPOINT_RETURN_REF(T, M)    MASKSPLINEPOINT_BASE_RETURN_BODY(T, &, M)
#define MASKSPLINEPOINT_RETURN_AS(T, M)     MASKSPLINEPOINT_BASE_RETURN_BODY(T,, M)
#define MASKSPLINEPOINT_RETURN(M)           MASKSPLINEPOINT_BASE_RETURN_BODY(,, M)

#define MASKSPLINE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MaskSpline3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MaskSpline4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MaskSpline4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MaskSpline4_4_0*>(data_ptr)-> C); 
#define MASKSPLINE_RETURN_REF(T, M)    MASKSPLINE_BASE_RETURN_BODY(T, &, M)
#define MASKSPLINE_RETURN_AS(T, M)     MASKSPLINE_BASE_RETURN_BODY(T,, M)
#define MASKSPLINE_RETURN(M)           MASKSPLINE_BASE_RETURN_BODY(,, M)

#define MASK_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Mask3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Mask4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Mask4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Mask4_4_0*>(data_ptr)-> C); 
#define MASK_RETURN_REF(T, M)    MASK_BASE_RETURN_BODY(T, &, M)
#define MASK_RETURN_AS(T, M)     MASK_BASE_RETURN_BODY(T,, M)
#define MASK_RETURN(M)           MASK_BASE_RETURN_BODY(,, M)

#define MATERIALGPENCILSTYLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MaterialGPencilStyle3_6_0*>(data_ptr)-> C); 
#define MATERIALGPENCILSTYLE_RETURN_REF(T, M)    MATERIALGPENCILSTYLE_BASE_RETURN_BODY(T, &, M)
#define MATERIALGPENCILSTYLE_RETURN_AS(T, M)     MATERIALGPENCILSTYLE_BASE_RETURN_BODY(T,, M)
#define MATERIALGPENCILSTYLE_RETURN(M)           MATERIALGPENCILSTYLE_BASE_RETURN_BODY(,, M)

#define MATERIALLINEART_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MaterialLineArt3_6_0*>(data_ptr)-> C); 
#define MATERIALLINEART_RETURN_REF(T, M)    MATERIALLINEART_BASE_RETURN_BODY(T, &, M)
#define MATERIALLINEART_RETURN_AS(T, M)     MATERIALLINEART_BASE_RETURN_BODY(T,, M)
#define MATERIALLINEART_RETURN(M)           MATERIALLINEART_BASE_RETURN_BODY(,, M)

#define MATERIAL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Material3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Material4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Material4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Material4_4_0*>(data_ptr)-> C); 
#define MATERIAL_RETURN_REF(T, M)    MATERIAL_BASE_RETURN_BODY(T, &, M)
#define MATERIAL_RETURN_AS(T, M)     MATERIAL_BASE_RETURN_BODY(T,, M)
#define MATERIAL_RETURN(M)           MATERIAL_BASE_RETURN_BODY(,, M)

#define MBOOLPROPERTY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MBoolProperty3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MBoolProperty4_1_0*>(data_ptr)-> C); 
#define MBOOLPROPERTY_RETURN_REF(T, M)    MBOOLPROPERTY_BASE_RETURN_BODY(T, &, M)
#define MBOOLPROPERTY_RETURN_AS(T, M)     MBOOLPROPERTY_BASE_RETURN_BODY(T,, M)
#define MBOOLPROPERTY_RETURN(M)           MBOOLPROPERTY_BASE_RETURN_BODY(,, M)

#define MCOL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MCol3_6_0*>(data_ptr)-> C); 
#define MCOL_RETURN_REF(T, M)    MCOL_BASE_RETURN_BODY(T, &, M)
#define MCOL_RETURN_AS(T, M)     MCOL_BASE_RETURN_BODY(T,, M)
#define MCOL_RETURN(M)           MCOL_BASE_RETURN_BODY(,, M)

#define MDEFCELL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MDefCell3_6_0*>(data_ptr)-> C); 
#define MDEFCELL_RETURN_REF(T, M)    MDEFCELL_BASE_RETURN_BODY(T, &, M)
#define MDEFCELL_RETURN_AS(T, M)     MDEFCELL_BASE_RETURN_BODY(T,, M)
#define MDEFCELL_RETURN(M)           MDEFCELL_BASE_RETURN_BODY(,, M)

#define MDEFINFLUENCE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MDefInfluence3_6_0*>(data_ptr)-> C); 
#define MDEFINFLUENCE_RETURN_REF(T, M)    MDEFINFLUENCE_BASE_RETURN_BODY(T, &, M)
#define MDEFINFLUENCE_RETURN_AS(T, M)     MDEFINFLUENCE_BASE_RETURN_BODY(T,, M)
#define MDEFINFLUENCE_RETURN(M)           MDEFINFLUENCE_BASE_RETURN_BODY(,, M)

#define MDEFORMVERT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MDeformVert3_6_0*>(data_ptr)-> C); 
#define MDEFORMVERT_RETURN_REF(T, M)    MDEFORMVERT_BASE_RETURN_BODY(T, &, M)
#define MDEFORMVERT_RETURN_AS(T, M)     MDEFORMVERT_BASE_RETURN_BODY(T,, M)
#define MDEFORMVERT_RETURN(M)           MDEFORMVERT_BASE_RETURN_BODY(,, M)

#define MDEFORMWEIGHT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MDeformWeight3_6_0*>(data_ptr)-> C); 
#define MDEFORMWEIGHT_RETURN_REF(T, M)    MDEFORMWEIGHT_BASE_RETURN_BODY(T, &, M)
#define MDEFORMWEIGHT_RETURN_AS(T, M)     MDEFORMWEIGHT_BASE_RETURN_BODY(T,, M)
#define MDEFORMWEIGHT_RETURN(M)           MDEFORMWEIGHT_BASE_RETURN_BODY(,, M)

#define MDISPS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MDisps3_6_0*>(data_ptr)-> C); 
#define MDISPS_RETURN_REF(T, M)    MDISPS_BASE_RETURN_BODY(T, &, M)
#define MDISPS_RETURN_AS(T, M)     MDISPS_BASE_RETURN_BODY(T,, M)
#define MDISPS_RETURN(M)           MDISPS_BASE_RETURN_BODY(,, M)

#define MESHCACHEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MeshCacheModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MeshCacheModifierData4_1_0*>(data_ptr)-> C); 
#define MESHCACHEMODIFIERDATA_RETURN_REF(T, M)    MESHCACHEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MESHCACHEMODIFIERDATA_RETURN_AS(T, M)     MESHCACHEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MESHCACHEMODIFIERDATA_RETURN(M)           MESHCACHEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MESHSEQCACHEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MeshSeqCacheModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MeshSeqCacheModifierData4_1_0*>(data_ptr)-> C); 
#define MESHSEQCACHEMODIFIERDATA_RETURN_REF(T, M)    MESHSEQCACHEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MESHSEQCACHEMODIFIERDATA_RETURN_AS(T, M)     MESHSEQCACHEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MESHSEQCACHEMODIFIERDATA_RETURN(M)           MESHSEQCACHEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MESHSTATVIS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MeshStatVis3_6_0*>(data_ptr)-> C); 
#define MESHSTATVIS_RETURN_REF(T, M)    MESHSTATVIS_BASE_RETURN_BODY(T, &, M)
#define MESHSTATVIS_RETURN_AS(T, M)     MESHSTATVIS_BASE_RETURN_BODY(T,, M)
#define MESHSTATVIS_RETURN(M)           MESHSTATVIS_BASE_RETURN_BODY(,, M)

#define MESHTOVOLUMEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<MeshToVolumeModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MeshToVolumeModifierData4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MeshToVolumeModifierData4_1_0*>(data_ptr)-> C); 
#define MESHTOVOLUMEMODIFIERDATA_RETURN_REF(T, M)    MESHTOVOLUMEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MESHTOVOLUMEMODIFIERDATA_RETURN_AS(T, M)     MESHTOVOLUMEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MESHTOVOLUMEMODIFIERDATA_RETURN(M)           MESHTOVOLUMEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MESH_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Mesh3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Mesh4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Mesh4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Mesh4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Mesh4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Mesh4_4_0*>(data_ptr)-> C); 
#define MESH_RETURN_REF(T, M)    MESH_BASE_RETURN_BODY(T, &, M)
#define MESH_RETURN_AS(T, M)     MESH_BASE_RETURN_BODY(T,, M)
#define MESH_RETURN(M)           MESH_BASE_RETURN_BODY(,, M)

#define METABALL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MetaBall3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MetaBall4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MetaBall4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MetaBall4_4_0*>(data_ptr)-> C); 
#define METABALL_RETURN_REF(T, M)    METABALL_BASE_RETURN_BODY(T, &, M)
#define METABALL_RETURN_AS(T, M)     METABALL_BASE_RETURN_BODY(T,, M)
#define METABALL_RETURN(M)           METABALL_BASE_RETURN_BODY(,, M)

#define METAELEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MetaElem3_6_0*>(data_ptr)-> C); 
#define METAELEM_RETURN_REF(T, M)    METAELEM_BASE_RETURN_BODY(T, &, M)
#define METAELEM_RETURN_AS(T, M)     METAELEM_BASE_RETURN_BODY(T,, M)
#define METAELEM_RETURN(M)           METAELEM_BASE_RETURN_BODY(,, M)

#define METASTACK_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<MetaStack3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MetaStack4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<MetaStack4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MetaStack4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MetaStack4_4_0*>(data_ptr)-> C); 
#define METASTACK_RETURN_REF(T, M)    METASTACK_BASE_RETURN_BODY(T, &, M)
#define METASTACK_RETURN_AS(T, M)     METASTACK_BASE_RETURN_BODY(T,, M)
#define METASTACK_RETURN(M)           METASTACK_BASE_RETURN_BODY(,, M)

#define MFACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MFace3_6_0*>(data_ptr)-> C); 
#define MFACE_RETURN_REF(T, M)    MFACE_BASE_RETURN_BODY(T, &, M)
#define MFACE_RETURN_AS(T, M)     MFACE_BASE_RETURN_BODY(T,, M)
#define MFACE_RETURN(M)           MFACE_BASE_RETURN_BODY(,, M)

#define MFLOATPROPERTY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MFloatProperty3_6_0*>(data_ptr)-> C); 
#define MFLOATPROPERTY_RETURN_REF(T, M)    MFLOATPROPERTY_BASE_RETURN_BODY(T, &, M)
#define MFLOATPROPERTY_RETURN_AS(T, M)     MFLOATPROPERTY_BASE_RETURN_BODY(T,, M)
#define MFLOATPROPERTY_RETURN(M)           MFLOATPROPERTY_BASE_RETURN_BODY(,, M)

#define MINT8PROPERTY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MInt8Property3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MInt8Property4_1_0*>(data_ptr)-> C); 
#define MINT8PROPERTY_RETURN_REF(T, M)    MINT8PROPERTY_BASE_RETURN_BODY(T, &, M)
#define MINT8PROPERTY_RETURN_AS(T, M)     MINT8PROPERTY_BASE_RETURN_BODY(T,, M)
#define MINT8PROPERTY_RETURN(M)           MINT8PROPERTY_BASE_RETURN_BODY(,, M)

#define MINTPROPERTY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MIntProperty3_6_0*>(data_ptr)-> C); 
#define MINTPROPERTY_RETURN_REF(T, M)    MINTPROPERTY_BASE_RETURN_BODY(T, &, M)
#define MINTPROPERTY_RETURN_AS(T, M)     MINTPROPERTY_BASE_RETURN_BODY(T,, M)
#define MINTPROPERTY_RETURN(M)           MINTPROPERTY_BASE_RETURN_BODY(,, M)

#define MIRRORGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MirrorGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define MIRRORGPENCILMODIFIERDATA_RETURN_REF(T, M)    MIRRORGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MIRRORGPENCILMODIFIERDATA_RETURN_AS(T, M)     MIRRORGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MIRRORGPENCILMODIFIERDATA_RETURN(M)           MIRRORGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MIRRORMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MirrorModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MirrorModifierData4_1_0*>(data_ptr)-> C); 
#define MIRRORMODIFIERDATA_RETURN_REF(T, M)    MIRRORMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MIRRORMODIFIERDATA_RETURN_AS(T, M)     MIRRORMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MIRRORMODIFIERDATA_RETURN(M)           MIRRORMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MLOOPCOL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MLoopCol3_6_0*>(data_ptr)-> C); 
#define MLOOPCOL_RETURN_REF(T, M)    MLOOPCOL_BASE_RETURN_BODY(T, &, M)
#define MLOOPCOL_RETURN_AS(T, M)     MLOOPCOL_BASE_RETURN_BODY(T,, M)
#define MLOOPCOL_RETURN(M)           MLOOPCOL_BASE_RETURN_BODY(,, M)

#define MLOOPTRI_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MLoopTri3_6_0*>(data_ptr)-> C); 
#define MLOOPTRI_RETURN_REF(T, M)    MLOOPTRI_BASE_RETURN_BODY(T, &, M)
#define MLOOPTRI_RETURN_AS(T, M)     MLOOPTRI_BASE_RETURN_BODY(T,, M)
#define MLOOPTRI_RETURN(M)           MLOOPTRI_BASE_RETURN_BODY(,, M)

#define MODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ModifierData4_1_0*>(data_ptr)-> C); 
#define MODIFIERDATA_RETURN_REF(T, M)    MODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MODIFIERDATA_RETURN_AS(T, M)     MODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MODIFIERDATA_RETURN(M)           MODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MODIFIERVIEWERPATHELEM_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ModifierViewerPathElem3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ModifierViewerPathElem4_0_0*>(data_ptr)-> C); 
#define MODIFIERVIEWERPATHELEM_RETURN_REF(T, M)    MODIFIERVIEWERPATHELEM_BASE_RETURN_BODY(T, &, M)
#define MODIFIERVIEWERPATHELEM_RETURN_AS(T, M)     MODIFIERVIEWERPATHELEM_BASE_RETURN_BODY(T,, M)
#define MODIFIERVIEWERPATHELEM_RETURN(M)           MODIFIERVIEWERPATHELEM_BASE_RETURN_BODY(,, M)

#define MOVIECLIPPROXY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieClipProxy3_6_0*>(data_ptr)-> C); 
#define MOVIECLIPPROXY_RETURN_REF(T, M)    MOVIECLIPPROXY_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIPPROXY_RETURN_AS(T, M)     MOVIECLIPPROXY_BASE_RETURN_BODY(T,, M)
#define MOVIECLIPPROXY_RETURN(M)           MOVIECLIPPROXY_BASE_RETURN_BODY(,, M)

#define MOVIECLIPSCOPES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieClipScopes3_6_0*>(data_ptr)-> C); 
#define MOVIECLIPSCOPES_RETURN_REF(T, M)    MOVIECLIPSCOPES_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIPSCOPES_RETURN_AS(T, M)     MOVIECLIPSCOPES_BASE_RETURN_BODY(T,, M)
#define MOVIECLIPSCOPES_RETURN(M)           MOVIECLIPSCOPES_BASE_RETURN_BODY(,, M)

#define MOVIECLIPUSER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieClipUser3_6_0*>(data_ptr)-> C); 
#define MOVIECLIPUSER_RETURN_REF(T, M)    MOVIECLIPUSER_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIPUSER_RETURN_AS(T, M)     MOVIECLIPUSER_BASE_RETURN_BODY(T,, M)
#define MOVIECLIPUSER_RETURN(M)           MOVIECLIPUSER_BASE_RETURN_BODY(,, M)

#define MOVIECLIP_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MovieClip3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MovieClip4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MovieClip4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MovieClip4_4_0*>(data_ptr)-> C); 
#define MOVIECLIP_RETURN_REF(T, M)    MOVIECLIP_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIP_RETURN_AS(T, M)     MOVIECLIP_BASE_RETURN_BODY(T,, M)
#define MOVIECLIP_RETURN(M)           MOVIECLIP_BASE_RETURN_BODY(,, M)

#define MOVIECLIP_RUNTIMEGPUTEXTURE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieClip_RuntimeGPUTexture3_6_0*>(data_ptr)-> C); 
#define MOVIECLIP_RUNTIMEGPUTEXTURE_RETURN_REF(T, M)    MOVIECLIP_RUNTIMEGPUTEXTURE_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIP_RUNTIMEGPUTEXTURE_RETURN_AS(T, M)     MOVIECLIP_RUNTIMEGPUTEXTURE_BASE_RETURN_BODY(T,, M)
#define MOVIECLIP_RUNTIMEGPUTEXTURE_RETURN(M)           MOVIECLIP_RUNTIMEGPUTEXTURE_BASE_RETURN_BODY(,, M)

#define MOVIECLIP_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieClip_Runtime3_6_0*>(data_ptr)-> C); 
#define MOVIECLIP_RUNTIME_RETURN_REF(T, M)    MOVIECLIP_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIP_RUNTIME_RETURN_AS(T, M)     MOVIECLIP_RUNTIME_BASE_RETURN_BODY(T,, M)
#define MOVIECLIP_RUNTIME_RETURN(M)           MOVIECLIP_RUNTIME_BASE_RETURN_BODY(,, M)

#define MOVIERECONSTRUCTEDCAMERA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieReconstructedCamera3_6_0*>(data_ptr)-> C); 
#define MOVIERECONSTRUCTEDCAMERA_RETURN_REF(T, M)    MOVIERECONSTRUCTEDCAMERA_BASE_RETURN_BODY(T, &, M)
#define MOVIERECONSTRUCTEDCAMERA_RETURN_AS(T, M)     MOVIERECONSTRUCTEDCAMERA_BASE_RETURN_BODY(T,, M)
#define MOVIERECONSTRUCTEDCAMERA_RETURN(M)           MOVIERECONSTRUCTEDCAMERA_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGCAMERA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingCamera3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGCAMERA_RETURN_REF(T, M)    MOVIETRACKINGCAMERA_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGCAMERA_RETURN_AS(T, M)     MOVIETRACKINGCAMERA_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGCAMERA_RETURN(M)           MOVIETRACKINGCAMERA_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGDOPESHEETCHANNEL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingDopesheetChannel3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGDOPESHEETCHANNEL_RETURN_REF(T, M)    MOVIETRACKINGDOPESHEETCHANNEL_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGDOPESHEETCHANNEL_RETURN_AS(T, M)     MOVIETRACKINGDOPESHEETCHANNEL_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGDOPESHEETCHANNEL_RETURN(M)           MOVIETRACKINGDOPESHEETCHANNEL_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingDopesheetCoverageSegment3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_RETURN_REF(T, M)    MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_RETURN_AS(T, M)     MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_RETURN(M)           MOVIETRACKINGDOPESHEETCOVERAGESEGMENT_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGDOPESHEET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingDopesheet3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGDOPESHEET_RETURN_REF(T, M)    MOVIETRACKINGDOPESHEET_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGDOPESHEET_RETURN_AS(T, M)     MOVIETRACKINGDOPESHEET_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGDOPESHEET_RETURN(M)           MOVIETRACKINGDOPESHEET_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGMARKER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingMarker3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGMARKER_RETURN_REF(T, M)    MOVIETRACKINGMARKER_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGMARKER_RETURN_AS(T, M)     MOVIETRACKINGMARKER_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGMARKER_RETURN(M)           MOVIETRACKINGMARKER_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGOBJECT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MovieTrackingObject3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MovieTrackingObject4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MovieTrackingObject4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MovieTrackingObject4_4_0*>(data_ptr)-> C); 
#define MOVIETRACKINGOBJECT_RETURN_REF(T, M)    MOVIETRACKINGOBJECT_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGOBJECT_RETURN_AS(T, M)     MOVIETRACKINGOBJECT_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGOBJECT_RETURN(M)           MOVIETRACKINGOBJECT_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGPLANEMARKER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingPlaneMarker3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGPLANEMARKER_RETURN_REF(T, M)    MOVIETRACKINGPLANEMARKER_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGPLANEMARKER_RETURN_AS(T, M)     MOVIETRACKINGPLANEMARKER_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGPLANEMARKER_RETURN(M)           MOVIETRACKINGPLANEMARKER_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGPLANETRACK_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MovieTrackingPlaneTrack3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MovieTrackingPlaneTrack4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MovieTrackingPlaneTrack4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MovieTrackingPlaneTrack4_4_0*>(data_ptr)-> C); 
#define MOVIETRACKINGPLANETRACK_RETURN_REF(T, M)    MOVIETRACKINGPLANETRACK_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGPLANETRACK_RETURN_AS(T, M)     MOVIETRACKINGPLANETRACK_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGPLANETRACK_RETURN(M)           MOVIETRACKINGPLANETRACK_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGRECONSTRUCTION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingReconstruction3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGRECONSTRUCTION_RETURN_REF(T, M)    MOVIETRACKINGRECONSTRUCTION_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGRECONSTRUCTION_RETURN_AS(T, M)     MOVIETRACKINGRECONSTRUCTION_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGRECONSTRUCTION_RETURN(M)           MOVIETRACKINGRECONSTRUCTION_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingSettings3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGSETTINGS_RETURN_REF(T, M)    MOVIETRACKINGSETTINGS_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGSETTINGS_RETURN_AS(T, M)     MOVIETRACKINGSETTINGS_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGSETTINGS_RETURN(M)           MOVIETRACKINGSETTINGS_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGSTABILIZATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingStabilization3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGSTABILIZATION_RETURN_REF(T, M)    MOVIETRACKINGSTABILIZATION_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGSTABILIZATION_RETURN_AS(T, M)     MOVIETRACKINGSTABILIZATION_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGSTABILIZATION_RETURN(M)           MOVIETRACKINGSTABILIZATION_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGSTATS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingStats3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGSTATS_RETURN_REF(T, M)    MOVIETRACKINGSTATS_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGSTATS_RETURN_AS(T, M)     MOVIETRACKINGSTATS_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGSTATS_RETURN(M)           MOVIETRACKINGSTATS_BASE_RETURN_BODY(,, M)

#define MOVIETRACKINGTRACK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MovieTrackingTrack3_6_0*>(data_ptr)-> C); 
#define MOVIETRACKINGTRACK_RETURN_REF(T, M)    MOVIETRACKINGTRACK_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKINGTRACK_RETURN_AS(T, M)     MOVIETRACKINGTRACK_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKINGTRACK_RETURN(M)           MOVIETRACKINGTRACK_BASE_RETURN_BODY(,, M)

#define MOVIETRACKING_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MovieTracking3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MovieTracking4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MovieTracking4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MovieTracking4_4_0*>(data_ptr)-> C); 
#define MOVIETRACKING_RETURN_REF(T, M)    MOVIETRACKING_BASE_RETURN_BODY(T, &, M)
#define MOVIETRACKING_RETURN_AS(T, M)     MOVIETRACKING_BASE_RETURN_BODY(T,, M)
#define MOVIETRACKING_RETURN(M)           MOVIETRACKING_BASE_RETURN_BODY(,, M)

#define MPROPCOL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MPropCol3_6_0*>(data_ptr)-> C); 
#define MPROPCOL_RETURN_REF(T, M)    MPROPCOL_BASE_RETURN_BODY(T, &, M)
#define MPROPCOL_RETURN_AS(T, M)     MPROPCOL_BASE_RETURN_BODY(T,, M)
#define MPROPCOL_RETURN(M)           MPROPCOL_BASE_RETURN_BODY(,, M)

#define MRECAST_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MRecast3_6_0*>(data_ptr)-> C); 
#define MRECAST_RETURN_REF(T, M)    MRECAST_BASE_RETURN_BODY(T, &, M)
#define MRECAST_RETURN_AS(T, M)     MRECAST_BASE_RETURN_BODY(T,, M)
#define MRECAST_RETURN(M)           MRECAST_BASE_RETURN_BODY(,, M)

#define MSELECT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MSelect3_6_0*>(data_ptr)-> C); 
#define MSELECT_RETURN_REF(T, M)    MSELECT_BASE_RETURN_BODY(T, &, M)
#define MSELECT_RETURN_AS(T, M)     MSELECT_BASE_RETURN_BODY(T,, M)
#define MSELECT_RETURN(M)           MSELECT_BASE_RETURN_BODY(,, M)

#define MSTRINGPROPERTY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MStringProperty3_6_0*>(data_ptr)-> C); 
#define MSTRINGPROPERTY_RETURN_REF(T, M)    MSTRINGPROPERTY_BASE_RETURN_BODY(T, &, M)
#define MSTRINGPROPERTY_RETURN_AS(T, M)     MSTRINGPROPERTY_BASE_RETURN_BODY(T,, M)
#define MSTRINGPROPERTY_RETURN(M)           MSTRINGPROPERTY_BASE_RETURN_BODY(,, M)

#define MTEX_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<MTex3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MTex4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<MTex4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<MTex4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<MTex4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MTex4_4_0*>(data_ptr)-> C); 
#define MTEX_RETURN_REF(T, M)    MTEX_BASE_RETURN_BODY(T, &, M)
#define MTEX_RETURN_AS(T, M)     MTEX_BASE_RETURN_BODY(T,, M)
#define MTEX_RETURN(M)           MTEX_BASE_RETURN_BODY(,, M)

#define MTFACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MTFace3_6_0*>(data_ptr)-> C); 
#define MTFACE_RETURN_REF(T, M)    MTFACE_BASE_RETURN_BODY(T, &, M)
#define MTFACE_RETURN_AS(T, M)     MTFACE_BASE_RETURN_BODY(T,, M)
#define MTFACE_RETURN(M)           MTFACE_BASE_RETURN_BODY(,, M)

#define MULTIPLYGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MultiplyGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define MULTIPLYGPENCILMODIFIERDATA_RETURN_REF(T, M)    MULTIPLYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MULTIPLYGPENCILMODIFIERDATA_RETURN_AS(T, M)     MULTIPLYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MULTIPLYGPENCILMODIFIERDATA_RETURN(M)           MULTIPLYGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MULTIRESMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<MultiresModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<MultiresModifierData4_1_0*>(data_ptr)-> C); 
#define MULTIRESMODIFIERDATA_RETURN_REF(T, M)    MULTIRESMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define MULTIRESMODIFIERDATA_RETURN_AS(T, M)     MULTIRESMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define MULTIRESMODIFIERDATA_RETURN(M)           MULTIRESMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define MVERTSKIN_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MVertSkin3_6_0*>(data_ptr)-> C); 
#define MVERTSKIN_RETURN_REF(T, M)    MVERTSKIN_BASE_RETURN_BODY(T, &, M)
#define MVERTSKIN_RETURN_AS(T, M)     MVERTSKIN_BASE_RETURN_BODY(T,, M)
#define MVERTSKIN_RETURN(M)           MVERTSKIN_BASE_RETURN_BODY(,, M)

#define MVERTTRI_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<MVertTri3_6_0*>(data_ptr)-> C); 
#define MVERTTRI_RETURN_REF(T, M)    MVERTTRI_BASE_RETURN_BODY(T, &, M)
#define MVERTTRI_RETURN_AS(T, M)     MVERTTRI_BASE_RETURN_BODY(T,, M)
#define MVERTTRI_RETURN(M)           MVERTTRI_BASE_RETURN_BODY(,, M)

#define NLASTRIP_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<NlaStrip3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<NlaStrip4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<NlaStrip4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<NlaStrip4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NlaStrip4_4_0*>(data_ptr)-> C); 
#define NLASTRIP_RETURN_REF(T, M)    NLASTRIP_BASE_RETURN_BODY(T, &, M)
#define NLASTRIP_RETURN_AS(T, M)     NLASTRIP_BASE_RETURN_BODY(T,, M)
#define NLASTRIP_RETURN(M)           NLASTRIP_BASE_RETURN_BODY(,, M)

#define NLATRACK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NlaTrack3_6_0*>(data_ptr)-> C); 
#define NLATRACK_RETURN_REF(T, M)    NLATRACK_BASE_RETURN_BODY(T, &, M)
#define NLATRACK_RETURN_AS(T, M)     NLATRACK_BASE_RETURN_BODY(T,, M)
#define NLATRACK_RETURN(M)           NLATRACK_BASE_RETURN_BODY(,, M)

#define NODEACCUMULATEFIELD_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeAccumulateField3_6_0*>(data_ptr)-> C); 
#define NODEACCUMULATEFIELD_RETURN_REF(T, M)    NODEACCUMULATEFIELD_BASE_RETURN_BODY(T, &, M)
#define NODEACCUMULATEFIELD_RETURN_AS(T, M)     NODEACCUMULATEFIELD_BASE_RETURN_BODY(T,, M)
#define NODEACCUMULATEFIELD_RETURN(M)           NODEACCUMULATEFIELD_BASE_RETURN_BODY(,, M)

#define NODEANTIALIASINGDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeAntiAliasingData3_6_0*>(data_ptr)-> C); 
#define NODEANTIALIASINGDATA_RETURN_REF(T, M)    NODEANTIALIASINGDATA_BASE_RETURN_BODY(T, &, M)
#define NODEANTIALIASINGDATA_RETURN_AS(T, M)     NODEANTIALIASINGDATA_BASE_RETURN_BODY(T,, M)
#define NODEANTIALIASINGDATA_RETURN(M)           NODEANTIALIASINGDATA_BASE_RETURN_BODY(,, M)

#define NODEBILATERALBLURDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeBilateralBlurData3_6_0*>(data_ptr)-> C); 
#define NODEBILATERALBLURDATA_RETURN_REF(T, M)    NODEBILATERALBLURDATA_BASE_RETURN_BODY(T, &, M)
#define NODEBILATERALBLURDATA_RETURN_AS(T, M)     NODEBILATERALBLURDATA_BASE_RETURN_BODY(T,, M)
#define NODEBILATERALBLURDATA_RETURN(M)           NODEBILATERALBLURDATA_BASE_RETURN_BODY(,, M)

#define NODEBLURDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeBlurData3_6_0*>(data_ptr)-> C); 
#define NODEBLURDATA_RETURN_REF(T, M)    NODEBLURDATA_BASE_RETURN_BODY(T, &, M)
#define NODEBLURDATA_RETURN_AS(T, M)     NODEBLURDATA_BASE_RETURN_BODY(T,, M)
#define NODEBLURDATA_RETURN(M)           NODEBLURDATA_BASE_RETURN_BODY(,, M)

#define NODEBOKEHIMAGE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeBokehImage3_6_0*>(data_ptr)-> C); 
#define NODEBOKEHIMAGE_RETURN_REF(T, M)    NODEBOKEHIMAGE_BASE_RETURN_BODY(T, &, M)
#define NODEBOKEHIMAGE_RETURN_AS(T, M)     NODEBOKEHIMAGE_BASE_RETURN_BODY(T,, M)
#define NODEBOKEHIMAGE_RETURN(M)           NODEBOKEHIMAGE_BASE_RETURN_BODY(,, M)

#define NODEBOXMASK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeBoxMask3_6_0*>(data_ptr)-> C); 
#define NODEBOXMASK_RETURN_REF(T, M)    NODEBOXMASK_BASE_RETURN_BODY(T, &, M)
#define NODEBOXMASK_RETURN_AS(T, M)     NODEBOXMASK_BASE_RETURN_BODY(T,, M)
#define NODEBOXMASK_RETURN(M)           NODEBOXMASK_BASE_RETURN_BODY(,, M)

#define NODECHROMA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeChroma3_6_0*>(data_ptr)-> C); 
#define NODECHROMA_RETURN_REF(T, M)    NODECHROMA_BASE_RETURN_BODY(T, &, M)
#define NODECHROMA_RETURN_AS(T, M)     NODECHROMA_BASE_RETURN_BODY(T,, M)
#define NODECHROMA_RETURN(M)           NODECHROMA_BASE_RETURN_BODY(,, M)

#define NODECMPCOMBSEPCOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeCMPCombSepColor3_6_0*>(data_ptr)-> C); 
#define NODECMPCOMBSEPCOLOR_RETURN_REF(T, M)    NODECMPCOMBSEPCOLOR_BASE_RETURN_BODY(T, &, M)
#define NODECMPCOMBSEPCOLOR_RETURN_AS(T, M)     NODECMPCOMBSEPCOLOR_BASE_RETURN_BODY(T,, M)
#define NODECMPCOMBSEPCOLOR_RETURN(M)           NODECMPCOMBSEPCOLOR_BASE_RETURN_BODY(,, M)

#define NODECOLORBALANCE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<NodeColorBalance3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeColorBalance4_3_0*>(data_ptr)-> C); 
#define NODECOLORBALANCE_RETURN_REF(T, M)    NODECOLORBALANCE_BASE_RETURN_BODY(T, &, M)
#define NODECOLORBALANCE_RETURN_AS(T, M)     NODECOLORBALANCE_BASE_RETURN_BODY(T,, M)
#define NODECOLORBALANCE_RETURN(M)           NODECOLORBALANCE_BASE_RETURN_BODY(,, M)

#define NODECOLORCORRECTION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeColorCorrection3_6_0*>(data_ptr)-> C); 
#define NODECOLORCORRECTION_RETURN_REF(T, M)    NODECOLORCORRECTION_BASE_RETURN_BODY(T, &, M)
#define NODECOLORCORRECTION_RETURN_AS(T, M)     NODECOLORCORRECTION_BASE_RETURN_BODY(T,, M)
#define NODECOLORCORRECTION_RETURN(M)           NODECOLORCORRECTION_BASE_RETURN_BODY(,, M)

#define NODECOLORSPILL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeColorspill3_6_0*>(data_ptr)-> C); 
#define NODECOLORSPILL_RETURN_REF(T, M)    NODECOLORSPILL_BASE_RETURN_BODY(T, &, M)
#define NODECOLORSPILL_RETURN_AS(T, M)     NODECOLORSPILL_BASE_RETURN_BODY(T,, M)
#define NODECOLORSPILL_RETURN(M)           NODECOLORSPILL_BASE_RETURN_BODY(,, M)

#define NODECOMBSEPCOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeCombSepColor3_6_0*>(data_ptr)-> C); 
#define NODECOMBSEPCOLOR_RETURN_REF(T, M)    NODECOMBSEPCOLOR_BASE_RETURN_BODY(T, &, M)
#define NODECOMBSEPCOLOR_RETURN_AS(T, M)     NODECOMBSEPCOLOR_BASE_RETURN_BODY(T,, M)
#define NODECOMBSEPCOLOR_RETURN(M)           NODECOMBSEPCOLOR_BASE_RETURN_BODY(,, M)

#define NODECONVERTCOLORSPACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeConvertColorSpace3_6_0*>(data_ptr)-> C); 
#define NODECONVERTCOLORSPACE_RETURN_REF(T, M)    NODECONVERTCOLORSPACE_BASE_RETURN_BODY(T, &, M)
#define NODECONVERTCOLORSPACE_RETURN_AS(T, M)     NODECONVERTCOLORSPACE_BASE_RETURN_BODY(T,, M)
#define NODECONVERTCOLORSPACE_RETURN(M)           NODECONVERTCOLORSPACE_BASE_RETURN_BODY(,, M)

#define NODECRYPTOMATTE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeCryptomatte3_6_0*>(data_ptr)-> C); 
#define NODECRYPTOMATTE_RETURN_REF(T, M)    NODECRYPTOMATTE_BASE_RETURN_BODY(T, &, M)
#define NODECRYPTOMATTE_RETURN_AS(T, M)     NODECRYPTOMATTE_BASE_RETURN_BODY(T,, M)
#define NODECRYPTOMATTE_RETURN(M)           NODECRYPTOMATTE_BASE_RETURN_BODY(,, M)

#define NODECRYPTOMATTE_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeCryptomatte_Runtime3_6_0*>(data_ptr)-> C); 
#define NODECRYPTOMATTE_RUNTIME_RETURN_REF(T, M)    NODECRYPTOMATTE_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define NODECRYPTOMATTE_RUNTIME_RETURN_AS(T, M)     NODECRYPTOMATTE_RUNTIME_BASE_RETURN_BODY(T,, M)
#define NODECRYPTOMATTE_RUNTIME_RETURN(M)           NODECRYPTOMATTE_RUNTIME_BASE_RETURN_BODY(,, M)

#define NODEDBLURDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeDBlurData3_6_0*>(data_ptr)-> C); 
#define NODEDBLURDATA_RETURN_REF(T, M)    NODEDBLURDATA_BASE_RETURN_BODY(T, &, M)
#define NODEDBLURDATA_RETURN_AS(T, M)     NODEDBLURDATA_BASE_RETURN_BODY(T,, M)
#define NODEDBLURDATA_RETURN(M)           NODEDBLURDATA_BASE_RETURN_BODY(,, M)

#define NODEDEFOCUS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeDefocus3_6_0*>(data_ptr)-> C); 
#define NODEDEFOCUS_RETURN_REF(T, M)    NODEDEFOCUS_BASE_RETURN_BODY(T, &, M)
#define NODEDEFOCUS_RETURN_AS(T, M)     NODEDEFOCUS_BASE_RETURN_BODY(T,, M)
#define NODEDEFOCUS_RETURN(M)           NODEDEFOCUS_BASE_RETURN_BODY(,, M)

#define NODEDENOISE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<NodeDenoise3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeDenoise4_4_0*>(data_ptr)-> C); 
#define NODEDENOISE_RETURN_REF(T, M)    NODEDENOISE_BASE_RETURN_BODY(T, &, M)
#define NODEDENOISE_RETURN_AS(T, M)     NODEDENOISE_BASE_RETURN_BODY(T,, M)
#define NODEDENOISE_RETURN(M)           NODEDENOISE_BASE_RETURN_BODY(,, M)

#define NODEDILATEERODE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeDilateErode3_6_0*>(data_ptr)-> C); 
#define NODEDILATEERODE_RETURN_REF(T, M)    NODEDILATEERODE_BASE_RETURN_BODY(T, &, M)
#define NODEDILATEERODE_RETURN_AS(T, M)     NODEDILATEERODE_BASE_RETURN_BODY(T,, M)
#define NODEDILATEERODE_RETURN(M)           NODEDILATEERODE_BASE_RETURN_BODY(,, M)

#define NODEELLIPSEMASK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeEllipseMask3_6_0*>(data_ptr)-> C); 
#define NODEELLIPSEMASK_RETURN_REF(T, M)    NODEELLIPSEMASK_BASE_RETURN_BODY(T, &, M)
#define NODEELLIPSEMASK_RETURN_AS(T, M)     NODEELLIPSEMASK_BASE_RETURN_BODY(T,, M)
#define NODEELLIPSEMASK_RETURN(M)           NODEELLIPSEMASK_BASE_RETURN_BODY(,, M)

#define NODEFRAME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeFrame3_6_0*>(data_ptr)-> C); 
#define NODEFRAME_RETURN_REF(T, M)    NODEFRAME_BASE_RETURN_BODY(T, &, M)
#define NODEFRAME_RETURN_AS(T, M)     NODEFRAME_BASE_RETURN_BODY(T,, M)
#define NODEFRAME_RETURN(M)           NODEFRAME_BASE_RETURN_BODY(,, M)

#define NODEFUNCTIONCOMPARE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeFunctionCompare3_6_0*>(data_ptr)-> C); 
#define NODEFUNCTIONCOMPARE_RETURN_REF(T, M)    NODEFUNCTIONCOMPARE_BASE_RETURN_BODY(T, &, M)
#define NODEFUNCTIONCOMPARE_RETURN_AS(T, M)     NODEFUNCTIONCOMPARE_BASE_RETURN_BODY(T,, M)
#define NODEFUNCTIONCOMPARE_RETURN(M)           NODEFUNCTIONCOMPARE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYATTRIBUTECAPTURE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<NodeGeometryAttributeCapture3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeGeometryAttributeCapture4_2_0*>(data_ptr)-> C); 
#define NODEGEOMETRYATTRIBUTECAPTURE_RETURN_REF(T, M)    NODEGEOMETRYATTRIBUTECAPTURE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYATTRIBUTECAPTURE_RETURN_AS(T, M)     NODEGEOMETRYATTRIBUTECAPTURE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYATTRIBUTECAPTURE_RETURN(M)           NODEGEOMETRYATTRIBUTECAPTURE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCOLLECTIONINFO_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCollectionInfo3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCOLLECTIONINFO_RETURN_REF(T, M)    NODEGEOMETRYCOLLECTIONINFO_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCOLLECTIONINFO_RETURN_AS(T, M)     NODEGEOMETRYCOLLECTIONINFO_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCOLLECTIONINFO_RETURN(M)           NODEGEOMETRYCOLLECTIONINFO_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEFILLET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveFillet3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEFILLET_RETURN_REF(T, M)    NODEGEOMETRYCURVEFILLET_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEFILLET_RETURN_AS(T, M)     NODEGEOMETRYCURVEFILLET_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEFILLET_RETURN(M)           NODEGEOMETRYCURVEFILLET_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEFILL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveFill3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEFILL_RETURN_REF(T, M)    NODEGEOMETRYCURVEFILL_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEFILL_RETURN_AS(T, M)     NODEGEOMETRYCURVEFILL_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEFILL_RETURN(M)           NODEGEOMETRYCURVEFILL_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEPRIMITIVEARC_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurvePrimitiveArc3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEPRIMITIVEARC_RETURN_REF(T, M)    NODEGEOMETRYCURVEPRIMITIVEARC_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEPRIMITIVEARC_RETURN_AS(T, M)     NODEGEOMETRYCURVEPRIMITIVEARC_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEPRIMITIVEARC_RETURN(M)           NODEGEOMETRYCURVEPRIMITIVEARC_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurvePrimitiveBezierSegment3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_RETURN_REF(T, M)    NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_RETURN_AS(T, M)     NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_RETURN(M)           NODEGEOMETRYCURVEPRIMITIVEBEZIERSEGMENT_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEPRIMITIVECIRCLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurvePrimitiveCircle3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEPRIMITIVECIRCLE_RETURN_REF(T, M)    NODEGEOMETRYCURVEPRIMITIVECIRCLE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEPRIMITIVECIRCLE_RETURN_AS(T, M)     NODEGEOMETRYCURVEPRIMITIVECIRCLE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEPRIMITIVECIRCLE_RETURN(M)           NODEGEOMETRYCURVEPRIMITIVECIRCLE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEPRIMITIVELINE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurvePrimitiveLine3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEPRIMITIVELINE_RETURN_REF(T, M)    NODEGEOMETRYCURVEPRIMITIVELINE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEPRIMITIVELINE_RETURN_AS(T, M)     NODEGEOMETRYCURVEPRIMITIVELINE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEPRIMITIVELINE_RETURN(M)           NODEGEOMETRYCURVEPRIMITIVELINE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVEPRIMITIVEQUAD_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurvePrimitiveQuad3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVEPRIMITIVEQUAD_RETURN_REF(T, M)    NODEGEOMETRYCURVEPRIMITIVEQUAD_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVEPRIMITIVEQUAD_RETURN_AS(T, M)     NODEGEOMETRYCURVEPRIMITIVEQUAD_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVEPRIMITIVEQUAD_RETURN(M)           NODEGEOMETRYCURVEPRIMITIVEQUAD_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVERESAMPLE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<NodeGeometryCurveResample3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeGeometryCurveResample4_4_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVERESAMPLE_RETURN_REF(T, M)    NODEGEOMETRYCURVERESAMPLE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVERESAMPLE_RETURN_AS(T, M)     NODEGEOMETRYCURVERESAMPLE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVERESAMPLE_RETURN(M)           NODEGEOMETRYCURVERESAMPLE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVESAMPLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveSample3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVESAMPLE_RETURN_REF(T, M)    NODEGEOMETRYCURVESAMPLE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVESAMPLE_RETURN_AS(T, M)     NODEGEOMETRYCURVESAMPLE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVESAMPLE_RETURN(M)           NODEGEOMETRYCURVESAMPLE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVESELECTHANDLES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveSelectHandles3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVESELECTHANDLES_RETURN_REF(T, M)    NODEGEOMETRYCURVESELECTHANDLES_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVESELECTHANDLES_RETURN_AS(T, M)     NODEGEOMETRYCURVESELECTHANDLES_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVESELECTHANDLES_RETURN(M)           NODEGEOMETRYCURVESELECTHANDLES_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVESETHANDLES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveSetHandles3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVESETHANDLES_RETURN_REF(T, M)    NODEGEOMETRYCURVESETHANDLES_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVESETHANDLES_RETURN_AS(T, M)     NODEGEOMETRYCURVESETHANDLES_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVESETHANDLES_RETURN(M)           NODEGEOMETRYCURVESETHANDLES_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVESPLINETYPE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveSplineType3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVESPLINETYPE_RETURN_REF(T, M)    NODEGEOMETRYCURVESPLINETYPE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVESPLINETYPE_RETURN_AS(T, M)     NODEGEOMETRYCURVESPLINETYPE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVESPLINETYPE_RETURN(M)           NODEGEOMETRYCURVESPLINETYPE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVETOPOINTS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveToPoints3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVETOPOINTS_RETURN_REF(T, M)    NODEGEOMETRYCURVETOPOINTS_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVETOPOINTS_RETURN_AS(T, M)     NODEGEOMETRYCURVETOPOINTS_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVETOPOINTS_RETURN(M)           NODEGEOMETRYCURVETOPOINTS_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYCURVETRIM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryCurveTrim3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYCURVETRIM_RETURN_REF(T, M)    NODEGEOMETRYCURVETRIM_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYCURVETRIM_RETURN_AS(T, M)     NODEGEOMETRYCURVETRIM_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYCURVETRIM_RETURN(M)           NODEGEOMETRYCURVETRIM_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYDELETEGEOMETRY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryDeleteGeometry3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYDELETEGEOMETRY_RETURN_REF(T, M)    NODEGEOMETRYDELETEGEOMETRY_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYDELETEGEOMETRY_RETURN_AS(T, M)     NODEGEOMETRYDELETEGEOMETRY_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYDELETEGEOMETRY_RETURN(M)           NODEGEOMETRYDELETEGEOMETRY_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryDistributePointsInVolume3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_RETURN_REF(T, M)    NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_RETURN_AS(T, M)     NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_RETURN(M)           NODEGEOMETRYDISTRIBUTEPOINTSINVOLUME_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYDUPLICATEELEMENTS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryDuplicateElements3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYDUPLICATEELEMENTS_RETURN_REF(T, M)    NODEGEOMETRYDUPLICATEELEMENTS_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYDUPLICATEELEMENTS_RETURN_AS(T, M)     NODEGEOMETRYDUPLICATEELEMENTS_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYDUPLICATEELEMENTS_RETURN(M)           NODEGEOMETRYDUPLICATEELEMENTS_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYEXTRUDEMESH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryExtrudeMesh3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYEXTRUDEMESH_RETURN_REF(T, M)    NODEGEOMETRYEXTRUDEMESH_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYEXTRUDEMESH_RETURN_AS(T, M)     NODEGEOMETRYEXTRUDEMESH_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYEXTRUDEMESH_RETURN(M)           NODEGEOMETRYEXTRUDEMESH_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYIMAGETEXTURE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryImageTexture3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYIMAGETEXTURE_RETURN_REF(T, M)    NODEGEOMETRYIMAGETEXTURE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYIMAGETEXTURE_RETURN_AS(T, M)     NODEGEOMETRYIMAGETEXTURE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYIMAGETEXTURE_RETURN(M)           NODEGEOMETRYIMAGETEXTURE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYINPUTNAMEDATTRIBUTE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryInputNamedAttribute3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYINPUTNAMEDATTRIBUTE_RETURN_REF(T, M)    NODEGEOMETRYINPUTNAMEDATTRIBUTE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYINPUTNAMEDATTRIBUTE_RETURN_AS(T, M)     NODEGEOMETRYINPUTNAMEDATTRIBUTE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYINPUTNAMEDATTRIBUTE_RETURN(M)           NODEGEOMETRYINPUTNAMEDATTRIBUTE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMERGEBYDISTANCE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMergeByDistance3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMERGEBYDISTANCE_RETURN_REF(T, M)    NODEGEOMETRYMERGEBYDISTANCE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMERGEBYDISTANCE_RETURN_AS(T, M)     NODEGEOMETRYMERGEBYDISTANCE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMERGEBYDISTANCE_RETURN(M)           NODEGEOMETRYMERGEBYDISTANCE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMESHCIRCLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMeshCircle3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMESHCIRCLE_RETURN_REF(T, M)    NODEGEOMETRYMESHCIRCLE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMESHCIRCLE_RETURN_AS(T, M)     NODEGEOMETRYMESHCIRCLE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMESHCIRCLE_RETURN(M)           NODEGEOMETRYMESHCIRCLE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMESHCONE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMeshCone3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMESHCONE_RETURN_REF(T, M)    NODEGEOMETRYMESHCONE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMESHCONE_RETURN_AS(T, M)     NODEGEOMETRYMESHCONE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMESHCONE_RETURN(M)           NODEGEOMETRYMESHCONE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMESHCYLINDER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMeshCylinder3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMESHCYLINDER_RETURN_REF(T, M)    NODEGEOMETRYMESHCYLINDER_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMESHCYLINDER_RETURN_AS(T, M)     NODEGEOMETRYMESHCYLINDER_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMESHCYLINDER_RETURN(M)           NODEGEOMETRYMESHCYLINDER_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMESHLINE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMeshLine3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMESHLINE_RETURN_REF(T, M)    NODEGEOMETRYMESHLINE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMESHLINE_RETURN_AS(T, M)     NODEGEOMETRYMESHLINE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMESHLINE_RETURN(M)           NODEGEOMETRYMESHLINE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMESHTOPOINTS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMeshToPoints3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMESHTOPOINTS_RETURN_REF(T, M)    NODEGEOMETRYMESHTOPOINTS_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMESHTOPOINTS_RETURN_AS(T, M)     NODEGEOMETRYMESHTOPOINTS_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMESHTOPOINTS_RETURN(M)           NODEGEOMETRYMESHTOPOINTS_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYMESHTOVOLUME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryMeshToVolume3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYMESHTOVOLUME_RETURN_REF(T, M)    NODEGEOMETRYMESHTOVOLUME_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYMESHTOVOLUME_RETURN_AS(T, M)     NODEGEOMETRYMESHTOVOLUME_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYMESHTOVOLUME_RETURN(M)           NODEGEOMETRYMESHTOVOLUME_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYOBJECTINFO_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryObjectInfo3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYOBJECTINFO_RETURN_REF(T, M)    NODEGEOMETRYOBJECTINFO_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYOBJECTINFO_RETURN_AS(T, M)     NODEGEOMETRYOBJECTINFO_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYOBJECTINFO_RETURN(M)           NODEGEOMETRYOBJECTINFO_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYPOINTSTOVOLUME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryPointsToVolume3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYPOINTSTOVOLUME_RETURN_REF(T, M)    NODEGEOMETRYPOINTSTOVOLUME_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYPOINTSTOVOLUME_RETURN_AS(T, M)     NODEGEOMETRYPOINTSTOVOLUME_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYPOINTSTOVOLUME_RETURN(M)           NODEGEOMETRYPOINTSTOVOLUME_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYPROXIMITY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryProximity3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYPROXIMITY_RETURN_REF(T, M)    NODEGEOMETRYPROXIMITY_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYPROXIMITY_RETURN_AS(T, M)     NODEGEOMETRYPROXIMITY_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYPROXIMITY_RETURN(M)           NODEGEOMETRYPROXIMITY_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYRAYCAST_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<NodeGeometryRaycast3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeGeometryRaycast4_0_0*>(data_ptr)-> C); 
#define NODEGEOMETRYRAYCAST_RETURN_REF(T, M)    NODEGEOMETRYRAYCAST_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYRAYCAST_RETURN_AS(T, M)     NODEGEOMETRYRAYCAST_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYRAYCAST_RETURN(M)           NODEGEOMETRYRAYCAST_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSAMPLEINDEX_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySampleIndex3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSAMPLEINDEX_RETURN_REF(T, M)    NODEGEOMETRYSAMPLEINDEX_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSAMPLEINDEX_RETURN_AS(T, M)     NODEGEOMETRYSAMPLEINDEX_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSAMPLEINDEX_RETURN(M)           NODEGEOMETRYSAMPLEINDEX_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSAMPLEVOLUME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySampleVolume3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSAMPLEVOLUME_RETURN_REF(T, M)    NODEGEOMETRYSAMPLEVOLUME_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSAMPLEVOLUME_RETURN_AS(T, M)     NODEGEOMETRYSAMPLEVOLUME_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSAMPLEVOLUME_RETURN(M)           NODEGEOMETRYSAMPLEVOLUME_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSEPARATEGEOMETRY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySeparateGeometry3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSEPARATEGEOMETRY_RETURN_REF(T, M)    NODEGEOMETRYSEPARATEGEOMETRY_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSEPARATEGEOMETRY_RETURN_AS(T, M)     NODEGEOMETRYSEPARATEGEOMETRY_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSEPARATEGEOMETRY_RETURN(M)           NODEGEOMETRYSEPARATEGEOMETRY_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSETCURVEHANDLEPOSITIONS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySetCurveHandlePositions3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSETCURVEHANDLEPOSITIONS_RETURN_REF(T, M)    NODEGEOMETRYSETCURVEHANDLEPOSITIONS_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSETCURVEHANDLEPOSITIONS_RETURN_AS(T, M)     NODEGEOMETRYSETCURVEHANDLEPOSITIONS_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSETCURVEHANDLEPOSITIONS_RETURN(M)           NODEGEOMETRYSETCURVEHANDLEPOSITIONS_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSIMULATIONINPUT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySimulationInput3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSIMULATIONINPUT_RETURN_REF(T, M)    NODEGEOMETRYSIMULATIONINPUT_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSIMULATIONINPUT_RETURN_AS(T, M)     NODEGEOMETRYSIMULATIONINPUT_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSIMULATIONINPUT_RETURN(M)           NODEGEOMETRYSIMULATIONINPUT_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSIMULATIONOUTPUT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySimulationOutput3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSIMULATIONOUTPUT_RETURN_REF(T, M)    NODEGEOMETRYSIMULATIONOUTPUT_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSIMULATIONOUTPUT_RETURN_AS(T, M)     NODEGEOMETRYSIMULATIONOUTPUT_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSIMULATIONOUTPUT_RETURN(M)           NODEGEOMETRYSIMULATIONOUTPUT_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSTORENAMEDATTRIBUTE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryStoreNamedAttribute3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSTORENAMEDATTRIBUTE_RETURN_REF(T, M)    NODEGEOMETRYSTORENAMEDATTRIBUTE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSTORENAMEDATTRIBUTE_RETURN_AS(T, M)     NODEGEOMETRYSTORENAMEDATTRIBUTE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSTORENAMEDATTRIBUTE_RETURN(M)           NODEGEOMETRYSTORENAMEDATTRIBUTE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSTRINGTOCURVES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryStringToCurves3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSTRINGTOCURVES_RETURN_REF(T, M)    NODEGEOMETRYSTRINGTOCURVES_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSTRINGTOCURVES_RETURN_AS(T, M)     NODEGEOMETRYSTRINGTOCURVES_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSTRINGTOCURVES_RETURN(M)           NODEGEOMETRYSTRINGTOCURVES_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYSUBDIVISIONSURFACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometrySubdivisionSurface3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYSUBDIVISIONSURFACE_RETURN_REF(T, M)    NODEGEOMETRYSUBDIVISIONSURFACE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYSUBDIVISIONSURFACE_RETURN_AS(T, M)     NODEGEOMETRYSUBDIVISIONSURFACE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYSUBDIVISIONSURFACE_RETURN(M)           NODEGEOMETRYSUBDIVISIONSURFACE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYTRANSFERATTRIBUTE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryTransferAttribute3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYTRANSFERATTRIBUTE_RETURN_REF(T, M)    NODEGEOMETRYTRANSFERATTRIBUTE_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYTRANSFERATTRIBUTE_RETURN_AS(T, M)     NODEGEOMETRYTRANSFERATTRIBUTE_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYTRANSFERATTRIBUTE_RETURN(M)           NODEGEOMETRYTRANSFERATTRIBUTE_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYUVUNWRAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryUVUnwrap3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYUVUNWRAP_RETURN_REF(T, M)    NODEGEOMETRYUVUNWRAP_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYUVUNWRAP_RETURN_AS(T, M)     NODEGEOMETRYUVUNWRAP_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYUVUNWRAP_RETURN(M)           NODEGEOMETRYUVUNWRAP_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYVIEWER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryViewer3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYVIEWER_RETURN_REF(T, M)    NODEGEOMETRYVIEWER_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYVIEWER_RETURN_AS(T, M)     NODEGEOMETRYVIEWER_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYVIEWER_RETURN(M)           NODEGEOMETRYVIEWER_BASE_RETURN_BODY(,, M)

#define NODEGEOMETRYVOLUMETOMESH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeGeometryVolumeToMesh3_6_0*>(data_ptr)-> C); 
#define NODEGEOMETRYVOLUMETOMESH_RETURN_REF(T, M)    NODEGEOMETRYVOLUMETOMESH_BASE_RETURN_BODY(T, &, M)
#define NODEGEOMETRYVOLUMETOMESH_RETURN_AS(T, M)     NODEGEOMETRYVOLUMETOMESH_BASE_RETURN_BODY(T,, M)
#define NODEGEOMETRYVOLUMETOMESH_RETURN(M)           NODEGEOMETRYVOLUMETOMESH_BASE_RETURN_BODY(,, M)

#define NODEGLARE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<NodeGlare3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeGlare4_4_0*>(data_ptr)-> C); 
#define NODEGLARE_RETURN_REF(T, M)    NODEGLARE_BASE_RETURN_BODY(T, &, M)
#define NODEGLARE_RETURN_AS(T, M)     NODEGLARE_BASE_RETURN_BODY(T,, M)
#define NODEGLARE_RETURN(M)           NODEGLARE_BASE_RETURN_BODY(,, M)

#define NODEHUESAT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeHueSat3_6_0*>(data_ptr)-> C); 
#define NODEHUESAT_RETURN_REF(T, M)    NODEHUESAT_BASE_RETURN_BODY(T, &, M)
#define NODEHUESAT_RETURN_AS(T, M)     NODEHUESAT_BASE_RETURN_BODY(T,, M)
#define NODEHUESAT_RETURN(M)           NODEHUESAT_BASE_RETURN_BODY(,, M)

#define NODEIMAGEANIM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeImageAnim3_6_0*>(data_ptr)-> C); 
#define NODEIMAGEANIM_RETURN_REF(T, M)    NODEIMAGEANIM_BASE_RETURN_BODY(T, &, M)
#define NODEIMAGEANIM_RETURN_AS(T, M)     NODEIMAGEANIM_BASE_RETURN_BODY(T,, M)
#define NODEIMAGEANIM_RETURN(M)           NODEIMAGEANIM_BASE_RETURN_BODY(,, M)

#define NODEIMAGEFILE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<NodeImageFile3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeImageFile4_3_0*>(data_ptr)-> C); 
#define NODEIMAGEFILE_RETURN_REF(T, M)    NODEIMAGEFILE_BASE_RETURN_BODY(T, &, M)
#define NODEIMAGEFILE_RETURN_AS(T, M)     NODEIMAGEFILE_BASE_RETURN_BODY(T,, M)
#define NODEIMAGEFILE_RETURN(M)           NODEIMAGEFILE_BASE_RETURN_BODY(,, M)

#define NODEIMAGELAYER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeImageLayer3_6_0*>(data_ptr)-> C); 
#define NODEIMAGELAYER_RETURN_REF(T, M)    NODEIMAGELAYER_BASE_RETURN_BODY(T, &, M)
#define NODEIMAGELAYER_RETURN_AS(T, M)     NODEIMAGELAYER_BASE_RETURN_BODY(T,, M)
#define NODEIMAGELAYER_RETURN(M)           NODEIMAGELAYER_BASE_RETURN_BODY(,, M)

#define NODEIMAGEMULTIFILESOCKET_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<NodeImageMultiFileSocket3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeImageMultiFileSocket4_3_0*>(data_ptr)-> C); 
#define NODEIMAGEMULTIFILESOCKET_RETURN_REF(T, M)    NODEIMAGEMULTIFILESOCKET_BASE_RETURN_BODY(T, &, M)
#define NODEIMAGEMULTIFILESOCKET_RETURN_AS(T, M)     NODEIMAGEMULTIFILESOCKET_BASE_RETURN_BODY(T,, M)
#define NODEIMAGEMULTIFILESOCKET_RETURN(M)           NODEIMAGEMULTIFILESOCKET_BASE_RETURN_BODY(,, M)

#define NODEIMAGEMULTIFILE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<NodeImageMultiFile3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeImageMultiFile4_3_0*>(data_ptr)-> C); 
#define NODEIMAGEMULTIFILE_RETURN_REF(T, M)    NODEIMAGEMULTIFILE_BASE_RETURN_BODY(T, &, M)
#define NODEIMAGEMULTIFILE_RETURN_AS(T, M)     NODEIMAGEMULTIFILE_BASE_RETURN_BODY(T,, M)
#define NODEIMAGEMULTIFILE_RETURN(M)           NODEIMAGEMULTIFILE_BASE_RETURN_BODY(,, M)

#define NODEINPUTBOOL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeInputBool3_6_0*>(data_ptr)-> C); 
#define NODEINPUTBOOL_RETURN_REF(T, M)    NODEINPUTBOOL_BASE_RETURN_BODY(T, &, M)
#define NODEINPUTBOOL_RETURN_AS(T, M)     NODEINPUTBOOL_BASE_RETURN_BODY(T,, M)
#define NODEINPUTBOOL_RETURN(M)           NODEINPUTBOOL_BASE_RETURN_BODY(,, M)

#define NODEINPUTCOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeInputColor3_6_0*>(data_ptr)-> C); 
#define NODEINPUTCOLOR_RETURN_REF(T, M)    NODEINPUTCOLOR_BASE_RETURN_BODY(T, &, M)
#define NODEINPUTCOLOR_RETURN_AS(T, M)     NODEINPUTCOLOR_BASE_RETURN_BODY(T,, M)
#define NODEINPUTCOLOR_RETURN(M)           NODEINPUTCOLOR_BASE_RETURN_BODY(,, M)

#define NODEINPUTINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeInputInt3_6_0*>(data_ptr)-> C); 
#define NODEINPUTINT_RETURN_REF(T, M)    NODEINPUTINT_BASE_RETURN_BODY(T, &, M)
#define NODEINPUTINT_RETURN_AS(T, M)     NODEINPUTINT_BASE_RETURN_BODY(T,, M)
#define NODEINPUTINT_RETURN(M)           NODEINPUTINT_BASE_RETURN_BODY(,, M)

#define NODEINPUTSTRING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeInputString3_6_0*>(data_ptr)-> C); 
#define NODEINPUTSTRING_RETURN_REF(T, M)    NODEINPUTSTRING_BASE_RETURN_BODY(T, &, M)
#define NODEINPUTSTRING_RETURN_AS(T, M)     NODEINPUTSTRING_BASE_RETURN_BODY(T,, M)
#define NODEINPUTSTRING_RETURN(M)           NODEINPUTSTRING_BASE_RETURN_BODY(,, M)

#define NODEINPUTVECTOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeInputVector3_6_0*>(data_ptr)-> C); 
#define NODEINPUTVECTOR_RETURN_REF(T, M)    NODEINPUTVECTOR_BASE_RETURN_BODY(T, &, M)
#define NODEINPUTVECTOR_RETURN_AS(T, M)     NODEINPUTVECTOR_BASE_RETURN_BODY(T,, M)
#define NODEINPUTVECTOR_RETURN(M)           NODEINPUTVECTOR_BASE_RETURN_BODY(,, M)

#define NODEKEYINGDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeKeyingData3_6_0*>(data_ptr)-> C); 
#define NODEKEYINGDATA_RETURN_REF(T, M)    NODEKEYINGDATA_BASE_RETURN_BODY(T, &, M)
#define NODEKEYINGDATA_RETURN_AS(T, M)     NODEKEYINGDATA_BASE_RETURN_BODY(T,, M)
#define NODEKEYINGDATA_RETURN(M)           NODEKEYINGDATA_BASE_RETURN_BODY(,, M)

#define NODEKEYINGSCREENDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<NodeKeyingScreenData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeKeyingScreenData4_1_0*>(data_ptr)-> C); 
#define NODEKEYINGSCREENDATA_RETURN_REF(T, M)    NODEKEYINGSCREENDATA_BASE_RETURN_BODY(T, &, M)
#define NODEKEYINGSCREENDATA_RETURN_AS(T, M)     NODEKEYINGSCREENDATA_BASE_RETURN_BODY(T,, M)
#define NODEKEYINGSCREENDATA_RETURN(M)           NODEKEYINGSCREENDATA_BASE_RETURN_BODY(,, M)

#define NODELENSDIST_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeLensDist3_6_0*>(data_ptr)-> C); 
#define NODELENSDIST_RETURN_REF(T, M)    NODELENSDIST_BASE_RETURN_BODY(T, &, M)
#define NODELENSDIST_RETURN_AS(T, M)     NODELENSDIST_BASE_RETURN_BODY(T,, M)
#define NODELENSDIST_RETURN(M)           NODELENSDIST_BASE_RETURN_BODY(,, M)

#define NODEMAPRANGE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeMapRange3_6_0*>(data_ptr)-> C); 
#define NODEMAPRANGE_RETURN_REF(T, M)    NODEMAPRANGE_BASE_RETURN_BODY(T, &, M)
#define NODEMAPRANGE_RETURN_AS(T, M)     NODEMAPRANGE_BASE_RETURN_BODY(T,, M)
#define NODEMAPRANGE_RETURN(M)           NODEMAPRANGE_BASE_RETURN_BODY(,, M)

#define NODEMASK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeMask3_6_0*>(data_ptr)-> C); 
#define NODEMASK_RETURN_REF(T, M)    NODEMASK_BASE_RETURN_BODY(T, &, M)
#define NODEMASK_RETURN_AS(T, M)     NODEMASK_BASE_RETURN_BODY(T,, M)
#define NODEMASK_RETURN(M)           NODEMASK_BASE_RETURN_BODY(,, M)

#define NODEPLANETRACKDEFORMDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodePlaneTrackDeformData3_6_0*>(data_ptr)-> C); 
#define NODEPLANETRACKDEFORMDATA_RETURN_REF(T, M)    NODEPLANETRACKDEFORMDATA_BASE_RETURN_BODY(T, &, M)
#define NODEPLANETRACKDEFORMDATA_RETURN_AS(T, M)     NODEPLANETRACKDEFORMDATA_BASE_RETURN_BODY(T,, M)
#define NODEPLANETRACKDEFORMDATA_RETURN(M)           NODEPLANETRACKDEFORMDATA_BASE_RETURN_BODY(,, M)

#define NODERANDOMVALUE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeRandomValue3_6_0*>(data_ptr)-> C); 
#define NODERANDOMVALUE_RETURN_REF(T, M)    NODERANDOMVALUE_BASE_RETURN_BODY(T, &, M)
#define NODERANDOMVALUE_RETURN_AS(T, M)     NODERANDOMVALUE_BASE_RETURN_BODY(T,, M)
#define NODERANDOMVALUE_RETURN(M)           NODERANDOMVALUE_BASE_RETURN_BODY(,, M)

#define NODESCRIPTDICT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeScriptDict3_6_0*>(data_ptr)-> C); 
#define NODESCRIPTDICT_RETURN_REF(T, M)    NODESCRIPTDICT_BASE_RETURN_BODY(T, &, M)
#define NODESCRIPTDICT_RETURN_AS(T, M)     NODESCRIPTDICT_BASE_RETURN_BODY(T,, M)
#define NODESCRIPTDICT_RETURN(M)           NODESCRIPTDICT_BASE_RETURN_BODY(,, M)

#define NODESETALPHA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeSetAlpha3_6_0*>(data_ptr)-> C); 
#define NODESETALPHA_RETURN_REF(T, M)    NODESETALPHA_BASE_RETURN_BODY(T, &, M)
#define NODESETALPHA_RETURN_AS(T, M)     NODESETALPHA_BASE_RETURN_BODY(T,, M)
#define NODESETALPHA_RETURN(M)           NODESETALPHA_BASE_RETURN_BODY(,, M)

#define NODESHADERATTRIBUTE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<NodeShaderAttribute3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeShaderAttribute4_2_0*>(data_ptr)-> C); 
#define NODESHADERATTRIBUTE_RETURN_REF(T, M)    NODESHADERATTRIBUTE_BASE_RETURN_BODY(T, &, M)
#define NODESHADERATTRIBUTE_RETURN_AS(T, M)     NODESHADERATTRIBUTE_BASE_RETURN_BODY(T,, M)
#define NODESHADERATTRIBUTE_RETURN(M)           NODESHADERATTRIBUTE_BASE_RETURN_BODY(,, M)

#define NODESHADERMIX_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderMix3_6_0*>(data_ptr)-> C); 
#define NODESHADERMIX_RETURN_REF(T, M)    NODESHADERMIX_BASE_RETURN_BODY(T, &, M)
#define NODESHADERMIX_RETURN_AS(T, M)     NODESHADERMIX_BASE_RETURN_BODY(T,, M)
#define NODESHADERMIX_RETURN(M)           NODESHADERMIX_BASE_RETURN_BODY(,, M)

#define NODESHADERNORMALMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderNormalMap3_6_0*>(data_ptr)-> C); 
#define NODESHADERNORMALMAP_RETURN_REF(T, M)    NODESHADERNORMALMAP_BASE_RETURN_BODY(T, &, M)
#define NODESHADERNORMALMAP_RETURN_AS(T, M)     NODESHADERNORMALMAP_BASE_RETURN_BODY(T,, M)
#define NODESHADERNORMALMAP_RETURN(M)           NODESHADERNORMALMAP_BASE_RETURN_BODY(,, M)

#define NODESHADEROUTPUTAOV_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderOutputAOV3_6_0*>(data_ptr)-> C); 
#define NODESHADEROUTPUTAOV_RETURN_REF(T, M)    NODESHADEROUTPUTAOV_BASE_RETURN_BODY(T, &, M)
#define NODESHADEROUTPUTAOV_RETURN_AS(T, M)     NODESHADEROUTPUTAOV_BASE_RETURN_BODY(T,, M)
#define NODESHADEROUTPUTAOV_RETURN(M)           NODESHADEROUTPUTAOV_BASE_RETURN_BODY(,, M)

#define NODESHADERPRINCIPLED_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderPrincipled3_6_0*>(data_ptr)-> C); 
#define NODESHADERPRINCIPLED_RETURN_REF(T, M)    NODESHADERPRINCIPLED_BASE_RETURN_BODY(T, &, M)
#define NODESHADERPRINCIPLED_RETURN_AS(T, M)     NODESHADERPRINCIPLED_BASE_RETURN_BODY(T,, M)
#define NODESHADERPRINCIPLED_RETURN(M)           NODESHADERPRINCIPLED_BASE_RETURN_BODY(,, M)

#define NODESHADERSCRIPT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderScript3_6_0*>(data_ptr)-> C); 
#define NODESHADERSCRIPT_RETURN_REF(T, M)    NODESHADERSCRIPT_BASE_RETURN_BODY(T, &, M)
#define NODESHADERSCRIPT_RETURN_AS(T, M)     NODESHADERSCRIPT_BASE_RETURN_BODY(T,, M)
#define NODESHADERSCRIPT_RETURN(M)           NODESHADERSCRIPT_BASE_RETURN_BODY(,, M)

#define NODESHADERTANGENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderTangent3_6_0*>(data_ptr)-> C); 
#define NODESHADERTANGENT_RETURN_REF(T, M)    NODESHADERTANGENT_BASE_RETURN_BODY(T, &, M)
#define NODESHADERTANGENT_RETURN_AS(T, M)     NODESHADERTANGENT_BASE_RETURN_BODY(T,, M)
#define NODESHADERTANGENT_RETURN(M)           NODESHADERTANGENT_BASE_RETURN_BODY(,, M)

#define NODESHADERTEXIES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderTexIES3_6_0*>(data_ptr)-> C); 
#define NODESHADERTEXIES_RETURN_REF(T, M)    NODESHADERTEXIES_BASE_RETURN_BODY(T, &, M)
#define NODESHADERTEXIES_RETURN_AS(T, M)     NODESHADERTEXIES_BASE_RETURN_BODY(T,, M)
#define NODESHADERTEXIES_RETURN(M)           NODESHADERTEXIES_BASE_RETURN_BODY(,, M)

#define NODESHADERUVMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderUVMap3_6_0*>(data_ptr)-> C); 
#define NODESHADERUVMAP_RETURN_REF(T, M)    NODESHADERUVMAP_BASE_RETURN_BODY(T, &, M)
#define NODESHADERUVMAP_RETURN_AS(T, M)     NODESHADERUVMAP_BASE_RETURN_BODY(T,, M)
#define NODESHADERUVMAP_RETURN(M)           NODESHADERUVMAP_BASE_RETURN_BODY(,, M)

#define NODESHADERVECTTRANSFORM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderVectTransform3_6_0*>(data_ptr)-> C); 
#define NODESHADERVECTTRANSFORM_RETURN_REF(T, M)    NODESHADERVECTTRANSFORM_BASE_RETURN_BODY(T, &, M)
#define NODESHADERVECTTRANSFORM_RETURN_AS(T, M)     NODESHADERVECTTRANSFORM_BASE_RETURN_BODY(T,, M)
#define NODESHADERVECTTRANSFORM_RETURN(M)           NODESHADERVECTTRANSFORM_BASE_RETURN_BODY(,, M)

#define NODESHADERVERTEXCOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeShaderVertexColor3_6_0*>(data_ptr)-> C); 
#define NODESHADERVERTEXCOLOR_RETURN_REF(T, M)    NODESHADERVERTEXCOLOR_BASE_RETURN_BODY(T, &, M)
#define NODESHADERVERTEXCOLOR_RETURN_AS(T, M)     NODESHADERVERTEXCOLOR_BASE_RETURN_BODY(T,, M)
#define NODESHADERVERTEXCOLOR_RETURN(M)           NODESHADERVERTEXCOLOR_BASE_RETURN_BODY(,, M)

#define NODESIMULATIONITEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeSimulationItem3_6_0*>(data_ptr)-> C); 
#define NODESIMULATIONITEM_RETURN_REF(T, M)    NODESIMULATIONITEM_BASE_RETURN_BODY(T, &, M)
#define NODESIMULATIONITEM_RETURN_AS(T, M)     NODESIMULATIONITEM_BASE_RETURN_BODY(T,, M)
#define NODESIMULATIONITEM_RETURN(M)           NODESIMULATIONITEM_BASE_RETURN_BODY(,, M)

#define NODESMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<NodesModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<NodesModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<NodesModifierData4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodesModifierData4_3_0*>(data_ptr)-> C); 
#define NODESMODIFIERDATA_RETURN_REF(T, M)    NODESMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define NODESMODIFIERDATA_RETURN_AS(T, M)     NODESMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define NODESMODIFIERDATA_RETURN(M)           NODESMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define NODESMODIFIERSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodesModifierSettings3_6_0*>(data_ptr)-> C); 
#define NODESMODIFIERSETTINGS_RETURN_REF(T, M)    NODESMODIFIERSETTINGS_BASE_RETURN_BODY(T, &, M)
#define NODESMODIFIERSETTINGS_RETURN_AS(T, M)     NODESMODIFIERSETTINGS_BASE_RETURN_BODY(T,, M)
#define NODESMODIFIERSETTINGS_RETURN(M)           NODESMODIFIERSETTINGS_BASE_RETURN_BODY(,, M)

#define NODESUNBEAMS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeSunBeams3_6_0*>(data_ptr)-> C); 
#define NODESUNBEAMS_RETURN_REF(T, M)    NODESUNBEAMS_BASE_RETURN_BODY(T, &, M)
#define NODESUNBEAMS_RETURN_AS(T, M)     NODESUNBEAMS_BASE_RETURN_BODY(T,, M)
#define NODESUNBEAMS_RETURN(M)           NODESUNBEAMS_BASE_RETURN_BODY(,, M)

#define NODESWITCH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeSwitch3_6_0*>(data_ptr)-> C); 
#define NODESWITCH_RETURN_REF(T, M)    NODESWITCH_BASE_RETURN_BODY(T, &, M)
#define NODESWITCH_RETURN_AS(T, M)     NODESWITCH_BASE_RETURN_BODY(T,, M)
#define NODESWITCH_RETURN(M)           NODESWITCH_BASE_RETURN_BODY(,, M)

#define NODETONEMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeTonemap3_6_0*>(data_ptr)-> C); 
#define NODETONEMAP_RETURN_REF(T, M)    NODETONEMAP_BASE_RETURN_BODY(T, &, M)
#define NODETONEMAP_RETURN_AS(T, M)     NODETONEMAP_BASE_RETURN_BODY(T,, M)
#define NODETONEMAP_RETURN(M)           NODETONEMAP_BASE_RETURN_BODY(,, M)

#define NODETRACKPOSDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeTrackPosData3_6_0*>(data_ptr)-> C); 
#define NODETRACKPOSDATA_RETURN_REF(T, M)    NODETRACKPOSDATA_BASE_RETURN_BODY(T, &, M)
#define NODETRACKPOSDATA_RETURN_AS(T, M)     NODETRACKPOSDATA_BASE_RETURN_BODY(T,, M)
#define NODETRACKPOSDATA_RETURN(M)           NODETRACKPOSDATA_BASE_RETURN_BODY(,, M)

#define NODETRANSLATEDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<NodeTranslateData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NodeTranslateData4_2_0*>(data_ptr)-> C); 
#define NODETRANSLATEDATA_RETURN_REF(T, M)    NODETRANSLATEDATA_BASE_RETURN_BODY(T, &, M)
#define NODETRANSLATEDATA_RETURN_AS(T, M)     NODETRANSLATEDATA_BASE_RETURN_BODY(T,, M)
#define NODETRANSLATEDATA_RETURN(M)           NODETRANSLATEDATA_BASE_RETURN_BODY(,, M)

#define NODETWOFLOATS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeTwoFloats3_6_0*>(data_ptr)-> C); 
#define NODETWOFLOATS_RETURN_REF(T, M)    NODETWOFLOATS_BASE_RETURN_BODY(T, &, M)
#define NODETWOFLOATS_RETURN_AS(T, M)     NODETWOFLOATS_BASE_RETURN_BODY(T,, M)
#define NODETWOFLOATS_RETURN(M)           NODETWOFLOATS_BASE_RETURN_BODY(,, M)

#define NODETWOXYS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeTwoXYs3_6_0*>(data_ptr)-> C); 
#define NODETWOXYS_RETURN_REF(T, M)    NODETWOXYS_BASE_RETURN_BODY(T, &, M)
#define NODETWOXYS_RETURN_AS(T, M)     NODETWOXYS_BASE_RETURN_BODY(T,, M)
#define NODETWOXYS_RETURN(M)           NODETWOXYS_BASE_RETURN_BODY(,, M)

#define NODEVERTEXCOL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeVertexCol3_6_0*>(data_ptr)-> C); 
#define NODEVERTEXCOL_RETURN_REF(T, M)    NODEVERTEXCOL_BASE_RETURN_BODY(T, &, M)
#define NODEVERTEXCOL_RETURN_AS(T, M)     NODEVERTEXCOL_BASE_RETURN_BODY(T,, M)
#define NODEVERTEXCOL_RETURN(M)           NODEVERTEXCOL_BASE_RETURN_BODY(,, M)

#define NODEVIEWERPATHELEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NodeViewerPathElem3_6_0*>(data_ptr)-> C); 
#define NODEVIEWERPATHELEM_RETURN_REF(T, M)    NODEVIEWERPATHELEM_BASE_RETURN_BODY(T, &, M)
#define NODEVIEWERPATHELEM_RETURN_AS(T, M)     NODEVIEWERPATHELEM_BASE_RETURN_BODY(T,, M)
#define NODEVIEWERPATHELEM_RETURN(M)           NODEVIEWERPATHELEM_BASE_RETURN_BODY(,, M)

#define NOISEGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<NoiseGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define NOISEGPENCILMODIFIERDATA_RETURN_REF(T, M)    NOISEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define NOISEGPENCILMODIFIERDATA_RETURN_AS(T, M)     NOISEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define NOISEGPENCILMODIFIERDATA_RETURN(M)           NOISEGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define NORMALEDITMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<NormalEditModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<NormalEditModifierData4_1_0*>(data_ptr)-> C); 
#define NORMALEDITMODIFIERDATA_RETURN_REF(T, M)    NORMALEDITMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define NORMALEDITMODIFIERDATA_RETURN_AS(T, M)     NORMALEDITMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define NORMALEDITMODIFIERDATA_RETURN(M)           NORMALEDITMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define NURB_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Nurb3_6_0*>(data_ptr)-> C); 
#define NURB_RETURN_REF(T, M)    NURB_BASE_RETURN_BODY(T, &, M)
#define NURB_RETURN_AS(T, M)     NURB_BASE_RETURN_BODY(T,, M)
#define NURB_RETURN(M)           NURB_BASE_RETURN_BODY(,, M)

#define OBHOOK_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ObHook3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ObHook4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ObHook4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ObHook4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ObHook4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ObHook4_4_0*>(data_ptr)-> C); 
#define OBHOOK_RETURN_REF(T, M)    OBHOOK_BASE_RETURN_BODY(T, &, M)
#define OBHOOK_RETURN_AS(T, M)     OBHOOK_BASE_RETURN_BODY(T,, M)
#define OBHOOK_RETURN(M)           OBHOOK_BASE_RETURN_BODY(,, M)

#define OBJECTLINEART_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ObjectLineArt3_6_0*>(data_ptr)-> C); 
#define OBJECTLINEART_RETURN_REF(T, M)    OBJECTLINEART_BASE_RETURN_BODY(T, &, M)
#define OBJECTLINEART_RETURN_AS(T, M)     OBJECTLINEART_BASE_RETURN_BODY(T,, M)
#define OBJECTLINEART_RETURN(M)           OBJECTLINEART_BASE_RETURN_BODY(,, M)

#define OBJECT_BASE_RETURN_BODY(A, B, C) \
    if (get_compatability_mode() < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Object3_6_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Object4_0_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Object4_1_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Object4_2_0*>(data_ptr)-> C); \
    if (get_compatability_mode() < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Object4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Object4_4_0*>(data_ptr)-> C); 
#define OBJECT_RETURN_REF(T, M)    OBJECT_BASE_RETURN_BODY(T, &, M)
#define OBJECT_RETURN_AS(T, M)     OBJECT_BASE_RETURN_BODY(T,, M)
#define OBJECT_RETURN(M)           OBJECT_BASE_RETURN_BODY(,, M)

#define OBJECT_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Object_Runtime3_6_0*>(data_ptr)-> C); 
#define OBJECT_RUNTIME_RETURN_REF(T, M)    OBJECT_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define OBJECT_RUNTIME_RETURN_AS(T, M)     OBJECT_RUNTIME_BASE_RETURN_BODY(T,, M)
#define OBJECT_RUNTIME_RETURN(M)           OBJECT_RUNTIME_BASE_RETURN_BODY(,, M)

#define OCEANMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<OceanModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<OceanModifierData4_1_0*>(data_ptr)-> C); 
#define OCEANMODIFIERDATA_RETURN_REF(T, M)    OCEANMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define OCEANMODIFIERDATA_RETURN_AS(T, M)     OCEANMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define OCEANMODIFIERDATA_RETURN(M)           OCEANMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define OFFSETGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<OffsetGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define OFFSETGPENCILMODIFIERDATA_RETURN_REF(T, M)    OFFSETGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define OFFSETGPENCILMODIFIERDATA_RETURN_AS(T, M)     OFFSETGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define OFFSETGPENCILMODIFIERDATA_RETURN(M)           OFFSETGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define OPACITYGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<OpacityGpencilModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<OpacityGpencilModifierData4_0_0*>(data_ptr)-> C); 
#define OPACITYGPENCILMODIFIERDATA_RETURN_REF(T, M)    OPACITYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define OPACITYGPENCILMODIFIERDATA_RETURN_AS(T, M)     OPACITYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define OPACITYGPENCILMODIFIERDATA_RETURN(M)           OPACITYGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define ORIGSPACEFACE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<OrigSpaceFace3_6_0*>(data_ptr)-> C); 
#define ORIGSPACEFACE_RETURN_REF(T, M)    ORIGSPACEFACE_BASE_RETURN_BODY(T, &, M)
#define ORIGSPACEFACE_RETURN_AS(T, M)     ORIGSPACEFACE_BASE_RETURN_BODY(T,, M)
#define ORIGSPACEFACE_RETURN(M)           ORIGSPACEFACE_BASE_RETURN_BODY(,, M)

#define ORIGSPACELOOP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<OrigSpaceLoop3_6_0*>(data_ptr)-> C); 
#define ORIGSPACELOOP_RETURN_REF(T, M)    ORIGSPACELOOP_BASE_RETURN_BODY(T, &, M)
#define ORIGSPACELOOP_RETURN_AS(T, M)     ORIGSPACELOOP_BASE_RETURN_BODY(T,, M)
#define ORIGSPACELOOP_RETURN(M)           ORIGSPACELOOP_BASE_RETURN_BODY(,, M)

#define OUTLINEGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<OutlineGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define OUTLINEGPENCILMODIFIERDATA_RETURN_REF(T, M)    OUTLINEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define OUTLINEGPENCILMODIFIERDATA_RETURN_AS(T, M)     OUTLINEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define OUTLINEGPENCILMODIFIERDATA_RETURN(M)           OUTLINEGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define PACKEDFILE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<PackedFile3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PackedFile4_3_0*>(data_ptr)-> C); 
#define PACKEDFILE_RETURN_REF(T, M)    PACKEDFILE_BASE_RETURN_BODY(T, &, M)
#define PACKEDFILE_RETURN_AS(T, M)     PACKEDFILE_BASE_RETURN_BODY(T,, M)
#define PACKEDFILE_RETURN(M)           PACKEDFILE_BASE_RETURN_BODY(,, M)

#define PAINTCURVEPOINT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PaintCurvePoint3_6_0*>(data_ptr)-> C); 
#define PAINTCURVEPOINT_RETURN_REF(T, M)    PAINTCURVEPOINT_BASE_RETURN_BODY(T, &, M)
#define PAINTCURVEPOINT_RETURN_AS(T, M)     PAINTCURVEPOINT_BASE_RETURN_BODY(T,, M)
#define PAINTCURVEPOINT_RETURN(M)           PAINTCURVEPOINT_BASE_RETURN_BODY(,, M)

#define PAINTCURVE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<PaintCurve3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<PaintCurve4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<PaintCurve4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PaintCurve4_4_0*>(data_ptr)-> C); 
#define PAINTCURVE_RETURN_REF(T, M)    PAINTCURVE_BASE_RETURN_BODY(T, &, M)
#define PAINTCURVE_RETURN_AS(T, M)     PAINTCURVE_BASE_RETURN_BODY(T,, M)
#define PAINTCURVE_RETURN(M)           PAINTCURVE_BASE_RETURN_BODY(,, M)

#define PAINTMODESETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<PaintModeSettings3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<PaintModeSettings4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<PaintModeSettings4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PaintModeSettings4_4_0*>(data_ptr)-> C); 
#define PAINTMODESETTINGS_RETURN_REF(T, M)    PAINTMODESETTINGS_BASE_RETURN_BODY(T, &, M)
#define PAINTMODESETTINGS_RETURN_AS(T, M)     PAINTMODESETTINGS_BASE_RETURN_BODY(T,, M)
#define PAINTMODESETTINGS_RETURN(M)           PAINTMODESETTINGS_BASE_RETURN_BODY(,, M)

#define PAINTTOOLSLOT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PaintToolSlot3_6_0*>(data_ptr)-> C); 
#define PAINTTOOLSLOT_RETURN_REF(T, M)    PAINTTOOLSLOT_BASE_RETURN_BODY(T, &, M)
#define PAINTTOOLSLOT_RETURN_AS(T, M)     PAINTTOOLSLOT_BASE_RETURN_BODY(T,, M)
#define PAINTTOOLSLOT_RETURN(M)           PAINTTOOLSLOT_BASE_RETURN_BODY(,, M)

#define PAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Paint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Paint4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Paint4_3_0*>(data_ptr)-> C); 
#define PAINT_RETURN_REF(T, M)    PAINT_BASE_RETURN_BODY(T, &, M)
#define PAINT_RETURN_AS(T, M)     PAINT_BASE_RETURN_BODY(T,, M)
#define PAINT_RETURN(M)           PAINT_BASE_RETURN_BODY(,, M)

#define PAINT_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Paint_Runtime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Paint_Runtime4_3_0*>(data_ptr)-> C); 
#define PAINT_RUNTIME_RETURN_REF(T, M)    PAINT_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define PAINT_RUNTIME_RETURN_AS(T, M)     PAINT_RUNTIME_BASE_RETURN_BODY(T,, M)
#define PAINT_RUNTIME_RETURN(M)           PAINT_RUNTIME_BASE_RETURN_BODY(,, M)

#define PALETTECOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PaletteColor3_6_0*>(data_ptr)-> C); 
#define PALETTECOLOR_RETURN_REF(T, M)    PALETTECOLOR_BASE_RETURN_BODY(T, &, M)
#define PALETTECOLOR_RETURN_AS(T, M)     PALETTECOLOR_BASE_RETURN_BODY(T,, M)
#define PALETTECOLOR_RETURN(M)           PALETTECOLOR_BASE_RETURN_BODY(,, M)

#define PALETTE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Palette3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Palette4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Palette4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Palette4_4_0*>(data_ptr)-> C); 
#define PALETTE_RETURN_REF(T, M)    PALETTE_BASE_RETURN_BODY(T, &, M)
#define PALETTE_RETURN_AS(T, M)     PALETTE_BASE_RETURN_BODY(T,, M)
#define PALETTE_RETURN(M)           PALETTE_BASE_RETURN_BODY(,, M)

#define PANELCATEGORYDYN_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PanelCategoryDyn3_6_0*>(data_ptr)-> C); 
#define PANELCATEGORYDYN_RETURN_REF(T, M)    PANELCATEGORYDYN_BASE_RETURN_BODY(T, &, M)
#define PANELCATEGORYDYN_RETURN_AS(T, M)     PANELCATEGORYDYN_BASE_RETURN_BODY(T,, M)
#define PANELCATEGORYDYN_RETURN(M)           PANELCATEGORYDYN_BASE_RETURN_BODY(,, M)

#define PANELCATEGORYSTACK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PanelCategoryStack3_6_0*>(data_ptr)-> C); 
#define PANELCATEGORYSTACK_RETURN_REF(T, M)    PANELCATEGORYSTACK_BASE_RETURN_BODY(T, &, M)
#define PANELCATEGORYSTACK_RETURN_AS(T, M)     PANELCATEGORYSTACK_BASE_RETURN_BODY(T,, M)
#define PANELCATEGORYSTACK_RETURN(M)           PANELCATEGORYSTACK_BASE_RETURN_BODY(,, M)

#define PANEL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Panel3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Panel4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Panel4_1_0*>(data_ptr)-> C); 
#define PANEL_RETURN_REF(T, M)    PANEL_BASE_RETURN_BODY(T, &, M)
#define PANEL_RETURN_AS(T, M)     PANEL_BASE_RETURN_BODY(T,, M)
#define PANEL_RETURN(M)           PANEL_BASE_RETURN_BODY(,, M)

#define PANEL_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Panel_Runtime3_6_0*>(data_ptr)-> C); 
#define PANEL_RUNTIME_RETURN_REF(T, M)    PANEL_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define PANEL_RUNTIME_RETURN_AS(T, M)     PANEL_RUNTIME_BASE_RETURN_BODY(T,, M)
#define PANEL_RUNTIME_RETURN(M)           PANEL_RUNTIME_BASE_RETURN_BODY(,, M)

#define PARTDEFLECT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<PartDeflect3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PartDeflect4_2_0*>(data_ptr)-> C); 
#define PARTDEFLECT_RETURN_REF(T, M)    PARTDEFLECT_BASE_RETURN_BODY(T, &, M)
#define PARTDEFLECT_RETURN_AS(T, M)     PARTDEFLECT_BASE_RETURN_BODY(T,, M)
#define PARTDEFLECT_RETURN(M)           PARTDEFLECT_BASE_RETURN_BODY(,, M)

#define PARTEFF_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PartEff3_6_0*>(data_ptr)-> C); 
#define PARTEFF_RETURN_REF(T, M)    PARTEFF_BASE_RETURN_BODY(T, &, M)
#define PARTEFF_RETURN_AS(T, M)     PARTEFF_BASE_RETURN_BODY(T,, M)
#define PARTEFF_RETURN(M)           PARTEFF_BASE_RETURN_BODY(,, M)

#define PARTICLEBRUSHDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ParticleBrushData3_6_0*>(data_ptr)-> C); 
#define PARTICLEBRUSHDATA_RETURN_REF(T, M)    PARTICLEBRUSHDATA_BASE_RETURN_BODY(T, &, M)
#define PARTICLEBRUSHDATA_RETURN_AS(T, M)     PARTICLEBRUSHDATA_BASE_RETURN_BODY(T,, M)
#define PARTICLEBRUSHDATA_RETURN(M)           PARTICLEBRUSHDATA_BASE_RETURN_BODY(,, M)

#define PARTICLEDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ParticleData3_6_0*>(data_ptr)-> C); 
#define PARTICLEDATA_RETURN_REF(T, M)    PARTICLEDATA_BASE_RETURN_BODY(T, &, M)
#define PARTICLEDATA_RETURN_AS(T, M)     PARTICLEDATA_BASE_RETURN_BODY(T,, M)
#define PARTICLEDATA_RETURN(M)           PARTICLEDATA_BASE_RETURN_BODY(,, M)

#define PARTICLEDUPLIWEIGHT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ParticleDupliWeight3_6_0*>(data_ptr)-> C); 
#define PARTICLEDUPLIWEIGHT_RETURN_REF(T, M)    PARTICLEDUPLIWEIGHT_BASE_RETURN_BODY(T, &, M)
#define PARTICLEDUPLIWEIGHT_RETURN_AS(T, M)     PARTICLEDUPLIWEIGHT_BASE_RETURN_BODY(T,, M)
#define PARTICLEDUPLIWEIGHT_RETURN(M)           PARTICLEDUPLIWEIGHT_BASE_RETURN_BODY(,, M)

#define PARTICLEINSTANCEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ParticleInstanceModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ParticleInstanceModifierData4_1_0*>(data_ptr)-> C); 
#define PARTICLEINSTANCEMODIFIERDATA_RETURN_REF(T, M)    PARTICLEINSTANCEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define PARTICLEINSTANCEMODIFIERDATA_RETURN_AS(T, M)     PARTICLEINSTANCEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define PARTICLEINSTANCEMODIFIERDATA_RETURN(M)           PARTICLEINSTANCEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define PARTICLEKEY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ParticleKey3_6_0*>(data_ptr)-> C); 
#define PARTICLEKEY_RETURN_REF(T, M)    PARTICLEKEY_BASE_RETURN_BODY(T, &, M)
#define PARTICLEKEY_RETURN_AS(T, M)     PARTICLEKEY_BASE_RETURN_BODY(T,, M)
#define PARTICLEKEY_RETURN(M)           PARTICLEKEY_BASE_RETURN_BODY(,, M)

#define PARTICLESETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ParticleSettings3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ParticleSettings4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ParticleSettings4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ParticleSettings4_4_0*>(data_ptr)-> C); 
#define PARTICLESETTINGS_RETURN_REF(T, M)    PARTICLESETTINGS_BASE_RETURN_BODY(T, &, M)
#define PARTICLESETTINGS_RETURN_AS(T, M)     PARTICLESETTINGS_BASE_RETURN_BODY(T,, M)
#define PARTICLESETTINGS_RETURN(M)           PARTICLESETTINGS_BASE_RETURN_BODY(,, M)

#define PARTICLESPRING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ParticleSpring3_6_0*>(data_ptr)-> C); 
#define PARTICLESPRING_RETURN_REF(T, M)    PARTICLESPRING_BASE_RETURN_BODY(T, &, M)
#define PARTICLESPRING_RETURN_AS(T, M)     PARTICLESPRING_BASE_RETURN_BODY(T,, M)
#define PARTICLESPRING_RETURN(M)           PARTICLESPRING_BASE_RETURN_BODY(,, M)

#define PARTICLESYSTEMMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ParticleSystemModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ParticleSystemModifierData4_1_0*>(data_ptr)-> C); 
#define PARTICLESYSTEMMODIFIERDATA_RETURN_REF(T, M)    PARTICLESYSTEMMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define PARTICLESYSTEMMODIFIERDATA_RETURN_AS(T, M)     PARTICLESYSTEMMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define PARTICLESYSTEMMODIFIERDATA_RETURN(M)           PARTICLESYSTEMMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define PARTICLETARGET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ParticleTarget3_6_0*>(data_ptr)-> C); 
#define PARTICLETARGET_RETURN_REF(T, M)    PARTICLETARGET_BASE_RETURN_BODY(T, &, M)
#define PARTICLETARGET_RETURN_AS(T, M)     PARTICLETARGET_BASE_RETURN_BODY(T,, M)
#define PARTICLETARGET_RETURN(M)           PARTICLETARGET_BASE_RETURN_BODY(,, M)

#define PARTICLE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Particle3_6_0*>(data_ptr)-> C); 
#define PARTICLE_RETURN_REF(T, M)    PARTICLE_BASE_RETURN_BODY(T, &, M)
#define PARTICLE_RETURN_AS(T, M)     PARTICLE_BASE_RETURN_BODY(T,, M)
#define PARTICLE_RETURN(M)           PARTICLE_BASE_RETURN_BODY(,, M)

#define PHYSICSSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PhysicsSettings3_6_0*>(data_ptr)-> C); 
#define PHYSICSSETTINGS_RETURN_REF(T, M)    PHYSICSSETTINGS_BASE_RETURN_BODY(T, &, M)
#define PHYSICSSETTINGS_RETURN_AS(T, M)     PHYSICSSETTINGS_BASE_RETURN_BODY(T,, M)
#define PHYSICSSETTINGS_RETURN(M)           PHYSICSSETTINGS_BASE_RETURN_BODY(,, M)

#define PIXELSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PixelShaderFxData3_6_0*>(data_ptr)-> C); 
#define PIXELSHADERFXDATA_RETURN_REF(T, M)    PIXELSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define PIXELSHADERFXDATA_RETURN_AS(T, M)     PIXELSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define PIXELSHADERFXDATA_RETURN(M)           PIXELSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define POINTCLOUD_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<PointCloud3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<PointCloud4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<PointCloud4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<PointCloud4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<PointCloud4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PointCloud4_4_0*>(data_ptr)-> C); 
#define POINTCLOUD_RETURN_REF(T, M)    POINTCLOUD_BASE_RETURN_BODY(T, &, M)
#define POINTCLOUD_RETURN_AS(T, M)     POINTCLOUD_BASE_RETURN_BODY(T,, M)
#define POINTCLOUD_RETURN(M)           POINTCLOUD_BASE_RETURN_BODY(,, M)

#define POINTDENSITY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<PointDensity3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<PointDensity4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<PointDensity4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<PointDensity4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<PointDensity4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PointDensity4_4_0*>(data_ptr)-> C); 
#define POINTDENSITY_RETURN_REF(T, M)    POINTDENSITY_BASE_RETURN_BODY(T, &, M)
#define POINTDENSITY_RETURN_AS(T, M)     POINTDENSITY_BASE_RETURN_BODY(T,, M)
#define POINTDENSITY_RETURN(M)           POINTDENSITY_BASE_RETURN_BODY(,, M)

#define PREVIEWIMAGE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<PreviewImage3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<PreviewImage4_2_0*>(data_ptr)-> C); 
#define PREVIEWIMAGE_RETURN_REF(T, M)    PREVIEWIMAGE_BASE_RETURN_BODY(T, &, M)
#define PREVIEWIMAGE_RETURN_AS(T, M)     PREVIEWIMAGE_BASE_RETURN_BODY(T,, M)
#define PREVIEWIMAGE_RETURN(M)           PREVIEWIMAGE_BASE_RETURN_BODY(,, M)

#define PTCACHEEXTRA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PTCacheExtra3_6_0*>(data_ptr)-> C); 
#define PTCACHEEXTRA_RETURN_REF(T, M)    PTCACHEEXTRA_BASE_RETURN_BODY(T, &, M)
#define PTCACHEEXTRA_RETURN_AS(T, M)     PTCACHEEXTRA_BASE_RETURN_BODY(T,, M)
#define PTCACHEEXTRA_RETURN(M)           PTCACHEEXTRA_BASE_RETURN_BODY(,, M)

#define PTCACHEMEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<PTCacheMem3_6_0*>(data_ptr)-> C); 
#define PTCACHEMEM_RETURN_REF(T, M)    PTCACHEMEM_BASE_RETURN_BODY(T, &, M)
#define PTCACHEMEM_RETURN_AS(T, M)     PTCACHEMEM_BASE_RETURN_BODY(T,, M)
#define PTCACHEMEM_RETURN(M)           PTCACHEMEM_BASE_RETURN_BODY(,, M)

#define RCTF_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<rctf3_6_0*>(data_ptr)-> C); 
#define RCTF_RETURN_REF(T, M)    RCTF_BASE_RETURN_BODY(T, &, M)
#define RCTF_RETURN_AS(T, M)     RCTF_BASE_RETURN_BODY(T,, M)
#define RCTF_RETURN(M)           RCTF_BASE_RETURN_BODY(,, M)

#define RCTI_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<rcti3_6_0*>(data_ptr)-> C); 
#define RCTI_RETURN_REF(T, M)    RCTI_BASE_RETURN_BODY(T, &, M)
#define RCTI_RETURN_AS(T, M)     RCTI_BASE_RETURN_BODY(T,, M)
#define RCTI_RETURN(M)           RCTI_BASE_RETURN_BODY(,, M)

#define REGIONVIEW3D_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<RegionView3D3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<RegionView3D4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<RegionView3D4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<RegionView3D4_4_0*>(data_ptr)-> C); 
#define REGIONVIEW3D_RETURN_REF(T, M)    REGIONVIEW3D_BASE_RETURN_BODY(T, &, M)
#define REGIONVIEW3D_RETURN_AS(T, M)     REGIONVIEW3D_BASE_RETURN_BODY(T,, M)
#define REGIONVIEW3D_RETURN(M)           REGIONVIEW3D_BASE_RETURN_BODY(,, M)

#define REMESHMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<RemeshModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<RemeshModifierData4_1_0*>(data_ptr)-> C); 
#define REMESHMODIFIERDATA_RETURN_REF(T, M)    REMESHMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define REMESHMODIFIERDATA_RETURN_AS(T, M)     REMESHMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define REMESHMODIFIERDATA_RETURN(M)           REMESHMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define RENDERPROFILE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<RenderProfile3_6_0*>(data_ptr)-> C); 
#define RENDERPROFILE_RETURN_REF(T, M)    RENDERPROFILE_BASE_RETURN_BODY(T, &, M)
#define RENDERPROFILE_RETURN_AS(T, M)     RENDERPROFILE_BASE_RETURN_BODY(T,, M)
#define RENDERPROFILE_RETURN(M)           RENDERPROFILE_BASE_RETURN_BODY(,, M)

#define RENDERSLOT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<RenderSlot3_6_0*>(data_ptr)-> C); 
#define RENDERSLOT_RETURN_REF(T, M)    RENDERSLOT_BASE_RETURN_BODY(T, &, M)
#define RENDERSLOT_RETURN_AS(T, M)     RENDERSLOT_BASE_RETURN_BODY(T,, M)
#define RENDERSLOT_RETURN(M)           RENDERSLOT_BASE_RETURN_BODY(,, M)

#define REPORTLIST_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ReportList3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ReportList4_1_0*>(data_ptr)-> C); 
#define REPORTLIST_RETURN_REF(T, M)    REPORTLIST_BASE_RETURN_BODY(T, &, M)
#define REPORTLIST_RETURN_AS(T, M)     REPORTLIST_BASE_RETURN_BODY(T,, M)
#define REPORTLIST_RETURN(M)           REPORTLIST_BASE_RETURN_BODY(,, M)

#define REPORTTIMERINFO_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ReportTimerInfo3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ReportTimerInfo4_0_0*>(data_ptr)-> C); 
#define REPORTTIMERINFO_RETURN_REF(T, M)    REPORTTIMERINFO_BASE_RETURN_BODY(T, &, M)
#define REPORTTIMERINFO_RETURN_AS(T, M)     REPORTTIMERINFO_BASE_RETURN_BODY(T,, M)
#define REPORTTIMERINFO_RETURN(M)           REPORTTIMERINFO_BASE_RETURN_BODY(,, M)

#define REPORT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Report3_6_0*>(data_ptr)-> C); 
#define REPORT_RETURN_REF(T, M)    REPORT_BASE_RETURN_BODY(T, &, M)
#define REPORT_RETURN_AS(T, M)     REPORT_BASE_RETURN_BODY(T,, M)
#define REPORT_RETURN(M)           REPORT_BASE_RETURN_BODY(,, M)

#define RIGIDBODYCON_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<RigidBodyCon3_6_0*>(data_ptr)-> C); 
#define RIGIDBODYCON_RETURN_REF(T, M)    RIGIDBODYCON_BASE_RETURN_BODY(T, &, M)
#define RIGIDBODYCON_RETURN_AS(T, M)     RIGIDBODYCON_BASE_RETURN_BODY(T,, M)
#define RIGIDBODYCON_RETURN(M)           RIGIDBODYCON_BASE_RETURN_BODY(,, M)

#define RIGIDBODYOB_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<RigidBodyOb3_6_0*>(data_ptr)-> C); 
#define RIGIDBODYOB_RETURN_REF(T, M)    RIGIDBODYOB_BASE_RETURN_BODY(T, &, M)
#define RIGIDBODYOB_RETURN_AS(T, M)     RIGIDBODYOB_BASE_RETURN_BODY(T,, M)
#define RIGIDBODYOB_RETURN(M)           RIGIDBODYOB_BASE_RETURN_BODY(,, M)

#define RIGIDBODYOB_SHARED_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<RigidBodyOb_Shared3_6_0*>(data_ptr)-> C); 
#define RIGIDBODYOB_SHARED_RETURN_REF(T, M)    RIGIDBODYOB_SHARED_BASE_RETURN_BODY(T, &, M)
#define RIGIDBODYOB_SHARED_RETURN_AS(T, M)     RIGIDBODYOB_SHARED_BASE_RETURN_BODY(T,, M)
#define RIGIDBODYOB_SHARED_RETURN(M)           RIGIDBODYOB_SHARED_BASE_RETURN_BODY(,, M)

#define RIGIDBODYWORLD_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<RigidBodyWorld3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<RigidBodyWorld4_4_0*>(data_ptr)-> C); 
#define RIGIDBODYWORLD_RETURN_REF(T, M)    RIGIDBODYWORLD_BASE_RETURN_BODY(T, &, M)
#define RIGIDBODYWORLD_RETURN_AS(T, M)     RIGIDBODYWORLD_BASE_RETURN_BODY(T,, M)
#define RIGIDBODYWORLD_RETURN(M)           RIGIDBODYWORLD_BASE_RETURN_BODY(,, M)

#define RIGIDBODYWORLD_SHARED_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<RigidBodyWorld_Shared3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<RigidBodyWorld_Shared4_4_0*>(data_ptr)-> C); 
#define RIGIDBODYWORLD_SHARED_RETURN_REF(T, M)    RIGIDBODYWORLD_SHARED_BASE_RETURN_BODY(T, &, M)
#define RIGIDBODYWORLD_SHARED_RETURN_AS(T, M)     RIGIDBODYWORLD_SHARED_BASE_RETURN_BODY(T,, M)
#define RIGIDBODYWORLD_SHARED_RETURN(M)           RIGIDBODYWORLD_SHARED_BASE_RETURN_BODY(,, M)

#define RIMSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<RimShaderFxData3_6_0*>(data_ptr)-> C); 
#define RIMSHADERFXDATA_RETURN_REF(T, M)    RIMSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define RIMSHADERFXDATA_RETURN_AS(T, M)     RIMSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define RIMSHADERFXDATA_RETURN(M)           RIMSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define SBVERTEX_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SBVertex3_6_0*>(data_ptr)-> C); 
#define SBVERTEX_RETURN_REF(T, M)    SBVERTEX_BASE_RETURN_BODY(T, &, M)
#define SBVERTEX_RETURN_AS(T, M)     SBVERTEX_BASE_RETURN_BODY(T,, M)
#define SBVERTEX_RETURN(M)           SBVERTEX_BASE_RETURN_BODY(,, M)

#define SCENECOLLECTION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SceneCollection3_6_0*>(data_ptr)-> C); 
#define SCENECOLLECTION_RETURN_REF(T, M)    SCENECOLLECTION_BASE_RETURN_BODY(T, &, M)
#define SCENECOLLECTION_RETURN_AS(T, M)     SCENECOLLECTION_BASE_RETURN_BODY(T,, M)
#define SCENECOLLECTION_RETURN(M)           SCENECOLLECTION_BASE_RETURN_BODY(,, M)

#define SCENEDISPLAY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SceneDisplay3_6_0*>(data_ptr)-> C); 
#define SCENEDISPLAY_RETURN_REF(T, M)    SCENEDISPLAY_BASE_RETURN_BODY(T, &, M)
#define SCENEDISPLAY_RETURN_AS(T, M)     SCENEDISPLAY_BASE_RETURN_BODY(T,, M)
#define SCENEDISPLAY_RETURN(M)           SCENEDISPLAY_BASE_RETURN_BODY(,, M)

#define SCENEEEVEE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SceneEEVEE3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SceneEEVEE4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SceneEEVEE4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SceneEEVEE4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SceneEEVEE4_3_0*>(data_ptr)-> C); 
#define SCENEEEVEE_RETURN_REF(T, M)    SCENEEEVEE_BASE_RETURN_BODY(T, &, M)
#define SCENEEEVEE_RETURN_AS(T, M)     SCENEEEVEE_BASE_RETURN_BODY(T,, M)
#define SCENEEEVEE_RETURN(M)           SCENEEEVEE_BASE_RETURN_BODY(,, M)

#define SCENEGPENCIL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SceneGpencil3_6_0*>(data_ptr)-> C); 
#define SCENEGPENCIL_RETURN_REF(T, M)    SCENEGPENCIL_BASE_RETURN_BODY(T, &, M)
#define SCENEGPENCIL_RETURN_AS(T, M)     SCENEGPENCIL_BASE_RETURN_BODY(T,, M)
#define SCENEGPENCIL_RETURN(M)           SCENEGPENCIL_BASE_RETURN_BODY(,, M)

#define SCENERENDERLAYER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SceneRenderLayer3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SceneRenderLayer4_2_0*>(data_ptr)-> C); 
#define SCENERENDERLAYER_RETURN_REF(T, M)    SCENERENDERLAYER_BASE_RETURN_BODY(T, &, M)
#define SCENERENDERLAYER_RETURN_AS(T, M)     SCENERENDERLAYER_BASE_RETURN_BODY(T,, M)
#define SCENERENDERLAYER_RETURN(M)           SCENERENDERLAYER_BASE_RETURN_BODY(,, M)

#define SCENERENDERVIEW_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SceneRenderView3_6_0*>(data_ptr)-> C); 
#define SCENERENDERVIEW_RETURN_REF(T, M)    SCENERENDERVIEW_BASE_RETURN_BODY(T, &, M)
#define SCENERENDERVIEW_RETURN_AS(T, M)     SCENERENDERVIEW_BASE_RETURN_BODY(T,, M)
#define SCENERENDERVIEW_RETURN(M)           SCENERENDERVIEW_BASE_RETURN_BODY(,, M)

#define SCOPES_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Scopes3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Scopes4_1_0*>(data_ptr)-> C); 
#define SCOPES_RETURN_REF(T, M)    SCOPES_BASE_RETURN_BODY(T, &, M)
#define SCOPES_RETURN_AS(T, M)     SCOPES_BASE_RETURN_BODY(T,, M)
#define SCOPES_RETURN(M)           SCOPES_BASE_RETURN_BODY(,, M)

#define SCRAREAMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ScrAreaMap3_6_0*>(data_ptr)-> C); 
#define SCRAREAMAP_RETURN_REF(T, M)    SCRAREAMAP_BASE_RETURN_BODY(T, &, M)
#define SCRAREAMAP_RETURN_AS(T, M)     SCRAREAMAP_BASE_RETURN_BODY(T,, M)
#define SCRAREAMAP_RETURN(M)           SCRAREAMAP_BASE_RETURN_BODY(,, M)

#define SCRAREA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ScrArea3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ScrArea4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ScrArea4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ScrArea4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ScrArea4_4_0*>(data_ptr)-> C); 
#define SCRAREA_RETURN_REF(T, M)    SCRAREA_BASE_RETURN_BODY(T, &, M)
#define SCRAREA_RETURN_AS(T, M)     SCRAREA_BASE_RETURN_BODY(T,, M)
#define SCRAREA_RETURN(M)           SCRAREA_BASE_RETURN_BODY(,, M)

#define SCRAREA_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ScrArea_Runtime3_6_0*>(data_ptr)-> C); 
#define SCRAREA_RUNTIME_RETURN_REF(T, M)    SCRAREA_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define SCRAREA_RUNTIME_RETURN_AS(T, M)     SCRAREA_RUNTIME_BASE_RETURN_BODY(T,, M)
#define SCRAREA_RUNTIME_RETURN(M)           SCRAREA_RUNTIME_BASE_RETURN_BODY(,, M)

#define SCREDGE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ScrEdge3_6_0*>(data_ptr)-> C); 
#define SCREDGE_RETURN_REF(T, M)    SCREDGE_BASE_RETURN_BODY(T, &, M)
#define SCREDGE_RETURN_AS(T, M)     SCREDGE_BASE_RETURN_BODY(T,, M)
#define SCREDGE_RETURN(M)           SCREDGE_BASE_RETURN_BODY(,, M)

#define SCREWMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ScrewModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ScrewModifierData4_1_0*>(data_ptr)-> C); 
#define SCREWMODIFIERDATA_RETURN_REF(T, M)    SCREWMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SCREWMODIFIERDATA_RETURN_AS(T, M)     SCREWMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SCREWMODIFIERDATA_RETURN(M)           SCREWMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SCRGLOBALAREADATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ScrGlobalAreaData3_6_0*>(data_ptr)-> C); 
#define SCRGLOBALAREADATA_RETURN_REF(T, M)    SCRGLOBALAREADATA_BASE_RETURN_BODY(T, &, M)
#define SCRGLOBALAREADATA_RETURN_AS(T, M)     SCRGLOBALAREADATA_BASE_RETURN_BODY(T,, M)
#define SCRGLOBALAREADATA_RETURN(M)           SCRGLOBALAREADATA_BASE_RETURN_BODY(,, M)

#define SCRIPT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Script3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Script4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Script4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Script4_4_0*>(data_ptr)-> C); 
#define SCRIPT_RETURN_REF(T, M)    SCRIPT_BASE_RETURN_BODY(T, &, M)
#define SCRIPT_RETURN_AS(T, M)     SCRIPT_BASE_RETURN_BODY(T,, M)
#define SCRIPT_RETURN(M)           SCRIPT_BASE_RETURN_BODY(,, M)

#define SCRVERT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ScrVert3_6_0*>(data_ptr)-> C); 
#define SCRVERT_RETURN_REF(T, M)    SCRVERT_BASE_RETURN_BODY(T, &, M)
#define SCRVERT_RETURN_AS(T, M)     SCRVERT_BASE_RETURN_BODY(T,, M)
#define SCRVERT_RETURN(M)           SCRVERT_BASE_RETURN_BODY(,, M)

#define SCULPT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Sculpt3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Sculpt4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Sculpt4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Sculpt4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Sculpt4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Sculpt4_4_0*>(data_ptr)-> C); 
#define SCULPT_RETURN_REF(T, M)    SCULPT_BASE_RETURN_BODY(T, &, M)
#define SCULPT_RETURN_AS(T, M)     SCULPT_BASE_RETURN_BODY(T,, M)
#define SCULPT_RETURN(M)           SCULPT_BASE_RETURN_BODY(,, M)

#define SDEFBIND_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SDefBind3_6_0*>(data_ptr)-> C); 
#define SDEFBIND_RETURN_REF(T, M)    SDEFBIND_BASE_RETURN_BODY(T, &, M)
#define SDEFBIND_RETURN_AS(T, M)     SDEFBIND_BASE_RETURN_BODY(T,, M)
#define SDEFBIND_RETURN(M)           SDEFBIND_BASE_RETURN_BODY(,, M)

#define SDEFVERT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SDefVert3_6_0*>(data_ptr)-> C); 
#define SDEFVERT_RETURN_REF(T, M)    SDEFVERT_BASE_RETURN_BODY(T, &, M)
#define SDEFVERT_RETURN_AS(T, M)     SDEFVERT_BASE_RETURN_BODY(T,, M)
#define SDEFVERT_RETURN(M)           SDEFVERT_BASE_RETURN_BODY(,, M)

#define SDNA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SDNA4_2_8*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SDNA4_4_0*>(data_ptr)-> C); 
#define SDNA_RETURN_REF(T, M)    SDNA_BASE_RETURN_BODY(T, &, M)
#define SDNA_RETURN_AS(T, M)     SDNA_BASE_RETURN_BODY(T,, M)
#define SDNA_RETURN(M)           SDNA_BASE_RETURN_BODY(,, M)

#define SDNA_STRUCTMEMBER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SDNA_StructMember3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SDNA_StructMember4_3_0*>(data_ptr)-> C); 
#define SDNA_STRUCTMEMBER_RETURN_REF(T, M)    SDNA_STRUCTMEMBER_BASE_RETURN_BODY(T, &, M)
#define SDNA_STRUCTMEMBER_RETURN_AS(T, M)     SDNA_STRUCTMEMBER_BASE_RETURN_BODY(T,, M)
#define SDNA_STRUCTMEMBER_RETURN(M)           SDNA_STRUCTMEMBER_BASE_RETURN_BODY(,, M)

#define SEQTIMELINECHANNEL_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SeqTimelineChannel3_6_0*>(data_ptr)-> C); 
#define SEQTIMELINECHANNEL_RETURN_REF(T, M)    SEQTIMELINECHANNEL_BASE_RETURN_BODY(T, &, M)
#define SEQTIMELINECHANNEL_RETURN_AS(T, M)     SEQTIMELINECHANNEL_BASE_RETURN_BODY(T,, M)
#define SEQTIMELINECHANNEL_RETURN(M)           SEQTIMELINECHANNEL_BASE_RETURN_BODY(,, M)

#define SEQUENCEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SequenceModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SequenceModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SequenceModifierData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SequenceModifierData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SequenceModifierData4_4_0*>(data_ptr)-> C); 
#define SEQUENCEMODIFIERDATA_RETURN_REF(T, M)    SEQUENCEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SEQUENCEMODIFIERDATA_RETURN_AS(T, M)     SEQUENCEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SEQUENCEMODIFIERDATA_RETURN(M)           SEQUENCEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SEQUENCERMASKMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SequencerMaskModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SequencerMaskModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SequencerMaskModifierData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SequencerMaskModifierData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SequencerMaskModifierData4_4_0*>(data_ptr)-> C); 
#define SEQUENCERMASKMODIFIERDATA_RETURN_REF(T, M)    SEQUENCERMASKMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERMASKMODIFIERDATA_RETURN_AS(T, M)     SEQUENCERMASKMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SEQUENCERMASKMODIFIERDATA_RETURN(M)           SEQUENCERMASKMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SEQUENCERPREVIEWOVERLAY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SequencerPreviewOverlay3_6_0*>(data_ptr)-> C); 
#define SEQUENCERPREVIEWOVERLAY_RETURN_REF(T, M)    SEQUENCERPREVIEWOVERLAY_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERPREVIEWOVERLAY_RETURN_AS(T, M)     SEQUENCERPREVIEWOVERLAY_BASE_RETURN_BODY(T,, M)
#define SEQUENCERPREVIEWOVERLAY_RETURN(M)           SEQUENCERPREVIEWOVERLAY_BASE_RETURN_BODY(,, M)

#define SEQUENCERSCOPES_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SequencerScopes3_6_0*>(data_ptr)-> C); 
#define SEQUENCERSCOPES_RETURN_REF(T, M)    SEQUENCERSCOPES_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERSCOPES_RETURN_AS(T, M)     SEQUENCERSCOPES_BASE_RETURN_BODY(T,, M)
#define SEQUENCERSCOPES_RETURN(M)           SEQUENCERSCOPES_BASE_RETURN_BODY(,, M)

#define SEQUENCERTIMELINEOVERLAY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SequencerTimelineOverlay3_6_0*>(data_ptr)-> C); 
#define SEQUENCERTIMELINEOVERLAY_RETURN_REF(T, M)    SEQUENCERTIMELINEOVERLAY_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERTIMELINEOVERLAY_RETURN_AS(T, M)     SEQUENCERTIMELINEOVERLAY_BASE_RETURN_BODY(T,, M)
#define SEQUENCERTIMELINEOVERLAY_RETURN(M)           SEQUENCERTIMELINEOVERLAY_BASE_RETURN_BODY(,, M)

#define SEQUENCERTONEMAPMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SequencerTonemapModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SequencerTonemapModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SequencerTonemapModifierData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SequencerTonemapModifierData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SequencerTonemapModifierData4_4_0*>(data_ptr)-> C); 
#define SEQUENCERTONEMAPMODIFIERDATA_RETURN_REF(T, M)    SEQUENCERTONEMAPMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERTONEMAPMODIFIERDATA_RETURN_AS(T, M)     SEQUENCERTONEMAPMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SEQUENCERTONEMAPMODIFIERDATA_RETURN(M)           SEQUENCERTONEMAPMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SEQUENCERTOOLSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SequencerToolSettings3_6_0*>(data_ptr)-> C); 
#define SEQUENCERTOOLSETTINGS_RETURN_REF(T, M)    SEQUENCERTOOLSETTINGS_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERTOOLSETTINGS_RETURN_AS(T, M)     SEQUENCERTOOLSETTINGS_BASE_RETURN_BODY(T,, M)
#define SEQUENCERTOOLSETTINGS_RETURN(M)           SEQUENCERTOOLSETTINGS_BASE_RETURN_BODY(,, M)

#define SEQUENCERUNTIME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SequenceRuntime3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SequenceRuntime4_1_0*>(data_ptr)-> C); 
#define SEQUENCERUNTIME_RETURN_REF(T, M)    SEQUENCERUNTIME_BASE_RETURN_BODY(T, &, M)
#define SEQUENCERUNTIME_RETURN_AS(T, M)     SEQUENCERUNTIME_BASE_RETURN_BODY(T,, M)
#define SEQUENCERUNTIME_RETURN(M)           SEQUENCERUNTIME_BASE_RETURN_BODY(,, M)

#define SEQUENCE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<Sequence3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Sequence4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<Sequence4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Sequence4_3_0*>(data_ptr)-> C); 
#define SEQUENCE_RETURN_REF(T, M)    SEQUENCE_BASE_RETURN_BODY(T, &, M)
#define SEQUENCE_RETURN_AS(T, M)     SEQUENCE_BASE_RETURN_BODY(T,, M)
#define SEQUENCE_RETURN(M)           SEQUENCE_BASE_RETURN_BODY(,, M)

#define SESSIONUUID_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SessionUUID3_6_0*>(data_ptr)-> C); 
#define SESSIONUUID_RETURN_REF(T, M)    SESSIONUUID_BASE_RETURN_BODY(T, &, M)
#define SESSIONUUID_RETURN_AS(T, M)     SESSIONUUID_BASE_RETURN_BODY(T,, M)
#define SESSIONUUID_RETURN(M)           SESSIONUUID_BASE_RETURN_BODY(,, M)

#define SHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ShaderFxData3_6_0*>(data_ptr)-> C); 
#define SHADERFXDATA_RETURN_REF(T, M)    SHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define SHADERFXDATA_RETURN_AS(T, M)     SHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define SHADERFXDATA_RETURN(M)           SHADERFXDATA_BASE_RETURN_BODY(,, M)

#define SHADERFXDATA_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ShaderFxData_Runtime3_6_0*>(data_ptr)-> C); 
#define SHADERFXDATA_RUNTIME_RETURN_REF(T, M)    SHADERFXDATA_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define SHADERFXDATA_RUNTIME_RETURN_AS(T, M)     SHADERFXDATA_RUNTIME_BASE_RETURN_BODY(T,, M)
#define SHADERFXDATA_RUNTIME_RETURN(M)           SHADERFXDATA_RUNTIME_BASE_RETURN_BODY(,, M)

#define SHADOWSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ShadowShaderFxData3_6_0*>(data_ptr)-> C); 
#define SHADOWSHADERFXDATA_RETURN_REF(T, M)    SHADOWSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define SHADOWSHADERFXDATA_RETURN_AS(T, M)     SHADOWSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define SHADOWSHADERFXDATA_RETURN(M)           SHADOWSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define SHAPEKEYMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ShapeKeyModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ShapeKeyModifierData4_1_0*>(data_ptr)-> C); 
#define SHAPEKEYMODIFIERDATA_RETURN_REF(T, M)    SHAPEKEYMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SHAPEKEYMODIFIERDATA_RETURN_AS(T, M)     SHAPEKEYMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SHAPEKEYMODIFIERDATA_RETURN(M)           SHAPEKEYMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SHRINKWRAPGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ShrinkwrapGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define SHRINKWRAPGPENCILMODIFIERDATA_RETURN_REF(T, M)    SHRINKWRAPGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SHRINKWRAPGPENCILMODIFIERDATA_RETURN_AS(T, M)     SHRINKWRAPGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SHRINKWRAPGPENCILMODIFIERDATA_RETURN(M)           SHRINKWRAPGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SHRINKWRAPMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ShrinkwrapModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ShrinkwrapModifierData4_1_0*>(data_ptr)-> C); 
#define SHRINKWRAPMODIFIERDATA_RETURN_REF(T, M)    SHRINKWRAPMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SHRINKWRAPMODIFIERDATA_RETURN_AS(T, M)     SHRINKWRAPMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SHRINKWRAPMODIFIERDATA_RETURN(M)           SHRINKWRAPMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SIMPLEDEFORMMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SimpleDeformModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SimpleDeformModifierData4_1_0*>(data_ptr)-> C); 
#define SIMPLEDEFORMMODIFIERDATA_RETURN_REF(T, M)    SIMPLEDEFORMMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SIMPLEDEFORMMODIFIERDATA_RETURN_AS(T, M)     SIMPLEDEFORMMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SIMPLEDEFORMMODIFIERDATA_RETURN(M)           SIMPLEDEFORMMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SIMPLIFYGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SimplifyGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define SIMPLIFYGPENCILMODIFIERDATA_RETURN_REF(T, M)    SIMPLIFYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SIMPLIFYGPENCILMODIFIERDATA_RETURN_AS(T, M)     SIMPLIFYGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SIMPLIFYGPENCILMODIFIERDATA_RETURN(M)           SIMPLIFYGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SIMULATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Simulation3_6_0*>(data_ptr)-> C); 
#define SIMULATION_RETURN_REF(T, M)    SIMULATION_BASE_RETURN_BODY(T, &, M)
#define SIMULATION_RETURN_AS(T, M)     SIMULATION_BASE_RETURN_BODY(T,, M)
#define SIMULATION_RETURN(M)           SIMULATION_BASE_RETURN_BODY(,, M)

#define SKINMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SkinModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SkinModifierData4_1_0*>(data_ptr)-> C); 
#define SKINMODIFIERDATA_RETURN_REF(T, M)    SKINMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SKINMODIFIERDATA_RETURN_AS(T, M)     SKINMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SKINMODIFIERDATA_RETURN(M)           SKINMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SMOKEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SmokeModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SmokeModifierData4_1_0*>(data_ptr)-> C); 
#define SMOKEMODIFIERDATA_RETURN_REF(T, M)    SMOKEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SMOKEMODIFIERDATA_RETURN_AS(T, M)     SMOKEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SMOKEMODIFIERDATA_RETURN(M)           SMOKEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SMOOTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SmoothGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define SMOOTHGPENCILMODIFIERDATA_RETURN_REF(T, M)    SMOOTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SMOOTHGPENCILMODIFIERDATA_RETURN_AS(T, M)     SMOOTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SMOOTHGPENCILMODIFIERDATA_RETURN(M)           SMOOTHGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SMOOTHMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SmoothModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SmoothModifierData4_1_0*>(data_ptr)-> C); 
#define SMOOTHMODIFIERDATA_RETURN_REF(T, M)    SMOOTHMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SMOOTHMODIFIERDATA_RETURN_AS(T, M)     SMOOTHMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SMOOTHMODIFIERDATA_RETURN(M)           SMOOTHMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SOFTBODYMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SoftbodyModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SoftbodyModifierData4_1_0*>(data_ptr)-> C); 
#define SOFTBODYMODIFIERDATA_RETURN_REF(T, M)    SOFTBODYMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SOFTBODYMODIFIERDATA_RETURN_AS(T, M)     SOFTBODYMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SOFTBODYMODIFIERDATA_RETURN(M)           SOFTBODYMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SOFTBODY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SoftBody3_6_0*>(data_ptr)-> C); 
#define SOFTBODY_RETURN_REF(T, M)    SOFTBODY_BASE_RETURN_BODY(T, &, M)
#define SOFTBODY_RETURN_AS(T, M)     SOFTBODY_BASE_RETURN_BODY(T,, M)
#define SOFTBODY_RETURN(M)           SOFTBODY_BASE_RETURN_BODY(,, M)

#define SOFTBODY_SHARED_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SoftBody_Shared3_6_0*>(data_ptr)-> C); 
#define SOFTBODY_SHARED_RETURN_REF(T, M)    SOFTBODY_SHARED_BASE_RETURN_BODY(T, &, M)
#define SOFTBODY_SHARED_RETURN_AS(T, M)     SOFTBODY_SHARED_BASE_RETURN_BODY(T,, M)
#define SOFTBODY_SHARED_RETURN(M)           SOFTBODY_SHARED_BASE_RETURN_BODY(,, M)

#define SOLIDCOLORVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SolidColorVars3_6_0*>(data_ptr)-> C); 
#define SOLIDCOLORVARS_RETURN_REF(T, M)    SOLIDCOLORVARS_BASE_RETURN_BODY(T, &, M)
#define SOLIDCOLORVARS_RETURN_AS(T, M)     SOLIDCOLORVARS_BASE_RETURN_BODY(T,, M)
#define SOLIDCOLORVARS_RETURN(M)           SOLIDCOLORVARS_BASE_RETURN_BODY(,, M)

#define SOLIDIFYMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SolidifyModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SolidifyModifierData4_1_0*>(data_ptr)-> C); 
#define SOLIDIFYMODIFIERDATA_RETURN_REF(T, M)    SOLIDIFYMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SOLIDIFYMODIFIERDATA_RETURN_AS(T, M)     SOLIDIFYMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SOLIDIFYMODIFIERDATA_RETURN(M)           SOLIDIFYMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SOLIDLIGHT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SolidLight3_6_0*>(data_ptr)-> C); 
#define SOLIDLIGHT_RETURN_REF(T, M)    SOLIDLIGHT_BASE_RETURN_BODY(T, &, M)
#define SOLIDLIGHT_RETURN_AS(T, M)     SOLIDLIGHT_BASE_RETURN_BODY(T,, M)
#define SOLIDLIGHT_RETURN(M)           SOLIDLIGHT_BASE_RETURN_BODY(,, M)

#define SPACEACTION_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceAction3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceAction4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceAction4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SpaceAction4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceAction4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceAction4_4_0*>(data_ptr)-> C); 
#define SPACEACTION_RETURN_REF(T, M)    SPACEACTION_BASE_RETURN_BODY(T, &, M)
#define SPACEACTION_RETURN_AS(T, M)     SPACEACTION_BASE_RETURN_BODY(T,, M)
#define SPACEACTION_RETURN(M)           SPACEACTION_BASE_RETURN_BODY(,, M)

#define SPACEACTION_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceAction_Runtime3_6_0*>(data_ptr)-> C); 
#define SPACEACTION_RUNTIME_RETURN_REF(T, M)    SPACEACTION_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define SPACEACTION_RUNTIME_RETURN_AS(T, M)     SPACEACTION_RUNTIME_BASE_RETURN_BODY(T,, M)
#define SPACEACTION_RUNTIME_RETURN(M)           SPACEACTION_RUNTIME_BASE_RETURN_BODY(,, M)

#define SPACECLIP_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceClip3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceClip4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceClip4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceClip4_4_0*>(data_ptr)-> C); 
#define SPACECLIP_RETURN_REF(T, M)    SPACECLIP_BASE_RETURN_BODY(T, &, M)
#define SPACECLIP_RETURN_AS(T, M)     SPACECLIP_BASE_RETURN_BODY(T,, M)
#define SPACECLIP_RETURN(M)           SPACECLIP_BASE_RETURN_BODY(,, M)

#define SPACECONSOLE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceConsole3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceConsole4_1_0*>(data_ptr)-> C); 
#define SPACECONSOLE_RETURN_REF(T, M)    SPACECONSOLE_BASE_RETURN_BODY(T, &, M)
#define SPACECONSOLE_RETURN_AS(T, M)     SPACECONSOLE_BASE_RETURN_BODY(T,, M)
#define SPACECONSOLE_RETURN(M)           SPACECONSOLE_BASE_RETURN_BODY(,, M)

#define SPACEFILE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceFile3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceFile4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceFile4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceFile4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceFile4_4_0*>(data_ptr)-> C); 
#define SPACEFILE_RETURN_REF(T, M)    SPACEFILE_BASE_RETURN_BODY(T, &, M)
#define SPACEFILE_RETURN_AS(T, M)     SPACEFILE_BASE_RETURN_BODY(T,, M)
#define SPACEFILE_RETURN(M)           SPACEFILE_BASE_RETURN_BODY(,, M)

#define SPACEGRAPH_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceGraph3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceGraph4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceGraph4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceGraph4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceGraph4_4_0*>(data_ptr)-> C); 
#define SPACEGRAPH_RETURN_REF(T, M)    SPACEGRAPH_BASE_RETURN_BODY(T, &, M)
#define SPACEGRAPH_RETURN_AS(T, M)     SPACEGRAPH_BASE_RETURN_BODY(T,, M)
#define SPACEGRAPH_RETURN(M)           SPACEGRAPH_BASE_RETURN_BODY(,, M)

#define SPACEGRAPH_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceGraph_Runtime3_6_0*>(data_ptr)-> C); 
#define SPACEGRAPH_RUNTIME_RETURN_REF(T, M)    SPACEGRAPH_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define SPACEGRAPH_RUNTIME_RETURN_AS(T, M)     SPACEGRAPH_RUNTIME_BASE_RETURN_BODY(T,, M)
#define SPACEGRAPH_RUNTIME_RETURN(M)           SPACEGRAPH_RUNTIME_BASE_RETURN_BODY(,, M)

#define SPACEIMAGEOVERLAY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceImageOverlay3_6_0*>(data_ptr)-> C); 
#define SPACEIMAGEOVERLAY_RETURN_REF(T, M)    SPACEIMAGEOVERLAY_BASE_RETURN_BODY(T, &, M)
#define SPACEIMAGEOVERLAY_RETURN_AS(T, M)     SPACEIMAGEOVERLAY_BASE_RETURN_BODY(T,, M)
#define SPACEIMAGEOVERLAY_RETURN(M)           SPACEIMAGEOVERLAY_BASE_RETURN_BODY(,, M)

#define SPACEIMAGE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceImage3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceImage4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceImage4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceImage4_4_0*>(data_ptr)-> C); 
#define SPACEIMAGE_RETURN_REF(T, M)    SPACEIMAGE_BASE_RETURN_BODY(T, &, M)
#define SPACEIMAGE_RETURN_AS(T, M)     SPACEIMAGE_BASE_RETURN_BODY(T,, M)
#define SPACEIMAGE_RETURN(M)           SPACEIMAGE_BASE_RETURN_BODY(,, M)

#define SPACEINFO_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceInfo3_6_0*>(data_ptr)-> C); 
#define SPACEINFO_RETURN_REF(T, M)    SPACEINFO_BASE_RETURN_BODY(T, &, M)
#define SPACEINFO_RETURN_AS(T, M)     SPACEINFO_BASE_RETURN_BODY(T,, M)
#define SPACEINFO_RETURN(M)           SPACEINFO_BASE_RETURN_BODY(,, M)

#define SPACELINK_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceLink3_6_0*>(data_ptr)-> C); 
#define SPACELINK_RETURN_REF(T, M)    SPACELINK_BASE_RETURN_BODY(T, &, M)
#define SPACELINK_RETURN_AS(T, M)     SPACELINK_BASE_RETURN_BODY(T,, M)
#define SPACELINK_RETURN(M)           SPACELINK_BASE_RETURN_BODY(,, M)

#define SPACENLA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceNla3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceNla4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceNla4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceNla4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceNla4_4_0*>(data_ptr)-> C); 
#define SPACENLA_RETURN_REF(T, M)    SPACENLA_BASE_RETURN_BODY(T, &, M)
#define SPACENLA_RETURN_AS(T, M)     SPACENLA_BASE_RETURN_BODY(T,, M)
#define SPACENLA_RETURN(M)           SPACENLA_BASE_RETURN_BODY(,, M)

#define SPACENODEOVERLAY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceNodeOverlay3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceNodeOverlay4_0_0*>(data_ptr)-> C); 
#define SPACENODEOVERLAY_RETURN_REF(T, M)    SPACENODEOVERLAY_BASE_RETURN_BODY(T, &, M)
#define SPACENODEOVERLAY_RETURN_AS(T, M)     SPACENODEOVERLAY_BASE_RETURN_BODY(T,, M)
#define SPACENODEOVERLAY_RETURN(M)           SPACENODEOVERLAY_BASE_RETURN_BODY(,, M)

#define SPACENODE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceNode3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceNode4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceNode4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SpaceNode4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceNode4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceNode4_4_0*>(data_ptr)-> C); 
#define SPACENODE_RETURN_REF(T, M)    SPACENODE_BASE_RETURN_BODY(T, &, M)
#define SPACENODE_RETURN_AS(T, M)     SPACENODE_BASE_RETURN_BODY(T,, M)
#define SPACENODE_RETURN(M)           SPACENODE_BASE_RETURN_BODY(,, M)

#define SPACEOUTLINER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceOutliner3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceOutliner4_0_0*>(data_ptr)-> C); 
#define SPACEOUTLINER_RETURN_REF(T, M)    SPACEOUTLINER_BASE_RETURN_BODY(T, &, M)
#define SPACEOUTLINER_RETURN_AS(T, M)     SPACEOUTLINER_BASE_RETURN_BODY(T,, M)
#define SPACEOUTLINER_RETURN(M)           SPACEOUTLINER_BASE_RETURN_BODY(,, M)

#define SPACEPROPERTIES_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceProperties3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceProperties4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceProperties4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceProperties4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceProperties4_4_0*>(data_ptr)-> C); 
#define SPACEPROPERTIES_RETURN_REF(T, M)    SPACEPROPERTIES_BASE_RETURN_BODY(T, &, M)
#define SPACEPROPERTIES_RETURN_AS(T, M)     SPACEPROPERTIES_BASE_RETURN_BODY(T,, M)
#define SPACEPROPERTIES_RETURN(M)           SPACEPROPERTIES_BASE_RETURN_BODY(,, M)

#define SPACESCRIPT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceScript3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceScript4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<SpaceScript4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceScript4_4_0*>(data_ptr)-> C); 
#define SPACESCRIPT_RETURN_REF(T, M)    SPACESCRIPT_BASE_RETURN_BODY(T, &, M)
#define SPACESCRIPT_RETURN_AS(T, M)     SPACESCRIPT_BASE_RETURN_BODY(T,, M)
#define SPACESCRIPT_RETURN(M)           SPACESCRIPT_BASE_RETURN_BODY(,, M)

#define SPACESEQRUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceSeqRuntime3_6_0*>(data_ptr)-> C); 
#define SPACESEQRUNTIME_RETURN_REF(T, M)    SPACESEQRUNTIME_BASE_RETURN_BODY(T, &, M)
#define SPACESEQRUNTIME_RETURN_AS(T, M)     SPACESEQRUNTIME_BASE_RETURN_BODY(T,, M)
#define SPACESEQRUNTIME_RETURN(M)           SPACESEQRUNTIME_BASE_RETURN_BODY(,, M)

#define SPACESEQ_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<SpaceSeq3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceSeq4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<SpaceSeq4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceSeq4_2_0*>(data_ptr)-> C); 
#define SPACESEQ_RETURN_REF(T, M)    SPACESEQ_BASE_RETURN_BODY(T, &, M)
#define SPACESEQ_RETURN_AS(T, M)     SPACESEQ_BASE_RETURN_BODY(T,, M)
#define SPACESEQ_RETURN(M)           SPACESEQ_BASE_RETURN_BODY(,, M)

#define SPACESPREADSHEET_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceSpreadsheet3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<SpaceSpreadsheet4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceSpreadsheet4_3_0*>(data_ptr)-> C); 
#define SPACESPREADSHEET_RETURN_REF(T, M)    SPACESPREADSHEET_BASE_RETURN_BODY(T, &, M)
#define SPACESPREADSHEET_RETURN_AS(T, M)     SPACESPREADSHEET_BASE_RETURN_BODY(T,, M)
#define SPACESPREADSHEET_RETURN(M)           SPACESPREADSHEET_BASE_RETURN_BODY(,, M)

#define SPACESTATUSBAR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceStatusBar3_6_0*>(data_ptr)-> C); 
#define SPACESTATUSBAR_RETURN_REF(T, M)    SPACESTATUSBAR_BASE_RETURN_BODY(T, &, M)
#define SPACESTATUSBAR_RETURN_AS(T, M)     SPACESTATUSBAR_BASE_RETURN_BODY(T,, M)
#define SPACESTATUSBAR_RETURN(M)           SPACESTATUSBAR_BASE_RETURN_BODY(,, M)

#define SPACETEXT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SpaceText3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SpaceText4_1_0*>(data_ptr)-> C); 
#define SPACETEXT_RETURN_REF(T, M)    SPACETEXT_BASE_RETURN_BODY(T, &, M)
#define SPACETEXT_RETURN_AS(T, M)     SPACETEXT_BASE_RETURN_BODY(T,, M)
#define SPACETEXT_RETURN(M)           SPACETEXT_BASE_RETURN_BODY(,, M)

#define SPACETEXT_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceText_Runtime3_6_0*>(data_ptr)-> C); 
#define SPACETEXT_RUNTIME_RETURN_REF(T, M)    SPACETEXT_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define SPACETEXT_RUNTIME_RETURN_AS(T, M)     SPACETEXT_RUNTIME_BASE_RETURN_BODY(T,, M)
#define SPACETEXT_RUNTIME_RETURN(M)           SPACETEXT_RUNTIME_BASE_RETURN_BODY(,, M)

#define SPACETOPBAR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceTopBar3_6_0*>(data_ptr)-> C); 
#define SPACETOPBAR_RETURN_REF(T, M)    SPACETOPBAR_BASE_RETURN_BODY(T, &, M)
#define SPACETOPBAR_RETURN_AS(T, M)     SPACETOPBAR_BASE_RETURN_BODY(T,, M)
#define SPACETOPBAR_RETURN(M)           SPACETOPBAR_BASE_RETURN_BODY(,, M)

#define SPACEUSERPREF_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpaceUserPref3_6_0*>(data_ptr)-> C); 
#define SPACEUSERPREF_RETURN_REF(T, M)    SPACEUSERPREF_BASE_RETURN_BODY(T, &, M)
#define SPACEUSERPREF_RETURN_AS(T, M)     SPACEUSERPREF_BASE_RETURN_BODY(T,, M)
#define SPACEUSERPREF_RETURN(M)           SPACEUSERPREF_BASE_RETURN_BODY(,, M)

#define SPEAKER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Speaker3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Speaker4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Speaker4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Speaker4_4_0*>(data_ptr)-> C); 
#define SPEAKER_RETURN_REF(T, M)    SPEAKER_BASE_RETURN_BODY(T, &, M)
#define SPEAKER_RETURN_AS(T, M)     SPEAKER_BASE_RETURN_BODY(T,, M)
#define SPEAKER_RETURN(M)           SPEAKER_BASE_RETURN_BODY(,, M)

#define SPEEDCONTROLVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpeedControlVars3_6_0*>(data_ptr)-> C); 
#define SPEEDCONTROLVARS_RETURN_REF(T, M)    SPEEDCONTROLVARS_BASE_RETURN_BODY(T, &, M)
#define SPEEDCONTROLVARS_RETURN_AS(T, M)     SPEEDCONTROLVARS_BASE_RETURN_BODY(T,, M)
#define SPEEDCONTROLVARS_RETURN(M)           SPEEDCONTROLVARS_BASE_RETURN_BODY(,, M)

#define SPHFLUIDSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SPHFluidSettings3_6_0*>(data_ptr)-> C); 
#define SPHFLUIDSETTINGS_RETURN_REF(T, M)    SPHFLUIDSETTINGS_BASE_RETURN_BODY(T, &, M)
#define SPHFLUIDSETTINGS_RETURN_AS(T, M)     SPHFLUIDSETTINGS_BASE_RETURN_BODY(T,, M)
#define SPHFLUIDSETTINGS_RETURN(M)           SPHFLUIDSETTINGS_BASE_RETURN_BODY(,, M)

#define SPREADSHEETCOLUMNID_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpreadsheetColumnID3_6_0*>(data_ptr)-> C); 
#define SPREADSHEETCOLUMNID_RETURN_REF(T, M)    SPREADSHEETCOLUMNID_BASE_RETURN_BODY(T, &, M)
#define SPREADSHEETCOLUMNID_RETURN_AS(T, M)     SPREADSHEETCOLUMNID_BASE_RETURN_BODY(T,, M)
#define SPREADSHEETCOLUMNID_RETURN(M)           SPREADSHEETCOLUMNID_BASE_RETURN_BODY(,, M)

#define SPREADSHEETCOLUMN_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpreadsheetColumn3_6_0*>(data_ptr)-> C); 
#define SPREADSHEETCOLUMN_RETURN_REF(T, M)    SPREADSHEETCOLUMN_BASE_RETURN_BODY(T, &, M)
#define SPREADSHEETCOLUMN_RETURN_AS(T, M)     SPREADSHEETCOLUMN_BASE_RETURN_BODY(T,, M)
#define SPREADSHEETCOLUMN_RETURN(M)           SPREADSHEETCOLUMN_BASE_RETURN_BODY(,, M)

#define SPREADSHEETROWFILTER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SpreadsheetRowFilter3_6_0*>(data_ptr)-> C); 
#define SPREADSHEETROWFILTER_RETURN_REF(T, M)    SPREADSHEETROWFILTER_BASE_RETURN_BODY(T, &, M)
#define SPREADSHEETROWFILTER_RETURN_AS(T, M)     SPREADSHEETROWFILTER_BASE_RETURN_BODY(T,, M)
#define SPREADSHEETROWFILTER_RETURN(M)           SPREADSHEETROWFILTER_BASE_RETURN_BODY(,, M)

#define STEREO3DFORMAT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Stereo3dFormat3_6_0*>(data_ptr)-> C); 
#define STEREO3DFORMAT_RETURN_REF(T, M)    STEREO3DFORMAT_BASE_RETURN_BODY(T, &, M)
#define STEREO3DFORMAT_RETURN_AS(T, M)     STEREO3DFORMAT_BASE_RETURN_BODY(T,, M)
#define STEREO3DFORMAT_RETURN(M)           STEREO3DFORMAT_BASE_RETURN_BODY(,, M)

#define STRIPANIM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<StripAnim3_6_0*>(data_ptr)-> C); 
#define STRIPANIM_RETURN_REF(T, M)    STRIPANIM_BASE_RETURN_BODY(T, &, M)
#define STRIPANIM_RETURN_AS(T, M)     STRIPANIM_BASE_RETURN_BODY(T,, M)
#define STRIPANIM_RETURN(M)           STRIPANIM_BASE_RETURN_BODY(,, M)

#define STRIPCOLORBALANCE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<StripColorBalance3_6_0*>(data_ptr)-> C); 
#define STRIPCOLORBALANCE_RETURN_REF(T, M)    STRIPCOLORBALANCE_BASE_RETURN_BODY(T, &, M)
#define STRIPCOLORBALANCE_RETURN_AS(T, M)     STRIPCOLORBALANCE_BASE_RETURN_BODY(T,, M)
#define STRIPCOLORBALANCE_RETURN(M)           STRIPCOLORBALANCE_BASE_RETURN_BODY(,, M)

#define STRIPCROP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<StripCrop3_6_0*>(data_ptr)-> C); 
#define STRIPCROP_RETURN_REF(T, M)    STRIPCROP_BASE_RETURN_BODY(T, &, M)
#define STRIPCROP_RETURN_AS(T, M)     STRIPCROP_BASE_RETURN_BODY(T,, M)
#define STRIPCROP_RETURN(M)           STRIPCROP_BASE_RETURN_BODY(,, M)

#define STRIPELEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<StripElem3_6_0*>(data_ptr)-> C); 
#define STRIPELEM_RETURN_REF(T, M)    STRIPELEM_BASE_RETURN_BODY(T, &, M)
#define STRIPELEM_RETURN_AS(T, M)     STRIPELEM_BASE_RETURN_BODY(T,, M)
#define STRIPELEM_RETURN(M)           STRIPELEM_BASE_RETURN_BODY(,, M)

#define STRIPPROXY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<StripProxy3_6_0*>(data_ptr)-> C); 
#define STRIPPROXY_RETURN_REF(T, M)    STRIPPROXY_BASE_RETURN_BODY(T, &, M)
#define STRIPPROXY_RETURN_AS(T, M)     STRIPPROXY_BASE_RETURN_BODY(T,, M)
#define STRIPPROXY_RETURN(M)           STRIPPROXY_BASE_RETURN_BODY(,, M)

#define STRIPTRANSFORM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<StripTransform3_6_0*>(data_ptr)-> C); 
#define STRIPTRANSFORM_RETURN_REF(T, M)    STRIPTRANSFORM_BASE_RETURN_BODY(T, &, M)
#define STRIPTRANSFORM_RETURN_AS(T, M)     STRIPTRANSFORM_BASE_RETURN_BODY(T,, M)
#define STRIPTRANSFORM_RETURN(M)           STRIPTRANSFORM_BASE_RETURN_BODY(,, M)

#define STRIP_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Strip3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Strip4_4_0*>(data_ptr)-> C); 
#define STRIP_RETURN_REF(T, M)    STRIP_BASE_RETURN_BODY(T, &, M)
#define STRIP_RETURN_AS(T, M)     STRIP_BASE_RETURN_BODY(T,, M)
#define STRIP_RETURN(M)           STRIP_BASE_RETURN_BODY(,, M)

#define SUBDIVGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SubdivGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define SUBDIVGPENCILMODIFIERDATA_RETURN_REF(T, M)    SUBDIVGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SUBDIVGPENCILMODIFIERDATA_RETURN_AS(T, M)     SUBDIVGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SUBDIVGPENCILMODIFIERDATA_RETURN(M)           SUBDIVGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SUBSURFMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SubsurfModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SubsurfModifierData4_1_0*>(data_ptr)-> C); 
#define SUBSURFMODIFIERDATA_RETURN_REF(T, M)    SUBSURFMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SUBSURFMODIFIERDATA_RETURN_AS(T, M)     SUBSURFMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SUBSURFMODIFIERDATA_RETURN(M)           SUBSURFMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SURFACEDEFORMMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SurfaceDeformModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SurfaceDeformModifierData4_1_0*>(data_ptr)-> C); 
#define SURFACEDEFORMMODIFIERDATA_RETURN_REF(T, M)    SURFACEDEFORMMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SURFACEDEFORMMODIFIERDATA_RETURN_AS(T, M)     SURFACEDEFORMMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SURFACEDEFORMMODIFIERDATA_RETURN(M)           SURFACEDEFORMMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SURFACEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<SurfaceModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<SurfaceModifierData4_1_0*>(data_ptr)-> C); 
#define SURFACEMODIFIERDATA_RETURN_REF(T, M)    SURFACEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define SURFACEMODIFIERDATA_RETURN_AS(T, M)     SURFACEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define SURFACEMODIFIERDATA_RETURN(M)           SURFACEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define SURFACEMODIFIERDATA_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SurfaceModifierData_Runtime3_6_0*>(data_ptr)-> C); 
#define SURFACEMODIFIERDATA_RUNTIME_RETURN_REF(T, M)    SURFACEMODIFIERDATA_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define SURFACEMODIFIERDATA_RUNTIME_RETURN_AS(T, M)     SURFACEMODIFIERDATA_RUNTIME_BASE_RETURN_BODY(T,, M)
#define SURFACEMODIFIERDATA_RUNTIME_RETURN(M)           SURFACEMODIFIERDATA_RUNTIME_BASE_RETURN_BODY(,, M)

#define SWIRLSHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<SwirlShaderFxData3_6_0*>(data_ptr)-> C); 
#define SWIRLSHADERFXDATA_RETURN_REF(T, M)    SWIRLSHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define SWIRLSHADERFXDATA_RETURN_AS(T, M)     SWIRLSHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define SWIRLSHADERFXDATA_RETURN(M)           SWIRLSHADERFXDATA_BASE_RETURN_BODY(,, M)

#define TEXMAPPING_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<TexMapping3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<TexMapping4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<TexMapping4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<TexMapping4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<TexMapping4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<TexMapping4_4_0*>(data_ptr)-> C); 
#define TEXMAPPING_RETURN_REF(T, M)    TEXMAPPING_BASE_RETURN_BODY(T, &, M)
#define TEXMAPPING_RETURN_AS(T, M)     TEXMAPPING_BASE_RETURN_BODY(T,, M)
#define TEXMAPPING_RETURN(M)           TEXMAPPING_BASE_RETURN_BODY(,, M)

#define TEXNODEOUTPUT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TexNodeOutput3_6_0*>(data_ptr)-> C); 
#define TEXNODEOUTPUT_RETURN_REF(T, M)    TEXNODEOUTPUT_BASE_RETURN_BODY(T, &, M)
#define TEXNODEOUTPUT_RETURN_AS(T, M)     TEXNODEOUTPUT_BASE_RETURN_BODY(T,, M)
#define TEXNODEOUTPUT_RETURN(M)           TEXNODEOUTPUT_BASE_RETURN_BODY(,, M)

#define TEXPAINTSLOT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TexPaintSlot3_6_0*>(data_ptr)-> C); 
#define TEXPAINTSLOT_RETURN_REF(T, M)    TEXPAINTSLOT_BASE_RETURN_BODY(T, &, M)
#define TEXPAINTSLOT_RETURN_AS(T, M)     TEXPAINTSLOT_BASE_RETURN_BODY(T,, M)
#define TEXPAINTSLOT_RETURN(M)           TEXPAINTSLOT_BASE_RETURN_BODY(,, M)

#define TEXTBOX_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TextBox3_6_0*>(data_ptr)-> C); 
#define TEXTBOX_RETURN_REF(T, M)    TEXTBOX_BASE_RETURN_BODY(T, &, M)
#define TEXTBOX_RETURN_AS(T, M)     TEXTBOX_BASE_RETURN_BODY(T,, M)
#define TEXTBOX_RETURN(M)           TEXTBOX_BASE_RETURN_BODY(,, M)

#define TEXTLINE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TextLine3_6_0*>(data_ptr)-> C); 
#define TEXTLINE_RETURN_REF(T, M)    TEXTLINE_BASE_RETURN_BODY(T, &, M)
#define TEXTLINE_RETURN_AS(T, M)     TEXTLINE_BASE_RETURN_BODY(T,, M)
#define TEXTLINE_RETURN(M)           TEXTLINE_BASE_RETURN_BODY(,, M)

#define TEXTUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TextureGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define TEXTUREGPENCILMODIFIERDATA_RETURN_REF(T, M)    TEXTUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define TEXTUREGPENCILMODIFIERDATA_RETURN_AS(T, M)     TEXTUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define TEXTUREGPENCILMODIFIERDATA_RETURN(M)           TEXTUREGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define TEXTVARS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<TextVars3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<TextVars4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<TextVars4_4_0*>(data_ptr)-> C); 
#define TEXTVARS_RETURN_REF(T, M)    TEXTVARS_BASE_RETURN_BODY(T, &, M)
#define TEXTVARS_RETURN_AS(T, M)     TEXTVARS_BASE_RETURN_BODY(T,, M)
#define TEXTVARS_RETURN(M)           TEXTVARS_BASE_RETURN_BODY(,, M)

#define TEXT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Text3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Text4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Text4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Text4_4_0*>(data_ptr)-> C); 
#define TEXT_RETURN_REF(T, M)    TEXT_BASE_RETURN_BODY(T, &, M)
#define TEXT_RETURN_AS(T, M)     TEXT_BASE_RETURN_BODY(T,, M)
#define TEXT_RETURN(M)           TEXT_BASE_RETURN_BODY(,, M)

#define TEX_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Tex3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Tex4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Tex4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Tex4_4_0*>(data_ptr)-> C); 
#define TEX_RETURN_REF(T, M)    TEX_BASE_RETURN_BODY(T, &, M)
#define TEX_RETURN_AS(T, M)     TEX_BASE_RETURN_BODY(T,, M)
#define TEX_RETURN(M)           TEX_BASE_RETURN_BODY(,, M)

#define THEMECOLLECTIONCOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ThemeCollectionColor3_6_0*>(data_ptr)-> C); 
#define THEMECOLLECTIONCOLOR_RETURN_REF(T, M)    THEMECOLLECTIONCOLOR_BASE_RETURN_BODY(T, &, M)
#define THEMECOLLECTIONCOLOR_RETURN_AS(T, M)     THEMECOLLECTIONCOLOR_BASE_RETURN_BODY(T,, M)
#define THEMECOLLECTIONCOLOR_RETURN(M)           THEMECOLLECTIONCOLOR_BASE_RETURN_BODY(,, M)

#define THEMESPACE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ThemeSpace3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ThemeSpace4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ThemeSpace4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ThemeSpace4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ThemeSpace4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ThemeSpace4_4_0*>(data_ptr)-> C); 
#define THEMESPACE_RETURN_REF(T, M)    THEMESPACE_BASE_RETURN_BODY(T, &, M)
#define THEMESPACE_RETURN_AS(T, M)     THEMESPACE_BASE_RETURN_BODY(T,, M)
#define THEMESPACE_RETURN(M)           THEMESPACE_BASE_RETURN_BODY(,, M)

#define THEMESTRIPCOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ThemeStripColor3_6_0*>(data_ptr)-> C); 
#define THEMESTRIPCOLOR_RETURN_REF(T, M)    THEMESTRIPCOLOR_BASE_RETURN_BODY(T, &, M)
#define THEMESTRIPCOLOR_RETURN_AS(T, M)     THEMESTRIPCOLOR_BASE_RETURN_BODY(T,, M)
#define THEMESTRIPCOLOR_RETURN(M)           THEMESTRIPCOLOR_BASE_RETURN_BODY(,, M)

#define THEMEUI_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ThemeUI3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ThemeUI4_0_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ThemeUI4_3_0*>(data_ptr)-> C); 
#define THEMEUI_RETURN_REF(T, M)    THEMEUI_BASE_RETURN_BODY(T, &, M)
#define THEMEUI_RETURN_AS(T, M)     THEMEUI_BASE_RETURN_BODY(T,, M)
#define THEMEUI_RETURN(M)           THEMEUI_BASE_RETURN_BODY(,, M)

#define THEMEWIRECOLOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ThemeWireColor3_6_0*>(data_ptr)-> C); 
#define THEMEWIRECOLOR_RETURN_REF(T, M)    THEMEWIRECOLOR_BASE_RETURN_BODY(T, &, M)
#define THEMEWIRECOLOR_RETURN_AS(T, M)     THEMEWIRECOLOR_BASE_RETURN_BODY(T,, M)
#define THEMEWIRECOLOR_RETURN(M)           THEMEWIRECOLOR_BASE_RETURN_BODY(,, M)

#define THICKGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ThickGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define THICKGPENCILMODIFIERDATA_RETURN_REF(T, M)    THICKGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define THICKGPENCILMODIFIERDATA_RETURN_AS(T, M)     THICKGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define THICKGPENCILMODIFIERDATA_RETURN(M)           THICKGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define TIMEGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TimeGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define TIMEGPENCILMODIFIERDATA_RETURN_REF(T, M)    TIMEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define TIMEGPENCILMODIFIERDATA_RETURN_AS(T, M)     TIMEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define TIMEGPENCILMODIFIERDATA_RETURN(M)           TIMEGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define TIMEGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TimeGpencilModifierSegment3_6_0*>(data_ptr)-> C); 
#define TIMEGPENCILMODIFIERSEGMENT_RETURN_REF(T, M)    TIMEGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(T, &, M)
#define TIMEGPENCILMODIFIERSEGMENT_RETURN_AS(T, M)     TIMEGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(T,, M)
#define TIMEGPENCILMODIFIERSEGMENT_RETURN(M)           TIMEGPENCILMODIFIERSEGMENT_BASE_RETURN_BODY(,, M)

#define TIMEMARKER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<TimeMarker3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<TimeMarker4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<TimeMarker4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<TimeMarker4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<TimeMarker4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<TimeMarker4_4_0*>(data_ptr)-> C); 
#define TIMEMARKER_RETURN_REF(T, M)    TIMEMARKER_BASE_RETURN_BODY(T, &, M)
#define TIMEMARKER_RETURN_AS(T, M)     TIMEMARKER_BASE_RETURN_BODY(T,, M)
#define TIMEMARKER_RETURN(M)           TIMEMARKER_BASE_RETURN_BODY(,, M)

#define TINTGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TintGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define TINTGPENCILMODIFIERDATA_RETURN_REF(T, M)    TINTGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define TINTGPENCILMODIFIERDATA_RETURN_AS(T, M)     TINTGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define TINTGPENCILMODIFIERDATA_RETURN(M)           TINTGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define TPALETTECOLORHSV_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<tPaletteColorHSV3_6_0*>(data_ptr)-> C); 
#define TPALETTECOLORHSV_RETURN_REF(T, M)    TPALETTECOLORHSV_BASE_RETURN_BODY(T, &, M)
#define TPALETTECOLORHSV_RETURN_AS(T, M)     TPALETTECOLORHSV_BASE_RETURN_BODY(T,, M)
#define TPALETTECOLORHSV_RETURN(M)           TPALETTECOLORHSV_BASE_RETURN_BODY(,, M)

#define TRANSFORMORIENTATIONSLOT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TransformOrientationSlot3_6_0*>(data_ptr)-> C); 
#define TRANSFORMORIENTATIONSLOT_RETURN_REF(T, M)    TRANSFORMORIENTATIONSLOT_BASE_RETURN_BODY(T, &, M)
#define TRANSFORMORIENTATIONSLOT_RETURN_AS(T, M)     TRANSFORMORIENTATIONSLOT_BASE_RETURN_BODY(T,, M)
#define TRANSFORMORIENTATIONSLOT_RETURN(M)           TRANSFORMORIENTATIONSLOT_BASE_RETURN_BODY(,, M)

#define TRANSFORMORIENTATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TransformOrientation3_6_0*>(data_ptr)-> C); 
#define TRANSFORMORIENTATION_RETURN_REF(T, M)    TRANSFORMORIENTATION_BASE_RETURN_BODY(T, &, M)
#define TRANSFORMORIENTATION_RETURN_AS(T, M)     TRANSFORMORIENTATION_BASE_RETURN_BODY(T,, M)
#define TRANSFORMORIENTATION_RETURN(M)           TRANSFORMORIENTATION_BASE_RETURN_BODY(,, M)

#define TRANSFORMVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<TransformVars3_6_0*>(data_ptr)-> C); 
#define TRANSFORMVARS_RETURN_REF(T, M)    TRANSFORMVARS_BASE_RETURN_BODY(T, &, M)
#define TRANSFORMVARS_RETURN_AS(T, M)     TRANSFORMVARS_BASE_RETURN_BODY(T,, M)
#define TRANSFORMVARS_RETURN(M)           TRANSFORMVARS_BASE_RETURN_BODY(,, M)

#define TREESTOREELEM_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<TreeStoreElem3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<TreeStoreElem4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<TreeStoreElem4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<TreeStoreElem4_4_0*>(data_ptr)-> C); 
#define TREESTOREELEM_RETURN_REF(T, M)    TREESTOREELEM_BASE_RETURN_BODY(T, &, M)
#define TREESTOREELEM_RETURN_AS(T, M)     TREESTOREELEM_BASE_RETURN_BODY(T,, M)
#define TREESTOREELEM_RETURN(M)           TREESTOREELEM_BASE_RETURN_BODY(,, M)

#define TREESTORE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<TreeStore3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<TreeStore4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<TreeStore4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<TreeStore4_4_0*>(data_ptr)-> C); 
#define TREESTORE_RETURN_REF(T, M)    TREESTORE_BASE_RETURN_BODY(T, &, M)
#define TREESTORE_RETURN_AS(T, M)     TREESTORE_BASE_RETURN_BODY(T,, M)
#define TREESTORE_RETURN(M)           TREESTORE_BASE_RETURN_BODY(,, M)

#define TRIANGULATEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<TriangulateModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<TriangulateModifierData4_1_0*>(data_ptr)-> C); 
#define TRIANGULATEMODIFIERDATA_RETURN_REF(T, M)    TRIANGULATEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define TRIANGULATEMODIFIERDATA_RETURN_AS(T, M)     TRIANGULATEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define TRIANGULATEMODIFIERDATA_RETURN(M)           TRIANGULATEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define UIFONTSTYLE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<uiFontStyle3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<uiFontStyle4_1_0*>(data_ptr)-> C); 
#define UIFONTSTYLE_RETURN_REF(T, M)    UIFONTSTYLE_BASE_RETURN_BODY(T, &, M)
#define UIFONTSTYLE_RETURN_AS(T, M)     UIFONTSTYLE_BASE_RETURN_BODY(T,, M)
#define UIFONTSTYLE_RETURN(M)           UIFONTSTYLE_BASE_RETURN_BODY(,, M)

#define UIFONT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<uiFont3_6_0*>(data_ptr)-> C); 
#define UIFONT_RETURN_REF(T, M)    UIFONT_BASE_RETURN_BODY(T, &, M)
#define UIFONT_RETURN_AS(T, M)     UIFONT_BASE_RETURN_BODY(T,, M)
#define UIFONT_RETURN(M)           UIFONT_BASE_RETURN_BODY(,, M)

#define UILIST_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_1) \
        return A ( B reinterpret_cast<uiList3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<uiList4_1_1*>(data_ptr)-> C); 
#define UILIST_RETURN_REF(T, M)    UILIST_BASE_RETURN_BODY(T, &, M)
#define UILIST_RETURN_AS(T, M)     UILIST_BASE_RETURN_BODY(T,, M)
#define UILIST_RETURN(M)           UILIST_BASE_RETURN_BODY(,, M)

#define UIPANELCOLORS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<uiPanelColors3_6_0*>(data_ptr)-> C); 
#define UIPANELCOLORS_RETURN_REF(T, M)    UIPANELCOLORS_BASE_RETURN_BODY(T, &, M)
#define UIPANELCOLORS_RETURN_AS(T, M)     UIPANELCOLORS_BASE_RETURN_BODY(T,, M)
#define UIPANELCOLORS_RETURN(M)           UIPANELCOLORS_BASE_RETURN_BODY(,, M)

#define UIPREVIEW_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_4) \
        return A ( B reinterpret_cast<uiPreview3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<uiPreview4_2_4*>(data_ptr)-> C); 
#define UIPREVIEW_RETURN_REF(T, M)    UIPREVIEW_BASE_RETURN_BODY(T, &, M)
#define UIPREVIEW_RETURN_AS(T, M)     UIPREVIEW_BASE_RETURN_BODY(T,, M)
#define UIPREVIEW_RETURN(M)           UIPREVIEW_BASE_RETURN_BODY(,, M)

#define UISTYLE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<uiStyle3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<uiStyle4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<uiStyle4_3_0*>(data_ptr)-> C); 
#define UISTYLE_RETURN_REF(T, M)    UISTYLE_BASE_RETURN_BODY(T, &, M)
#define UISTYLE_RETURN_AS(T, M)     UISTYLE_BASE_RETURN_BODY(T,, M)
#define UISTYLE_RETURN(M)           UISTYLE_BASE_RETURN_BODY(,, M)

#define UIWIDGETCOLORS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<uiWidgetColors3_6_0*>(data_ptr)-> C); 
#define UIWIDGETCOLORS_RETURN_REF(T, M)    UIWIDGETCOLORS_BASE_RETURN_BODY(T, &, M)
#define UIWIDGETCOLORS_RETURN_AS(T, M)     UIWIDGETCOLORS_BASE_RETURN_BODY(T,, M)
#define UIWIDGETCOLORS_RETURN(M)           UIWIDGETCOLORS_BASE_RETURN_BODY(,, M)

#define UIWIDGETSTATECOLORS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<uiWidgetStateColors3_6_0*>(data_ptr)-> C); 
#define UIWIDGETSTATECOLORS_RETURN_REF(T, M)    UIWIDGETSTATECOLORS_BASE_RETURN_BODY(T, &, M)
#define UIWIDGETSTATECOLORS_RETURN_AS(T, M)     UIWIDGETSTATECOLORS_BASE_RETURN_BODY(T,, M)
#define UIWIDGETSTATECOLORS_RETURN(M)           UIWIDGETSTATECOLORS_BASE_RETURN_BODY(,, M)

#define UNIFIEDPAINTSETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<UnifiedPaintSettings3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<UnifiedPaintSettings4_1_0*>(data_ptr)-> C); 
#define UNIFIEDPAINTSETTINGS_RETURN_REF(T, M)    UNIFIEDPAINTSETTINGS_BASE_RETURN_BODY(T, &, M)
#define UNIFIEDPAINTSETTINGS_RETURN_AS(T, M)     UNIFIEDPAINTSETTINGS_BASE_RETURN_BODY(T,, M)
#define UNIFIEDPAINTSETTINGS_RETURN(M)           UNIFIEDPAINTSETTINGS_BASE_RETURN_BODY(,, M)

#define UNITSETTINGS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<UnitSettings3_6_0*>(data_ptr)-> C); 
#define UNITSETTINGS_RETURN_REF(T, M)    UNITSETTINGS_BASE_RETURN_BODY(T, &, M)
#define UNITSETTINGS_RETURN_AS(T, M)     UNITSETTINGS_BASE_RETURN_BODY(T,, M)
#define UNITSETTINGS_RETURN(M)           UNITSETTINGS_BASE_RETURN_BODY(,, M)

#define USERDEF_EXPERIMENTAL_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<UserDef_Experimental3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<UserDef_Experimental4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<UserDef_Experimental4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<UserDef_Experimental4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<UserDef_Experimental4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<UserDef_Experimental4_4_0*>(data_ptr)-> C); 
#define USERDEF_EXPERIMENTAL_RETURN_REF(T, M)    USERDEF_EXPERIMENTAL_BASE_RETURN_BODY(T, &, M)
#define USERDEF_EXPERIMENTAL_RETURN_AS(T, M)     USERDEF_EXPERIMENTAL_BASE_RETURN_BODY(T,, M)
#define USERDEF_EXPERIMENTAL_RETURN(M)           USERDEF_EXPERIMENTAL_BASE_RETURN_BODY(,, M)

#define USERDEF_FILESPACEDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<UserDef_FileSpaceData3_6_0*>(data_ptr)-> C); 
#define USERDEF_FILESPACEDATA_RETURN_REF(T, M)    USERDEF_FILESPACEDATA_BASE_RETURN_BODY(T, &, M)
#define USERDEF_FILESPACEDATA_RETURN_AS(T, M)     USERDEF_FILESPACEDATA_BASE_RETURN_BODY(T,, M)
#define USERDEF_FILESPACEDATA_RETURN(M)           USERDEF_FILESPACEDATA_BASE_RETURN_BODY(,, M)

#define USERDEF_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<UserDef_Runtime3_6_0*>(data_ptr)-> C); 
#define USERDEF_RUNTIME_RETURN_REF(T, M)    USERDEF_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define USERDEF_RUNTIME_RETURN_AS(T, M)     USERDEF_RUNTIME_BASE_RETURN_BODY(T,, M)
#define USERDEF_RUNTIME_RETURN(M)           USERDEF_RUNTIME_BASE_RETURN_BODY(,, M)

#define USERDEF_SPACEDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<UserDef_SpaceData3_6_0*>(data_ptr)-> C); 
#define USERDEF_SPACEDATA_RETURN_REF(T, M)    USERDEF_SPACEDATA_BASE_RETURN_BODY(T, &, M)
#define USERDEF_SPACEDATA_RETURN_AS(T, M)     USERDEF_SPACEDATA_BASE_RETURN_BODY(T,, M)
#define USERDEF_SPACEDATA_RETURN(M)           USERDEF_SPACEDATA_BASE_RETURN_BODY(,, M)

#define UVPROJECTMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<UVProjectModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<UVProjectModifierData4_1_0*>(data_ptr)-> C); 
#define UVPROJECTMODIFIERDATA_RETURN_REF(T, M)    UVPROJECTMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define UVPROJECTMODIFIERDATA_RETURN_AS(T, M)     UVPROJECTMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define UVPROJECTMODIFIERDATA_RETURN(M)           UVPROJECTMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define UVSCULPT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<UvSculpt3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<UvSculpt4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<UvSculpt4_2_0*>(data_ptr)-> C); 
#define UVSCULPT_RETURN_REF(T, M)    UVSCULPT_BASE_RETURN_BODY(T, &, M)
#define UVSCULPT_RETURN_AS(T, M)     UVSCULPT_BASE_RETURN_BODY(T,, M)
#define UVSCULPT_RETURN(M)           UVSCULPT_BASE_RETURN_BODY(,, M)

#define UVWARPMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<UVWarpModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<UVWarpModifierData4_1_0*>(data_ptr)-> C); 
#define UVWARPMODIFIERDATA_RETURN_REF(T, M)    UVWARPMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define UVWARPMODIFIERDATA_RETURN_AS(T, M)     UVWARPMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define UVWARPMODIFIERDATA_RETURN(M)           UVWARPMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define VEC2F_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<vec2f3_6_0*>(data_ptr)-> C); 
#define VEC2F_RETURN_REF(T, M)    VEC2F_BASE_RETURN_BODY(T, &, M)
#define VEC2F_RETURN_AS(T, M)     VEC2F_BASE_RETURN_BODY(T,, M)
#define VEC2F_RETURN(M)           VEC2F_BASE_RETURN_BODY(,, M)

#define VEC2I_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<vec2i3_6_0*>(data_ptr)-> C); 
#define VEC2I_RETURN_REF(T, M)    VEC2I_BASE_RETURN_BODY(T, &, M)
#define VEC2I_RETURN_AS(T, M)     VEC2I_BASE_RETURN_BODY(T,, M)
#define VEC2I_RETURN(M)           VEC2I_BASE_RETURN_BODY(,, M)

#define VEC2S_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<vec2s3_6_0*>(data_ptr)-> C); 
#define VEC2S_RETURN_REF(T, M)    VEC2S_BASE_RETURN_BODY(T, &, M)
#define VEC2S_RETURN_AS(T, M)     VEC2S_BASE_RETURN_BODY(T,, M)
#define VEC2S_RETURN(M)           VEC2S_BASE_RETURN_BODY(,, M)

#define VEC3F_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<vec3f3_6_0*>(data_ptr)-> C); 
#define VEC3F_RETURN_REF(T, M)    VEC3F_BASE_RETURN_BODY(T, &, M)
#define VEC3F_RETURN_AS(T, M)     VEC3F_BASE_RETURN_BODY(T,, M)
#define VEC3F_RETURN(M)           VEC3F_BASE_RETURN_BODY(,, M)

#define VEC3I_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<vec3i4_1_0*>(data_ptr)-> C); 
#define VEC3I_RETURN_REF(T, M)    VEC3I_BASE_RETURN_BODY(T, &, M)
#define VEC3I_RETURN_AS(T, M)     VEC3I_BASE_RETURN_BODY(T,, M)
#define VEC3I_RETURN(M)           VEC3I_BASE_RETURN_BODY(,, M)

#define VEC4F_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<vec4f4_0_0*>(data_ptr)-> C); 
#define VEC4F_RETURN_REF(T, M)    VEC4F_BASE_RETURN_BODY(T, &, M)
#define VEC4F_RETURN_AS(T, M)     VEC4F_BASE_RETURN_BODY(T,, M)
#define VEC4F_RETURN(M)           VEC4F_BASE_RETURN_BODY(,, M)

#define VFONT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<VFont3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<VFont4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<VFont4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<VFont4_4_0*>(data_ptr)-> C); 
#define VFONT_RETURN_REF(T, M)    VFONT_BASE_RETURN_BODY(T, &, M)
#define VFONT_RETURN_AS(T, M)     VFONT_BASE_RETURN_BODY(T,, M)
#define VFONT_RETURN(M)           VFONT_BASE_RETURN_BODY(,, M)

#define VIEW2D_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<View2D3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<View2D4_0_0*>(data_ptr)-> C); 
#define VIEW2D_RETURN_REF(T, M)    VIEW2D_BASE_RETURN_BODY(T, &, M)
#define VIEW2D_RETURN_AS(T, M)     VIEW2D_BASE_RETURN_BODY(T,, M)
#define VIEW2D_RETURN(M)           VIEW2D_BASE_RETURN_BODY(,, M)

#define VIEW3DCURSOR_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<View3DCursor3_6_0*>(data_ptr)-> C); 
#define VIEW3DCURSOR_RETURN_REF(T, M)    VIEW3DCURSOR_BASE_RETURN_BODY(T, &, M)
#define VIEW3DCURSOR_RETURN_AS(T, M)     VIEW3DCURSOR_BASE_RETURN_BODY(T,, M)
#define VIEW3DCURSOR_RETURN(M)           VIEW3DCURSOR_BASE_RETURN_BODY(,, M)

#define VIEW3DOVERLAY_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<View3DOverlay3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<View3DOverlay4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<View3DOverlay4_3_0*>(data_ptr)-> C); 
#define VIEW3DOVERLAY_RETURN_REF(T, M)    VIEW3DOVERLAY_BASE_RETURN_BODY(T, &, M)
#define VIEW3DOVERLAY_RETURN_AS(T, M)     VIEW3DOVERLAY_BASE_RETURN_BODY(T,, M)
#define VIEW3DOVERLAY_RETURN(M)           VIEW3DOVERLAY_BASE_RETURN_BODY(,, M)

#define VIEW3DSHADING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<View3DShading3_6_0*>(data_ptr)-> C); 
#define VIEW3DSHADING_RETURN_REF(T, M)    VIEW3DSHADING_BASE_RETURN_BODY(T, &, M)
#define VIEW3DSHADING_RETURN_AS(T, M)     VIEW3DSHADING_BASE_RETURN_BODY(T,, M)
#define VIEW3DSHADING_RETURN(M)           VIEW3DSHADING_BASE_RETURN_BODY(,, M)

#define VIEW3D_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<View3D3_6_0*>(data_ptr)-> C); 
#define VIEW3D_RETURN_REF(T, M)    VIEW3D_BASE_RETURN_BODY(T, &, M)
#define VIEW3D_RETURN_AS(T, M)     VIEW3D_BASE_RETURN_BODY(T,, M)
#define VIEW3D_RETURN(M)           VIEW3D_BASE_RETURN_BODY(,, M)

#define VIEW3D_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<View3D_Runtime3_6_0*>(data_ptr)-> C); 
#define VIEW3D_RUNTIME_RETURN_REF(T, M)    VIEW3D_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define VIEW3D_RUNTIME_RETURN_AS(T, M)     VIEW3D_RUNTIME_BASE_RETURN_BODY(T,, M)
#define VIEW3D_RUNTIME_RETURN(M)           VIEW3D_RUNTIME_BASE_RETURN_BODY(,, M)

#define VIEWERPATHELEM_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ViewerPathElem3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ViewerPathElem4_0_0*>(data_ptr)-> C); 
#define VIEWERPATHELEM_RETURN_REF(T, M)    VIEWERPATHELEM_BASE_RETURN_BODY(T, &, M)
#define VIEWERPATHELEM_RETURN_AS(T, M)     VIEWERPATHELEM_BASE_RETURN_BODY(T,, M)
#define VIEWERPATHELEM_RETURN(M)           VIEWERPATHELEM_BASE_RETURN_BODY(,, M)

#define VIEWERPATH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ViewerPath3_6_0*>(data_ptr)-> C); 
#define VIEWERPATH_RETURN_REF(T, M)    VIEWERPATH_BASE_RETURN_BODY(T, &, M)
#define VIEWERPATH_RETURN_AS(T, M)     VIEWERPATH_BASE_RETURN_BODY(T,, M)
#define VIEWERPATH_RETURN(M)           VIEWERPATH_BASE_RETURN_BODY(,, M)

#define VIEWLAYERAOV_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ViewLayerAOV3_6_0*>(data_ptr)-> C); 
#define VIEWLAYERAOV_RETURN_REF(T, M)    VIEWLAYERAOV_BASE_RETURN_BODY(T, &, M)
#define VIEWLAYERAOV_RETURN_AS(T, M)     VIEWLAYERAOV_BASE_RETURN_BODY(T,, M)
#define VIEWLAYERAOV_RETURN(M)           VIEWLAYERAOV_BASE_RETURN_BODY(,, M)

#define VIEWLAYEREEVEE_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ViewLayerEEVEE3_6_0*>(data_ptr)-> C); 
#define VIEWLAYEREEVEE_RETURN_REF(T, M)    VIEWLAYEREEVEE_BASE_RETURN_BODY(T, &, M)
#define VIEWLAYEREEVEE_RETURN_AS(T, M)     VIEWLAYEREEVEE_BASE_RETURN_BODY(T,, M)
#define VIEWLAYEREEVEE_RETURN(M)           VIEWLAYEREEVEE_BASE_RETURN_BODY(,, M)

#define VIEWLAYERLIGHTGROUP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<ViewLayerLightgroup3_6_0*>(data_ptr)-> C); 
#define VIEWLAYERLIGHTGROUP_RETURN_REF(T, M)    VIEWLAYERLIGHTGROUP_BASE_RETURN_BODY(T, &, M)
#define VIEWLAYERLIGHTGROUP_RETURN_AS(T, M)     VIEWLAYERLIGHTGROUP_BASE_RETURN_BODY(T,, M)
#define VIEWLAYERLIGHTGROUP_RETURN(M)           VIEWLAYERLIGHTGROUP_BASE_RETURN_BODY(,, M)

#define VIEWLAYER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<ViewLayer3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<ViewLayer4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<ViewLayer4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<ViewLayer4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<ViewLayer4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<ViewLayer4_4_0*>(data_ptr)-> C); 
#define VIEWLAYER_RETURN_REF(T, M)    VIEWLAYER_BASE_RETURN_BODY(T, &, M)
#define VIEWLAYER_RETURN_AS(T, M)     VIEWLAYER_BASE_RETURN_BODY(T,, M)
#define VIEWLAYER_RETURN(M)           VIEWLAYER_BASE_RETURN_BODY(,, M)

#define VOLUMEDISPLACEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<VolumeDisplaceModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<VolumeDisplaceModifierData4_1_0*>(data_ptr)-> C); 
#define VOLUMEDISPLACEMODIFIERDATA_RETURN_REF(T, M)    VOLUMEDISPLACEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define VOLUMEDISPLACEMODIFIERDATA_RETURN_AS(T, M)     VOLUMEDISPLACEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define VOLUMEDISPLACEMODIFIERDATA_RETURN(M)           VOLUMEDISPLACEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define VOLUMEDISPLAY_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<VolumeDisplay3_6_0*>(data_ptr)-> C); 
#define VOLUMEDISPLAY_RETURN_REF(T, M)    VOLUMEDISPLAY_BASE_RETURN_BODY(T, &, M)
#define VOLUMEDISPLAY_RETURN_AS(T, M)     VOLUMEDISPLAY_BASE_RETURN_BODY(T,, M)
#define VOLUMEDISPLAY_RETURN(M)           VOLUMEDISPLAY_BASE_RETURN_BODY(,, M)

#define VOLUMERENDER_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<VolumeRender3_6_0*>(data_ptr)-> C); 
#define VOLUMERENDER_RETURN_REF(T, M)    VOLUMERENDER_BASE_RETURN_BODY(T, &, M)
#define VOLUMERENDER_RETURN_AS(T, M)     VOLUMERENDER_BASE_RETURN_BODY(T,, M)
#define VOLUMERENDER_RETURN(M)           VOLUMERENDER_BASE_RETURN_BODY(,, M)

#define VOLUMETOMESHMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<VolumeToMeshModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<VolumeToMeshModifierData4_1_0*>(data_ptr)-> C); 
#define VOLUMETOMESHMODIFIERDATA_RETURN_REF(T, M)    VOLUMETOMESHMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define VOLUMETOMESHMODIFIERDATA_RETURN_AS(T, M)     VOLUMETOMESHMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define VOLUMETOMESHMODIFIERDATA_RETURN(M)           VOLUMETOMESHMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define VOLUME_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<Volume3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<Volume4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<Volume4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<Volume4_4_0*>(data_ptr)-> C); 
#define VOLUME_RETURN_REF(T, M)    VOLUME_BASE_RETURN_BODY(T, &, M)
#define VOLUME_RETURN_AS(T, M)     VOLUME_BASE_RETURN_BODY(T,, M)
#define VOLUME_RETURN(M)           VOLUME_BASE_RETURN_BODY(,, M)

#define VOLUME_RUNTIME_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<Volume_Runtime3_6_0*>(data_ptr)-> C); 
#define VOLUME_RUNTIME_RETURN_REF(T, M)    VOLUME_RUNTIME_BASE_RETURN_BODY(T, &, M)
#define VOLUME_RUNTIME_RETURN_AS(T, M)     VOLUME_RUNTIME_BASE_RETURN_BODY(T,, M)
#define VOLUME_RUNTIME_RETURN(M)           VOLUME_RUNTIME_BASE_RETURN_BODY(,, M)

#define VPAINT_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<VPaint3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<VPaint4_1_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<VPaint4_3_0*>(data_ptr)-> C); 
#define VPAINT_RETURN_REF(T, M)    VPAINT_BASE_RETURN_BODY(T, &, M)
#define VPAINT_RETURN_AS(T, M)     VPAINT_BASE_RETURN_BODY(T,, M)
#define VPAINT_RETURN(M)           VPAINT_BASE_RETURN_BODY(,, M)

#define WALKNAVIGATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WalkNavigation3_6_0*>(data_ptr)-> C); 
#define WALKNAVIGATION_RETURN_REF(T, M)    WALKNAVIGATION_BASE_RETURN_BODY(T, &, M)
#define WALKNAVIGATION_RETURN_AS(T, M)     WALKNAVIGATION_BASE_RETURN_BODY(T,, M)
#define WALKNAVIGATION_RETURN(M)           WALKNAVIGATION_BASE_RETURN_BODY(,, M)

#define WARPMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WarpModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WarpModifierData4_1_0*>(data_ptr)-> C); 
#define WARPMODIFIERDATA_RETURN_REF(T, M)    WARPMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WARPMODIFIERDATA_RETURN_AS(T, M)     WARPMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WARPMODIFIERDATA_RETURN(M)           WARPMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WAVEEFF_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WaveEff3_6_0*>(data_ptr)-> C); 
#define WAVEEFF_RETURN_REF(T, M)    WAVEEFF_BASE_RETURN_BODY(T, &, M)
#define WAVEEFF_RETURN_AS(T, M)     WAVEEFF_BASE_RETURN_BODY(T,, M)
#define WAVEEFF_RETURN(M)           WAVEEFF_BASE_RETURN_BODY(,, M)

#define WAVEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WaveModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WaveModifierData4_1_0*>(data_ptr)-> C); 
#define WAVEMODIFIERDATA_RETURN_REF(T, M)    WAVEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WAVEMODIFIERDATA_RETURN_AS(T, M)     WAVEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WAVEMODIFIERDATA_RETURN(M)           WAVEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WAVESHADERFXDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WaveShaderFxData3_6_0*>(data_ptr)-> C); 
#define WAVESHADERFXDATA_RETURN_REF(T, M)    WAVESHADERFXDATA_BASE_RETURN_BODY(T, &, M)
#define WAVESHADERFXDATA_RETURN_AS(T, M)     WAVESHADERFXDATA_BASE_RETURN_BODY(T,, M)
#define WAVESHADERFXDATA_RETURN(M)           WAVESHADERFXDATA_BASE_RETURN_BODY(,, M)

#define WEIGHTANGLEGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WeightAngleGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define WEIGHTANGLEGPENCILMODIFIERDATA_RETURN_REF(T, M)    WEIGHTANGLEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WEIGHTANGLEGPENCILMODIFIERDATA_RETURN_AS(T, M)     WEIGHTANGLEGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WEIGHTANGLEGPENCILMODIFIERDATA_RETURN(M)           WEIGHTANGLEGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WEIGHTEDNORMALMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WeightedNormalModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WeightedNormalModifierData4_1_0*>(data_ptr)-> C); 
#define WEIGHTEDNORMALMODIFIERDATA_RETURN_REF(T, M)    WEIGHTEDNORMALMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WEIGHTEDNORMALMODIFIERDATA_RETURN_AS(T, M)     WEIGHTEDNORMALMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WEIGHTEDNORMALMODIFIERDATA_RETURN(M)           WEIGHTEDNORMALMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WEIGHTPROXGPENCILMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WeightProxGpencilModifierData3_6_0*>(data_ptr)-> C); 
#define WEIGHTPROXGPENCILMODIFIERDATA_RETURN_REF(T, M)    WEIGHTPROXGPENCILMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WEIGHTPROXGPENCILMODIFIERDATA_RETURN_AS(T, M)     WEIGHTPROXGPENCILMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WEIGHTPROXGPENCILMODIFIERDATA_RETURN(M)           WEIGHTPROXGPENCILMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WEIGHTVGEDITMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WeightVGEditModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WeightVGEditModifierData4_1_0*>(data_ptr)-> C); 
#define WEIGHTVGEDITMODIFIERDATA_RETURN_REF(T, M)    WEIGHTVGEDITMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WEIGHTVGEDITMODIFIERDATA_RETURN_AS(T, M)     WEIGHTVGEDITMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WEIGHTVGEDITMODIFIERDATA_RETURN(M)           WEIGHTVGEDITMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WEIGHTVGMIXMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WeightVGMixModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WeightVGMixModifierData4_1_0*>(data_ptr)-> C); 
#define WEIGHTVGMIXMODIFIERDATA_RETURN_REF(T, M)    WEIGHTVGMIXMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WEIGHTVGMIXMODIFIERDATA_RETURN_AS(T, M)     WEIGHTVGMIXMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WEIGHTVGMIXMODIFIERDATA_RETURN(M)           WEIGHTVGMIXMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WEIGHTVGPROXIMITYMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WeightVGProximityModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WeightVGProximityModifierData4_1_0*>(data_ptr)-> C); 
#define WEIGHTVGPROXIMITYMODIFIERDATA_RETURN_REF(T, M)    WEIGHTVGPROXIMITYMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WEIGHTVGPROXIMITYMODIFIERDATA_RETURN_AS(T, M)     WEIGHTVGPROXIMITYMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WEIGHTVGPROXIMITYMODIFIERDATA_RETURN(M)           WEIGHTVGPROXIMITYMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WELDMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WeldModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WeldModifierData4_1_0*>(data_ptr)-> C); 
#define WELDMODIFIERDATA_RETURN_REF(T, M)    WELDMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WELDMODIFIERDATA_RETURN_AS(T, M)     WELDMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WELDMODIFIERDATA_RETURN(M)           WELDMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WHITEBALANCEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<WhiteBalanceModifierData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WhiteBalanceModifierData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<WhiteBalanceModifierData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<WhiteBalanceModifierData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WhiteBalanceModifierData4_4_0*>(data_ptr)-> C); 
#define WHITEBALANCEMODIFIERDATA_RETURN_REF(T, M)    WHITEBALANCEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WHITEBALANCEMODIFIERDATA_RETURN_AS(T, M)     WHITEBALANCEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WHITEBALANCEMODIFIERDATA_RETURN(M)           WHITEBALANCEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WIPEVARS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WipeVars3_6_0*>(data_ptr)-> C); 
#define WIPEVARS_RETURN_REF(T, M)    WIPEVARS_BASE_RETURN_BODY(T, &, M)
#define WIPEVARS_RETURN_AS(T, M)     WIPEVARS_BASE_RETURN_BODY(T,, M)
#define WIPEVARS_RETURN(M)           WIPEVARS_BASE_RETURN_BODY(,, M)

#define WIREFRAMEMODIFIERDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WireframeModifierData3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WireframeModifierData4_1_0*>(data_ptr)-> C); 
#define WIREFRAMEMODIFIERDATA_RETURN_REF(T, M)    WIREFRAMEMODIFIERDATA_BASE_RETURN_BODY(T, &, M)
#define WIREFRAMEMODIFIERDATA_RETURN_AS(T, M)     WIREFRAMEMODIFIERDATA_BASE_RETURN_BODY(T,, M)
#define WIREFRAMEMODIFIERDATA_RETURN(M)           WIREFRAMEMODIFIERDATA_BASE_RETURN_BODY(,, M)

#define WMKEYCONFIGPREF_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<wmKeyConfigPref3_6_0*>(data_ptr)-> C); 
#define WMKEYCONFIGPREF_RETURN_REF(T, M)    WMKEYCONFIGPREF_BASE_RETURN_BODY(T, &, M)
#define WMKEYCONFIGPREF_RETURN_AS(T, M)     WMKEYCONFIGPREF_BASE_RETURN_BODY(T,, M)
#define WMKEYCONFIGPREF_RETURN(M)           WMKEYCONFIGPREF_BASE_RETURN_BODY(,, M)

#define WMKEYCONFIG_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<wmKeyConfig3_6_0*>(data_ptr)-> C); 
#define WMKEYCONFIG_RETURN_REF(T, M)    WMKEYCONFIG_BASE_RETURN_BODY(T, &, M)
#define WMKEYCONFIG_RETURN_AS(T, M)     WMKEYCONFIG_BASE_RETURN_BODY(T,, M)
#define WMKEYCONFIG_RETURN(M)           WMKEYCONFIG_BASE_RETURN_BODY(,, M)

#define WMKEYMAPDIFFITEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<wmKeyMapDiffItem3_6_0*>(data_ptr)-> C); 
#define WMKEYMAPDIFFITEM_RETURN_REF(T, M)    WMKEYMAPDIFFITEM_BASE_RETURN_BODY(T, &, M)
#define WMKEYMAPDIFFITEM_RETURN_AS(T, M)     WMKEYMAPDIFFITEM_BASE_RETURN_BODY(T,, M)
#define WMKEYMAPDIFFITEM_RETURN(M)           WMKEYMAPDIFFITEM_BASE_RETURN_BODY(,, M)

#define WMKEYMAPITEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<wmKeyMapItem3_6_0*>(data_ptr)-> C); 
#define WMKEYMAPITEM_RETURN_REF(T, M)    WMKEYMAPITEM_BASE_RETURN_BODY(T, &, M)
#define WMKEYMAPITEM_RETURN_AS(T, M)     WMKEYMAPITEM_BASE_RETURN_BODY(T,, M)
#define WMKEYMAPITEM_RETURN(M)           WMKEYMAPITEM_BASE_RETURN_BODY(,, M)

#define WMOPERATORTYPEMACRO_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<wmOperatorTypeMacro3_6_0*>(data_ptr)-> C); 
#define WMOPERATORTYPEMACRO_RETURN_REF(T, M)    WMOPERATORTYPEMACRO_BASE_RETURN_BODY(T, &, M)
#define WMOPERATORTYPEMACRO_RETURN_AS(T, M)     WMOPERATORTYPEMACRO_BASE_RETURN_BODY(T,, M)
#define WMOPERATORTYPEMACRO_RETURN(M)           WMOPERATORTYPEMACRO_BASE_RETURN_BODY(,, M)

#define WMOPERATOR_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<wmOperator3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<wmOperator4_1_0*>(data_ptr)-> C); 
#define WMOPERATOR_RETURN_REF(T, M)    WMOPERATOR_BASE_RETURN_BODY(T, &, M)
#define WMOPERATOR_RETURN_AS(T, M)     WMOPERATOR_BASE_RETURN_BODY(T,, M)
#define WMOPERATOR_RETURN(M)           WMOPERATOR_BASE_RETURN_BODY(,, M)

#define WMOWNERID_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_2_1) \
        return A ( B reinterpret_cast<wmOwnerID3_6_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<wmOwnerID4_2_1*>(data_ptr)-> C); 
#define WMOWNERID_RETURN_REF(T, M)    WMOWNERID_BASE_RETURN_BODY(T, &, M)
#define WMOWNERID_RETURN_AS(T, M)     WMOWNERID_BASE_RETURN_BODY(T,, M)
#define WMOWNERID_RETURN(M)           WMOWNERID_BASE_RETURN_BODY(,, M)

#define WMWINDOWMANAGER_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<wmWindowManager3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<wmWindowManager4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<wmWindowManager4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_1) \
        return A ( B reinterpret_cast<wmWindowManager4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_4) \
        return A ( B reinterpret_cast<wmWindowManager4_2_1*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<wmWindowManager4_2_4*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<wmWindowManager4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<wmWindowManager4_4_0*>(data_ptr)-> C); 
#define WMWINDOWMANAGER_RETURN_REF(T, M)    WMWINDOWMANAGER_BASE_RETURN_BODY(T, &, M)
#define WMWINDOWMANAGER_RETURN_AS(T, M)     WMWINDOWMANAGER_BASE_RETURN_BODY(T,, M)
#define WMWINDOWMANAGER_RETURN(M)           WMWINDOWMANAGER_BASE_RETURN_BODY(,, M)

#define WMWINDOW_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<wmWindow3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<wmWindow4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<wmWindow4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<wmWindow4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<wmWindow4_4_0*>(data_ptr)-> C); 
#define WMWINDOW_RETURN_REF(T, M)    WMWINDOW_BASE_RETURN_BODY(T, &, M)
#define WMWINDOW_RETURN_AS(T, M)     WMWINDOW_BASE_RETURN_BODY(T,, M)
#define WMWINDOW_RETURN(M)           WMWINDOW_BASE_RETURN_BODY(,, M)

#define WMXRDATA_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<wmXrData3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<wmXrData4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<wmXrData4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<wmXrData4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<wmXrData4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<wmXrData4_4_0*>(data_ptr)-> C); 
#define WMXRDATA_RETURN_REF(T, M)    WMXRDATA_BASE_RETURN_BODY(T, &, M)
#define WMXRDATA_RETURN_AS(T, M)     WMXRDATA_BASE_RETURN_BODY(T,, M)
#define WMXRDATA_RETURN(M)           WMXRDATA_BASE_RETURN_BODY(,, M)

#define WORKSPACEDATARELATION_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WorkSpaceDataRelation3_6_0*>(data_ptr)-> C); 
#define WORKSPACEDATARELATION_RETURN_REF(T, M)    WORKSPACEDATARELATION_BASE_RETURN_BODY(T, &, M)
#define WORKSPACEDATARELATION_RETURN_AS(T, M)     WORKSPACEDATARELATION_BASE_RETURN_BODY(T,, M)
#define WORKSPACEDATARELATION_RETURN(M)           WORKSPACEDATARELATION_BASE_RETURN_BODY(,, M)

#define WORKSPACEINSTANCEHOOK_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WorkSpaceInstanceHook3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<WorkSpaceInstanceHook4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<WorkSpaceInstanceHook4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WorkSpaceInstanceHook4_4_0*>(data_ptr)-> C); 
#define WORKSPACEINSTANCEHOOK_RETURN_REF(T, M)    WORKSPACEINSTANCEHOOK_BASE_RETURN_BODY(T, &, M)
#define WORKSPACEINSTANCEHOOK_RETURN_AS(T, M)     WORKSPACEINSTANCEHOOK_BASE_RETURN_BODY(T,, M)
#define WORKSPACEINSTANCEHOOK_RETURN(M)           WORKSPACEINSTANCEHOOK_BASE_RETURN_BODY(,, M)

#define WORKSPACELAYOUT_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<WorkSpaceLayout3_6_0*>(data_ptr)-> C); 
#define WORKSPACELAYOUT_RETURN_REF(T, M)    WORKSPACELAYOUT_BASE_RETURN_BODY(T, &, M)
#define WORKSPACELAYOUT_RETURN_AS(T, M)     WORKSPACELAYOUT_BASE_RETURN_BODY(T,, M)
#define WORKSPACELAYOUT_RETURN(M)           WORKSPACELAYOUT_BASE_RETURN_BODY(,, M)

#define WORKSPACE_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<WorkSpace3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<WorkSpace4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<WorkSpace4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<WorkSpace4_4_0*>(data_ptr)-> C); 
#define WORKSPACE_RETURN_REF(T, M)    WORKSPACE_BASE_RETURN_BODY(T, &, M)
#define WORKSPACE_RETURN_AS(T, M)     WORKSPACE_BASE_RETURN_BODY(T,, M)
#define WORKSPACE_RETURN(M)           WORKSPACE_BASE_RETURN_BODY(,, M)

#define WORLD_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<World3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<World4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<World4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<World4_2_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<World4_4_0*>(data_ptr)-> C); 
#define WORLD_RETURN_REF(T, M)    WORLD_BASE_RETURN_BODY(T, &, M)
#define WORLD_RETURN_AS(T, M)     WORLD_BASE_RETURN_BODY(T,, M)
#define WORLD_RETURN(M)           WORLD_BASE_RETURN_BODY(,, M)

#define XRACTIONMAPBINDING_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<XrActionMapBinding3_6_0*>(data_ptr)-> C); 
#define XRACTIONMAPBINDING_RETURN_REF(T, M)    XRACTIONMAPBINDING_BASE_RETURN_BODY(T, &, M)
#define XRACTIONMAPBINDING_RETURN_AS(T, M)     XRACTIONMAPBINDING_BASE_RETURN_BODY(T,, M)
#define XRACTIONMAPBINDING_RETURN(M)           XRACTIONMAPBINDING_BASE_RETURN_BODY(,, M)

#define XRACTIONMAPITEM_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<XrActionMapItem3_6_0*>(data_ptr)-> C); 
#define XRACTIONMAPITEM_RETURN_REF(T, M)    XRACTIONMAPITEM_BASE_RETURN_BODY(T, &, M)
#define XRACTIONMAPITEM_RETURN_AS(T, M)     XRACTIONMAPITEM_BASE_RETURN_BODY(T,, M)
#define XRACTIONMAPITEM_RETURN(M)           XRACTIONMAPITEM_BASE_RETURN_BODY(,, M)

#define XRACTIONMAP_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<XrActionMap3_6_0*>(data_ptr)-> C); 
#define XRACTIONMAP_RETURN_REF(T, M)    XRACTIONMAP_BASE_RETURN_BODY(T, &, M)
#define XRACTIONMAP_RETURN_AS(T, M)     XRACTIONMAP_BASE_RETURN_BODY(T,, M)
#define XRACTIONMAP_RETURN(M)           XRACTIONMAP_BASE_RETURN_BODY(,, M)

#define XRCOMPONENTPATH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<XrComponentPath3_6_0*>(data_ptr)-> C); 
#define XRCOMPONENTPATH_RETURN_REF(T, M)    XRCOMPONENTPATH_BASE_RETURN_BODY(T, &, M)
#define XRCOMPONENTPATH_RETURN_AS(T, M)     XRCOMPONENTPATH_BASE_RETURN_BODY(T,, M)
#define XRCOMPONENTPATH_RETURN(M)           XRCOMPONENTPATH_BASE_RETURN_BODY(,, M)

#define XRSESSIONSETTINGS_BASE_RETURN_BODY(A, B, C) \
    if (blender_ver < BlenderVersion::VER_4_0_0) \
        return A ( B reinterpret_cast<XrSessionSettings3_6_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_1_0) \
        return A ( B reinterpret_cast<XrSessionSettings4_0_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_2_0) \
        return A ( B reinterpret_cast<XrSessionSettings4_1_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_3_0) \
        return A ( B reinterpret_cast<XrSessionSettings4_2_0*>(data_ptr)-> C); \
    if (blender_ver < BlenderVersion::VER_4_4_0) \
        return A ( B reinterpret_cast<XrSessionSettings4_3_0*>(data_ptr)-> C); \
    return A ( B reinterpret_cast<XrSessionSettings4_4_0*>(data_ptr)-> C); 
#define XRSESSIONSETTINGS_RETURN_REF(T, M)    XRSESSIONSETTINGS_BASE_RETURN_BODY(T, &, M)
#define XRSESSIONSETTINGS_RETURN_AS(T, M)     XRSESSIONSETTINGS_BASE_RETURN_BODY(T,, M)
#define XRSESSIONSETTINGS_RETURN(M)           XRSESSIONSETTINGS_BASE_RETURN_BODY(,, M)

#define XRUSERPATH_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<XrUserPath3_6_0*>(data_ptr)-> C); 
#define XRUSERPATH_RETURN_REF(T, M)    XRUSERPATH_BASE_RETURN_BODY(T, &, M)
#define XRUSERPATH_RETURN_AS(T, M)     XRUSERPATH_BASE_RETURN_BODY(T,, M)
#define XRUSERPATH_RETURN(M)           XRUSERPATH_BASE_RETURN_BODY(,, M)

#define _MBSTATET_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<_Mbstatet3_6_0*>(data_ptr)-> C); 
#define _MBSTATET_RETURN_REF(T, M)    _MBSTATET_BASE_RETURN_BODY(T, &, M)
#define _MBSTATET_RETURN_AS(T, M)     _MBSTATET_BASE_RETURN_BODY(T,, M)
#define _MBSTATET_RETURN(M)           _MBSTATET_BASE_RETURN_BODY(,, M)

#define __CRT_LOCALE_DATA_PUBLIC_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<__crt_locale_data_public3_6_0*>(data_ptr)-> C); 
#define __CRT_LOCALE_DATA_PUBLIC_RETURN_REF(T, M)    __CRT_LOCALE_DATA_PUBLIC_BASE_RETURN_BODY(T, &, M)
#define __CRT_LOCALE_DATA_PUBLIC_RETURN_AS(T, M)     __CRT_LOCALE_DATA_PUBLIC_BASE_RETURN_BODY(T,, M)
#define __CRT_LOCALE_DATA_PUBLIC_RETURN(M)           __CRT_LOCALE_DATA_PUBLIC_BASE_RETURN_BODY(,, M)

#define __CRT_LOCALE_POINTERS_BASE_RETURN_BODY(A, B, C) \
    return A ( B reinterpret_cast<__crt_locale_pointers3_6_0*>(data_ptr)-> C); 
#define __CRT_LOCALE_POINTERS_RETURN_REF(T, M)    __CRT_LOCALE_POINTERS_BASE_RETURN_BODY(T, &, M)
#define __CRT_LOCALE_POINTERS_RETURN_AS(T, M)     __CRT_LOCALE_POINTERS_BASE_RETURN_BODY(T,, M)
#define __CRT_LOCALE_POINTERS_RETURN(M)           __CRT_LOCALE_POINTERS_BASE_RETURN_BODY(,, M)

#endif