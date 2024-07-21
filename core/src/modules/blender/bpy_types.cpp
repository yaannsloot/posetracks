/*
Copyright (C) 2024 Ian Sloat

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
*/

#include "bpy_types.hpp"

// Global compatability mode (can be changed)
BlenderVersion blender_ver = BlenderVersion::VER_2_93_0;

void set_compatibility_mode(BlenderVersion version) {
	blender_ver = version;
}

BlenderVersion get_compatability_mode() {
	return blender_ver;
}

// -------------------- MovieTrackingMarker --------------------

#define MARKER_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<MovieTrackingMarker2_93_0*>(data_ptr)-> C);

#define MARKER_RETURN(M) MARKER_BASE_RETURN_BODY(,, M)

float* MovieTrackingMarker::pos() const 
{
	MARKER_RETURN(pos)
}

Corners MovieTrackingMarker::pattern_corners() const
{
	MARKER_RETURN(pattern_corners)
}

float* MovieTrackingMarker::search_min() const
{
	MARKER_RETURN(search_min)
}

float* MovieTrackingMarker::search_max() const
{
	MARKER_RETURN(search_max)
}

int& MovieTrackingMarker::framenr() const
{
	MARKER_RETURN(framenr)
}

int& MovieTrackingMarker::flag() const
{
	MARKER_RETURN(flag)
}

// -------------------- MovieTrackingTrack --------------------

#define TRACK_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_6_0) \
		return A ( B reinterpret_cast<MovieTrackingTrack2_93_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieTrackingTrack3_6_0*>(data_ptr)-> C);

#define TRACK_BASE_RETURN_BODY_LEGACY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_6_0) \
		return A ( B reinterpret_cast<MovieTrackingTrack2_93_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieTrackingTrack3_6_0*>(data_ptr)-> C##_legacy);

#define TRACK_RETURN_REF(T, C) TRACK_BASE_RETURN_BODY(T, &, C)
#define TRACK_RETURN_AS(T, C) TRACK_BASE_RETURN_BODY(T,, C)
#define TRACK_RETURN(M) TRACK_BASE_RETURN_BODY(,, M)
#define TRACK_RETURN_LEGACY(M) TRACK_BASE_RETURN_BODY_LEGACY(,, M)

MovieTrackingTrack MovieTrackingTrack::next() const
{
	TRACK_RETURN_AS(MovieTrackingTrack, next)
}

MovieTrackingTrack MovieTrackingTrack::prev() const
{
	TRACK_RETURN_AS(MovieTrackingTrack, prev)
}

const std::string MovieTrackingTrack::name() const
{
	TRACK_RETURN_AS(std::string, name)
}

float* MovieTrackingTrack::offset() const
{
	TRACK_RETURN(offset)
}

const int MovieTrackingTrack::markersnr() const
{
	TRACK_RETURN(markersnr)
}

MovieTrackingMarker MovieTrackingTrack::markers(size_t idx) const
{
	TRACK_RETURN_REF(MovieTrackingMarker, markers[idx])
}

float* MovieTrackingTrack::bundle_pos() const
{
	TRACK_RETURN(bundle_pos)
}

float& MovieTrackingTrack::error() const
{
	TRACK_RETURN(error)
}

int& MovieTrackingTrack::flag() const
{
	TRACK_RETURN(flag)
}

int& MovieTrackingTrack::pat_flag() const
{
	TRACK_RETURN(pat_flag)
}

int& MovieTrackingTrack::search_flag() const
{
	TRACK_RETURN(search_flag)
}

float* MovieTrackingTrack::color() const
{
	TRACK_RETURN(color)
}

short& MovieTrackingTrack::frames_limit() const
{
	TRACK_RETURN(frames_limit)
}

short& MovieTrackingTrack::margin() const
{
	TRACK_RETURN(margin)
}

short& MovieTrackingTrack::pattern_match() const
{
	TRACK_RETURN(pattern_match)
}

short& MovieTrackingTrack::motion_model() const
{
	TRACK_RETURN(motion_model)
}

int& MovieTrackingTrack::algorithm_flag() const
{
	TRACK_RETURN(algorithm_flag)
}

float& MovieTrackingTrack::minimum_correlation() const
{
	TRACK_RETURN(minimum_correlation)
}

bGPdata MovieTrackingTrack::gpd() const
{
	TRACK_RETURN_AS(bGPdata, gpd)
}

float& MovieTrackingTrack::weight() const
{
	TRACK_RETURN(weight)
}

float& MovieTrackingTrack::weight_stab() const
{
	TRACK_RETURN(weight_stab)
}

// -------------------- MovieTrackingObject --------------------

#define TOBJECT_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<MovieTrackingObject2_93_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieTrackingObject3_5_0*>(data_ptr)-> C);

#define TOBJECT_RETURN_REF(T, M) TOBJECT_BASE_RETURN_BODY(T, &, M)
#define TOBJECT_RETURN_AS(T, M) TOBJECT_BASE_RETURN_BODY(T,, M)
#define TOBJECT_RETURN(M) TOBJECT_BASE_RETURN_BODY(,, M)

MovieTrackingObject MovieTrackingObject::next() const
{
	TOBJECT_RETURN_AS(MovieTrackingObject, next)
}

MovieTrackingObject MovieTrackingObject::prev() const
{
	TOBJECT_RETURN_AS(MovieTrackingObject, prev)
}

const std::string MovieTrackingObject::name() const
{
	TOBJECT_RETURN_AS(std::string, name)
}

int& MovieTrackingObject::flag() const
{
	TOBJECT_RETURN(flag)
}

float& MovieTrackingObject::scale() const
{
	TOBJECT_RETURN(scale)
}

ListBase<MovieTrackingTrack> MovieTrackingObject::tracks() const
{
	TOBJECT_RETURN_REF(ListBase<MovieTrackingTrack>, tracks)
}

ListBase<MovieTrackingPlaneTrack> MovieTrackingObject::plane_tracks() const
{
	TOBJECT_RETURN_REF(ListBase<MovieTrackingPlaneTrack>, plane_tracks)
}

MovieTrackingReconstruction MovieTrackingObject::reconstruction() const
{
	TOBJECT_RETURN_REF(MovieTrackingReconstruction, reconstruction)
}

int& MovieTrackingObject::keyframe1() const
{
	TOBJECT_RETURN(keyframe1)
}

int& MovieTrackingObject::keyframe2() const
{
	TOBJECT_RETURN(keyframe2)
}

// -------------------- MovieTracking --------------------

#define TRACKING_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<MovieTracking2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_6_0) \
		return A ( B reinterpret_cast<MovieTracking3_5_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieTracking3_6_0*>(data_ptr)-> C);

#define TRACKING_BASE_RETURN_BODY_LEGACY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<MovieTracking2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_6_0) \
		return A ( B reinterpret_cast<MovieTracking3_5_0*>(data_ptr)-> C##_legacy); \
	return A ( B reinterpret_cast<MovieTracking3_6_0*>(data_ptr)-> C##_legacy);

#define TRACKING_RETURN_REF(T, M) TRACKING_BASE_RETURN_BODY(T, &, M)
#define TRACKING_RETURN_REF_LEGACY(T, M) TRACKING_BASE_RETURN_BODY_LEGACY(T, &, M)
#define TRACKING_RETURN_AS(T, M) TRACKING_BASE_RETURN_BODY(T,, M)
#define TRACKING_RETURN_AS_LEGACY(T, M) TRACKING_BASE_RETURN_BODY_LEGACY(T,, M)
#define TRACKING_RETURN(M) TRACKING_BASE_RETURN_BODY(,, M)

MovieTrackingSettings MovieTracking::settings() const
{
	TRACKING_RETURN_REF(MovieTrackingSettings, settings)
}

MovieTrackingCamera MovieTracking::camera() const
{
	TRACKING_RETURN_REF(MovieTrackingCamera, camera)
}

ListBase<MovieTrackingTrack> MovieTracking::tracks() const
{
	TRACKING_RETURN_REF_LEGACY(ListBase<MovieTrackingTrack>, tracks)
}

ListBase<MovieTrackingPlaneTrack> MovieTracking::plane_tracks() const
{
	TRACKING_RETURN_REF_LEGACY(ListBase<MovieTrackingPlaneTrack>, plane_tracks)
}

MovieTrackingReconstruction MovieTracking::reconstruction() const
{
	TRACKING_RETURN_REF_LEGACY(MovieTrackingReconstruction, reconstruction)
}

MovieTrackingStabilization MovieTracking::stabilization() const
{
	TRACKING_RETURN_REF(MovieTrackingStabilization, stabilization)
}

MovieTrackingTrack MovieTracking::active_track() const
{
	TRACKING_RETURN_AS_LEGACY(MovieTrackingTrack, act_track)
}

MovieTrackingPlaneTrack MovieTracking::active_plane_track() const
{
	TRACKING_RETURN_AS_LEGACY(MovieTrackingPlaneTrack, act_plane_track)
}

ListBase<MovieTrackingObject> MovieTracking::objects() const
{
	TRACKING_RETURN_REF(ListBase<MovieTrackingObject>, objects)
}

const int MovieTracking::objectnr() const
{
	TRACKING_RETURN(objectnr)
}

const int MovieTracking::tot_objects() const
{
	TRACKING_RETURN(tot_object)
}

MovieTrackingStats MovieTracking::stats() const
{
	TRACKING_RETURN_AS(MovieTrackingStats, stats)
}

MovieTrackingDopesheet MovieTracking::dopesheet()
{
	TRACKING_RETURN_REF(MovieTrackingDopesheet, dopesheet)
}

// -------------------- MovieClip --------------------

#define MOVIECLIP_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_0_0) \
		return A ( B reinterpret_cast<MovieClip2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_2_0) \
		return A ( B reinterpret_cast<MovieClip3_0_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_4_0) \
		return A ( B reinterpret_cast<MovieClip3_2_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<MovieClip3_4_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_6_0) \
		return A ( B reinterpret_cast<MovieClip3_5_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_1_0) \
		return A ( B reinterpret_cast<MovieClip3_6_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_2_0) \
		return A ( B reinterpret_cast<MovieClip4_1_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieClip4_2_0*>(data_ptr)-> C);

#define MOVIECLIP_RETURN_REF(T, M) MOVIECLIP_BASE_RETURN_BODY(T, &, M)
#define MOVIECLIP_RETURN_AS(T, M) MOVIECLIP_BASE_RETURN_BODY(T,, M)
#define MOVIECLIP_RETURN(M) MOVIECLIP_BASE_RETURN_BODY(,, M)

ID<MovieClip> MovieClip::id() const
{
	MOVIECLIP_RETURN_REF(ID<MovieClip>, id)
}

AnimData MovieClip::anim_data() const
{
	MOVIECLIP_RETURN_AS(AnimData, adt)
}

const std::string MovieClip::filepath() const
{
	MOVIECLIP_RETURN_AS(std::string, filepath)
}

int MovieClip::source() const
{
	MOVIECLIP_RETURN(source)
}

int *MovieClip::last_size() const
{

	MOVIECLIP_RETURN(lastsize)
}

float MovieClip::aspect_x() const
{
	MOVIECLIP_RETURN(aspx)
}

float MovieClip::aspect_y() const
{
	MOVIECLIP_RETURN(aspy)
}

bGPdata MovieClip::grease_pencil_data() const
{
	MOVIECLIP_RETURN_AS(bGPdata, gpd)
}

MovieTracking MovieClip::tracking() const
{
	MOVIECLIP_RETURN_REF(MovieTracking, tracking)
}

int MovieClip::flag() const
{
	MOVIECLIP_RETURN(flag)
}

int MovieClip::length() const
{
	MOVIECLIP_RETURN(len)
}

int &MovieClip::start_frame() const
{
	MOVIECLIP_RETURN(start_frame)
}

int &MovieClip::frame_offset() const
{
	MOVIECLIP_RETURN(frame_offset)
}

ColorManagedColorspaceSettings MovieClip::colorspace_settings() const
{
	MOVIECLIP_RETURN_REF(ColorManagedColorspaceSettings, colorspace_settings)
}

MovieClip_Runtime MovieClip::runtime() const
{
	MOVIECLIP_RETURN_REF(MovieClip_Runtime, runtime)
}

// -------------------- MaterialSlot --------------------

Material MaterialSlot::material() const 
{
	return Material(materials[mat_idx]);
}

bool MaterialSlot::is_linked() const 
{
	return (matbits[mat_idx / 8] & (1 << (mat_idx % 8))) != 0;
}

size_t MaterialSlotArray::size() const 
{
	return totcol;
}

MaterialSlot MaterialSlotArray::operator[](const size_t idx) const 
{
	return MaterialSlot(materials, matbits, idx);
}

// -------------------- Object --------------------

#define OBJECT_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_2_93_4) \
		return A ( B reinterpret_cast<Object2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_0_0) \
		return A ( B reinterpret_cast<Object2_93_4*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<Object3_0_0*>(data_ptr)-> C);

#define OBJECT_RETURN_REF(T, M) OBJECT_BASE_RETURN_BODY(T, &, M)
#define OBJECT_RETURN_AS(T, M) OBJECT_BASE_RETURN_BODY(T,, M)
#define OBJECT_RETURN(M) OBJECT_BASE_RETURN_BODY(,, M)

ID<Object> Object::id() const 
{
	OBJECT_RETURN_REF(ID<Object>, id)
}

AnimData Object::adt() const
{
	OBJECT_RETURN_AS(AnimData, adt);
}

DrawDataList Object::drawdata() const
{
	OBJECT_RETURN_REF(DrawDataList, drawdata)
}

short Object::type() const
{
	OBJECT_RETURN(type)
}

short Object::partype() const
{
	OBJECT_RETURN(partype)
}

int Object::par1() const
{
	OBJECT_RETURN(par1)
}

int Object::par2() const
{
	OBJECT_RETURN(par2)
}

int Object::par3() const
{
	OBJECT_RETURN(par3)
}

std::string Object::parsubstr() const
{
	OBJECT_RETURN_AS(std::string, parsubstr)
}

Object Object::parent() const
{
	OBJECT_RETURN_AS(Object, parent);
}

Object Object::track() const
{
	OBJECT_RETURN_AS(Object, track);
}

Object Object::proxy() const
{
	OBJECT_RETURN_AS(Object, proxy);
}

Object Object::proxy_group() const
{
	OBJECT_RETURN_AS(Object, proxy_group);
}

Object Object::proxy_from() const
{
	OBJECT_RETURN_AS(Object, proxy_from);
}

Ipo Object::ipo() const
{
	OBJECT_RETURN_AS(Ipo, ipo);
}

bAction Object::action() const
{
	OBJECT_RETURN_AS(bAction, action);
}

bAction Object::poselib() const
{
	OBJECT_RETURN_AS(bAction, poselib);
}

bPose Object::pose() const
{
	OBJECT_RETURN_AS(bPose, pose);
}

bGPdata Object::gpd() const
{
	OBJECT_RETURN_AS(bGPdata, gpd);
}

bAnimVizSettings Object::avs() const
{
	OBJECT_RETURN_REF(bAnimVizSettings, avs)
}

bMotionPath Object::mpath() const
{
	OBJECT_RETURN_AS(bMotionPath, mpath);
}

ListBase<ModifierData> Object::modifiers() const
{
	OBJECT_RETURN_REF(ListBase<ModifierData>, modifiers)
}

ListBase<GpencilModifierData> Object::greasepencil_modifiers() const
{
	OBJECT_RETURN_REF(ListBase<GpencilModifierData>, greasepencil_modifiers)
}

ListBase<bFaceMap> Object::fmaps() const
{
	OBJECT_RETURN_REF(ListBase<bFaceMap>, fmaps)
}

ListBase<ShaderFxData> Object::shader_fx() const
{
	OBJECT_RETURN_REF(ListBase<ShaderFxData>, shader_fx)
}

int& Object::mode() const
{
	OBJECT_RETURN(mode)
}

int& Object::restore_mode() const
{
	OBJECT_RETURN(restore_mode)
}

MaterialSlotArray Object::materials() const
{
	return MaterialSlotArray(mat(), matbits(), totcol());
}

MaterialSlot Object::active_mat() const
{
	return materials()[actcol()];
}

float* Object::loc() const
{
	OBJECT_RETURN(loc)
}

float* Object::dloc() const
{
	OBJECT_RETURN(dloc)
}

float* Object::scale() const
{
	OBJECT_RETURN(scale)
}

float* Object::dscale() const
{
	OBJECT_RETURN(dscale)
}

float* Object::rot() const
{
	OBJECT_RETURN(rot)
}

float* Object::drot() const
{
	OBJECT_RETURN(drot)
}

float* Object::quat() const
{
	OBJECT_RETURN(quat)
}

float* Object::dquat() const
{
	OBJECT_RETURN(dquat)
}

float* Object::rotAxis() const
{
	OBJECT_RETURN(rotAxis)
}

float* Object::drotAxis() const
{
	OBJECT_RETURN(drotAxis)
}

float& Object::rotAngle() const
{
	OBJECT_RETURN(rotAngle)
}

float& Object::drotAngle() const
{
	OBJECT_RETURN(drotAngle)
}

Mat Object::obmat() const
{
	OBJECT_RETURN(obmat)
}

Mat Object::parentinv() const
{
	OBJECT_RETURN(parentinv)
}

Mat Object::constinv() const
{
	OBJECT_RETURN(constinv)
}

Mat Object::imat() const
{
	OBJECT_RETURN(imat)
}

short& Object::flag() const
{
	OBJECT_RETURN(flag)
}

short& Object::transflag() const
{
	OBJECT_RETURN(transflag)
}

short& Object::protectflag() const
{
	OBJECT_RETURN(protectflag)
}

short& Object::trackflag() const
{
	OBJECT_RETURN(trackflag)
}

short& Object::upflag() const
{
	OBJECT_RETURN(upflag)
}

short& Object::nlaflag() const
{
	OBJECT_RETURN(nlaflag)
}

char& Object::duplicator_visibility_flag() const
{
	OBJECT_RETURN(duplicator_visibility_flag)
}

short& Object::base_flag() const
{
	OBJECT_RETURN(base_flag)
}

unsigned short Object::base_local_view_bits() const
{
	OBJECT_RETURN(base_local_view_bits)
}

unsigned short& Object::col_group() const
{
	OBJECT_RETURN(col_group)
}

unsigned short& Object::col_mask() const
{
	OBJECT_RETURN(col_mask)
}

short& Object::rotmode() const
{
	OBJECT_RETURN(rotmode)
}

char& Object::boundtype() const
{
	OBJECT_RETURN(boundtype)
}

char& Object::collision_boundtype() const
{
	OBJECT_RETURN(collision_boundtype)
}

short& Object::dtx() const
{
	OBJECT_RETURN(dtx)
}

char& Object::dt() const
{
	OBJECT_RETURN(dt)
}

char& Object::empty_drawtype() const
{
	OBJECT_RETURN(empty_drawtype)
}

float& Object::empty_drawsize() const
{
	OBJECT_RETURN(empty_drawsize)
}

float& Object::instance_faces_scale() const
{
	OBJECT_RETURN(instance_faces_scale)
}

short& Object::index() const
{
	OBJECT_RETURN(index)
}

unsigned short& Object::actdef() const
{
	OBJECT_RETURN(actdef)
}

unsigned short& Object::actfmap() const
{
	OBJECT_RETURN(actfmap)
}

float* Object::color() const
{
	OBJECT_RETURN(color)
}

short& Object::softflag() const
{
	OBJECT_RETURN(softflag)
}

char& Object::shapeflag() const
{
	OBJECT_RETURN(shapeflag)
}

short& Object::shapenr() const
{
	OBJECT_RETURN(shapenr)
}

// Add ListBase constraints and particlesystem once exact type is identified

PartDeflect Object::pd() const
{
	OBJECT_RETURN_AS(PartDeflect, pd);
}

SoftBody Object::soft() const
{
	OBJECT_RETURN_AS(SoftBody, soft);
}

Collection Object::instance_collection() const
{
	OBJECT_RETURN_AS(Collection, instance_collection);
}

// Add ListBase pc_ids once exact type is identified

RigidBodyOb Object::rigidbody_object() const
{
	OBJECT_RETURN_AS(RigidBodyOb, rigidbody_object);
}

RigidBodyCon Object::rigidbody_constraint() const
{
	OBJECT_RETURN_AS(RigidBodyCon, rigidbody_constraint);
}

float* Object::ima_ofs() const
{
	OBJECT_RETURN(ima_ofs)
}

ImageUser Object::iuser() const
{
	OBJECT_RETURN_AS(ImageUser, iuser);
}

char& Object::empty_image_visibility_flag() const
{
	OBJECT_RETURN(empty_image_visibility_flag)
}

char& Object::empty_image_depth() const
{
	OBJECT_RETURN(empty_image_depth)
}

char& Object::empty_image_flag() const
{
	OBJECT_RETURN(empty_image_flag)
}

PreviewImage Object::preview() const
{
	OBJECT_RETURN_AS(PreviewImage, preview);
}

ObjectLineArt Object::lineart() const
{
	OBJECT_RETURN_REF(ObjectLineArt, lineart)
}

Object_Rumtime Object::runtime() const
{
	OBJECT_RETURN_REF(Object_Rumtime, runtime)
}

void** Object::mat() const 
{
	OBJECT_RETURN_AS((void**), mat)
}

char* Object::matbits() const 
{
	OBJECT_RETURN(matbits)
}

int Object::totcol() const 
{
	OBJECT_RETURN(totcol)
}

int Object::actcol() const 
{
	OBJECT_RETURN(actcol)
}
