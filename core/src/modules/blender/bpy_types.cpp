/* Copyright (C) 2025 Ian Sloat
* Licensed under the GNU GPLv3 or later. See <https://www.gnu.org/licenses/>. */

#include "bpy_types.hpp"
#include "generated/makesdna_3_6_0.h"
#include "generated/makesdna_4_3_0.h"

// Global compatability mode (can be changed)
BlenderVersion blender_ver = BlenderVersion::VER_3_6_0;

void set_compatibility_mode(BlenderVersion version) {
	blender_ver = version;
}

BlenderVersion get_compatability_mode() {
	return blender_ver;
}

// -------------------- BezTriple --------------------

BezTripleVecs BezTriple::vec() const {
	BEZTRIPLE_RETURN(vec)
}

float& BezTriple::tilt() const {
	BEZTRIPLE_RETURN(tilt)
}

float& BezTriple::weight() const {
	BEZTRIPLE_RETURN(weight)
}

float& BezTriple::radius() const {
	BEZTRIPLE_RETURN(radius)
}

char& BezTriple::ipo() const {
	BEZTRIPLE_RETURN(ipo)
}

uint8_t& BezTriple::h1() const {
	BEZTRIPLE_RETURN(h1)
}

uint8_t& BezTriple::h2() const {
	BEZTRIPLE_RETURN(h2)
}

uint8_t& BezTriple::f1() const {
	BEZTRIPLE_RETURN(f1)
}

uint8_t& BezTriple::f2() const {
	BEZTRIPLE_RETURN(f2)
}

uint8_t& BezTriple::f3() const {
	BEZTRIPLE_RETURN(f3)
}

char& BezTriple::hide() const {
	BEZTRIPLE_RETURN(hide)
}

char& BezTriple::easing() const {
	BEZTRIPLE_RETURN(easing)
}

float& BezTriple::back() const {
	BEZTRIPLE_RETURN(back)
}

float& BezTriple::amplitude() const {
	BEZTRIPLE_RETURN(amplitude)
}

float& BezTriple::period() const {
	BEZTRIPLE_RETURN(period)
}

char& BezTriple::auto_handle_type() const {
	BEZTRIPLE_RETURN(auto_handle_type)
}

// -------------------- FPoint --------------------

float* FPoint::vec() const {
	FPOINT_RETURN(vec)
}

int& FPoint::flag() const {
	FPOINT_RETURN(flag)
}

// -------------------- FCurve --------------------

FCurve FCurve::next() const {
	FCURVE_RETURN_AS(FCurve, next)
}

FCurve FCurve::prev() const {
	FCURVE_RETURN_AS(FCurve, prev)
}

bActionGroup FCurve::grp() const {
	FCURVE_RETURN_AS(bActionGroup, grp)
}

ChannelDriver FCurve::driver() const {
	FCURVE_RETURN_AS(ChannelDriver, driver)
}

// add modifiers when type is identified

BezTriple FCurve::bezt(size_t idx) const {
	FCURVE_RETURN_REF(BezTriple, bezt[idx])
}

FPoint FCurve::fpt(size_t idx) const {
	FCURVE_RETURN_REF(FPoint, fpt[idx])
}

unsigned int FCurve::totvert() const {
	FCURVE_RETURN(totvert)
}

int& FCurve::active_keyframe_index() const {
	FCURVE_RETURN(active_keyframe_index)
}

float FCurve::curval() const {
	FCURVE_RETURN(curval)
}

short& FCurve::flag() const {
	FCURVE_RETURN(flag)
}

short& FCurve::extend() const {
	FCURVE_RETURN(extend)
}

char& FCurve::auto_smoothing() const {
	FCURVE_RETURN(auto_smoothing)
}

int& FCurve::color_mode() const {
	FCURVE_RETURN(color_mode)
}

float* FCurve::color() const {
	FCURVE_RETURN(color)
}

float FCurve::prev_norm_factor() const {
	FCURVE_RETURN(prev_norm_factor)
}

float FCurve::prev_offset() const {
	FCURVE_RETURN(prev_offset)
}

// -------------------- bConstraintChannel --------------------

bConstraintChannel bConstraintChannel::next() const {
	BCONSTRAINT_RETURN_AS(bConstraintChannel, next)
}

bConstraintChannel bConstraintChannel::prev() const {
	BCONSTRAINT_RETURN_AS(bConstraintChannel, prev)
}

Ipo bConstraintChannel::ipo() const {
	BCONSTRAINT_RETURN_AS(Ipo, ipo)
}

short& bConstraintChannel::flag() const {
	BCONSTRAINT_RETURN(flag)
}

std::string bConstraintChannel::name() const {
	BCONSTRAINT_RETURN_AS(std::string, name)
}

// -------------------- bActionChannel --------------------

bActionChannel bActionChannel::next() const {
	BACTIONCHANNEL_RETURN_AS(bActionChannel, next)
}

bActionChannel bActionChannel::prev() const {
	BACTIONCHANNEL_RETURN_AS(bActionChannel, prev)
}

bActionGroup bActionChannel::grp() const {
	BACTIONCHANNEL_RETURN_AS(bActionGroup, grp)
}

Ipo bActionChannel::ipo() const {
	BACTIONCHANNEL_RETURN_AS(Ipo, ipo)
}

ListBase<bConstraintChannel> bActionChannel::constraintChannels() const {
	BACTIONCHANNEL_RETURN_REF(ListBase<bConstraintChannel>, constraintChannels)
}

int& bActionChannel::flag() const {
	BACTIONCHANNEL_RETURN(flag)
}

std::string bActionChannel::name() const {
	BACTIONCHANNEL_RETURN_AS(std::string, name)
}

int& bActionChannel::temp() const {
	BACTIONCHANNEL_RETURN(temp)
}

// -------------------- bActionGroup --------------------

bActionGroup bActionGroup::next() const {
	BACTIONGROUP_RETURN_AS(bActionGroup, next)
}

bActionGroup bActionGroup::prev() const {
	BACTIONGROUP_RETURN_AS(bActionGroup, prev)
}

ListBase<bActionChannel> bActionGroup::channels() const {
	BACTIONGROUP_RETURN_REF(ListBase<bActionChannel>, channels)
}

int& bActionGroup::flag() const {
	BACTIONGROUP_RETURN(flag)
}

int& bActionGroup::customCol() const {
	BACTIONGROUP_RETURN(customCol)
}

std::string bActionGroup::name() const {
	BACTIONGROUP_RETURN_AS(std::string, name)
}

ThemeWireColor bActionGroup::cs() const {
	BACTIONGROUP_RETURN_REF(ThemeWireColor, cs)
}

// -------------------- bAction --------------------

ID<bAction> bAction::id() const {
	BACTION_RETURN_REF(ID<bAction>, id)
}

ListBase<FCurve> bAction::curves() const {
	BACTION_RETURN_REF(ListBase<FCurve>, curves)
}

ListBase<bActionGroup> bAction::groups() const {
	BACTION_RETURN_REF(ListBase<bActionGroup>, groups)
}

// Add markers when type is determined

int& bAction::flag() const {
	BACTION_RETURN(flag)
}

int& bAction::active_marker() const {
	BACTION_RETURN(active_marker)
}

int bAction::idroot() const {
	BACTION_RETURN(idroot)
}

PreviewImage bAction::preview() const {
	BACTION_RETURN_AS(PreviewImage, preview)
}

// -------------------- AnimData --------------------

bAction AnimData::action() const {
	ANIMDATA_RETURN_AS(bAction, action)
}

bAction AnimData::tmpact() const {
	ANIMDATA_RETURN_AS(bAction, tmpact)
}

ListBase<NlaTrack> AnimData::nla_tracks() const {
	ANIMDATA_RETURN_REF(ListBase<NlaTrack>, nla_tracks)
}

NlaTrack AnimData::act_track() const {
	ANIMDATA_RETURN_AS(NlaTrack, act_track)
}

NlaStrip AnimData::actstrip() const {
	ANIMDATA_RETURN_AS(NlaStrip, actstrip)
}

ListBase<ChannelDriver> AnimData::drivers() const {
	ANIMDATA_RETURN_REF(ListBase<ChannelDriver>, drivers)
}

ListBase<AnimOverride> AnimData::overrides() const {
	ANIMDATA_RETURN_REF(ListBase<AnimOverride>, overrides)
}

// -------------------- MovieTrackingCamera --------------------

short& MovieTrackingCamera::distortion_model() const {
	MOVIETRACKINGCAMERA_RETURN(distortion_model)
}

float& MovieTrackingCamera::sensor_width() const {
	MOVIETRACKINGCAMERA_RETURN(sensor_width)
}

float& MovieTrackingCamera::pixel_aspect() const {
	MOVIETRACKINGCAMERA_RETURN(pixel_aspect)
}

float& MovieTrackingCamera::focal() const {
	MOVIETRACKINGCAMERA_RETURN(focal)
}

short& MovieTrackingCamera::units() const {
	MOVIETRACKINGCAMERA_RETURN(units)
}

float& MovieTrackingCamera::k1() const {
	MOVIETRACKINGCAMERA_RETURN(k1)
}

float& MovieTrackingCamera::k2() const {
	MOVIETRACKINGCAMERA_RETURN(k2)
}

float& MovieTrackingCamera::k3() const {
	MOVIETRACKINGCAMERA_RETURN(k3)
}

float& MovieTrackingCamera::division_k1() const {
	MOVIETRACKINGCAMERA_RETURN(division_k1)
}

float& MovieTrackingCamera::division_k2() const {
	MOVIETRACKINGCAMERA_RETURN(division_k2)
}

float& MovieTrackingCamera::nuke_k1() const {
	MOVIETRACKINGCAMERA_RETURN(nuke_k1)
}

float& MovieTrackingCamera::nuke_k2() const {
	MOVIETRACKINGCAMERA_RETURN(nuke_k2)
}

float& MovieTrackingCamera::brown_k1() const {
	MOVIETRACKINGCAMERA_RETURN(brown_k1)
}

float& MovieTrackingCamera::brown_k2() const {
	MOVIETRACKINGCAMERA_RETURN(brown_k2)
}

float& MovieTrackingCamera::brown_k3() const {
	MOVIETRACKINGCAMERA_RETURN(brown_k3)
}

float& MovieTrackingCamera::brown_k4() const {
	MOVIETRACKINGCAMERA_RETURN(brown_k4)
}

float& MovieTrackingCamera::brown_p1() const {
	MOVIETRACKINGCAMERA_RETURN(brown_p1)
}

float& MovieTrackingCamera::brown_p2() const {
	MOVIETRACKINGCAMERA_RETURN(brown_p2)
}

// -------------------- MovieTrackingMarker --------------------

float* MovieTrackingMarker::pos() const {
	MOVIETRACKINGMARKER_RETURN(pos)
}

Corners MovieTrackingMarker::pattern_corners() const {
	MOVIETRACKINGMARKER_RETURN_REF(Corners, pattern_corners)
}

float* MovieTrackingMarker::search_min() const {
	MOVIETRACKINGMARKER_RETURN(search_min)
}

float* MovieTrackingMarker::search_max() const {
	MOVIETRACKINGMARKER_RETURN(search_max)
}

int& MovieTrackingMarker::framenr() const {
	MOVIETRACKINGMARKER_RETURN(framenr)
}

int& MovieTrackingMarker::flag() const {
	MOVIETRACKINGMARKER_RETURN(flag)
}

// -------------------- MovieTrackingTrack --------------------

MovieTrackingTrack MovieTrackingTrack::next() const {
	MOVIETRACKINGTRACK_RETURN_AS(MovieTrackingTrack, next)
}

MovieTrackingTrack MovieTrackingTrack::prev() const {
	MOVIETRACKINGTRACK_RETURN_AS(MovieTrackingTrack, prev)
}

const std::string MovieTrackingTrack::name() const {
	MOVIETRACKINGTRACK_RETURN_AS(std::string, name)
}

float* MovieTrackingTrack::offset() const {
	MOVIETRACKINGTRACK_RETURN(offset)
}

int &MovieTrackingTrack::markersnr() const {
	MOVIETRACKINGTRACK_RETURN(markersnr)
}

MovieTrackingMarker MovieTrackingTrack::marker(size_t idx) const {
	MOVIETRACKINGTRACK_RETURN_REF(MovieTrackingMarker, markers[idx])
}

float* MovieTrackingTrack::bundle_pos() const {
	MOVIETRACKINGTRACK_RETURN(bundle_pos)
}

float& MovieTrackingTrack::error() const {
	MOVIETRACKINGTRACK_RETURN(error)
}

int& MovieTrackingTrack::flag() const {
	MOVIETRACKINGTRACK_RETURN(flag)
}

int& MovieTrackingTrack::pat_flag() const {
	MOVIETRACKINGTRACK_RETURN(pat_flag)
}

int& MovieTrackingTrack::search_flag() const {
	MOVIETRACKINGTRACK_RETURN(search_flag)
}

float* MovieTrackingTrack::color() const {
	MOVIETRACKINGTRACK_RETURN(color)
}

short& MovieTrackingTrack::frames_limit() const {
	MOVIETRACKINGTRACK_RETURN(frames_limit)
}

short& MovieTrackingTrack::margin() const {
	MOVIETRACKINGTRACK_RETURN(margin)
}

short& MovieTrackingTrack::pattern_match() const {
	MOVIETRACKINGTRACK_RETURN(pattern_match)
}

short& MovieTrackingTrack::motion_model() const {
	MOVIETRACKINGTRACK_RETURN(motion_model)
}

int& MovieTrackingTrack::algorithm_flag() const {
	MOVIETRACKINGTRACK_RETURN(algorithm_flag)
}

float& MovieTrackingTrack::minimum_correlation() const {
	MOVIETRACKINGTRACK_RETURN(minimum_correlation)
}

bGPdata MovieTrackingTrack::gpd() const {
	MOVIETRACKINGTRACK_RETURN_AS(bGPdata, gpd)
}

float& MovieTrackingTrack::weight() const {
	MOVIETRACKINGTRACK_RETURN(weight)
}

float& MovieTrackingTrack::weight_stab() const {
	MOVIETRACKINGTRACK_RETURN(weight_stab)
}

// -------------------- MovieTrackingObject --------------------

MovieTrackingObject MovieTrackingObject::next() const {
	MOVIETRACKINGOBJECT_RETURN_AS(MovieTrackingObject, next)
}

MovieTrackingObject MovieTrackingObject::prev() const {
	MOVIETRACKINGOBJECT_RETURN_AS(MovieTrackingObject, prev)
}

const std::string MovieTrackingObject::name() const {
	MOVIETRACKINGOBJECT_RETURN_AS(std::string, name)
}

int& MovieTrackingObject::flag() const {
	MOVIETRACKINGOBJECT_RETURN(flag)
}

float& MovieTrackingObject::scale() const {
	MOVIETRACKINGOBJECT_RETURN(scale)
}

ListBase<MovieTrackingTrack> MovieTrackingObject::tracks() const {
	MOVIETRACKINGOBJECT_RETURN_REF(ListBase<MovieTrackingTrack>, tracks)
}

ListBase<MovieTrackingPlaneTrack> MovieTrackingObject::plane_tracks() const {
	MOVIETRACKINGOBJECT_RETURN_REF(ListBase<MovieTrackingPlaneTrack>, plane_tracks)
}

MovieTrackingReconstruction MovieTrackingObject::reconstruction() const {
	MOVIETRACKINGOBJECT_RETURN_REF(MovieTrackingReconstruction, reconstruction)
}

int& MovieTrackingObject::keyframe1() const {
	MOVIETRACKINGOBJECT_RETURN(keyframe1)
}

int& MovieTrackingObject::keyframe2() const {
	MOVIETRACKINGOBJECT_RETURN(keyframe2)
}

// -------------------- MovieTracking --------------------

MovieTrackingSettings MovieTracking::settings() const {
	MOVIETRACKING_RETURN_REF(MovieTrackingSettings, settings)
}

MovieTrackingCamera MovieTracking::camera() const {
	MOVIETRACKING_RETURN_REF(MovieTrackingCamera, camera)
}

ListBase<MovieTrackingTrack> MovieTracking::tracks() const {
	MOVIETRACKING_RETURN_REF(ListBase<MovieTrackingTrack>, tracks_legacy)
}

ListBase<MovieTrackingPlaneTrack> MovieTracking::plane_tracks() const {
	MOVIETRACKING_RETURN_REF(ListBase<MovieTrackingPlaneTrack>, plane_tracks_legacy)
}

MovieTrackingReconstruction MovieTracking::reconstruction() const {
	MOVIETRACKING_RETURN_REF(MovieTrackingReconstruction, reconstruction_legacy)
}

MovieTrackingStabilization MovieTracking::stabilization() const {
	MOVIETRACKING_RETURN_REF(MovieTrackingStabilization, stabilization)
}

MovieTrackingTrack MovieTracking::active_track() const {
	MOVIETRACKING_RETURN_AS(MovieTrackingTrack, act_track_legacy)
}

MovieTrackingPlaneTrack MovieTracking::active_plane_track() const {
	MOVIETRACKING_RETURN_AS(MovieTrackingPlaneTrack, act_plane_track_legacy)
}

ListBase<MovieTrackingObject> MovieTracking::objects() const {
	MOVIETRACKING_RETURN_REF(ListBase<MovieTrackingObject>, objects)
}

const int MovieTracking::objectnr() const {
	MOVIETRACKING_RETURN(objectnr)
}

const int MovieTracking::tot_objects() const {
	MOVIETRACKING_RETURN(tot_object)
}

MovieTrackingStats MovieTracking::stats() const {
	MOVIETRACKING_RETURN_AS(MovieTrackingStats, stats)
}

MovieTrackingDopesheet MovieTracking::dopesheet() {
	MOVIETRACKING_RETURN_REF(MovieTrackingDopesheet, dopesheet)
}

// -------------------- MovieClipUser --------------------

int& MovieClipUser::framenr() const {
	MOVIECLIPUSER_RETURN(framenr)
}

short& MovieClipUser::render_size() const {
	MOVIECLIPUSER_RETURN(render_size)
}

short& MovieClipUser::render_flag() const {
	MOVIECLIPUSER_RETURN(render_flag)
}

// -------------------- MovieClip --------------------

ID<MovieClip> MovieClip::id() const {
	MOVIECLIP_RETURN_REF(ID<MovieClip>, id)
}

AnimData MovieClip::anim_data() const {
	MOVIECLIP_RETURN_AS(AnimData, adt)
}

const std::string MovieClip::filepath() const {
	MOVIECLIP_RETURN_AS(std::string, filepath)
}

int MovieClip::source() const {
	MOVIECLIP_RETURN(source)
}

int *MovieClip::last_size() const {

	MOVIECLIP_RETURN(lastsize)
}

float MovieClip::aspect_x() const {
	MOVIECLIP_RETURN(aspx)
}

float MovieClip::aspect_y() const {
	MOVIECLIP_RETURN(aspy)
}

bGPdata MovieClip::grease_pencil_data() const {
	MOVIECLIP_RETURN_AS(bGPdata, gpd)
}

MovieTracking MovieClip::tracking() const {
	MOVIECLIP_RETURN_REF(MovieTracking, tracking)
}

int MovieClip::flag() const {
	MOVIECLIP_RETURN(flag)
}

int MovieClip::length() const {
	MOVIECLIP_RETURN(len)
}

int &MovieClip::start_frame() const {
	MOVIECLIP_RETURN(start_frame)
}

int &MovieClip::frame_offset() const {
	MOVIECLIP_RETURN(frame_offset)
}

ColorManagedColorspaceSettings MovieClip::colorspace_settings() const {
	MOVIECLIP_RETURN_REF(ColorManagedColorspaceSettings, colorspace_settings)
}

MovieClip_Runtime MovieClip::runtime() const {
	MOVIECLIP_RETURN_REF(MovieClip_Runtime, runtime)
}

// -------------------- Collection --------------------

ID<Collection> Collection::id() const {
	COLLECTION_RETURN_REF(ID<Collection>, id);
}

// -------------------- MaterialSlot --------------------

Material MaterialSlot::material() const {
	return Material(materials[mat_idx]);
}

bool MaterialSlot::is_linked() const {
	return (matbits[mat_idx / 8] & (1 << (mat_idx % 8))) != 0;
}

size_t MaterialSlotArray::size() const {
	return totcol;
}

MaterialSlot MaterialSlotArray::operator[](const size_t idx) const {
	return MaterialSlot(materials, matbits, idx);
}

// -------------------- Object --------------------

ID<Object> Object::id() const {
	OBJECT_RETURN_REF(ID<Object>, id)
}

AnimData Object::adt() const {
	OBJECT_RETURN_AS(AnimData, adt);
}

DrawDataList Object::drawdata() const {
	OBJECT_RETURN_REF(DrawDataList, drawdata)
}

short Object::type() const {
	OBJECT_RETURN(type)
}

short Object::partype() const {
	OBJECT_RETURN(partype)
}

int Object::par1() const {
	OBJECT_RETURN(par1)
}

int Object::par2() const {
	OBJECT_RETURN(par2)
}

int Object::par3() const {
	OBJECT_RETURN(par3)
}

std::string Object::parsubstr() const {
	OBJECT_RETURN_AS(std::string, parsubstr)
}

Object Object::parent() const {
	OBJECT_RETURN_AS(Object, parent);
}

Object Object::track() const {
	OBJECT_RETURN_AS(Object, track);
}

Object Object::proxy() const {
	OBJECT_RETURN_AS(Object, proxy);
}

Object Object::proxy_group() const {
	OBJECT_RETURN_AS(Object, proxy_group);
}

Object Object::proxy_from() const {
	OBJECT_RETURN_AS(Object, proxy_from);
}

Ipo Object::ipo() const {
	OBJECT_RETURN_AS(Ipo, ipo);
}

bAction Object::action() const {
	OBJECT_RETURN_AS(bAction, action);
}

bAction Object::poselib() const {
	OBJECT_RETURN_AS(bAction, poselib);
}

bPose Object::pose() const {
	OBJECT_RETURN_AS(bPose, pose);
}

bGPdata Object::gpd() const {
	OBJECT_RETURN_AS(bGPdata, gpd);
}

bAnimVizSettings Object::avs() const {
	OBJECT_RETURN_REF(bAnimVizSettings, avs)
}

bMotionPath Object::mpath() const {
	OBJECT_RETURN_AS(bMotionPath, mpath);
}

ListBase<ModifierData> Object::modifiers() const {
	OBJECT_RETURN_REF(ListBase<ModifierData>, modifiers)
}

ListBase<GpencilModifierData> Object::greasepencil_modifiers() const {
	OBJECT_RETURN_REF(ListBase<GpencilModifierData>, greasepencil_modifiers)
}

ListBase<bFaceMap> Object::fmaps() const {
	OBJECT_RETURN_REF(ListBase<bFaceMap>, fmaps)
}

ListBase<ShaderFxData> Object::shader_fx() const {
	OBJECT_RETURN_REF(ListBase<ShaderFxData>, shader_fx)
}

int& Object::mode() const {
	OBJECT_RETURN(mode)
}

int& Object::restore_mode() const {
	OBJECT_RETURN(restore_mode)
}

MaterialSlotArray Object::materials() const {
	return MaterialSlotArray(mat(), matbits(), totcol());
}

MaterialSlot Object::active_mat() const {
	return materials()[actcol()];
}

float* Object::loc() const {
	OBJECT_RETURN(loc)
}

float* Object::dloc() const {
	OBJECT_RETURN(dloc)
}

float* Object::scale() const {
	OBJECT_RETURN(scale)
}

float* Object::dscale() const {
	OBJECT_RETURN(dscale)
}

float* Object::rot() const {
	OBJECT_RETURN(rot)
}

float* Object::drot() const {
	OBJECT_RETURN(drot)
}

float* Object::quat() const {
	OBJECT_RETURN(quat)
}

float* Object::dquat() const {
	OBJECT_RETURN(dquat)
}

float* Object::rotAxis() const {
	OBJECT_RETURN(rotAxis)
}

float* Object::drotAxis() const {
	OBJECT_RETURN(drotAxis)
}

float& Object::rotAngle() const {
	OBJECT_RETURN(rotAngle)
}

float& Object::drotAngle() const {
	OBJECT_RETURN(drotAngle)
}

Mat Object::parentinv() const {
	OBJECT_RETURN(parentinv)
}

Mat Object::constinv() const {
	OBJECT_RETURN(constinv)
}

short& Object::flag() const {
	OBJECT_RETURN(flag)
}

short& Object::transflag() const {
	OBJECT_RETURN(transflag)
}

short& Object::protectflag() const {
	OBJECT_RETURN(protectflag)
}

short& Object::trackflag() const {
	OBJECT_RETURN(trackflag)
}

short& Object::upflag() const {
	OBJECT_RETURN(upflag)
}

short& Object::nlaflag() const {
	OBJECT_RETURN(nlaflag)
}

char& Object::duplicator_visibility_flag() const {
	OBJECT_RETURN(duplicator_visibility_flag)
}

short& Object::base_flag() const {
	OBJECT_RETURN(base_flag)
}

unsigned short Object::base_local_view_bits() const {
	OBJECT_RETURN(base_local_view_bits)
}

unsigned short& Object::col_group() const {
	OBJECT_RETURN(col_group)
}

unsigned short& Object::col_mask() const {
	OBJECT_RETURN(col_mask)
}

short& Object::rotmode() const {
	OBJECT_RETURN(rotmode)
}

char& Object::boundtype() const {
	OBJECT_RETURN(boundtype)
}

char& Object::collision_boundtype() const {
	OBJECT_RETURN(collision_boundtype)
}

short& Object::dtx() const {
	OBJECT_RETURN(dtx)
}

char& Object::dt() const {
	OBJECT_RETURN(dt)
}

char& Object::empty_drawtype() const {
	OBJECT_RETURN(empty_drawtype)
}

float& Object::empty_drawsize() const {
	OBJECT_RETURN(empty_drawsize)
}

float& Object::instance_faces_scale() const {
	OBJECT_RETURN(instance_faces_scale)
}

short& Object::index() const {
	OBJECT_RETURN(index)
}

unsigned short& Object::actdef() const {
	OBJECT_RETURN(actdef)
}

float* Object::color() const {
	OBJECT_RETURN(color)
}

short& Object::softflag() const {
	OBJECT_RETURN(softflag)
}

char& Object::shapeflag() const {
	OBJECT_RETURN(shapeflag)
}

short& Object::shapenr() const {
	OBJECT_RETURN(shapenr)
}

// Add ListBase constraints and particlesystem once exact type is identified

PartDeflect Object::pd() const {
	OBJECT_RETURN_AS(PartDeflect, pd);
}

SoftBody Object::soft() const {
	OBJECT_RETURN_AS(SoftBody, soft);
}

Collection Object::instance_collection() const {
	OBJECT_RETURN_AS(Collection, instance_collection);
}

// Add ListBase pc_ids once exact type is identified

RigidBodyOb Object::rigidbody_object() const {
	OBJECT_RETURN_AS(RigidBodyOb, rigidbody_object);
}

RigidBodyCon Object::rigidbody_constraint() const {
	OBJECT_RETURN_AS(RigidBodyCon, rigidbody_constraint);
}

float* Object::ima_ofs() const {
	OBJECT_RETURN(ima_ofs)
}

ImageUser Object::iuser() const {
	OBJECT_RETURN_AS(ImageUser, iuser);
}

char& Object::empty_image_visibility_flag() const {
	OBJECT_RETURN(empty_image_visibility_flag)
}

char& Object::empty_image_depth() const {
	OBJECT_RETURN(empty_image_depth)
}

char& Object::empty_image_flag() const {
	OBJECT_RETURN(empty_image_flag)
}

PreviewImage Object::preview() const {
	OBJECT_RETURN_AS(PreviewImage, preview);
}

ObjectLineArt Object::lineart() const {
	OBJECT_RETURN_REF(ObjectLineArt, lineart)
}

void** Object::mat() const {
	OBJECT_RETURN_AS((void**), mat)
}

char* Object::matbits() const {
	OBJECT_RETURN(matbits)
}

int Object::totcol() const {
	OBJECT_RETURN(totcol)
}

int Object::actcol() const {
	OBJECT_RETURN(actcol)
}

// -------------------- CameraBGImage --------------------

CameraBGImage CameraBGImage::next() const {
	CAMERABGIMAGE_RETURN_AS(CameraBGImage, next)
}

CameraBGImage CameraBGImage::prev() const {
	CAMERABGIMAGE_RETURN_AS(CameraBGImage, prev)
}

// Add ima and iuser later

MovieClip CameraBGImage::clip() const {
	CAMERABGIMAGE_RETURN_AS(MovieClip, clip)
}

MovieClipUser CameraBGImage::cuser() const {
	CAMERABGIMAGE_RETURN_REF(MovieClipUser, cuser)
}

float* CameraBGImage::offset() const {
	CAMERABGIMAGE_RETURN(offset);
}

float& CameraBGImage::scale() const {
	CAMERABGIMAGE_RETURN(scale);
}

float& CameraBGImage::rotation() const {
	CAMERABGIMAGE_RETURN(rotation);
}

float& CameraBGImage::alpha() const {
	CAMERABGIMAGE_RETURN(alpha);
}

short& CameraBGImage::flag() const {
	CAMERABGIMAGE_RETURN(flag);
}

short& CameraBGImage::source() const {
	CAMERABGIMAGE_RETURN(source);
}

// -------------------- Camera --------------------

ID<Camera> Camera::id() const {
	CAMERA_RETURN_REF(ID<Camera>, id)
}

AnimData Camera::adt() const {
	CAMERA_RETURN_AS(AnimData, adt)
}

char& Camera::type() const {
	CAMERA_RETURN(type)
}

char& Camera::dtx() const {
	CAMERA_RETURN(dtx)
}

short& Camera::flag() const {
	CAMERA_RETURN(flag)
}

float& Camera::passepartalpha() const {
	CAMERA_RETURN(passepartalpha)
}

float& Camera::clip_start() const {
	CAMERA_RETURN(clip_start)
}

float& Camera::clip_end() const {
	CAMERA_RETURN(clip_end)
}

float& Camera::lens() const {
	CAMERA_RETURN(lens)
}

float& Camera::ortho_scale() const {
	CAMERA_RETURN(ortho_scale)
}

float& Camera::drawsize() const {
	CAMERA_RETURN(drawsize)
}

float& Camera::sensor_x() const {
	CAMERA_RETURN(sensor_x)
}

float& Camera::sensor_y() const {
	CAMERA_RETURN(sensor_y)
}

float& Camera::shiftx() const {
	CAMERA_RETURN(shiftx)
}

float& Camera::shifty() const {
	CAMERA_RETURN(shifty)
}

float& Camera::dof_distance() const {
	CAMERA_RETURN(dof_distance)
}

Ipo Camera::ipo() const {
	CAMERA_RETURN_AS(Ipo, ipo)
}

ListBase<CameraBGImage> Camera::bg_images() const {
	CAMERA_RETURN_REF(ListBase<CameraBGImage>, bg_images)
}

char& Camera::sensor_fit() const {
	CAMERA_RETURN(sensor_fit)
}
