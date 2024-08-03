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
#include <iostream>
#include <algorithm>

// Global compatability mode (can be changed)
BlenderVersion blender_ver = BlenderVersion::VER_2_93_0;

void set_compatibility_mode(BlenderVersion version) {
	blender_ver = version;
}

BlenderVersion get_compatability_mode() {
	return blender_ver;
}

// -------------------- BezTriple --------------------

#define BZTRP_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<BezTriple2_93_0*>(data_ptr)-> C);

#define BZTRP_RETURN(M) BZTRP_BASE_RETURN_BODY(,, M)

BezTripleVecs BezTriple::vec() const {
	BZTRP_RETURN(vec)
}

float& BezTriple::tilt() const {
	BZTRP_RETURN(tilt)
}

float& BezTriple::weight() const {
	BZTRP_RETURN(weight)
}

float& BezTriple::radius() const {
	BZTRP_RETURN(radius)
}

char& BezTriple::ipo() const {
	BZTRP_RETURN(ipo)
}

uint8_t& BezTriple::h1() const {
	BZTRP_RETURN(h1)
}

uint8_t& BezTriple::h2() const {
	BZTRP_RETURN(h2)
}

uint8_t& BezTriple::f1() const {
	BZTRP_RETURN(f1)
}

uint8_t& BezTriple::f2() const {
	BZTRP_RETURN(f2)
}

uint8_t& BezTriple::f3() const {
	BZTRP_RETURN(f3)
}

char& BezTriple::hide() const {
	BZTRP_RETURN(hide)
}

char& BezTriple::easing() const {
	BZTRP_RETURN(easing)
}

float& BezTriple::back() const {
	BZTRP_RETURN(back)
}

float& BezTriple::amplitude() const {
	BZTRP_RETURN(amplitude)
}

float& BezTriple::period() const {
	BZTRP_RETURN(period)
}

char& BezTriple::auto_handle_type() const {
	BZTRP_RETURN(auto_handle_type)
}

// -------------------- FPoint --------------------

#define FPOINT_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<FPoint2_93_0*>(data_ptr)-> C);

#define FPOINT_RETURN(M) FPOINT_BASE_RETURN_BODY(,, M)

float* FPoint::vec() const {
	FPOINT_RETURN(vec)
}

int& FPoint::flag() const {
	FPOINT_RETURN(flag)
}

// -------------------- FCurve --------------------

#define FCURVE_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<FCurve2_93_0*>(data_ptr)-> C);

#define FCURVE_RETURN_REF(T, M) FCURVE_BASE_RETURN_BODY(T, &, M)
#define FCURVE_RETURN_AS(T, M) FCURVE_BASE_RETURN_BODY(T,, M)
#define FCURVE_RETURN(M) FCURVE_BASE_RETURN_BODY(,, M)

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

#define BCCH_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<bConstraintChannel2_93_0*>(data_ptr)-> C);

#define BCCH_RETURN_AS(T, M) BCCH_BASE_RETURN_BODY(T,, M)
#define BCCH_RETURN(M) BCCH_BASE_RETURN_BODY(,, M)

bConstraintChannel bConstraintChannel::next() const {
	BCCH_RETURN_AS(bConstraintChannel, next)
}

bConstraintChannel bConstraintChannel::prev() const {
	BCCH_RETURN_AS(bConstraintChannel, prev)
}

Ipo bConstraintChannel::ipo() const {
	BCCH_RETURN_AS(Ipo, ipo)
}

short& bConstraintChannel::flag() const {
	BCCH_RETURN(flag)
}

std::string bConstraintChannel::name() const {
	BCCH_RETURN_AS(std::string, name)
}

// -------------------- bActionChannel --------------------

#define BACH_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<bActionChannel2_93_0*>(data_ptr)-> C);

#define BACH_RETURN_REF(T, M) BACH_BASE_RETURN_BODY(T, &, M)
#define BACH_RETURN_AS(T, M) BACH_BASE_RETURN_BODY(T,, M)
#define BACH_RETURN(M) BACH_BASE_RETURN_BODY(,, M)

bActionChannel bActionChannel::next() const {
	BACH_RETURN_AS(bActionChannel, next)
}

bActionChannel bActionChannel::prev() const {
	BACH_RETURN_AS(bActionChannel, prev)
}

bActionGroup bActionChannel::grp() const {
	BACH_RETURN_AS(bActionGroup, grp)
}

Ipo bActionChannel::ipo() const {
	BACH_RETURN_AS(Ipo, ipo)
}

ListBase<bConstraintChannel> bActionChannel::constraintChannels() const {
	BACH_RETURN_REF(ListBase<bConstraintChannel>, constraintChannels)
}

int& bActionChannel::flag() const {
	BACH_RETURN(flag)
}

std::string bActionChannel::name() const {
	BACH_RETURN_AS(std::string, name)
}

int& bActionChannel::temp() const {
	BACH_RETURN(temp)
}

// -------------------- bActionGroup --------------------

#define BAGRP_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<bActionGroup2_93_0*>(data_ptr)-> C);

#define BAGRP_RETURN_REF(T, M) BAGRP_BASE_RETURN_BODY(T, &, M)
#define BAGRP_RETURN_AS(T, M) BAGRP_BASE_RETURN_BODY(T,, M)
#define BAGRP_RETURN(M) BAGRP_BASE_RETURN_BODY(,, M)

bActionGroup bActionGroup::next() const {
	BAGRP_RETURN_AS(bActionGroup, next)
}

bActionGroup bActionGroup::prev() const {
	BAGRP_RETURN_AS(bActionGroup, prev)
}

ListBase<bActionChannel> bActionGroup::channels() const {
	BAGRP_RETURN_REF(ListBase<bActionChannel>, channels)
}

int& bActionGroup::flag() const {
	BAGRP_RETURN(flag)
}

int& bActionGroup::customCol() const {
	BAGRP_RETURN(customCol)
}

std::string bActionGroup::name() const {
	BAGRP_RETURN_AS(std::string, name)
}

ThemeWireColor bActionGroup::cs() const {
	BAGRP_RETURN_REF(ThemeWireColor, cs)
}

// -------------------- bAction --------------------

#define BACT_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_0_0) \
		return A ( B reinterpret_cast<bAction2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_1_0) \
		return A ( B reinterpret_cast<bAction3_0_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_2_0) \
		return A ( B reinterpret_cast<bAction3_1_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_4_0) \
		return A ( B reinterpret_cast<bAction3_2_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_1_0) \
		return A ( B reinterpret_cast<bAction3_4_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_2_0) \
		return A ( B reinterpret_cast<bAction4_1_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<bAction4_2_0*>(data_ptr)-> C);

#define BACT_RETURN_REF(T, M) BACT_BASE_RETURN_BODY(T, &, M)
#define BACT_RETURN_AS(T, M) BACT_BASE_RETURN_BODY(T,, M)
#define BACT_RETURN(M) BACT_BASE_RETURN_BODY(,, M)

ID<bAction> bAction::id() const {
	BACT_RETURN_REF(ID<bAction>, id)
}

ListBase<FCurve> bAction::curves() const {
	BACT_RETURN_REF(ListBase<FCurve>, curves)
}

ListBase<bActionGroup> bAction::groups() const {
	BACT_RETURN_REF(ListBase<bActionGroup>, groups)
}

// Add markers when type is determined

int& bAction::flag() const {
	BACT_RETURN(flag)
}

int& bAction::active_marker() const {
	BACT_RETURN(active_marker)
}

int bAction::idroot() const {
	BACT_RETURN(idroot)
}

PreviewImage bAction::preview() const {
	BACT_RETURN_AS(PreviewImage, preview)
}

// -------------------- AnimData --------------------

#define ADAT_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_4_2_0) \
		return A ( B reinterpret_cast<AnimData2_93_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<AnimData4_2_0*>(data_ptr)-> C);

#define ADAT_RETURN_REF(T, M) ADAT_BASE_RETURN_BODY(T, &, M)
#define ADAT_RETURN_AS(T, M) ADAT_BASE_RETURN_BODY(T,, M)
#define ADAT_RETURN(M) ADAT_BASE_RETURN_BODY(,, M)

bAction AnimData::action() const {
	ADAT_RETURN_AS(bAction, action)
}

bAction AnimData::tmpact() const {
	ADAT_RETURN_AS(bAction, tmpact)
}

ListBase<NlaTrack> AnimData::nla_tracks() const {
	ADAT_RETURN_REF(ListBase<NlaTrack>, nla_tracks)
}

NlaTrack AnimData::act_track() const {
	ADAT_RETURN_AS(NlaTrack, act_track)
}

NlaStrip AnimData::actstrip() const {
	ADAT_RETURN_AS(NlaStrip, actstrip)
}

ListBase<ChannelDriver> AnimData::drivers() const {
	ADAT_RETURN_REF(ListBase<ChannelDriver>, drivers)
}

ListBase<AnimOverride> AnimData::overrides() const {
	ADAT_RETURN_REF(ListBase<AnimOverride>, overrides)
}

// -------------------- MovieTrackingCamera --------------------

#define TRCAM_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<MovieTrackingCamera2_93_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieTrackingCamera3_5_0*>(data_ptr)-> C);

#define TRCAM_RETURN(M) TRCAM_BASE_RETURN_BODY(,, M)

short& MovieTrackingCamera::distortion_model() const {
	TRCAM_RETURN(distortion_model)
}

float& MovieTrackingCamera::sensor_width() const {
	TRCAM_RETURN(sensor_width)
}

float& MovieTrackingCamera::pixel_aspect() const {
	TRCAM_RETURN(pixel_aspect)
}

float& MovieTrackingCamera::focal() const {
	TRCAM_RETURN(focal)
}

short& MovieTrackingCamera::units() const {
	TRCAM_RETURN(units)
}

float& MovieTrackingCamera::k1() const {
	TRCAM_RETURN(k1)
}

float& MovieTrackingCamera::k2() const {
	TRCAM_RETURN(k2)
}

float& MovieTrackingCamera::k3() const {
	TRCAM_RETURN(k3)
}

float& MovieTrackingCamera::division_k1() const {
	TRCAM_RETURN(division_k1)
}

float& MovieTrackingCamera::division_k2() const {
	TRCAM_RETURN(division_k2)
}

float& MovieTrackingCamera::nuke_k1() const {
	TRCAM_RETURN(nuke_k1)
}

float& MovieTrackingCamera::nuke_k2() const {
	TRCAM_RETURN(nuke_k2)
}

float& MovieTrackingCamera::brown_k1() const {
	TRCAM_RETURN(brown_k1)
}

float& MovieTrackingCamera::brown_k2() const {
	TRCAM_RETURN(brown_k2)
}

float& MovieTrackingCamera::brown_k3() const {
	TRCAM_RETURN(brown_k3)
}

float& MovieTrackingCamera::brown_k4() const {
	TRCAM_RETURN(brown_k4)
}

float& MovieTrackingCamera::brown_p1() const {
	TRCAM_RETURN(brown_p1)
}

float& MovieTrackingCamera::brown_p2() const {
	TRCAM_RETURN(brown_p2)
}

// -------------------- MovieTrackingMarker --------------------

#define MARKER_BASE_RETURN_BODY(A, B, C) \
	return A ( B reinterpret_cast<MovieTrackingMarker2_93_0*>(data_ptr)-> C);

#define MARKER_RETURN(M) MARKER_BASE_RETURN_BODY(,, M)

float* MovieTrackingMarker::pos() const {
	MARKER_RETURN(pos)
}

Corners MovieTrackingMarker::pattern_corners() const {
	MARKER_RETURN(pattern_corners)
}

float* MovieTrackingMarker::search_min() const {
	MARKER_RETURN(search_min)
}

float* MovieTrackingMarker::search_max() const {
	MARKER_RETURN(search_max)
}

int& MovieTrackingMarker::framenr() const {
	MARKER_RETURN(framenr)
}

int& MovieTrackingMarker::flag() const {
	MARKER_RETURN(flag)
}

void* new_marker() 
{ 
	// The struct hasn't changed since 2.93 so this is fine
	MovieTrackingMarker2_93_0* marker = new MovieTrackingMarker2_93_0;

	// Zero all fields
	std::fill(&marker->pos[0], &marker->pos[0] + sizeof(marker->pos) / sizeof(float), 0.0f);
	std::fill(&marker->pattern_corners[0][0], &marker->pattern_corners[0][0] + sizeof(marker->pattern_corners) / sizeof(float), 0.0f);
	std::fill(&marker->search_min[0], &marker->search_min[0] + sizeof(marker->search_min) / sizeof(float), 0.0f);
	std::fill(&marker->search_max[0], &marker->search_max[0] + sizeof(marker->search_max) / sizeof(float), 0.0f);
	marker->framenr = 0;
	marker->flag = 0;

	// return as void ptr
	return marker;
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

MovieTrackingTrack MovieTrackingTrack::next() const {
	TRACK_RETURN_AS(MovieTrackingTrack, next)
}

MovieTrackingTrack MovieTrackingTrack::prev() const {
	TRACK_RETURN_AS(MovieTrackingTrack, prev)
}

const std::string MovieTrackingTrack::name() const {
	TRACK_RETURN_AS(std::string, name)
}

float* MovieTrackingTrack::offset() const {
	TRACK_RETURN(offset)
}

int &MovieTrackingTrack::markersnr() const {
	TRACK_RETURN(markersnr)
}

MovieTrackingMarker MovieTrackingTrack::marker(size_t idx) const {
	TRACK_RETURN_REF(MovieTrackingMarker, markers[idx])
}

float* MovieTrackingTrack::bundle_pos() const {
	TRACK_RETURN(bundle_pos)
}

float& MovieTrackingTrack::error() const {
	TRACK_RETURN(error)
}

int& MovieTrackingTrack::flag() const {
	TRACK_RETURN(flag)
}

int& MovieTrackingTrack::pat_flag() const {
	TRACK_RETURN(pat_flag)
}

int& MovieTrackingTrack::search_flag() const {
	TRACK_RETURN(search_flag)
}

float* MovieTrackingTrack::color() const {
	TRACK_RETURN(color)
}

short& MovieTrackingTrack::frames_limit() const {
	TRACK_RETURN(frames_limit)
}

short& MovieTrackingTrack::margin() const {
	TRACK_RETURN(margin)
}

short& MovieTrackingTrack::pattern_match() const {
	TRACK_RETURN(pattern_match)
}

short& MovieTrackingTrack::motion_model() const {
	TRACK_RETURN(motion_model)
}

int& MovieTrackingTrack::algorithm_flag() const {
	TRACK_RETURN(algorithm_flag)
}

float& MovieTrackingTrack::minimum_correlation() const {
	TRACK_RETURN(minimum_correlation)
}

bGPdata MovieTrackingTrack::gpd() const {
	TRACK_RETURN_AS(bGPdata, gpd)
}

float& MovieTrackingTrack::weight() const {
	TRACK_RETURN(weight)
}

float& MovieTrackingTrack::weight_stab() const {
	TRACK_RETURN(weight_stab)
}

// Next 2 functions are based on code from tracking.cc
// Copyright (C) 2024 Blender Foundation. All rights reserved.
MovieTrackingMarker MovieTrackingTrack::tracking_marker_get(int framenr) const {
	const int num_markers = markersnr();
	if (num_markers == 0) {
		std::cerr << "Detected degenerated track, should never happen." << std::endl;
		return nullptr;
	}
	int left_boundary = 0;
	int right_boundary = num_markers;
	while (left_boundary < right_boundary) {
		const int median_index = (left_boundary + right_boundary) / 2;
		MovieTrackingMarker m = marker(median_index);
		int m_framenr = m.framenr();
		if (m_framenr == framenr)
			return m;
		if (m_framenr < framenr)
			left_boundary = median_index + 1;
		else
			right_boundary = median_index - 1;
	}
	const int closest_index = std::clamp(right_boundary, 0, num_markers - 1);
	return marker(closest_index);
}

MovieTrackingMarker MovieTrackingTrack::find_frame(int frame, bool exact) const {
	MovieTrackingMarker marker = tracking_marker_get(frame);
	if (exact && marker.framenr() != frame)
		return MovieTrackingMarker();
	return marker;
}

// -------------------- MovieTrackingObject --------------------

#define TOBJECT_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<MovieTrackingObject2_93_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<MovieTrackingObject3_5_0*>(data_ptr)-> C);

#define TOBJECT_RETURN_REF(T, M) TOBJECT_BASE_RETURN_BODY(T, &, M)
#define TOBJECT_RETURN_AS(T, M) TOBJECT_BASE_RETURN_BODY(T,, M)
#define TOBJECT_RETURN(M) TOBJECT_BASE_RETURN_BODY(,, M)

MovieTrackingObject MovieTrackingObject::next() const {
	TOBJECT_RETURN_AS(MovieTrackingObject, next)
}

MovieTrackingObject MovieTrackingObject::prev() const {
	TOBJECT_RETURN_AS(MovieTrackingObject, prev)
}

const std::string MovieTrackingObject::name() const {
	TOBJECT_RETURN_AS(std::string, name)
}

int& MovieTrackingObject::flag() const {
	TOBJECT_RETURN(flag)
}

float& MovieTrackingObject::scale() const {
	TOBJECT_RETURN(scale)
}

ListBase<MovieTrackingTrack> MovieTrackingObject::tracks() const {
	TOBJECT_RETURN_REF(ListBase<MovieTrackingTrack>, tracks)
}

ListBase<MovieTrackingPlaneTrack> MovieTrackingObject::plane_tracks() const {
	TOBJECT_RETURN_REF(ListBase<MovieTrackingPlaneTrack>, plane_tracks)
}

MovieTrackingReconstruction MovieTrackingObject::reconstruction() const {
	TOBJECT_RETURN_REF(MovieTrackingReconstruction, reconstruction)
}

int& MovieTrackingObject::keyframe1() const {
	TOBJECT_RETURN(keyframe1)
}

int& MovieTrackingObject::keyframe2() const {
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

MovieTrackingSettings MovieTracking::settings() const {
	TRACKING_RETURN_REF(MovieTrackingSettings, settings)
}

MovieTrackingCamera MovieTracking::camera() const {
	TRACKING_RETURN_REF(MovieTrackingCamera, camera)
}

ListBase<MovieTrackingTrack> MovieTracking::tracks() const {
	TRACKING_RETURN_REF_LEGACY(ListBase<MovieTrackingTrack>, tracks)
}

ListBase<MovieTrackingPlaneTrack> MovieTracking::plane_tracks() const {
	TRACKING_RETURN_REF_LEGACY(ListBase<MovieTrackingPlaneTrack>, plane_tracks)
}

MovieTrackingReconstruction MovieTracking::reconstruction() const {
	TRACKING_RETURN_REF_LEGACY(MovieTrackingReconstruction, reconstruction)
}

MovieTrackingStabilization MovieTracking::stabilization() const {
	TRACKING_RETURN_REF(MovieTrackingStabilization, stabilization)
}

MovieTrackingTrack MovieTracking::active_track() const {
	TRACKING_RETURN_AS_LEGACY(MovieTrackingTrack, act_track)
}

MovieTrackingPlaneTrack MovieTracking::active_plane_track() const {
	TRACKING_RETURN_AS_LEGACY(MovieTrackingPlaneTrack, act_plane_track)
}

ListBase<MovieTrackingObject> MovieTracking::objects() const {
	TRACKING_RETURN_REF(ListBase<MovieTrackingObject>, objects)
}

const int MovieTracking::objectnr() const {
	TRACKING_RETURN(objectnr)
}

const int MovieTracking::tot_objects() const {
	TRACKING_RETURN(tot_object)
}

MovieTrackingStats MovieTracking::stats() const {
	TRACKING_RETURN_AS(MovieTrackingStats, stats)
}

MovieTrackingDopesheet MovieTracking::dopesheet() {
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

#define COLL_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_0_0) \
		return A ( B reinterpret_cast<Collection2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_2_0) \
		return A ( B reinterpret_cast<Collection3_0_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_3_0) \
		return A ( B reinterpret_cast<Collection3_2_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_4_0) \
		return A ( B reinterpret_cast<Collection3_3_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_5_0) \
		return A ( B reinterpret_cast<Collection3_4_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_6_0) \
		return A ( B reinterpret_cast<Collection3_5_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_0_0) \
		return A ( B reinterpret_cast<Collection3_6_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_1_0) \
		return A(B reinterpret_cast<Collection4_0_0*>(data_ptr)->C); \
	else if (blender_ver < BlenderVersion::VER_4_2_0) \
		return A(B reinterpret_cast<Collection4_1_0*>(data_ptr)->C); \
	return A ( B reinterpret_cast<Collection4_2_0*>(data_ptr)-> C);

#define COLL_RETURN_REF(T, M) COLL_BASE_RETURN_BODY(T, &, M)
#define COLL_RETURN_AS(T, M) COLL_BASE_RETURN_BODY(T,, M)
#define COLL_RETURN(M) COLL_BASE_RETURN_BODY(,, M)

ID<Collection> Collection::id() const {
	COLL_RETURN_REF(ID<Collection>, id);
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

// Macros are defined in header

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

Mat Object::obmat() const {
	OBJECT_RETURN_MATS(obmat, object_to_world)
}

Mat Object::parentinv() const {
	OBJECT_RETURN(parentinv)
}

Mat Object::constinv() const {
	OBJECT_RETURN(constinv)
}

Mat Object::imat() const {
	OBJECT_RETURN_MATS(imat, world_to_object)
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

Object_Rumtime Object::runtime() const {
	OBJECT_RETURN_REF(Object_Rumtime, runtime)
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

// -------------------- Camera --------------------

#define CAM_BASE_RETURN_BODY(A, B, C) \
	if (blender_ver < BlenderVersion::VER_3_0_0) \
		return A ( B reinterpret_cast<Camera2_93_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_2_0) \
		return A ( B reinterpret_cast<Camera3_0_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_3_0) \
		return A ( B reinterpret_cast<Camera3_2_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_3_4_0) \
		return A ( B reinterpret_cast<Camera3_3_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_0_0) \
		return A ( B reinterpret_cast<Camera3_4_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_1_0) \
		return A ( B reinterpret_cast<Camera4_0_0*>(data_ptr)-> C); \
	else if (blender_ver < BlenderVersion::VER_4_2_0) \
		return A ( B reinterpret_cast<Camera4_1_0*>(data_ptr)-> C); \
	return A ( B reinterpret_cast<Camera4_2_0*>(data_ptr)-> C);

#define CAM_RETURN_REF(T, M) CAM_BASE_RETURN_BODY(T, &, M)
#define CAM_RETURN_AS(T, M) CAM_BASE_RETURN_BODY(T,, M)
#define CAM_RETURN(M) CAM_BASE_RETURN_BODY(,, M)

ID<Camera> Camera::id() const {
	CAM_RETURN_REF(ID<Camera>, id)
}

AnimData Camera::adt() const {
	CAM_RETURN_AS(AnimData, adt)
}

char& Camera::type() const {
	CAM_RETURN(type)
}

char& Camera::dtx() const {
	CAM_RETURN(dtx)
}

short& Camera::flag() const {
	CAM_RETURN(flag)
}

float& Camera::passepartalpha() const {
	CAM_RETURN(passepartalpha)
}

float& Camera::clip_start() const {
	CAM_RETURN(clip_start)
}

float& Camera::clip_end() const {
	CAM_RETURN(clip_end)
}

float& Camera::lens() const {
	CAM_RETURN(lens)
}

float& Camera::ortho_scale() const {
	CAM_RETURN(ortho_scale)
}

float& Camera::drawsize() const {
	CAM_RETURN(drawsize)
}

float& Camera::sensor_x() const {
	CAM_RETURN(sensor_x)
}

float& Camera::sensor_y() const {
	CAM_RETURN(sensor_y)
}

float& Camera::shiftx() const {
	CAM_RETURN(shiftx)
}

float& Camera::shifty() const {
	CAM_RETURN(shifty)
}

float& Camera::dof_distance() const {
	CAM_RETURN(dof_distance)
}

Ipo Camera::ipo() const {
	CAM_RETURN_AS(Ipo, ipo)
}

char& Camera::sensor_fit() const {
	CAM_RETURN(sensor_fit)
}
