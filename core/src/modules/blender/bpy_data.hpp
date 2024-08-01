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

----------------------------------------------------------------------

Python wrapper classes for bpy data access
Includes conversions to wrapper classes from bpy_types.hpp
*/

#pragma once

#include "bpy_types.hpp"
#include <set>

template <typename T>
class ListBaseData {
public:
	ListBaseData(T first, T last) : f(first), l(last) {}
	T first() const { return this->f; }
	T last() const { return this->l; }
private:
	T f;
	T l;
};

class PyRef {
public:
	PyRef() : py_obj(nullptr) {}
	PyRef(void* py_obj);
	PyRef(const PyRef& other) : PyRef(other.py_obj) {}
	~PyRef();
	bool is_null() const; // Check if py_obj is valid
	PyRef& operator=(const PyRef& other);
protected:
	void* py_obj = nullptr;
	void clear_ref();
};

class PyID : public PyRef {
public:
	using PyRef::PyRef;

	template <typename T>
	T as() const {
		return T(py_obj);
	}

	std::string get_property_str(const std::string& prop_name) const;
	bool get_property_bool (const std::string& prop_name) const;
	float get_property_float(const std::string& prop_name) const;
	int get_property_int(const std::string& prop_name) const;
};

// -------------------- BlendDataActions --------------------

class PyKeyframe;
class PyFCurveKeyframePoints;
class PyActionGroup;
class PyFCurve;
class PyActionFCurves;
class PyAction;
class PyBlendDataActions;
class PyAnimData;

class PyAnimData : public PyRef { // Part of blender object
public:
	using PyRef::PyRef;
	AnimData intern() const;
	PyAction action() const;
	void set_active_action(PyAction action) const;
};

class PyKeyframe : public PyRef {
public:
	using PyRef::PyRef;
	friend PyFCurveKeyframePoints;
	BezTriple intern() const;
};

class PyFCurveKeyframePoints : public PyRef {
public:
	using PyRef::PyRef;
	PyKeyframe insert(const float frame, const float value, 
		const std::set<std::string>& options = {},
		const std::string& keyframe_type = std::string("KEYFRAME"));
	void add(const int count);
	void remove(PyKeyframe keyframe, bool fast = false);
	void clear();
	void sort();
	void deduplicate();
	void handles_recalc();
	PyKeyframe operator[](const int& idx) const;
};

class PyActionGroup : public PyRef {
public:
	using PyRef::PyRef;
	bActionGroup intern() const;
};

class PyFCurve : public PyRef {
public:
	using PyRef::PyRef;
	friend PyActionFCurves;
	FCurve intern() const;
	// add driver func if necessary
	PyActionGroup group() const;
	PyFCurveKeyframePoints keyframe_points() const;
	// add modifiers func if necessary
	// There are more members of FCurve in python
};

class PyActionFCurves : public PyRef {
public:
	PyRef::PyRef;
	ListBaseData<FCurve> items() const;
	PyFCurve new_fcurve(const std::string& data_path, const int index = 0, const std::string& action_group = std::string());
	PyFCurve find(const std::string& data_path, const int index = 0) const;
	void remove(PyFCurve fcurve);
	void clear();
	PyFCurve operator[](const int& idx) const;
	PyFCurve operator[](const std::string& name) const;
};

class PyAction : public PyRef {
public:
	using PyRef::PyRef;
	friend PyAnimData;
	bAction intern() const;
	PyActionFCurves fcurves() const;
	// Add groups
	// Add pose_markers
	// Add flip_with_pose
};

class PyBlendDataActions : public PyRef {
public:
	using PyRef::PyRef;
	ListBaseData<bAction> items() const;
	PyAction new_action(const std::string& name = std::string());
	void remove(bAction action, bool do_unlink = true, bool do_id_user = true, bool do_ui_user = true);
	PyAction operator[](const int& idx) const;
	PyAction operator[](const std::string& name) const;
};

// -------------------- BlendDataMovieClips --------------------

class PyMovieTrackingMarkers;
class PyMovieTrackingTrack;
class PyMovieTrackingTracks;
class PyMovieTrackingPlaneTracks;
class PyMovieTrackingObject;
class PyMovieTrackingObjects;
class PyMovieTracking;
class PyMovieClip;
class PyBlendDataMovieClips;

class PyMovieTrackingMarkers : public PyRef {
public:
	using PyRef::PyRef;
	MovieTrackingMarker insert_frame(int frame, float x = 0.0f, float y = 0.0f);
	void delete_frame(int frame);
	ListBaseData<MovieTrackingMarker> items() const;
};

class PyMovieTrackingTrack : public PyRef {
public:
	using PyRef::PyRef;
	MovieTrackingTrack intern() const;
	PyMovieTrackingMarkers markers() const;
};

class PyMovieTrackingTracks : public PyRef {
public:
	using PyRef::PyRef;
	PyMovieTrackingTrack new_track(const std::string& name = std::string(), int frame = 1);
	ListBaseData<MovieTrackingTrack> items() const;
	PyMovieTrackingTrack operator[](const int& idx) const;
	PyMovieTrackingTrack operator[](const std::string& name) const;
};

class PyMovieTrackingPlaneTracks : public PyRef {
public:
	using PyRef::PyRef;
	ListBaseData<MovieTrackingPlaneTrack> items() const;
	// Incomplete declaration, expand if needed
};

class PyMovieTrackingObject : public PyRef {
public:
	using PyRef::PyRef;
	MovieTrackingObject intern() const;
	PyMovieTrackingPlaneTracks plane_tracks() const;
	PyMovieTrackingTracks tracks() const;
};

class PyMovieTrackingObjects : public PyRef {
public:
	using PyRef::PyRef;
	ListBaseData<MovieTrackingObject> items() const;
	PyMovieTrackingObject new_object(const std::string& name = std::string());
	void remove(MovieTrackingObject object);
	PyMovieTrackingObject operator[](const int& idx) const;
	PyMovieTrackingObject operator[](const std::string& name) const;
};

class PyMovieTracking : public PyRef {
public:
	using PyRef::PyRef;
	MovieTracking intern() const;
	PyMovieTrackingObjects objects() const;
	PyMovieTrackingPlaneTracks plane_tracks() const;
	PyMovieTrackingTracks tracks() const;
};

class PyMovieClip : public PyRef {
public:
	using PyRef::PyRef;
	MovieClip intern() const;
	PyMovieTracking tracking() const;
};

class PyBlendDataMovieClips : public PyRef {
public:
	using PyRef::PyRef;
	ListBaseData<MovieClip> items() const;
	void remove(MovieClip clip, bool do_unlink = true, bool do_id_user = true, bool do_ui_user = true);
	PyMovieClip load(const std::string& filepath, bool check_existing = false);
	PyMovieClip operator[](const int& idx) const;
	PyMovieClip operator[](const std::string& name) const;
};

// -------------------- BlendDataObjects --------------------

class PyBObject; // BObject, as to not to redefine PyObject in the C API
class PyBlendDataObjects;

class PyBObject : public PyRef {
public:
	using PyRef::PyRef;
	Object intern() const;
	PyAnimData animation_data() const;
	PyAnimData animation_data_create() const;
	void animation_data_clear() const;
	PyID data() const;
};

class PyBlendDataObjects : public PyRef {
public:
	using PyRef::PyRef;
	ListBaseData<Object> items() const;
	PyBObject new_object(const std::string& name = std::string()); // This function also accepts existing ID data in the API
	void remove(Object object, bool do_unlink = true, bool do_id_user = true, bool do_ui_user = true);
	PyBObject operator[](const int& idx) const;
	PyBObject operator[](const std::string& name) const;
};

// -------------------- BlendDataScenes --------------------

class PySceneObjects : public PyRef {
public:
	using PyRef::PyRef;
	const int size();
	PyBObject operator[](const int& idx) const;
	PyBObject operator[](const std::string& name) const;
};

class PyScene : public PyRef {
public:
	using PyRef::PyRef;
	// add intern later
	PySceneObjects objects() const;
};

// -------------------- BlendData --------------------

class PyBlendData;
class PyBlendContext;
class PyFCurveSeq;

class PyBlendData : public PyRef {
public:
	PyBlendData(); // Obtain python ref to bpy.data
	PyBlendDataActions actions() const;
	PyBlendDataMovieClips movieclips() const;
	PyBlendDataObjects objects() const;
};

// Sequence types for context access

class PyFCurveSeq : public PyRef {
public:
	using PyRef::PyRef;
	const int size();
	PyFCurve operator[](const int& idx) const;
};

// Miscellaneous types used with context access

class PyBlendArea : public PyRef {
public:
	using PyRef::PyRef;
	void tag_redraw() const;
	void header_set_text(const std::string& text = std::string());
};

// Context access

class PyBlendContext : public PyRef {
public:
	PyBlendContext(); // Obtain python ref to bpy.context

	// Global context area access
	PyBlendArea area() const;
	PyScene scene() const;

	// In clip editor context, this is the active clip. Can be null.
	PyMovieClip edit_movieclip() const;

	// In screen context, this is a sequence of FCurves. Can be empty or null.
	PyFCurveSeq selected_editable_fcurves() const;

};

// Operator reference
class PyBOperator : public PyRef {
public:
	using PyRef::PyRef;
	void report(const std::string& type, const std::string& message) const;
};
