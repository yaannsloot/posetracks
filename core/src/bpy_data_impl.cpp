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

Function implementations for bpy_data.hpp
*/

#include "modules/blender/bpy_data.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>

namespace py = pybind11;

#define NOARG

#define REF_FUNC_RET_HEADER(default_ret) \
	py::gil_scoped_acquire lock; \
	if (is_null()) \
		return default_ret; \
	py::object* obj = reinterpret_cast<py::object*>(py_obj)

#define REF_FUNC_HEADER REF_FUNC_RET_HEADER(NOARG)

#define REF_FUNC_LISTBASE(T) \
	REF_FUNC_RET_HEADER(ListBaseData(T(), T())); \
	if (obj->attr("__len__")().cast<int>() == 0) \
		return ListBaseData(T(), T()); \
	T first = bpy_wrap<T>(obj->attr("__getitem__")(0)); \
	T last = bpy_wrap<T>(obj->attr("__getitem__")(-1)); \
	return ListBaseData(first, last)

#define REF_FUNC_INTERN(T) \
	REF_FUNC_RET_HEADER(T()); \
	return bpy_wrap<T>(*obj)

#define DEF_INDEX_OP_INT(class_name, ret_type) \
ret_type class_name::operator[](const int& idx) const { \
	REF_FUNC_RET_HEADER(ret_type()); \
	py::object ret_obj = obj->attr("__getitem__")(idx); \
	return ret_type(&ret_obj); \
}

#define DEF_INDEX_OP_STR(class_name, ret_type) \
ret_type class_name::operator[](const std::string& name) const { \
	REF_FUNC_RET_HEADER(ret_type()); \
	py::object ret_obj = obj->attr("get")(name, py::none()); \
	return ret_type(&ret_obj); \
}

#define DEF_INDEX_OP_ALL(class_name, ret_type) \
	DEF_INDEX_OP_INT(class_name, ret_type) \
	DEF_INDEX_OP_STR(class_name, ret_type)

#define DEF_INTERN(class_name, ret_type) \
ret_type class_name::intern() const { \
	REF_FUNC_INTERN(ret_type); \
}

#define DEF_ITEMS(class_name, list_type) \
ListBaseData<list_type> class_name::items() const { \
	REF_FUNC_LISTBASE(list_type); \
}

template<typename T>
T bpy_wrap(py::object py_obj) {
	return T(reinterpret_cast<void*>(py_obj.attr("as_pointer")().cast<uintptr_t>()));
}

template<typename T>
T* bpy_ptr(py::object py_obj) {
	return (T*)reinterpret_cast<void*>(py_obj.attr("as_pointer")().cast<uintptr_t>());
}

PyRef::PyRef(void* py_obj) {
	if (py_obj == nullptr)
		return;
	py::gil_scoped_acquire lock;
	this->py_obj = new py::object;
	*reinterpret_cast<py::object*>(this->py_obj) = *reinterpret_cast<py::object*>(py_obj);
}

void PyRef::clear_ref() {
	if (py_obj == nullptr)
		return;
	py::gil_scoped_acquire lock;
	delete reinterpret_cast<py::object*>(py_obj);
	py_obj = nullptr;
}

PyRef::~PyRef() {
	clear_ref();
}

bool PyRef::is_null() const {
	if (py_obj == nullptr)
		return true;
	py::gil_scoped_acquire lock;
	return reinterpret_cast<py::object*>(py_obj)->is_none();
}

PyRef& PyRef::operator=(const PyRef& other) {
	clear_ref();
	if (!other.is_null()) {
		py::gil_scoped_acquire lock;
		this->py_obj = new py::object;
		*reinterpret_cast<py::object*>(this->py_obj) = *reinterpret_cast<py::object*>(other.py_obj);
	}
	return *this;
}

std::string PyID::get_property_str(const std::string& prop_name) const {
	REF_FUNC_RET_HEADER(std::string());
	py::object ret_obj = obj->attr("get")(prop_name, py::none());
	return (ret_obj.is_none()) ? std::string() : ret_obj.cast<std::string>();
}

bool PyID::get_property_bool(const std::string& prop_name) const {
	REF_FUNC_RET_HEADER(false);
	py::object ret_obj = obj->attr("get")(prop_name, py::none());
	return (ret_obj.is_none()) ? false : ret_obj.cast<bool>();
}

float PyID::get_property_float(const std::string& prop_name) const {
	REF_FUNC_RET_HEADER(0);
	py::object ret_obj = obj->attr("get")(prop_name, py::none());
	return (ret_obj.is_none()) ? 0 : ret_obj.cast<float>();
}

int PyID::get_property_int(const std::string& prop_name) const {
	REF_FUNC_RET_HEADER(0);
	py::object ret_obj = obj->attr("get")(prop_name, py::none());
	return (ret_obj.is_none()) ? 0 : ret_obj.cast<int>();
}

void PyID::set_property_str(const std::string& prop_name, const std::string& val) {
	REF_FUNC_HEADER;
	obj->attr("__setitem__")(prop_name, val);
}

void PyID::set_property_bool(const std::string& prop_name, const bool val) {
	REF_FUNC_HEADER;
	obj->attr("__setitem__")(prop_name, val);
}

void PyID::set_property_float(const std::string& prop_name, const float val) {
	REF_FUNC_HEADER;
	obj->attr("__setitem__")(prop_name, val);
}

void PyID::set_property_int(const std::string& prop_name, const int val) {
	REF_FUNC_HEADER;
	obj->attr("__setitem__")(prop_name, val);
}

// -------------------- BlendDataActions --------------------

// PyKeyframe

DEF_INTERN(PyKeyframe, BezTriple)

// PyFCurveKeyframePoints

PyKeyframe PyFCurveKeyframePoints::insert(const float frame, const float value, const std::set<std::string>& options, const std::string& keyframe_type) {
	REF_FUNC_RET_HEADER(PyKeyframe());
	py::object ret_obj = obj->attr("insert")(frame, value, py::arg("options") = options, py::arg("keyframe_type") = keyframe_type);
	return PyKeyframe(&ret_obj);
}

void PyFCurveKeyframePoints::add(const int count) {
	REF_FUNC_HEADER;
	obj->attr("add")(count);
}

void PyFCurveKeyframePoints::remove(PyKeyframe keyframe, bool fast) {
	REF_FUNC_HEADER;
	auto* kf_obj = reinterpret_cast<py::object*>(keyframe.py_obj);
	obj->attr("remove")(*kf_obj, py::arg("fast") = fast);
}

void PyFCurveKeyframePoints::clear() {
	REF_FUNC_HEADER;
	obj->attr("clear")();
}

void PyFCurveKeyframePoints::sort() {
	REF_FUNC_HEADER;
	obj->attr("sort")();
}

void PyFCurveKeyframePoints::deduplicate() {
	REF_FUNC_HEADER;
	obj->attr("deduplicate")();
}

void PyFCurveKeyframePoints::handles_recalc() {
	REF_FUNC_HEADER;
	obj->attr("handles_recalc")();
}

DEF_INDEX_OP_INT(PyFCurveKeyframePoints, PyKeyframe)

// PyActionGroup

DEF_INTERN(PyActionGroup, bActionGroup)

// PyFCurve

PyActionGroup PyFCurve::group() const {
	REF_FUNC_RET_HEADER(PyActionGroup());
	py::object ret_obj = obj->attr("group");
	return PyActionGroup(&ret_obj);
}

PyFCurveKeyframePoints PyFCurve::keyframe_points() const {
	REF_FUNC_RET_HEADER(PyFCurveKeyframePoints());
	py::object ret_obj = obj->attr("keyframe_points");
	return PyFCurveKeyframePoints(&ret_obj);
}

DEF_INTERN(PyFCurve, FCurve)

// PyActionFCurves

PyFCurve PyActionFCurves::new_fcurve(const std::string& data_path, const int index, const std::string& action_group) {
	REF_FUNC_RET_HEADER(PyFCurve());
	py::object ret_obj = obj->attr("new")(data_path, py::arg("index") = index, py::arg("action_group") = action_group);
	return PyFCurve(&ret_obj);
}

PyFCurve PyActionFCurves::find(const std::string& data_path, const int index) const {
	REF_FUNC_RET_HEADER(PyFCurve());
	py::object ret_obj = obj->attr("find")(data_path, py::arg("index") = index);
	return PyFCurve(&ret_obj);
}

void PyActionFCurves::remove(PyFCurve fcurve) {
	REF_FUNC_HEADER;
	auto* py_fcurve = reinterpret_cast<py::object*>(fcurve.py_obj);
	obj->attr("remove")(*py_fcurve);
}

void PyActionFCurves::clear() {
	REF_FUNC_HEADER;
	obj->attr("clear")();
}

DEF_ITEMS(PyActionFCurves, FCurve)
DEF_INDEX_OP_ALL(PyActionFCurves, PyFCurve)

// PyAction

PyActionFCurves PyAction::fcurves() const {
	REF_FUNC_RET_HEADER(PyActionFCurves());
	py::object ret_obj = obj->attr("fcurves");
	return PyActionFCurves(&ret_obj);
}

DEF_INTERN(PyAction, bAction)

// PyBlendDataActions

PyAction PyBlendDataActions::new_action(const std::string& name) {
	REF_FUNC_RET_HEADER(PyAction());
	py::object ret_obj = obj->attr("new")(name);
	return PyAction(&ret_obj);
}

void PyBlendDataActions::remove(bAction action, bool do_unlink, bool do_id_user, bool do_ui_user) {
	REF_FUNC_HEADER;
	std::string name = action.id().name();
	name = name.substr(2);
	py::object py_action = obj->attr("__getitem__")(name);
	obj->attr("remove")(py_action, py::arg("do_unlink") = do_unlink, py::arg("do_id_user") = do_id_user, py::arg("do_id_user") = do_ui_user);
}

void PyBlendDataActions::remove(PyAction action, bool do_unlink, bool do_id_user, bool do_ui_user) {
	REF_FUNC_HEADER;
	py::object py_action = *reinterpret_cast<py::object*>(action.py_obj);
	obj->attr("remove")(py_action, py::arg("do_unlink") = do_unlink, py::arg("do_id_user") = do_id_user, py::arg("do_id_user") = do_ui_user);
}

DEF_ITEMS(PyBlendDataActions, bAction)
DEF_INDEX_OP_ALL(PyBlendDataActions, PyAction)

// -------------------- BlendDataMovieClips --------------------

// PyMovieTrackingMarkers

MovieTrackingMarker PyMovieTrackingMarkers::insert_frame(int frame, float x, float y) {
	REF_FUNC_RET_HEADER(MovieTrackingMarker());
	return bpy_wrap<MovieTrackingMarker>(obj->attr("insert_frame")(frame, py::arg("co") = py::make_tuple(x, y)));
}

void PyMovieTrackingMarkers::delete_frame(int frame) {
	REF_FUNC_HEADER;
	obj->attr("delete_frame")(frame);
}

DEF_ITEMS(PyMovieTrackingMarkers, MovieTrackingMarker)

// PyMovieTrackingTrack

PyMovieTrackingMarkers PyMovieTrackingTrack::markers() const {
	REF_FUNC_RET_HEADER(PyMovieTrackingMarkers());
	py::object markers_obj = obj->attr("markers");
	return PyMovieTrackingMarkers(&markers_obj);
}

DEF_INTERN(PyMovieTrackingTrack, MovieTrackingTrack)

// PyMovieTrackingTracks

PyMovieTrackingTrack PyMovieTrackingTracks::new_track(const std::string& name, int frame) {
	REF_FUNC_RET_HEADER(PyMovieTrackingTrack());
	py::object track_obj = obj->attr("new")(py::arg("name") = name, py::arg("frame") = frame);
	return PyMovieTrackingTrack(&track_obj);
}

DEF_ITEMS(PyMovieTrackingTracks, MovieTrackingTrack)
DEF_INDEX_OP_ALL(PyMovieTrackingTracks, PyMovieTrackingTrack)

// PyMovieTrackingPlaneTracks

DEF_ITEMS(PyMovieTrackingPlaneTracks, MovieTrackingPlaneTrack)

// PyMovieTrackingObject

PyMovieTrackingPlaneTracks PyMovieTrackingObject::plane_tracks() const {
	REF_FUNC_RET_HEADER(PyMovieTrackingPlaneTracks());
	py::object tracks_obj = obj->attr("plane_tracks");
	return PyMovieTrackingPlaneTracks(&tracks_obj);
}

PyMovieTrackingTracks PyMovieTrackingObject::tracks() const {
	REF_FUNC_RET_HEADER(PyMovieTrackingTracks());
	py::object tracks_obj = obj->attr("tracks");
	return PyMovieTrackingTracks(&tracks_obj);
}

DEF_INTERN(PyMovieTrackingObject, MovieTrackingObject)

// PyMovieTrackingObjects

PyMovieTrackingObject PyMovieTrackingObjects::new_object(const std::string& name) {
	REF_FUNC_RET_HEADER(PyMovieTrackingObject());
	py::object tracking_obj = obj->attr("new")(name);
	return PyMovieTrackingObject(&tracking_obj);
}

void PyMovieTrackingObjects::remove(MovieTrackingObject object) {
	REF_FUNC_HEADER;
	py::object py_obj = obj->attr("__getitem__")(object.name());
	obj->attr("remove")(py_obj);
}

DEF_ITEMS(PyMovieTrackingObjects, MovieTrackingObject)
DEF_INDEX_OP_ALL(PyMovieTrackingObjects, PyMovieTrackingObject)

// PyMovieTracking

PyMovieTrackingObjects PyMovieTracking::objects() const {
	REF_FUNC_RET_HEADER(PyMovieTrackingObjects());
	py::object py_obj = obj->attr("objects");
	return PyMovieTrackingObjects(&py_obj);
}

PyMovieTrackingPlaneTracks PyMovieTracking::plane_tracks() const {
	REF_FUNC_RET_HEADER(PyMovieTrackingPlaneTracks());
	py::object py_obj = obj->attr("plane_tracks");
	return PyMovieTrackingPlaneTracks(&py_obj);
}

PyMovieTrackingTracks PyMovieTracking::tracks() const {
	REF_FUNC_RET_HEADER(PyMovieTrackingTracks());
	py::object py_obj = obj->attr("tracks");
	return PyMovieTrackingTracks(&py_obj);
}

DEF_INTERN(PyMovieTracking, MovieTracking)

// PyMovieClip

PyMovieTracking PyMovieClip::tracking() const {
	REF_FUNC_RET_HEADER(PyMovieTracking());
	py::object py_obj = obj->attr("tracking");
	return PyMovieTracking(&py_obj);
};

DEF_INTERN(PyMovieClip, MovieClip)

// PyBlendDataMovieClips

void PyBlendDataMovieClips::remove(MovieClip clip, bool do_unlink, bool do_id_user, bool do_ui_user) {
	REF_FUNC_HEADER;
	std::string name = clip.id().name();
	name = name.substr(2);
	py::object py_clip = obj->attr("__getitem__")(name);
	obj->attr("remove")(py_clip, py::arg("do_unlink") = do_unlink, py::arg("do_id_user") = do_id_user, py::arg("do_id_user") = do_ui_user);
}

PyMovieClip PyBlendDataMovieClips::load(const std::string& filepath, bool check_existing) {
	REF_FUNC_RET_HEADER(PyMovieClip());
	py::object py_clip = obj->attr("load")(filepath, py::arg("check_existing") = check_existing);
	return PyMovieClip(&py_clip);
}

DEF_ITEMS(PyBlendDataMovieClips, MovieClip)
DEF_INDEX_OP_ALL(PyBlendDataMovieClips, PyMovieClip)

// -------------------- BlendDataCollections --------------------

// PyCollectionChildren

const int PyCollectionChildren::size() {
	REF_FUNC_RET_HEADER(0);
	return obj->attr("__len__")().cast<int>();
}

void PyCollectionChildren::link(PyBCollection child) {
	REF_FUNC_HEADER;
	py::object ref_obj = *reinterpret_cast<py::object*>(child.py_obj);
	obj->attr("link")(ref_obj);
}

void PyCollectionChildren::unlink(PyBCollection child) {
	REF_FUNC_HEADER;
	py::object ref_obj = *reinterpret_cast<py::object*>(child.py_obj);
	obj->attr("unlink")(ref_obj);
}

DEF_INDEX_OP_ALL(PyCollectionChildren, PyBCollection)

// PyCollectionObjects

const int PyCollectionObjects::size() {
	REF_FUNC_RET_HEADER(0);
	return obj->attr("__len__")().cast<int>();
}

void PyCollectionObjects::link(PyBObject child) {
	REF_FUNC_HEADER;
	py::object ref_obj = *reinterpret_cast<py::object*>(child.py_obj);
	obj->attr("link")(ref_obj);
}

void PyCollectionObjects::unlink(PyBObject child) {
	REF_FUNC_HEADER;
	py::object ref_obj = *reinterpret_cast<py::object*>(child.py_obj);
	obj->attr("unlink")(ref_obj);
}

DEF_INDEX_OP_ALL(PyCollectionObjects, PyBObject)

// PyBCollection

PyCollectionChildren PyBCollection::children() const {
	REF_FUNC_RET_HEADER(PyCollectionChildren());
	py::object ret_obj = obj->attr("children");
	return PyCollectionChildren(&ret_obj);
}

PyCollectionObjects PyBCollection::objects() const {
	REF_FUNC_RET_HEADER(PyCollectionObjects());
	py::object ret_obj = obj->attr("objects");
	return PyCollectionObjects(&ret_obj);
}

DEF_INTERN(PyBCollection, Collection)

// PyBlendDataCollections

PyBCollection PyBlendDataCollections::new_collection(const std::string& name) {
	REF_FUNC_RET_HEADER(PyBCollection());
	py::object ret_obj = obj->attr("new")(name);
	return PyBCollection(&ret_obj);
}

void PyBlendDataCollections::remove(Collection collection, bool do_unlink, bool do_id_user, bool do_ui_user) {
	REF_FUNC_HEADER;
	std::string name = collection.id().name();
	name = name.substr(2);
	py::object b_obj = obj->attr("__getitem__")(name);
	obj->attr("remove")(b_obj, py::arg("do_unlink") = do_unlink, py::arg("do_id_user") = do_id_user, py::arg("do_id_user") = do_ui_user);
}

DEF_ITEMS(PyBlendDataCollections, Collection)
DEF_INDEX_OP_ALL(PyBlendDataCollections, PyBCollection)

// -------------------- BlendDataObjects --------------------

// PyAnimData

PyAction PyAnimData::action() const {
	REF_FUNC_RET_HEADER(PyAction());
	py::object ret_obj = obj->attr("action");
	return PyAction(&ret_obj);
}

void PyAnimData::set_active_action(PyAction action) const {
	REF_FUNC_HEADER;
	py::object ref_action = *reinterpret_cast<py::object*>(action.py_obj);
	obj->attr("action") = ref_action;
}

DEF_INTERN(PyAnimData, AnimData)

// PyBMat

float PyBMat::get(const int row, const int col) const {
	REF_FUNC_RET_HEADER(0.0f);
	return obj->attr("__getitem__")(row).attr("__getitem__")(col).cast<float>();
}

void PyBMat::set(const int row, const int col, const float val) {
	REF_FUNC_HEADER;
	obj->attr("__getitem__")(row).attr("__setitem__")(col, val);
}

// PyBObject

PyID PyBObject::as_id() const {
	REF_FUNC_RET_HEADER(PyID());
	return PyID(py_obj);
}

PyAnimData PyBObject::animation_data() const {
	REF_FUNC_RET_HEADER(PyAnimData());
	py::object ret_obj = obj->attr("animation_data");
	return PyAnimData(&ret_obj);
}

PyAnimData PyBObject::animation_data_create() const {
	REF_FUNC_RET_HEADER(PyAnimData());
	py::object ret_obj = obj->attr("animation_data_create")();
	return PyAnimData(&ret_obj);
}

void PyBObject::animation_data_clear() const {
	REF_FUNC_HEADER;
	obj->attr("animation_data_clear")();
}

PyID PyBObject::data() const {
	REF_FUNC_RET_HEADER(PyID());
	py::object ret_obj = obj->attr("data");
	return PyID(&ret_obj);
}

PyBMat PyBObject::matrix_world() const {
	REF_FUNC_RET_HEADER(PyBMat());
	py::object ret_obj = obj->attr("matrix_world");
	return PyBMat(&ret_obj);
}

void PyBObject::select_set(bool val) {
	REF_FUNC_HEADER;
	obj->attr("select_set")(val);
}

DEF_INTERN(PyBObject, Object)

// PyBlendDataObjects

PyBObject PyBlendDataObjects::new_object(const std::string& name) {
	REF_FUNC_RET_HEADER(PyBObject());
	py::object ret_obj = obj->attr("new")(name, py::none());
	return PyBObject(&ret_obj);
}

void PyBlendDataObjects::remove(Object object, bool do_unlink, bool do_id_user, bool do_ui_user) {
	REF_FUNC_HEADER;
	std::string name = object.id().name();
	name = name.substr(2);
	py::object b_obj = obj->attr("__getitem__")(name);
	obj->attr("remove")(b_obj, py::arg("do_unlink") = do_unlink, py::arg("do_id_user") = do_id_user, py::arg("do_id_user") = do_ui_user);
}

DEF_ITEMS(PyBlendDataObjects, Object)
DEF_INDEX_OP_ALL(PyBlendDataObjects, PyBObject)

// -------------------- BlendDataScenes --------------------

const int PySceneObjects::size() {
	REF_FUNC_RET_HEADER(0);
	return obj->attr("__len__")().cast<int>();
}

DEF_INDEX_OP_ALL(PySceneObjects, PyBObject)

PySceneObjects PyScene::objects() const {
	REF_FUNC_RET_HEADER(PySceneObjects());
	py::object ret_obj = obj->attr("objects");
	return PySceneObjects(&ret_obj);
}

PyBCollection PyScene::collection() const {
	REF_FUNC_RET_HEADER(PyBCollection());
	py::object ret_obj = obj->attr("collection");
	return PyBCollection(&ret_obj);
}

// -------------------- BlendData --------------------

PyBlendData::PyBlendData() {
	py::gil_scoped_acquire lock;
	this->py_obj = new py::object;
	py::object bpy = py::module::import("bpy");
	py::object data = bpy.attr("data");
	*reinterpret_cast<py::object*>(this->py_obj) = data;
}

PyBlendDataActions PyBlendData::actions() const {
	REF_FUNC_RET_HEADER(PyBlendDataActions());
	py::object ret_obj = obj->attr("actions");
	return PyBlendDataActions(&ret_obj);
}

PyBlendDataCollections PyBlendData::collections() const {
	REF_FUNC_RET_HEADER(PyBlendDataCollections());
	py::object ret_obj = obj->attr("collections");
	return PyBlendDataCollections(&ret_obj);
}

PyBlendDataMovieClips PyBlendData::movieclips() const {
	REF_FUNC_RET_HEADER(PyBlendDataMovieClips());
	py::object clip_data = obj->attr("movieclips");
	return PyBlendDataMovieClips(&clip_data);
}

PyBlendDataObjects PyBlendData::objects() const {
	REF_FUNC_RET_HEADER(PyBlendDataObjects());
	py::object ret_obj = obj->attr("objects");
	return PyBlendDataObjects(&ret_obj);
}

const int PyFCurveSeq::size() {
	REF_FUNC_RET_HEADER(0);
	return obj->attr("__len__")().cast<int>();
}

PyFCurve PyFCurveSeq::operator[](const int& idx) const {
	REF_FUNC_RET_HEADER(PyFCurve());
	py::object ret_obj = obj->attr("__getitem__")(idx);
	return PyFCurve(&ret_obj);
}

void PyBlendArea::tag_redraw() const {
	REF_FUNC_HEADER;
	obj->attr("tag_redraw")();
}

void PyBlendArea::header_set_text(const std::string& text) {
	REF_FUNC_HEADER;
	obj->attr("header_set_text")(text);
}

PyBlendContext::PyBlendContext() {
	py::gil_scoped_acquire lock;
	this->py_obj = new py::object;
	py::object bpy = py::module::import("bpy");
	py::object context = bpy.attr("context");
	*reinterpret_cast<py::object*>(this->py_obj) = context;
}

PyBlendArea PyBlendContext::area() const {
	REF_FUNC_RET_HEADER(PyBlendArea());
	py::object ret_obj = obj->attr("area");
	return PyBlendArea(&ret_obj);
}

PyBObject PyLayerObjects::get_active() const {
	REF_FUNC_RET_HEADER(PyBObject());
	py::object ret_obj = obj->attr("active");
	return PyBObject(&ret_obj);
}

void PyLayerObjects::set_active(PyBObject ref) {
	REF_FUNC_HEADER;
	obj->attr("active") = *reinterpret_cast<py::object*>(ref.py_obj);
}

PyLayerObjects PyViewLayer::objects() const {
	REF_FUNC_RET_HEADER(PyLayerObjects());
	py::object ret_obj = obj->attr("objects");
	return PyLayerObjects(&ret_obj);
}

PyScene PyBlendContext::scene() const {
	REF_FUNC_RET_HEADER(PyScene());
	py::object ret_obj = obj->attr("scene");
	return PyScene(&ret_obj);
}

PyViewLayer PyBlendContext::view_layer() const {
	REF_FUNC_RET_HEADER(PyViewLayer());
	py::object ret_obj = obj->attr("view_layer");
	return PyViewLayer(&ret_obj);
}

PyMovieClip PyBlendContext::edit_movieclip() const {
	REF_FUNC_RET_HEADER(PyMovieClip());
	py::object clip = obj->attr("edit_movieclip");
	return PyMovieClip(&clip);
}

PyFCurveSeq PyBlendContext::selected_editable_fcurves() const {
	REF_FUNC_RET_HEADER(PyFCurveSeq());
	py::object ret_obj = obj->attr("selected_editable_fcurves");
	return PyFCurveSeq(&ret_obj);
}

void PyBOperator::report(const std::string& type, const std::string& message) const {
	REF_FUNC_HEADER; 
	std::vector<std::string> type_enum{ type };
	py::set py_type_enum = py::cast(type_enum);
	obj->attr("report")(py_type_enum, message);
}

PyBlendOps::PyBlendOps() {
	py::gil_scoped_acquire lock;
	this->py_obj = new py::object;
	py::object bpy = py::module::import("bpy");
	py::object ops = bpy.attr("ops");
	*reinterpret_cast<py::object*>(this->py_obj) = ops;
}

void PyBlendOps::OP_OBJ_ParentSet(const std::string& type, bool xmirror, bool keep_transform) {
	REF_FUNC_HEADER;
	obj->attr("object").attr("parent_set")(py::arg("type") = type, py::arg("xmirror") = xmirror, py::arg("keep_transform") = keep_transform);
}
