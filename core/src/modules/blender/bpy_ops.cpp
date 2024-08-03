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

Operator invoke and execute functions used within the add-on code
*/

#include "bpy_ops.hpp"
#include "bpy_types.hpp"
#include "bpy_utils.hpp"
#include "../tracking/filters.hpp"
#include "../tracking/data.hpp"
#include "../tracking/triangulation.hpp"
#include <iostream>
#include <vector>
#include <map>
#include <unordered_set>
#include <algorithm>
#include <execution>

void OP_FilterTrackGaussian(int kernel_width) {
	PyBlendData data; // Points to bpy.data
	PyBlendContext context; // Points to bpy.context
	MovieClip active_clip = context.edit_movieclip().intern();
	if (active_clip.is_null()) {
		std::cout << "ACTIVE CLIP IS NONE! THIS SHOULD NOT HAPPEN!" << std::endl;
		return;
	}
	int* last_size = active_clip.last_size();
	std::vector<MovieTrackingTrack> selected_tracks = get_selected_tracks(active_clip.tracking().objects().first());
	for (auto& track : selected_tracks) {
		std::vector<std::vector<MovieTrackingMarker>> track_frames;
		std::vector<std::vector<double>> track_x;
		std::vector<std::vector<double>> track_y;
		std::vector<MovieTrackingMarker> frames_inter;
		std::vector<double> x_inter;
		std::vector<double> y_inter;
		MovieTrackingMarker first_marker = track.marker(0);
		int last_frame = first_marker.framenr();
		int markersnr = track.markersnr();
		for (int i = 0; i < markersnr; ++i) {
			auto marker = track.marker(i);
			int frame = marker.framenr();
			float* pos = marker.pos();
			if (frame - last_frame > 1) {
				track_frames.push_back(frames_inter);
				track_x.push_back(x_inter);
				track_y.push_back(y_inter);
				frames_inter.clear();
				x_inter.clear();
				y_inter.clear();
			}
			frames_inter.push_back(marker);
			x_inter.push_back(static_cast<double>(pos[0]) * last_size[0]);
			y_inter.push_back(static_cast<double>(pos[1]) * last_size[1]);
			last_frame = frame;
		}
		if (!frames_inter.empty()) {
			track_frames.push_back(frames_inter);
			track_x.push_back(x_inter);
			track_y.push_back(y_inter);
		}
		for (size_t i = 0; i < track_frames.size(); ++i) {
			auto& m_inter = track_frames[i];
			auto& x_inter = track_x[i];
			auto& y_inter = track_y[i];
			int k_width = std::min(kernel_width, static_cast<int>(m_inter.size()));
			auto x_filtered = me::tracking::g_conv_1d(x_inter, k_width);
			auto y_filtered = me::tracking::g_conv_1d(y_inter, k_width);
			for (size_t j = 0; j < m_inter.size(); ++j) {
				auto& marker = m_inter[j];
				float* pos = marker.pos();
				pos[0] = static_cast<float>(x_filtered[j]) / last_size[0];
				pos[1] = static_cast<float>(y_filtered[j]) / last_size[1];
			}
		}
	}
	context.area().tag_redraw();
}

void OP_FilterFCurvesGaussian(int kernel_width) {
	PyBlendContext context;
	PyFCurveSeq editable_fcurves = context.selected_editable_fcurves();
	if (editable_fcurves.is_null()) {
		std::cout << "EDITABLE FCURVES IS NONE! THIS SHOULD NOT HAPPEN!" << std::endl;
		return;
	}
	const int num_fcurves = editable_fcurves.size();
	for (int i = 0; i < num_fcurves; ++i) {
		PyFCurve py_curve = editable_fcurves[i];
		FCurve fcurve = py_curve.intern();
		const size_t num_points = fcurve.totvert();
		if (num_points < 1)
			continue;
		PyFCurveKeyframePoints py_keys = py_curve.keyframe_points();
		py_keys.sort();
		py_keys.deduplicate();
		std::vector<float*> points(num_points);
		std::vector<std::vector<double>> intervals;
		std::vector<double> inter;
		float last_x = fcurve.bezt(0).vec()[1][0];
		for (size_t j = 0; j < num_points; ++j) {
			BezTriple bezt = fcurve.bezt(j);
			BezTripleVecs vecs = bezt.vec();
			// X location (vecs[1][0]) is expected to be in multiples of 1
			// Y location (vecs[1][1]) will be added to interval vector (inter)
			// Snapping is not required in the graph editor, so an optional switch could be added to disable
			// interval splitting.
			// The curves module could also come back into play here for an optional slower method
			if (vecs[1][0] - last_x > 1) {
				intervals.push_back(inter);
				inter.clear();
			}
			inter.push_back(static_cast<double>(vecs[1][1]));
			last_x = vecs[1][0];
			points[j] = &vecs[1][1];
		}
		if (!inter.empty())
			intervals.push_back(inter);
		size_t j = 0;
		for (auto& iv : intervals) {
			int k_width = std::min(kernel_width, static_cast<int>(iv.size()));
			auto filtered = me::tracking::g_conv_1d(iv, k_width);
			for (double& y : filtered) {
				*points[j] = static_cast<float>(y);
				++j;
			}
		}
		py_keys.handles_recalc();
	}
	context.area().tag_redraw();
}

void OP_TriangulatePoints(PyBOperator calling_op, PyMovieClip py_anchor_clip) {

	// Prep data and triangulate

	PyBlendData data;
	std::unordered_map<std::string, MovieClip> clip_map;
	auto clips_list = data.movieclips().items();
	for (MovieClip clip = clips_list.first(); !clip.is_null(); clip = clip.id().next()) {
		clip_map[clip.id().name().substr(2)] = clip;
	}
	PyBlendContext context;
	PyScene scene = context.scene();
	std::unordered_map<std::string, PyBObject> scene_object_map;
	PySceneObjects scene_objects = scene.objects();
	const int scene_obj_num = scene_objects.size();
	for (int i = 0; i < scene_obj_num; ++i) {
		PyBObject py_cam_obj = scene_objects[i];
		py_cam_obj.select_set(false); // Clear selection on all objects
		Object cam_obj = py_cam_obj.intern();
		if (cam_obj.type() != OB_CAMERA)
			continue;
		Camera cam_data = cam_obj.data<Camera>();
		scene_object_map[cam_data.id().name().substr(2)] = py_cam_obj;
	}
	MovieClip anchor_clip = py_anchor_clip.intern();
	const std::string anchor_name = anchor_clip.id().name().substr(2);
	if (scene_object_map.find(anchor_name) == scene_object_map.end()) {
		std::cout << "ANCHOR NOT IN SCENE! TRIANGULATION ABORTED!" << std::endl;
		calling_op.report("ERROR", "Triangulation aborted. Check the console for more information.");
		return;
	}
	PyBObject anchor_obj = scene_object_map[anchor_name];
	const std::string solution_id = anchor_obj.data().get_property_str("solution_id");
	if (solution_id.empty()) {
		std::cout << "ANCHOR HAS NO SOLUTION ID! TRIANGULATION ABORTED!" << std::endl;
		calling_op.report("ERROR", "Triangulation aborted. Check the console for more information.");
		return;
	}
	std::vector<MovieClip> clips;
	std::vector<Object> cams;
	std::vector<me::tracking::TrackingData> t_data;
	std::vector<me::tracking::Kk> cam_Kk;
	std::vector<me::tracking::Rt> cam_Rt;
	std::mutex lock;
	std::for_each(std::execution::par_unseq, clip_map.begin(), clip_map.end(), [&](auto& c_map_pair) {
		if (scene_object_map.find(c_map_pair.first) == scene_object_map.end())
			return;
		auto& py_cam = scene_object_map[c_map_pair.first];
		if (py_cam.data().get_property_str("solution_id") != solution_id)
			return;
		auto& clip = c_map_pair.second;
		auto data = clip_tracking_data(clip, 0.0, true);
		auto cam_intrinsics = get_clip_Kk(clip);
		auto cam_t = get_obj_Rt(py_cam, true);
		std::lock_guard<std::mutex> lock_guard(lock);
		if (c_map_pair.first == anchor_name) {
			clips.insert(clips.begin(), clip);
			cams.insert(cams.begin(), py_cam.intern());
			t_data.insert(t_data.begin(), data);
			cam_Kk.insert(cam_Kk.begin(), cam_intrinsics);
			cam_Rt.insert(cam_Rt.begin(), cam_t);
		}
		else {
			clips.push_back(clip);
			cams.push_back(py_cam.intern());
			t_data.push_back(data);
			cam_Kk.push_back(cam_intrinsics);
			cam_Rt.push_back(cam_t);
		}
	});
	if (clips.size() < 2) {
		std::cout << "SOLUTION CAMERAS NOT FOUND! TRIANGULATION ABORTED!" << std::endl;
		calling_op.report("ERROR", "Triangulation aborted. Check the console for more information.");
		return;
	}
	auto t_data_3d = me::tracking::triangulateStatic(t_data, cam_Kk, cam_Rt);

	// Prepare empties and collect location data

	auto& poses = t_data_3d.poses;
	auto& detections = t_data_3d.detections;
	auto& tags = t_data_3d.tags;

	std::unordered_map<std::string, PyBObject> empty_map; // Note that names should never clash
	std::unordered_map<std::string, std::vector<cv::Point3d>> loc_map;
	std::unordered_map<std::string, std::vector<int>> f_map;

	// If possible, redo all of these to reduce calls to empty_map.find()

	// Poses

	std::vector<std::string> base_pose_collection_path{ "MotionEngine", "Tracking", "Poses" };

	for (auto& f_data : poses) {
		int frame = f_data.first;
		for (auto& id_data : f_data.second) {
			const std::string& pose_id = id_data.first;
			for (auto& joint_data : id_data.second) {
				int joint_id = joint_data.first;
				auto& joint = joint_data.second;
				std::string empty_name = pose_id + '.' + std::to_string(joint_id);
				PyBObject &py_empty = empty_map[empty_name];
				if (py_empty.is_null()) {
					auto collection_path = base_pose_collection_path;
					collection_path.push_back(pose_id);
					auto new_empty = get_empty(empty_name, collection_path);
					new_empty.select_set(true); // Select empties that were modified
					auto split_name = split_str(empty_name);
					auto id_empty = new_empty.as_id();
					id_empty.set_property_str("pose_source", split_name.back());
					id_empty.set_property_str("pose_name", join_string(split_name.begin(), std::prev(split_name.end())));
					id_empty.set_property_int("joint_id", joint_id);
					id_empty.set_property_str("cam_solution_id", solution_id);
					py_empty = new_empty;
				}
				loc_map[empty_name].push_back(joint);
				f_map[empty_name].push_back(frame);
			}
		}
	}

	// Write animations

	for (auto& e_data : empty_map) {
		const auto& e_name = e_data.first;
		auto& empty = e_data.second;
		auto& locations = loc_map[e_name];
		auto& frames = f_map[e_name];
		auto anim_data = empty.animation_data();
		auto f_curves = anim_data.action().fcurves();
		f_curves.clear();
		auto f_curve_x = f_curves.new_fcurve("location", 0);
		auto f_curve_y = f_curves.new_fcurve("location", 1);
		auto f_curve_z = f_curves.new_fcurve("location", 2);
		auto x_keypoints = f_curve_x.keyframe_points();
		auto y_keypoints = f_curve_y.keyframe_points();
		auto z_keypoints = f_curve_z.keyframe_points();
		int num_frames = static_cast<int>(locations.size());
		x_keypoints.add(num_frames);
		y_keypoints.add(num_frames);
		z_keypoints.add(num_frames);
		auto fc_intern_x = f_curve_x.intern();
		auto fc_intern_y = f_curve_y.intern();
		auto fc_intern_z = f_curve_z.intern();
		std::vector<int> kf_idxs(locations.size());
		std::iota(kf_idxs.begin(), kf_idxs.end(), 0);
		std::for_each(std::execution::par_unseq, kf_idxs.begin(), kf_idxs.end(), [&](int i) {
			int frame = frames[i];
			auto& pt = locations[i];
			auto& bezt_x = fc_intern_x.bezt(i);
			auto& bezt_y = fc_intern_y.bezt(i);
			auto& bezt_z = fc_intern_z.bezt(i);
			auto vec_x = bezt_x.vec();
			auto vec_y = bezt_y.vec();
			auto vec_z = bezt_z.vec();
			vec_x[1][0] = static_cast<float>(frame);
			vec_y[1][0] = static_cast<float>(frame);
			vec_z[1][0] = static_cast<float>(frame);
			vec_x[1][1] = static_cast<float>(pt.x);
			vec_y[1][1] = static_cast<float>(-pt.y);
			vec_z[1][1] = static_cast<float>(-pt.z);
		});
		x_keypoints.sort();
		y_keypoints.sort();
		z_keypoints.sort();
		x_keypoints.deduplicate();
		y_keypoints.deduplicate();
		z_keypoints.deduplicate();
		x_keypoints.handles_recalc();
		y_keypoints.handles_recalc();
		z_keypoints.handles_recalc();
	}

	context.view_layer().objects().set_active(anchor_obj);
	PyBlendOps ops;
	ops.OP_OBJ_ParentSet("OBJECT", false, true);

	context.area().tag_redraw();

}

