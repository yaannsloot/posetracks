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
#include "../tracking/camera.hpp"
#include "../crypto/sha1.hpp"
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

// Be sure to add an option to only filter selected keys later
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
		py_curve.update();
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
		py_curve.update();
	}
	context.area().tag_redraw();
}

void OP_TriangulatePoints(PyBOperator calling_op, const std::string& anchor) {

	if (anchor.empty())
		return;

	// Prep data and triangulate

	PyBlendData data;
	PyBlendContext context;
	context.view_layer().update();
	PyMovieClip py_anchor_clip = data.movieclips()[anchor];
	std::unordered_map<std::string, MovieClip> clip_map;
	auto clips_list = data.movieclips().items();
	for (MovieClip clip = clips_list.first(); !clip.is_null(); clip = clip.id().next()) {
		clip_map[clip.id().name().substr(2)] = clip;
	}
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
		calling_op.report("WARNING", "Triangulation aborted. Check the console for more information.");
		return;
	}
	PyBObject anchor_obj = scene_object_map[anchor_name];
	const std::string solution_id = anchor_obj.data().get_property_str("solution_id");
	if (solution_id.empty()) {
		std::cout << "ANCHOR HAS NO SOLUTION ID! TRIANGULATION ABORTED!" << std::endl;
		calling_op.report("WARNING", "Triangulation aborted. Check the console for more information.");
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
		std::cout << "NO OTHER CAMERAS BESIDES ANCHOR HAVE A MATCHING SOLUTION ID! TRIANGULATION ABORTED!" << std::endl;
		calling_op.report("WARNING", "Triangulation aborted. Check the console for more information.");
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
					id_empty.set_property_str("pose_source", *std::prev(split_name.end(), 2));
					id_empty.set_property_str("pose_name", join_string(split_name.begin(), std::prev(split_name.end(), 2)));
					id_empty.set_property_int("joint_id", joint_id);
					id_empty.set_property_str("cam_solution_id", solution_id);
					py_empty = new_empty;
				}
				loc_map[empty_name].push_back(joint);
				f_map[empty_name].push_back(frame);
			}
		}
	}

	// Detections

	std::vector<std::string> base_det_collection_path{ "MotionEngine", "Tracking", "Detections" };

	for (auto& f_data : detections) {
		int frame = f_data.first;
		for (auto& det_data : f_data.second) {
			auto& det_name = det_data.first;
			auto& det = det_data.second;
			PyBObject& py_empty = empty_map[det_name];
			if (py_empty.is_null()) {
				auto collection_path = base_det_collection_path;
				collection_path.push_back(det_name);
				auto new_empty = get_empty(det_name, collection_path);
				new_empty.select_set(true); // Select empties that were modified
				auto id_empty = new_empty.as_id();
				id_empty.set_property_str("cam_solution_id", solution_id);
				py_empty = new_empty;
			}
			loc_map[det_name].push_back(det.second);
			f_map[det_name].push_back(frame);
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
			auto bezt_x = fc_intern_x.bezt(i);
			auto bezt_y = fc_intern_y.bezt(i);
			auto bezt_z = fc_intern_z.bezt(i);
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
		f_curve_x.update();
		f_curve_y.update();
		f_curve_z.update();
	}
	
	context.view_layer().objects().set_active(anchor_obj);
	PyBlendOps ops;
	ops.OP_OBJ_ParentSet("OBJECT", false, true);

	context.area().tag_redraw();

}

std::string solvecameras_anchor;
std::vector<std::string> camera_names;
std::vector<me::tracking::Mat4x4> camera_transforms;

void OP_SolveCameras_Invoke(const std::string& anchor) {
	solvecameras_anchor = anchor;
	camera_names.clear();
	camera_transforms.clear();
	if (anchor.empty())
		return;
	PyBlendData data;
	PyBlendContext context;
	context.view_layer().update();
	std::vector<MovieClip> clips;
	std::vector<std::string> clip_names;
	for (MovieClip clip = data.movieclips().items().first(); !clip.is_null(); clip = clip.id().next()) {
		std::string clip_name = clip.id().name().substr(2);
		if (clip_name == anchor) {
			clips.insert(clips.begin(), clip);
			clip_names.insert(clip_names.begin(), clip_name);
		} 
		else {
			clips.push_back(clip);
			clip_names.push_back(clip_name);
		}
	}
	const size_t num_clips = clips.size();
	std::vector<size_t> clip_idxs(num_clips);
	std::iota(clip_idxs.begin(), clip_idxs.end(), 0);
	std::vector<me::tracking::TrackingData> t_data(num_clips);
	std::vector<me::tracking::Kk> cam_Kk(num_clips);
	std::for_each(std::execution::par_unseq, clip_idxs.begin(), clip_idxs.end(), [&](size_t i) {
		t_data[i] = clip_tracking_data(clips[i], 0.9, true);
		cam_Kk[i] = get_clip_Kk(clips[i]);
	});
	auto cam_transforms = me::tracking::solveStaticSet(t_data, cam_Kk);
	for (size_t i = 1; i < num_clips; ++i) {
		if (cam_transforms[i - 1].is_identity())
			continue;
		camera_names.push_back(clip_names[i]);
		auto& tf = cam_transforms[i - 1];
		tf.invert();
		camera_transforms.push_back(tf.to4x4().mul(flip_mtx));
	}
}

void OP_SolveCameras_Execute(PyBOperator calling_op, const std::string& anchor, float solution_scale) {
	
	if (anchor.empty())
		return;
	
	// Update solution if anchor was changed
	if (solvecameras_anchor != anchor)
		OP_SolveCameras_Invoke(anchor);

	// Abort on empty solution transforms
	if (camera_transforms.empty()) {
		calling_op.report("WARNING", "Solver could not find any valid camera transforms");
		return;
	}

	PyBlendData data;
	PyBlendContext context;
	context.view_layer().update();

	// Get new solution id
	const std::string solution_id = me::crypto::generateRandomSHA1().to_string();

	PyBObject anchor_obj = prepare_camera_for_clip(anchor);
	anchor_obj.data().set_property_str("solution_id", solution_id);
	PyBMat anchor_tf = anchor_obj.matrix_world();
	me::tracking::Mat4x4 base_tf;

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			base_tf(i, j) = anchor_tf.get(i, j);
		}
	}

	anchor_obj.set_parent(PyBObject());

	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			anchor_tf.set(i, j, static_cast<float>(base_tf(i, j)));
		}
	}

	const size_t num_cams = camera_transforms.size();
	std::vector<PyBObject> cameras(num_cams);
	for (size_t i = 0; i < num_cams; ++i) {
		PyBObject cam_obj = prepare_camera_for_clip(camera_names[i]);
		cameras[i] = cam_obj;
		cam_obj.data().set_property_str("solution_id", solution_id);
		cam_obj.set_parent(PyBObject());
		cam_obj.set_parent(anchor_obj);
		auto cam_tf = camera_transforms[i];
		cam_tf(0, 3) *= solution_scale;
		cam_tf(1, 3) *= solution_scale;
		cam_tf(2, 3) *= solution_scale;
		cam_tf = base_tf * cam_tf;
		PyBMat cam_world = cam_obj.matrix_world();
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				cam_world.set(i, j, static_cast<float>(cam_tf(i, j)));
			}
		}
	}

	if (base_tf == me::tracking::Mat4x4::eye()) {
		anchor_tf.set(1, 1, 0);
		anchor_tf.set(2, 2, 0);
		anchor_tf.set(1, 2, -1);
		anchor_tf.set(2, 1, 1);
		context.view_layer().update();
		float min_x = 0;
		float min_y = 0;
		float min_z = 0;
		float max_x = 0;
		float max_y = 0;
		float max_z = 0;
		for (size_t i = 0; i < num_cams; ++i) {
			PyBObject cam_obj = cameras[i];
			PyBMat cam_tf = cam_obj.matrix_world();
			const float cam_x = cam_tf.get(0, 3);
			const float cam_y = cam_tf.get(1, 3);
			const float cam_z = cam_tf.get(2, 3);
			min_x = (cam_x < min_x) ? cam_x : min_x;
			min_y = (cam_y < min_y) ? cam_y : min_y;
			min_z = (cam_z < min_z) ? cam_z : min_z;
			max_x = (cam_x > max_x) ? cam_x : max_x;
			max_y = (cam_y > max_y) ? cam_y : max_y;
			max_z = (cam_z > max_z) ? cam_z : max_z;
		}
		anchor_tf.set(0, 3, -(min_x + max_x) / 2);
		anchor_tf.set(1, 3, -(min_y + max_y) / 2);
		anchor_tf.set(2, 3, -(min_z + max_z) / 2);
	}

	context.area().tag_redraw();

}
