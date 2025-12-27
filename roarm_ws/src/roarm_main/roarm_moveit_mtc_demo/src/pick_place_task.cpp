/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
  Desc:   A demo to show MoveIt Task Constructor in action
*/

#include <Eigen/Geometry>
#include <roarm_moveit_mtc_demo/pick_place_task.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include "pick_place_parameters.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("roarm_moveit_mtc_demo");

namespace {
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
	return Eigen::Translation3d(values[0], values[1], values[2]) *
	       Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
	       Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
	       Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
	return tf2::toMsg(vectorToEigen(values));
};
}  // namespace

namespace roarm_moveit_mtc_demo {

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                 const moveit_msgs::msg::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::msg::CollisionObject createTable(const pick_place_task_demo::Params& params) {
	geometry_msgs::msg::Pose pose = vectorToPose(params.table_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = params.table_name;
	object.header.frame_id = params.table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { params.table_dimensions.at(0), params.table_dimensions.at(1),
		                                 params.table_dimensions.at(2) };
	pose.position.z -= 0.5 * params.table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createObject(const pick_place_task_demo::Params& params) {
	geometry_msgs::msg::Pose pose = vectorToPose(params.object_pose);
	moveit_msgs::msg::CollisionObject object;
	object.id = params.object_name;
	object.header.frame_id = params.object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = { params.object_dimensions.at(0), params.object_dimensions.at(1) };
	pose.position.z += 0.5 * params.object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createBoard(const pick_place_task_demo::Params& params) {
	geometry_msgs::msg::Pose pose;
	pose.position.x = params.chess_board_origin.at(0) + 0.5 * params.chess_board_size.at(0);
	pose.position.y = params.chess_board_origin.at(1) + 0.5 * params.chess_board_size.at(1);
	pose.position.z = params.chess_board_origin.at(2) + 0.5 * params.chess_board_thickness;
	moveit_msgs::msg::CollisionObject object;
	object.id = "chess_board";
	object.header.frame_id = params.chess_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { params.chess_board_size.at(0), params.chess_board_size.at(1),
		                                 params.chess_board_thickness };
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createCaptureBox(const pick_place_task_demo::Params& params,
                                                  const std::string& id,
                                                  const std::vector<double>& offset) {
	geometry_msgs::msg::Pose pose;
	pose.position.x = params.chess_board_origin.at(0) + offset.at(0);
	pose.position.y = params.chess_board_origin.at(1) + offset.at(1);
	pose.position.z = params.chess_board_origin.at(2) + 0.5 * params.chess_capture_box_size.at(2);
	moveit_msgs::msg::CollisionObject object;
	object.id = id;
	object.header.frame_id = params.chess_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { params.chess_capture_box_size.at(0), params.chess_capture_box_size.at(1),
		                                 params.chess_capture_box_size.at(2) };
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createSquare(const pick_place_task_demo::Params& params,
                                               const std::string& id,
                                               int file_index,
                                               int rank_index) {
	geometry_msgs::msg::Pose pose;
	pose.position.x = params.chess_board_origin.at(0) + (file_index + 0.5) * params.chess_square_size;
	pose.position.y = params.chess_board_origin.at(1) + (rank_index + 0.5) * params.chess_square_size;
	pose.position.z = params.chess_board_origin.at(2) + params.chess_board_thickness +
	                  0.5 * params.chess_square_thickness;

	moveit_msgs::msg::CollisionObject object;
	object.id = id;
	object.header.frame_id = params.chess_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { params.chess_square_size, params.chess_square_size,
		                                 params.chess_square_thickness };
	object.primitive_poses.push_back(pose);
	return object;
}

shape_msgs::msg::Mesh loadMesh(const std::string& resource, double scale) {
	shapes::Mesh* mesh = shapes::createMeshFromResource(resource, Eigen::Vector3d(scale, scale, scale));
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(mesh, shape_msg);
	delete mesh;
	return boost::get<shape_msgs::msg::Mesh>(shape_msg);
}

moveit_msgs::msg::CollisionObject createPiece(const pick_place_task_demo::Params& params,
                                              const std::string& id,
                                              const std::string& mesh_path,
                                              int file_index,
                                              int rank_index) {
	geometry_msgs::msg::Pose pose;
	pose.position.x = params.chess_board_origin.at(0) + (file_index + 0.5) * params.chess_square_size;
	pose.position.y = params.chess_board_origin.at(1) + (rank_index + 0.5) * params.chess_square_size;
	pose.position.z = params.chess_board_origin.at(2) + params.chess_board_thickness;

	moveit_msgs::msg::CollisionObject object;
	object.id = id;
	object.header.frame_id = params.chess_reference_frame;
	object.meshes.resize(1);
	object.meshes[0] = loadMesh(mesh_path, params.chess_piece_scale);
	object.mesh_poses.push_back(pose);
	return object;
}

void setupDemoScene(const pick_place_task_demo::Params& params) {
	// Add table and object to planning scene
	rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	if (params.spawn_chess_scene) {
		spawnObject(psi, createBoard(params));
		spawnObject(psi, createCaptureBox(params, "capture_left", params.chess_capture_box_left));
		spawnObject(psi, createCaptureBox(params, "capture_right", params.chess_capture_box_right));
		if (params.spawn_chess_squares) {
			for (int rank = 0; rank < 8; ++rank) {
				for (int file = 0; file < 8; ++file) {
					const std::string id = "square_" + std::to_string(file) + "_" + std::to_string(rank);
					spawnObject(psi, createSquare(params, id, file, rank));
				}
			}
		}

		const std::string base = params.chess_piece_mesh_dir;
		const std::string rook = "file://" + base + "/Rook-NewStandard.stl";
		const std::string knight = "file://" + base + "/Knight-NewStandard.stl";
		const std::string bishop = "file://" + base + "/Bishop-NewStandard.stl";
		const std::string queen = "file://" + base + "/Queen-NewStandard.stl";
		const std::string king = "file://" + base + "/King-NewStandard.stl";
		const std::string pawn = "file://" + base + "/Pawn-NewStandard.stl";

		// White pieces (ranks 1-2)
		spawnObject(psi, createPiece(params, "white_rook_a1", rook, 0, 0));
		spawnObject(psi, createPiece(params, "white_knight_b1", knight, 1, 0));
		spawnObject(psi, createPiece(params, "white_bishop_c1", bishop, 2, 0));
		spawnObject(psi, createPiece(params, "white_queen_d1", queen, 3, 0));
		spawnObject(psi, createPiece(params, "white_king_e1", king, 4, 0));
		spawnObject(psi, createPiece(params, "white_bishop_f1", bishop, 5, 0));
		spawnObject(psi, createPiece(params, "white_knight_g1", knight, 6, 0));
		spawnObject(psi, createPiece(params, "white_rook_h1", rook, 7, 0));
		for (int file = 0; file < 8; ++file) {
			const std::string id = "white_pawn_" + std::to_string(file) + "_2";
			spawnObject(psi, createPiece(params, id, pawn, file, 1));
		}

		// Black pieces (ranks 7-8)
		spawnObject(psi, createPiece(params, "black_rook_a8", rook, 0, 7));
		spawnObject(psi, createPiece(params, "black_knight_b8", knight, 1, 7));
		spawnObject(psi, createPiece(params, "black_bishop_c8", bishop, 2, 7));
		spawnObject(psi, createPiece(params, "black_queen_d8", queen, 3, 7));
		spawnObject(psi, createPiece(params, "black_king_e8", king, 4, 7));
		spawnObject(psi, createPiece(params, "black_bishop_f8", bishop, 5, 7));
		spawnObject(psi, createPiece(params, "black_knight_g8", knight, 6, 7));
		spawnObject(psi, createPiece(params, "black_rook_h8", rook, 7, 7));
		for (int file = 0; file < 8; ++file) {
			const std::string id = "black_pawn_" + std::to_string(file) + "_7";
			spawnObject(psi, createPiece(params, id, pawn, file, 6));
		}
		return;
	}

	if (params.spawn_table)
		spawnObject(psi, createTable(params));
	spawnObject(psi, createObject(params));
}

PickPlaceTask::PickPlaceTask(const std::string& task_name) : task_name_(task_name) {}

bool PickPlaceTask::init(const rclcpp::Node::SharedPtr& node, const pick_place_task_demo::Params& params) {
	RCLCPP_INFO(LOGGER, "Initializing task pipeline");

	// Reset ROS introspection before constructing the new object
	// TODO(v4hn): global storage for Introspection services to enable one-liner
	task_.reset();
	task_.reset(new moveit::task_constructor::Task());

	// Individual movement stages are collected within the Task object
	Task& t = *task_;
	t.stages()->setName(task_name_);
	t.loadRobotModel(node);

	/* Create planners used in various stages. Various options are available,
	   namely Cartesian, MoveIt pipeline, and joint interpolation. */
	// Sampling planner
	auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
	sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

	// Cartesian planner
	auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
	cartesian_planner->setMaxVelocityScalingFactor(1.0);
	cartesian_planner->setMaxAccelerationScalingFactor(1.0);
	cartesian_planner->setStepSize(.01);
	
	// JointInterpolation planner
	auto JointInterpolation_planner = std::make_shared<solvers::JointInterpolationPlanner>();
	
	// Set task properties
	t.setProperty("group", params.arm_group_name);
	t.setProperty("eef", params.eef_name);
	t.setProperty("hand", params.hand_group_name);
	t.setProperty("hand_grasping_frame", params.hand_frame);
	t.setProperty("ik_frame", params.hand_frame);

	/****************************************************
	 *                                                  *
	 *               Current State                      *
	 *                                                  *
	 ***************************************************/
	{
		auto current_state = std::make_unique<stages::CurrentState>("current state");

		// Verify that object is not attached
		auto applicability_filter =
		    std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
		applicability_filter->setPredicate([object = params.object_name](const SolutionBase& s, std::string& comment) {
			if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
				comment = "object with id '" + object + "' is already attached and cannot be picked";
				return false;
			}
			return true;
		});
		t.add(std::move(applicability_filter));
	}

	/****************************************************
	 *                                                  *
	 *               Open Hand                          *
	 *                                                  *
	 ***************************************************/
	Stage* initial_state_ptr = nullptr;
	{
		auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
		stage->setGroup(params.hand_group_name);
		stage->setGoal(params.hand_open_pose);
		initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Move to Pick                       *
	 *                                                  *
	 ***************************************************/
	// Connect initial open-hand state with pre-grasp pose defined in the following
	{
		auto stage = std::make_unique<stages::Connect>(
		    "move to pick", stages::Connect::GroupPlannerVector{ { params.arm_group_name, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *               Pick Object                        *
	 *                                                  *
	 ***************************************************/
	Stage* pick_stage_ptr = nullptr;
	{
		// A SerialContainer combines several sub-stages, here for picking the object
		auto grasp = std::make_unique<SerialContainer>("pick object");
		t.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
		grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

		/****************************************************
  ---- *               Approach Object                    *
		 ***************************************************/
		{
			// Move the eef link forward along its z-axis by an amount within the given min-max range
			auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
			stage->properties().set("marker_ns", "approach_object");
			stage->properties().set("link", params.hand_frame);  // link to perform IK for
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });  // inherit group from parent stage
			stage->setMinMaxDistance(params.approach_object_min_dist, params.approach_object_max_dist);

			// Set hand forward direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.hand_frame;
			vec.vector.x = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Generate Grasp Pose                *
		 ***************************************************/
		{
			// Sample grasp pose candidates in angle increments around the z-axis of the object
			auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->properties().set("marker_ns", "grasp_pose");
			stage->setPreGraspPose(params.hand_open_pose);
			stage->setObject(params.object_name);  // object to sample grasps for
			stage->setAngleDelta(M_PI / 12);
			stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

			// Compute IK for sampled grasp poses
			auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(8);  // limit number of solutions
			wrapper->setMinSolutionDistance(1.0);
			// define virtual frame to reach the target_pose
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });  // inherit properties from parent
			wrapper->properties().configureInitFrom(Stage::INTERFACE,
			                                        { "target_pose" });  // inherit property from child solution
			grasp->insert(std::move(wrapper));
		}

		/****************************************************
  ---- *               Allow Collision (hand object)   *
		 ***************************************************/
		{
			// Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
			stage->allowCollisions(
			    params.object_name,
			    t.getRobotModel()->getJointModelGroup(params.hand_group_name)->getLinkModelNamesWithCollisionGeometry(),
			    true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  ---- *               Close Hand                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
			stage->setGroup(params.hand_group_name);
			stage->setGoal(params.hand_close_pose);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Attach Object                      *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
			stage->attachObject(params.object_name, params.hand_frame);  // attach object to hand_frame_
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Allow collision (object support)   *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
			stage->allowCollisions({ params.object_name }, { params.surface_link }, true);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Lift object                        *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(params.lift_object_min_dist, params.lift_object_max_dist);
			stage->setIKFrame(params.hand_frame);
			stage->properties().set("marker_ns", "lift_object");

			// Set upward direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.world_frame;
			vec.vector.z = 1.0;
			stage->setDirection(vec);
			grasp->insert(std::move(stage));
		}

		/****************************************************
  .... *               Forbid collision (object support)  *
		 ***************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,surface)");
			stage->allowCollisions({ params.object_name }, { params.surface_link }, false);
			grasp->insert(std::move(stage));
		}

		pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

		// Add grasp container to task
		t.add(std::move(grasp));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Place                             *
	 *                                                    *
	 *****************************************************/
	{
		// Connect the grasped state to the pre-place state, i.e. realize the object transport
		auto stage = std::make_unique<stages::Connect>(
		    "move to place", stages::Connect::GroupPlannerVector{ { params.arm_group_name, sampling_planner } });
		stage->setTimeout(5.0);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	/******************************************************
	 *                                                    *
	 *          Place Object                              *
	 *                                                    *
	 *****************************************************/
	// All placing sub-stages are collected within a serial container again
	{
		auto place = std::make_unique<SerialContainer>("place object");
		t.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
		place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

		/******************************************************
  ---- *          Lower Object                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
			stage->properties().set("marker_ns", "lower_object");
			stage->properties().set("link", params.hand_frame);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.03, .13);

			// Set downward direction
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.world_frame;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Generate Place Pose                       *
		 *****************************************************/
		{
			// Generate Place Pose
			auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
			stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
			stage->properties().set("marker_ns", "place_pose");
			stage->setObject(params.object_name);

			// Set target pose
			geometry_msgs::msg::PoseStamped p;
			p.header.frame_id = params.object_reference_frame;
			p.pose = vectorToPose(params.place_pose);
			p.pose.position.z += 0.5 * params.object_dimensions[0] + params.place_surface_offset;
			stage->setPose(p);
			stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

			// Compute IK
			auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
			wrapper->setMaxIKSolutions(2);
			wrapper->setIKFrame(vectorToEigen(params.grasp_frame_transform), params.hand_frame);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			place->insert(std::move(wrapper));
		}

		/******************************************************
  ---- *          Open Hand                              *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
			stage->setGroup(params.hand_group_name);
			stage->setGoal(params.hand_open_pose);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Forbid collision (hand, object)        *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
			stage->allowCollisions(params.object_name, *t.getRobotModel()->getJointModelGroup(params.hand_group_name),
			                       false);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Detach Object                             *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
			stage->detachObject(params.object_name, params.hand_frame);
			place->insert(std::move(stage));
		}

		/******************************************************
  ---- *          Retreat Motion                            *
		 *****************************************************/
		{
			auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
			stage->properties().configureInitFrom(Stage::PARENT, { "group" });
			stage->setMinMaxDistance(.12, .25);
			stage->setIKFrame(params.hand_frame);
			stage->properties().set("marker_ns", "retreat");
			geometry_msgs::msg::Vector3Stamped vec;
			vec.header.frame_id = params.hand_frame;
			vec.vector.z = -1.0;
			stage->setDirection(vec);
			//place->insert(std::move(stage));
		}

		// Add place container to task
		t.add(std::move(place));
	}

	/******************************************************
	 *                                                    *
	 *          Move to Home                              *
	 *                                                    *
	 *****************************************************/
	{
		auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		stage->setGoal(params.arm_home_pose);
		stage->restrictDirection(stages::MoveTo::FORWARD);
		t.add(std::move(stage));
	}

	// prepare Task structure for planning
	try {
		t.init();
	} catch (InitStageException& e) {
		RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
		return false;
	}

	return true;
}

bool PickPlaceTask::plan(const std::size_t max_solutions) {
	RCLCPP_INFO(LOGGER, "Start searching for task solutions");

	return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTask::execute() {
	RCLCPP_INFO(LOGGER, "Executing solution trajectory");
	moveit_msgs::msg::MoveItErrorCodes execute_result;

	execute_result = task_->execute(*task_->solutions().front());

	if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
		RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed and returned: " << execute_result.val);
		return false;
	}

	return true;
}
}  // namespace roarm_moveit_mtc_demo
