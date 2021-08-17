#define stdmsg std_msgs::msg
#define stdsrv std_srvs::srv
#define geomsg geometry_msgs::msg
#define ssrmsg sensor_msgs::msg

#if 1
#include <webots/GPS.hpp>
#include <webots/LED.hpp>
#include <webots/Pen.hpp>
#include <webots/Gyro.hpp>
#include <webots/Node.hpp>
#include <webots/Skin.hpp>
#include <webots/Brake.hpp>
#include <webots/Field.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Mouse.hpp>
#include <webots/Radar.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/Device.hpp>
#include <webots/Compass.hpp>
#include <webots/Display.hpp>
#include <webots/Emitter.hpp>
#include <webots/Speaker.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Joystick.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Receiver.hpp>
#include <webots/Connector.hpp>
#include <webots/Supervisor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/vehicle/Car.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/utils/Motion.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/utils/AnsiCodes.hpp>
#include <webots/DifferentialWheels.hpp>
#endif

#if 1

//ament_index_cpp
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/has_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#include <ament_index_cpp/get_search_paths.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//rclcpp
#include <rclcpp/qos.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/event.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/wait_set.hpp>
#include <rclcpp/waitable.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp/exceptions.hpp>
#include <rclcpp/scope_exit.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp/wait_result.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/init_options.hpp>
#include <rclcpp/message_info.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/parameter_map.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/any_executable.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_service.hpp>
#include <rclcpp/graph_listener.hpp>
#include <rclcpp/loaned_message.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/function_traits.hpp>
#include <rclcpp/guard_condition.hpp>
#include <rclcpp/memory_strategy.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/wait_result_kind.hpp>
#include <rclcpp/memory_strategies.hpp>
#include <rclcpp/parameter_service.hpp>
#include <rclcpp/publisher_factory.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/type_support_decl.hpp>
#include <rclcpp/wait_set_template.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/subscription_traits.hpp>
#include <rclcpp/any_service_callback.hpp>
#include <rclcpp/subscription_factory.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/intra_process_setting.hpp>
#include <rclcpp/topic_statistics_state.hpp>
#include <rclcpp/message_memory_strategy.hpp>
#include <rclcpp/parameter_events_filter.hpp>
#include <rclcpp/any_subscription_callback.hpp>
#include <rclcpp/intra_process_buffer_type.hpp>
#include <rclcpp/subscription_wait_set_mask.hpp>
#include <rclcpp/expand_topic_or_service_name.hpp>

//rclcpp_action
#include <rclcpp_action/qos.hpp>
#include <rclcpp_action/types.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/exceptions.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/visibility_control.hpp>

//std_msgs
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>

//std_srvs
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

//builtin_interfaces
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>

//sensor_msgs
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>
#include <sensor_msgs/point_field_conversion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>

//geometry_msgs
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

//trajectory_msgs
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

//move_base_msgs
#include <move_base_msgs/action/move_base.hpp>

//stereo_msgs
#include <stereo_msgs/msg/disparity_image.hpp>

//shape_msgs
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh_triangle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

//map_msgs
#include <map_msgs/msg/projected_map.hpp>
#include <map_msgs/msg/projected_map_info.hpp>
#include <map_msgs/msg/point_cloud2_update.hpp>
#include <map_msgs/msg/occupancy_grid_update.hpp>
#include <map_msgs/srv/save_map.hpp>
#include <map_msgs/srv/get_map_roi.hpp>
#include <map_msgs/srv/get_point_map.hpp>
#include <map_msgs/srv/get_point_map_roi.hpp>
#include <map_msgs/srv/projected_maps_info.hpp>
#include <map_msgs/srv/set_map_projections.hpp>

//nav_msgs
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <nav_msgs/srv/set_map.hpp>
#include <nav_msgs/srv/get_plan.hpp>

//visualization_msgs
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_pose.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/srv/get_interactive_markers.hpp>

//diagnostic_msgs
#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/srv/self_test.hpp>
#include <diagnostic_msgs/srv/add_diagnostics.hpp>

//test_msgs
#include <test_msgs/msg/empty.hpp>
#include <test_msgs/msg/arrays.hpp>
#include <test_msgs/msg/nested.hpp>
#include <test_msgs/msg/strings.hpp>
#include <test_msgs/msg/builtins.hpp>
#include <test_msgs/msg/defaults.hpp>
#include <test_msgs/msg/constants.hpp>
#include <test_msgs/msg/w_strings.hpp>
#include <test_msgs/msg/basic_types.hpp>
#include <test_msgs/msg/multi_nested.hpp>
#include <test_msgs/msg/bounded_sequences.hpp>
#include <test_msgs/msg/unbounded_sequences.hpp>
#include <test_msgs/srv/empty.hpp>
#include <test_msgs/srv/arrays.hpp>
#include <test_msgs/srv/basic_types.hpp>
#include <test_msgs/action/fibonacci.hpp>
#include <test_msgs/action/nested_message.hpp>

//action_msgs
#include <action_msgs/msg/goal_info.hpp>
#include <action_msgs/msg/goal_status.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>

//actionlib_msgs
#include <actionlib_msgs/msg/goal_id.hpp>
#include <actionlib_msgs/msg/goal_status.hpp>
#include <actionlib_msgs/msg/goal_status_array.hpp>

//lifecycle_msgs
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/msg/transition_description.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_available_states.hpp>
#include <lifecycle_msgs/srv/get_available_transitions.hpp>

//rosgraph_msgs
#include <rosgraph_msgs/msg/clock.hpp>

//pendulum_msgs
#include <pendulum_msgs/msg/joint_state.hpp>
#include <pendulum_msgs/msg/joint_command.hpp>
#include <pendulum_msgs/msg/rttest_results.hpp>

//composition_interfaces
#include <composition_interfaces/srv/load_node.hpp>
#include <composition_interfaces/srv/list_nodes.hpp>
#include <composition_interfaces/srv/unload_node.hpp>

//unique_identifier_msgs
#include <unique_identifier_msgs/msg/uuid.hpp>

//tf2_msgs
#include <tf2_msgs/msg/tf2_error.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_msgs/srv/frame_graph.hpp>
#include <tf2_msgs/action/lookup_transform.hpp>

//tf2_sensor_msgs

//tf2_geometry_msgs

#endif
