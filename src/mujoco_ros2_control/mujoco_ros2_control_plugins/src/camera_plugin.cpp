/**
 * Copyright (c) 2026, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "camera_plugin.hpp"

#include <atomic>
#include <cerrno>
#include <cstring>
#include <rclcpp/expand_topic_or_service_name.hpp>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace mujoco_ros2_control_plugins
{

namespace
{
void copy_c_string(char* dst, std::size_t dst_len, const std::string& src)
{
  if (dst_len == 0)
  {
    return;
  }
  const std::size_t n = std::min(src.size(), dst_len - 1);
  std::memcpy(dst, src.data(), n);
  dst[n] = '\0';
}
}  // namespace

bool CameraPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data)
{
  // by default use `glfwInit` to check if the GLFW is initialized.
  return this->init(node, model, data, glfwInit);
}

bool CameraPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data, GlfwInitFn glfw_init_fn)
{
  node_ = node;
  mj_model_ = model;
  mj_data_ = data;

  // Ensure the logger has a name
  logger_ = node_->get_logger().get_child(node->get_sub_namespace());
  RCLCPP_INFO(node_->get_logger(), "CameraPlugin initializing cameras...");

  // Read the mj_model_, identify the number of cameras, and populate containers for them.
  if (!register_cameras())
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to register cameras.");
    return false;
  }
  if (cameras_.empty())
  {
    return true;
  }

  // Start the rendering thread process
  // Try GLFW first, fall back to EGL for headless environments
  if (glfw_init_fn())
  {
    use_egl_ = false;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Failed to initialize GLFW. Attempting EGL for headless rendering.");
    use_egl_ = true;
  }
  rendering_thread_ = std::thread(&CameraPlugin::update_loop, this);
  return true;
}

void CameraPlugin::update(const mjModel* model_arg, mjData* data)
{
  if (!publish_images_)
  {
    return;
  }

  // Streaming cameras render on a fixed-rate clock; polled cameras render as soon as a
  // trigger has been received. Both are serviced here, on the sim thread, because this is
  // the only place we can safely snapshot the live mjData without racing the simulation.
  // TODO: Support per-camera publish rates?
  // Schedule from the state being sampled, not the asynchronously delivered /clock.
  const rclcpp::Time now(rclcpp::Duration::from_seconds(data->time).nanoseconds(), RCL_ROS_TIME);
  const bool stream_due =
      has_streaming_cameras_ &&
      (now < last_publish_time_ || (now - last_publish_time_).seconds() >= (1.0 / camera_publish_rate_));
  const bool poll_due = poll_pending_.load();

  // Nothing to do this step: avoid taking the lock or copying data.
  if (!stream_due && !poll_due)
  {
    return;
  }

  bool any_selected = false;
  {
    // Rendering owns this snapshot until all cameras finish. Never make physics
    // wait for the GPU or image transport; retry with fresh state on a later step.
    std::unique_lock<std::mutex> lock(data_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
      return;
    }
    poll_pending_.exchange(false);

    // Flag streaming cameras when their interval is due and any polled cameras that have a
    // pending trigger (consuming the one-shot request so they render exactly once).
    for (auto& camera : cameras_)
    {
      if (camera.policy == CameraPolicy::STREAMING && stream_due)
      {
        camera.render_pending = true;
        any_selected = true;
      }
      else if (camera.policy == CameraPolicy::POLLED && camera.poll_requested)
      {
        camera.poll_requested = false;
        camera.render_pending = true;
        any_selected = true;
      }
    }

    if (stream_due)
    {
      last_publish_time_ = now;
    }

    // Only snapshot the simulation data when there is actually a camera to render.
    if (any_selected)
    {
      mjv_copyData(mj_camera_data_, model_arg, data);
      new_data_ = true;
    }
  }

  if (any_selected)
  {
    data_cv_.notify_one();
  }
}

void CameraPlugin::cleanup()
{
  close();
}

void CameraPlugin::trigger_update()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Force every streaming camera and consume any pending polls, then render synchronously.
  for (auto& camera : cameras_)
  {
    if (camera.policy == CameraPolicy::STREAMING)
    {
      camera.render_pending = true;
    }
    else if (camera.policy == CameraPolicy::POLLED && camera.poll_requested)
    {
      camera.poll_requested = false;
      camera.render_pending = true;
    }
  }
  update_cameras();
}

bool CameraPlugin::register_cameras()
{
  const std::string param_prefix = "mujoco_plugins." + node_->get_sub_namespace() + ".";
  const std::string camera_publish_rate_param = param_prefix + "camera_publish_rate";
  if (!node_->has_parameter(camera_publish_rate_param))
  {
    node_->declare_parameter(camera_publish_rate_param, 5.0);
  }

  camera_publish_rate_ = node_->get_parameter(camera_publish_rate_param).as_double();
  RCLCPP_INFO(node_->get_logger(), "Publishing camera data at rate %f per second.", camera_publish_rate_);

  const std::string output_param = param_prefix + "output";
  if (!node_->has_parameter(output_param))
  {
    node_->declare_parameter(output_param, "ros");
  }
  const std::string output_str = node_->get_parameter(output_param).as_string();
  if (output_str == "ros")
  {
    output_mode_ = CameraOutput::ROS;
  }
  else if (output_str == "shm")
  {
    output_mode_ = CameraOutput::SHM;
  }
  else if (output_str == "both")
  {
    output_mode_ = CameraOutput::BOTH;
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid camera output mode '%s' (expected ros|shm|both)", output_str.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Camera output mode: %s", output_str.c_str());

  if (!node_->has_parameter(param_prefix + "shm_prefix"))
  {
    node_->declare_parameter(param_prefix + "shm_prefix", "/pai_mj_cam_");
  }
  shm_prefix_ = node_->get_parameter(param_prefix + "shm_prefix").as_string();
  if (shm_prefix_.size() < 2 || shm_prefix_[0] != '/' || shm_prefix_.find('/', 1) != std::string::npos)
  {
    RCLCPP_ERROR(node_->get_logger(), "shm_prefix must start with / and contain no other slash");
    return false;
  }
  if (!node_->has_parameter(param_prefix + "default_policy"))
  {
    node_->declare_parameter(param_prefix + "default_policy", "streaming");
  }
  const auto default_policy = node_->get_parameter(param_prefix + "default_policy").as_string();
  cameras_.resize(0);
  for (auto i = 0; i < mj_model_->ncam; ++i)
  {
    const char* cam_name = mj_model_->names + mj_model_->name_camadr[i];
    const int* cam_resolution = mj_model_->cam_resolution + 2 * i;
    const mjtNum cam_fovy = mj_model_->cam_fovy[i];

    // Construct CameraData wrapper and set defaults
    CameraData camera;
    camera.name = cam_name;
    camera.mjv_cam.type = mjCAMERA_FIXED;
    camera.mjv_cam.fixedcamid = i;
    camera.width = static_cast<uint32_t>(cam_resolution[0]);
    camera.height = static_cast<uint32_t>(cam_resolution[1]);
    camera.viewport = { 0, 0, cam_resolution[0], cam_resolution[1] };

    const std::string param_ns = param_prefix + cam_name + ".";

    const std::string policy_param = param_ns + "policy";
    if (!node_->has_parameter(policy_param))
    {
      node_->declare_parameter(policy_param, default_policy);
    }
    std::string policy_str = node_->get_parameter(policy_param).as_string();
    if (policy_str == "streaming")
    {
      camera.policy = CameraPolicy::STREAMING;
    }
    else if (policy_str == "polled")
    {
      camera.policy = CameraPolicy::POLLED;
    }
    else if (policy_str == "disabled")
    {
      camera.policy = CameraPolicy::DISABLED;
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid policy for camera '%s'", cam_name);
      return false;
    }

    const std::string frame_param = param_ns + "frame_name";
    if (!node_->has_parameter(frame_param))
    {
      node_->declare_parameter(frame_param, camera.name + "_frame");
    }
    camera.frame_name = node_->get_parameter(frame_param).as_string();

    if (!node_->has_parameter(param_ns + "info_topic"))
    {
      node_->declare_parameter(param_ns + "info_topic", camera.name + "/camera_info");
    }
    camera.info_topic = node_->get_parameter(param_ns + "info_topic").as_string();

    if (!node_->has_parameter(param_ns + "image_topic"))
    {
      node_->declare_parameter(param_ns + "image_topic", camera.name + "/color");
    }
    camera.image_topic = node_->get_parameter(param_ns + "image_topic").as_string();

    if (!node_->has_parameter(param_ns + "depth_topic"))
    {
      node_->declare_parameter(param_ns + "depth_topic", camera.name + "/depth");
    }
    camera.depth_topic = node_->get_parameter(param_ns + "depth_topic").as_string();

    if (!node_->has_parameter(param_ns + "trigger_service_name"))
    {
      node_->declare_parameter(param_ns + "trigger_service_name", camera.name + "/trigger");
    }
    camera.trigger_service_name = node_->get_parameter(param_ns + "trigger_service_name").as_string();

    RCLCPP_INFO(node_->get_logger(), "Adding camera: '%s'", cam_name);
    RCLCPP_INFO(node_->get_logger(), "    policy: '%s'", policy_str.c_str());
    RCLCPP_INFO(node_->get_logger(), "    frame_name: '%s'", camera.frame_name.c_str());
    if (camera.policy == CameraPolicy::DISABLED)
    {
      continue;
    }
    RCLCPP_INFO(node_->get_logger(), "    info_topic: '%s'", camera.info_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "    image_topic: '%s'", camera.image_topic.c_str());
    RCLCPP_INFO(node_->get_logger(), "    depth_topic: '%s'", camera.depth_topic.c_str());
    if (camera.policy == CameraPolicy::POLLED)
    {
      RCLCPP_INFO(node_->get_logger(), "    trigger_service_name: '%s'", camera.trigger_service_name.c_str());
    }

    // Configure publishers and services. SHM-only mode skips Image publishers so ROS
    // fan-out happens in mujoco_image_bridge instead of ros2_control_node.
    const bool publish_ros = (output_mode_ == CameraOutput::ROS || output_mode_ == CameraOutput::BOTH);
    const bool publish_shm = (output_mode_ == CameraOutput::SHM || output_mode_ == CameraOutput::BOTH);
    if (publish_ros)
    {
      camera.camera_info_pub = node_->create_publisher<sensor_msgs::msg::CameraInfo>(camera.info_topic, 1);
      camera.image_pub = node_->create_publisher<sensor_msgs::msg::Image>(camera.image_topic, 1);
      camera.depth_image_pub = node_->create_publisher<sensor_msgs::msg::Image>(camera.depth_topic, 1);
    }
    if (camera.policy == CameraPolicy::POLLED)
    {
      camera.trigger_service = node_->create_service<std_srvs::srv::Trigger>(
          camera.trigger_service_name, [this, camera_index = cameras_.size()](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            this->handle_trigger(request, response, static_cast<int>(camera_index));
          });
    }
    // Setup containers for color image data
    camera.image.header.frame_id = camera.frame_name;

    const auto image_size = camera.width * camera.height * 3;
    camera.image_buffer.resize(image_size);
    camera.image.data.resize(image_size);
    camera.image.width = camera.width;
    camera.image.height = camera.height;
    camera.image.step = camera.width * 3;
    camera.image.encoding = sensor_msgs::image_encodings::RGB8;

    // Depth image data
    camera.depth_image.header.frame_id = camera.frame_name;
    camera.depth_buffer.resize(camera.width * camera.height);
    camera.depth_image.data.resize(camera.width * camera.height * sizeof(float));
    camera.depth_image.width = camera.width;
    camera.depth_image.height = camera.height;
    camera.depth_image.step = camera.width * sizeof(float);
    camera.depth_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    // Camera info
    camera.camera_info.header.frame_id = camera.frame_name;
    camera.camera_info.width = camera.width;
    camera.camera_info.height = camera.height;
    camera.camera_info.distortion_model = "plumb_bob";
    camera.camera_info.k.fill(0.0);
    camera.camera_info.r.fill(0.0);
    camera.camera_info.p.fill(0.0);
    camera.camera_info.d.resize(5, 0.0);

    double focal_scaling = (1.0 / std::tan((cam_fovy * M_PI / 180.0) / 2.0)) * camera.height / 2.0;
    camera.camera_info.k[0] = camera.camera_info.p[0] = focal_scaling;
    camera.camera_info.k[2] = camera.camera_info.p[2] = static_cast<double>(camera.width) / 2.0;
    camera.camera_info.k[4] = camera.camera_info.p[5] = focal_scaling;
    camera.camera_info.k[5] = camera.camera_info.p[6] = static_cast<double>(camera.height) / 2.0;
    camera.camera_info.k[8] = camera.camera_info.p[10] = 1.0;

    if (publish_shm)
    {
      camera.shm_name = shm_prefix_ + camera.name;
      if (!init_camera_shm(camera))
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to create camera SHM '%s'", camera.shm_name.c_str());
        return false;
      }
      RCLCPP_INFO(node_->get_logger(), "    shm: '%s'", camera.shm_name.c_str());
    }

    // Add to list of cameras
    cameras_.push_back(camera);
  }

  has_streaming_cameras_ = std::any_of(cameras_.begin(), cameras_.end(),
                                       [](const CameraData& cam) { return cam.policy == CameraPolicy::STREAMING; });

  // Reserve once so the per-pass render bookkeeping in `update_cameras()` never allocates.
  render_indices_.reserve(cameras_.size());

  return true;
}

void CameraPlugin::close()
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    stop_requested_ = true;
    publish_images_ = false;
  }
  data_cv_.notify_one();
  if (rendering_thread_.joinable())
  {
    rendering_thread_.join();
  }
  for (auto& camera : cameras_)
  {
    close_camera_shm(camera);
  }
}

bool CameraPlugin::init_egl_context()
{
  // Get EGL display
  egl_display_ = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, NULL);

  // Initialize EGL
  EGLint major, minor;
  if (!eglInitialize(egl_display_, &major, &minor))
  {
    RCLCPP_ERROR(node_->get_logger(), "EGL: Failed to initialize (error: 0x%x)", eglGetError());
    return false;
  }
  const char* client_apis = eglQueryString(egl_display_, EGL_CLIENT_APIS);
  const char* vendor = eglQueryString(egl_display_, EGL_VENDOR);
  RCLCPP_INFO(node_->get_logger(), "EGL: Initialized version %d.%d", major, minor);
  RCLCPP_INFO(node_->get_logger(), "EGL: Vendor: %s, APIs: %s", vendor, client_apis);
  // Choose EGL config for offscreen rendering
  const EGLint config_attribs[] = { EGL_SURFACE_TYPE,
                                    EGL_PBUFFER_BIT,
                                    EGL_RED_SIZE,
                                    8,
                                    EGL_GREEN_SIZE,
                                    8,
                                    EGL_BLUE_SIZE,
                                    8,
                                    EGL_ALPHA_SIZE,
                                    8,
                                    EGL_DEPTH_SIZE,
                                    24,
                                    EGL_RENDERABLE_TYPE,
                                    EGL_OPENGL_BIT,
                                    EGL_NONE };

  EGLConfig egl_config;
  EGLint num_configs;
  if (!eglChooseConfig(egl_display_, config_attribs, &egl_config, 1, &num_configs) || num_configs == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "EGL: Failed to choose config (error: 0x%x)", eglGetError());
    eglTerminate(egl_display_);
    egl_display_ = EGL_NO_DISPLAY;
    return false;
  }

  // Bind OpenGL API
  if (!eglBindAPI(EGL_OPENGL_API))
  {
    RCLCPP_ERROR(node_->get_logger(), "EGL: Failed to bind OpenGL API (error: 0x%x)", eglGetError());
    eglTerminate(egl_display_);
    egl_display_ = EGL_NO_DISPLAY;
    return false;
  }

  // Create EGL context
  egl_context_ = eglCreateContext(egl_display_, egl_config, EGL_NO_CONTEXT, nullptr);
  if (egl_context_ == EGL_NO_CONTEXT)
  {
    RCLCPP_ERROR(node_->get_logger(), "EGL: Failed to create context (error: 0x%x)", eglGetError());
    eglTerminate(egl_display_);
    egl_display_ = EGL_NO_DISPLAY;
    return false;
  }

  // Create PBuffer surface for offscreen rendering
  const EGLint pbuffer_attribs[] = { EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE };
  egl_surface_ = eglCreatePbufferSurface(egl_display_, egl_config, pbuffer_attribs);
  if (egl_surface_ == EGL_NO_SURFACE)
  {
    RCLCPP_ERROR(node_->get_logger(), "EGL: Failed to create PBuffer surface (error: 0x%x)", eglGetError());
    eglDestroyContext(egl_display_, egl_context_);
    eglTerminate(egl_display_);
    egl_context_ = EGL_NO_CONTEXT;
    egl_display_ = EGL_NO_DISPLAY;
    return false;
  }

  // Make the context current
  if (!eglMakeCurrent(egl_display_, egl_surface_, egl_surface_, egl_context_))
  {
    RCLCPP_ERROR(node_->get_logger(), "EGL: Failed to make context current (error: 0x%x)", eglGetError());
    cleanup_egl_context();
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "EGL: Successfully initialized headless OpenGL context");
  return true;
}

void CameraPlugin::cleanup_egl_context()
{
  if (egl_display_ != EGL_NO_DISPLAY)
  {
    eglMakeCurrent(egl_display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (egl_surface_ != EGL_NO_SURFACE)
    {
      eglDestroySurface(egl_display_, egl_surface_);
      egl_surface_ = EGL_NO_SURFACE;
    }
    if (egl_context_ != EGL_NO_CONTEXT)
    {
      eglDestroyContext(egl_display_, egl_context_);
      egl_context_ = EGL_NO_CONTEXT;
    }
    eglTerminate(egl_display_);
    egl_display_ = EGL_NO_DISPLAY;
  }
}

void CameraPlugin::update_loop()
{
  GLFWwindow* window = nullptr;

  if (use_egl_)
  {
    // Initialize EGL for headless rendering
    if (!init_egl_context())
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to initialize EGL context. Disabling camera publishing.");
      publish_images_ = false;
      return;
    }
  }
  else
  {
    // Use GLFW for offscreen context (display available)
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window = glfwCreateWindow(1, 1, "", NULL, NULL);
    if (!window)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to create GLFW window. Disabling camera publishing.");
      publish_images_ = false;
      return;
    }
    glfwMakeContextCurrent(window);
  }

  // Determine rendering backend name for logging
  const char* backend = use_egl_ ? "EGL" : "GLFW";

  // Initialization of the context and data structures has to happen in the rendering thread
  RCLCPP_INFO(node_->get_logger(), "Initializing rendering for cameras (using %s)", backend);
  mjv_defaultOption(&mjv_opt_);
  mjv_defaultScene(&mjv_scn_);
  mjr_defaultContext(&mjr_con_);

  // Turn rangefinder rendering off so we don't get rays in camera images
  mjv_opt_.flags[mjtVisFlag::mjVIS_RANGEFINDER] = 0;
  // Turn off site rendering so that visualization is more realistic in cameras for testing perception.
  for (int i = 0; i < mjNGROUP; i++)
  {
    mjv_opt_.sitegroup[i] = 0;
  }

  // Initialize data for camera rendering
  mj_camera_data_ = mj_makeData(mj_model_);
  RCLCPP_INFO(node_->get_logger(), "Starting the camera rendering loop, publishing at %f Hz", camera_publish_rate_);

  // create scene and context
  mjv_makeScene(mj_model_, &mjv_scn_, 2000);

  mjr_makeContext(mj_model_, &mjr_con_, mjFONTSCALE_150);

  // Ensure the context will support the largest cameras
  int max_width = 1, max_height = 1;
  for (const auto& cam : cameras_)
  {
    max_width = std::max(max_width, static_cast<int>(cam.width));
    max_height = std::max(max_height, static_cast<int>(cam.height));
  }
  mjr_resizeOffscreen(max_width, max_height, &mjr_con_);
  RCLCPP_INFO(node_->get_logger(), "Resized offscreen buffer to %d x %d", max_width, max_height);

  // Only process images once all data has been initialized, and do it until told to stop.
  // Publishing is enabled under the lock so that a shutdown requested during the (possibly
  // slow) initialization above is observed here rather than being clobbered.
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    publish_images_ = !stop_requested_;
  }
  while (rclcpp::ok() && !stop_requested_)
  {
    std::unique_lock<std::mutex> lock(data_mutex_);

    // Wait for the main thread to copy the data and trigger this to process and publish
    // images. Note that condition_variables can be awoken spuriously, so the additional
    // checks are necessary to avoid doing work excepting when updated rendering data has
    // been made available.
    data_cv_.wait(lock, [this] { return new_data_ || stop_requested_; });

    // Shutdown triggered, kill the loop and clean up.
    if (stop_requested_)
    {
      break;
    }
    new_data_ = false;
    // Keep the snapshot and render flags immutable throughout this pass.
    // The physics-side producer uses try_lock and skips a busy renderer.
    update_cameras();
  }
  publish_images_ = false;

  mjv_freeScene(&mjv_scn_);
  mjr_freeContext(&mjr_con_);
  mj_deleteData(mj_camera_data_);

  if (use_egl_)
  {
    cleanup_egl_context();
  }
  else if (window)
  {
    glfwDestroyWindow(window);
  }
}

void CameraPlugin::update_cameras()
{
  // Gather the cameras flagged for rendering this pass, clearing the flag as we go.
  render_indices_.clear();
  for (size_t i = 0; i < cameras_.size(); ++i)
  {
    auto& camera = cameras_[i];
    if (camera.render_pending)
    {
      camera.render_pending = false;
      render_indices_.push_back(i);
    }
  }
  if (render_indices_.empty())
  {
    return;
  }

  // Rendering is done offscreen using the data snapshot taken in `update`.
  mjr_setBuffer(mjFB_OFFSCREEN, &mjr_con_);

  camera_near_distance_ = static_cast<float>(mj_model_->vis.map.znear * mj_model_->stat.extent);
  const float far = static_cast<float>(mj_model_->vis.map.zfar * mj_model_->stat.extent);
  camera_depth_scale_ = 1.0f - camera_near_distance_ / far;

  // Use the snapshotted data's timestamp in the ROS header, as that is when the data was actually pulled.
  const rclcpp::Duration duration = rclcpp::Duration::from_seconds(mj_camera_data_->time);
  rclcpp::Time stamp(duration.nanoseconds(), RCL_ROS_TIME);

  for (const auto idx : render_indices_)
  {
    render_and_publish_camera(cameras_[idx], stamp);
  }
}

void CameraPlugin::render_and_publish_camera(CameraData& camera, const rclcpp::Time& stamp)
{
  // Step 1: Render the scene and copy images to relevant camera data containers.
  // Render scene
  mjv_updateScene(mj_model_, mj_camera_data_, &mjv_opt_, NULL, &camera.mjv_cam, mjCAT_ALL, &mjv_scn_);
  mjr_render(camera.viewport, &mjv_scn_, &mjr_con_);

  // Copy image into relevant buffers
  mjr_readPixels(camera.image_buffer.data(), camera.depth_buffer.data(), camera.viewport, &mjr_con_);

  // Step 2: Adjust the images and copy depth data.
  // Fix non-linear depth buffer and flip it vertically (OpenGL's origin is the bottom left)
  // https://github.com/google-deepmind/mujoco/blob/3.4.0/python/mujoco/renderer.py#L190
  auto* depth_out = reinterpret_cast<float*>(camera.depth_image.data.data());
  for (uint32_t h = 0; h < camera.height; ++h)
  {
    const float* src_row = camera.depth_buffer.data() + static_cast<size_t>(h) * camera.width;
    float* dst_row = depth_out + static_cast<size_t>(camera.height - 1 - h) * camera.width;
    for (uint32_t w = 0; w < camera.width; ++w)
    {
      dst_row[w] = camera_near_distance_ / (1.0f - src_row[w] * camera_depth_scale_);
    }
  }

  // OpenGL's coordinate system's origin is in the bottom left, so we invert the images row-by-row
  const auto row_size = camera.width * 3;
  for (uint32_t h = 0; h < camera.height; ++h)
  {
    const auto src_idx = h * row_size;
    const auto dest_idx = (camera.height - 1 - h) * row_size;
    std::memcpy(&camera.image.data[dest_idx], &camera.image_buffer[src_idx], row_size);
  }

  // Step 3: Deliver frames via ROS publishers and/or SHM for an out-of-process bridge.
  camera.image.header.stamp = stamp;
  camera.depth_image.header.stamp = stamp;
  camera.camera_info.header.stamp = stamp;

  if (output_mode_ == CameraOutput::ROS || output_mode_ == CameraOutput::BOTH)
  {
    camera.image_pub->publish(camera.image);
    camera.depth_image_pub->publish(camera.depth_image);
    camera.camera_info_pub->publish(camera.camera_info);
  }
  if (output_mode_ == CameraOutput::SHM || output_mode_ == CameraOutput::BOTH)
  {
    write_camera_shm(camera, stamp);
  }
}

bool CameraPlugin::init_camera_shm(CameraData& camera)
{
  const auto expand = [this](const std::string& topic) {
    return rclcpp::expand_topic_or_service_name(topic, node_->get_name(), node_->get_effective_namespace(), false);
  };
  const auto image_topic = expand(camera.image_topic);
  const auto depth_topic = expand(camera.depth_topic);
  const auto info_topic = expand(camera.info_topic);
  if (camera.frame_name.size() >= kCameraShmFrameIdLen || image_topic.size() >= kCameraShmTopicLen ||
      depth_topic.size() >= kCameraShmTopicLen || info_topic.size() >= kCameraShmTopicLen)
  {
    RCLCPP_ERROR(node_->get_logger(), "Camera '%s' metadata exceeds SHM layout limits", camera.name.c_str());
    return false;
  }
  const uint32_t color_bytes = camera.width * camera.height * 3u;
  const uint32_t depth_bytes = camera.width * camera.height * static_cast<uint32_t>(sizeof(float));
  const std::size_t total = camera_shm_total_bytes(color_bytes, depth_bytes);

  // Recreate so a stale segment from a previous run cannot keep an old layout.
  shm_unlink(camera.shm_name.c_str());
  camera.shm_fd = shm_open(camera.shm_name.c_str(), O_CREAT | O_EXCL | O_RDWR | O_CLOEXEC, 0660);
  if (camera.shm_fd < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "shm_open(%s) failed: %s", camera.shm_name.c_str(), std::strerror(errno));
    return false;
  }
  if (ftruncate(camera.shm_fd, static_cast<off_t>(total)) != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "ftruncate(%s) failed: %s", camera.shm_name.c_str(), std::strerror(errno));
    ::close(camera.shm_fd);
    camera.shm_fd = -1;
    shm_unlink(camera.shm_name.c_str());
    return false;
  }
  camera.shm_map = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, camera.shm_fd, 0);
  if (camera.shm_map == MAP_FAILED)
  {
    RCLCPP_ERROR(node_->get_logger(), "mmap(%s) failed: %s", camera.shm_name.c_str(), std::strerror(errno));
    ::close(camera.shm_fd);
    camera.shm_fd = -1;
    camera.shm_map = nullptr;
    shm_unlink(camera.shm_name.c_str());
    return false;
  }
  camera.shm_map_size = total;
  std::memset(camera.shm_map, 0, total);

  camera.shm_static = reinterpret_cast<CameraShmStatic*>(camera.shm_map);
  // Publish magic last so readers never attach to partially initialized metadata.
  camera.shm_static->version = kCameraShmVersion;
  camera.shm_static->width = camera.width;
  camera.shm_static->height = camera.height;
  camera.shm_static->color_bytes = color_bytes;
  camera.shm_static->depth_bytes = depth_bytes;
  camera.shm_static->num_slots = kCameraShmNumSlots;
  camera.shm_static->slot_bytes = static_cast<uint32_t>(camera_shm_slot_bytes(color_bytes, depth_bytes));
  copy_c_string(camera.shm_static->frame_id, sizeof(camera.shm_static->frame_id), camera.frame_name);
  copy_c_string(camera.shm_static->image_topic, sizeof(camera.shm_static->image_topic), image_topic);
  copy_c_string(camera.shm_static->depth_topic, sizeof(camera.shm_static->depth_topic), depth_topic);
  copy_c_string(camera.shm_static->info_topic, sizeof(camera.shm_static->info_topic), info_topic);
  copy_c_string(camera.shm_static->distortion_model, sizeof(camera.shm_static->distortion_model),
                camera.camera_info.distortion_model);
  for (std::size_t i = 0; i < 9; ++i)
  {
    camera.shm_static->k[i] = camera.camera_info.k[i];
  }
  for (std::size_t i = 0; i < 12; ++i)
  {
    camera.shm_static->p[i] = camera.camera_info.p[i];
  }
  for (std::size_t i = 0; i < 5; ++i)
  {
    camera.shm_static->d[i] = camera.camera_info.d[i];
  }
  camera.shm_sequence = 0;
  camera_shm_set_magic(camera.shm_map, kCameraShmMagic);
  return true;
}

void CameraPlugin::close_camera_shm(CameraData& camera)
{
  if (camera.shm_map != nullptr && camera.shm_map != MAP_FAILED)
  {
    camera_shm_set_magic(camera.shm_map, 0);
    munmap(camera.shm_map, camera.shm_map_size);
  }
  camera.shm_map = nullptr;
  camera.shm_static = nullptr;
  camera.shm_map_size = 0;
  if (camera.shm_fd >= 0)
  {
    struct stat owned{}, current{};
    const auto path = "/dev/shm" + camera.shm_name;
    if (fstat(camera.shm_fd, &owned) == 0 && stat(path.c_str(), &current) == 0 &&
        owned.st_dev == current.st_dev && owned.st_ino == current.st_ino)
    {
      shm_unlink(camera.shm_name.c_str());
    }
    ::close(camera.shm_fd);
    camera.shm_fd = -1;
  }
}

void CameraPlugin::write_camera_shm(CameraData& camera, const rclcpp::Time& stamp)
{
  if (camera.shm_static == nullptr || camera.shm_map == nullptr)
  {
    return;
  }

  const int64_t total_ns = stamp.nanoseconds();
  camera_shm_write_frame(camera.shm_map, camera.shm_sequence,
      static_cast<int32_t>(total_ns / 1000000000LL), static_cast<uint32_t>(total_ns % 1000000000LL),
      camera.image.data.data(), camera.depth_image.data.data());
}

void CameraPlugin::handle_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response, const int camera_idx)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cameras_.at(camera_idx).poll_requested = true;
  }
  // Wake the fast path in `update()` so the request is picked up on the next sim step.
  poll_pending_.store(true);
  response->success = true;
}

}  // namespace mujoco_ros2_control_plugins

// Export the plugin
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::CameraPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
