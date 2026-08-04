// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>


#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;

    descr.description = description;
    descr.read_only = read_only;

    return descr;
}

const static std::unordered_map<std::string, rmw_qos_profile_t> qos_profiles{
    {"default", rmw_qos_profile_default},
    {"sensor_data", rmw_qos_profile_sensor_data},
    {"system_default", rmw_qos_profile_system_default},
};

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam{
#ifdef image_transport_NODE_INTERFACE
        image_transport::RequiredInterfaces{*this},
#else
        this,
#endif
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
#ifdef image_transport_QoS
        rclcpp::QoS{rclcpp::QoSInitialization::from_rmw(
#endif
            qos_profiles.at(declare_parameter("qos_profile", "default", descr("qos profile to use. 'default', 'sensor_data' or 'system_default'", true)))
#ifdef image_transport_QoS
                )}
#endif
    },
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    pub_debug_image(create_publisher<sensor_msgs::msg::Image>("debug_image", rclcpp::QoS(1))),
    tf_broadcaster(
#ifdef tf2_ros_NODE_INTERFACE
        tf2_ros::TransformBroadcaster::RequiredInterfaces { *this }
#else
        this
#endif
    )
{
    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    const std::string& pose_estimation_method =
        declare_parameter("pose_estimation_method", "pnp",
                          descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster), "
                                "set to \"\" (empty) to disable pose estimation",
                                true));

    if(!pose_estimation_method.empty()) {
        if(pose_estimation_methods.count(pose_estimation_method)) {
            estimate_pose = pose_estimation_methods.at(pose_estimation_method);
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown pose estimation method '" << pose_estimation_method << "'.");
        }
    }

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // camera intrinsics for rectified images
    const std::array<double, 4> intrinsics = {msg_ci->p[0], msg_ci->p[5], msg_ci->p[2], msg_ci->p[6]};

    // check for valid intrinsics
    const bool calibrated = msg_ci->width && msg_ci->height &&
                            intrinsics[0] && intrinsics[1] && intrinsics[2] && intrinsics[3];

    if(estimate_pose != nullptr && !calibrated) {
        RCLCPP_WARN_STREAM(get_logger(), "The camera is not calibrated! Set 'pose_estimation_method' to \"\" (empty) to disable pose estimation and this warning.");
    }

    // convert to BGR image to draw on it
    cv::Mat img_color;
    try {
        img_color = cv_bridge::toCvCopy(msg_img, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    zarray_t* detections = apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    for(int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        // Draw tag outlines
        for (int j = 0; j < 4; j++) {
            cv::line(img_color,
                     cv::Point(det->p[j][0], det->p[j][1]),
                     cv::Point(det->p[(j + 1) % 4][0], det->p[(j + 1) % 4][1]),
                     cv::Scalar(0, 0, 255), 2);
        }
        // Draw tag ID
        cv::putText(img_color, std::to_string(det->id),
                    cv::Point(det->c[0] - 10, det->c[1] + 10),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        // Draw 3D pose coordinate axes if camera is calibrated
        if (calibrated) {
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            const std::vector<cv::Point3d> objectPoints{
                {-size / 2, -size / 2, 0},
                {+size / 2, -size / 2, 0},
                {+size / 2, +size / 2, 0},
                {-size / 2, +size / 2, 0},
            };
            const std::vector<cv::Point2d> imagePoints{
                {det->p[0][0], det->p[0][1]},
                {det->p[1][0], det->p[1][1]},
                {det->p[2][0], det->p[2][1]},
                {det->p[3][0], det->p[3][1]},
            };
            cv::Mat rvec, tvec;
            cv::Matx33d cameraMatrix = cv::Matx33d::eye();
            cameraMatrix(0, 0) = intrinsics[0];// fx
            cameraMatrix(1, 1) = intrinsics[1];// fy
            cameraMatrix(0, 2) = intrinsics[2];// cx
            cameraMatrix(1, 2) = intrinsics[3];// cy

            cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);
            cv::drawFrameAxes(img_color, cameraMatrix, cv::Mat(), rvec, tvec, size * 1.5, 3);
        }

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
        apriltag_msgs::msg::AprilTagDetection msg_detection;
        msg_detection.family = std::string(det->family->name);
        msg_detection.id = det->id;
        msg_detection.hamming = det->hamming;
        msg_detection.decision_margin = det->decision_margin;
        msg_detection.centre.x = det->c[0];
        msg_detection.centre.y = det->c[1];
        std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
        std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
        msg_detections.detections.push_back(msg_detection);

        // 3D orientation and position
        if(estimate_pose != nullptr && calibrated) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            // set child frame name by generic tag name or configured tag name
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            tf.transform = estimate_pose(det, intrinsics, size);
            tfs.push_back(tf);
        }
    }

    pub_detections->publish(msg_detections);

    if(estimate_pose != nullptr)
        tf_broadcaster.sendTransform(tfs);

    pub_debug_image->publish(*cv_bridge::CvImage(msg_img->header, "bgr8", img_color).toImageMsg());

    apriltag_detections_destroy(detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;

    mutex.lock();

    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);

        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
    }

    mutex.unlock();

    result.successful = true;

    return result;
}
