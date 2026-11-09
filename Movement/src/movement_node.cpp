#include "movement/movement_node.hpp"

#include <utility>

namespace movement {

MovementNode::MovementNode(std::unique_ptr<VehicleInterface> vehicle,
                           const rclcpp::NodeOptions& options)
    : Node("movement", options), vehicle_(std::move(vehicle)) {
    const auto detection_topic = declare_parameter("detection_topic",
                                                   std::string("/perception/detection"));
    const auto camera_info_topic = declare_parameter("camera_info_topic",
                                                     std::string("/camera/front/camera_info"));

    FollowParams requested;
    requested.safe_distance_m = declare_parameter("safe_distance_m", kDefaultDistanceM);
    requested.min_distance_m = declare_parameter("min_distance_m", kMinDistanceM);
    requested.max_distance_m = declare_parameter("max_distance_m", kMaxDistanceM);
    requested.max_forward_m_s = declare_parameter("max_forward_m_s", kMaxForwardSpeedMps);
    requested.max_strafe_m_s = declare_parameter("max_strafe_m_s", kMaxStrafeSpeedMps);
    requested.max_yaw_rate_deg_s = declare_parameter("max_yaw_rate_deg_s", kMaxYawRateDegS);
    requested.search_yaw_rate_deg_s =
        declare_parameter("search_yaw_rate_deg_s", kDefaultSearchYawRateDegS);
    requested.search_forward_m_s =
        declare_parameter("search_forward_m_s", kDefaultSearchForwardMps);
    requested.search_strafe_m_s = declare_parameter("search_strafe_m_s", kDefaultSearchStrafeMps);
    requested.search_orbit_hz = declare_parameter("search_orbit_hz", kDefaultSearchOrbitHz);
    requested.forward_gain = declare_parameter("forward_gain", kForwardGain);
    requested.lateral_gain = declare_parameter("lateral_gain", kLateralGain);
    requested.yaw_gain = declare_parameter("yaw_gain", kYawGain);
    requested.orbit_yaw_ki = declare_parameter("orbit_yaw_ki", kOrbitYawIntegralGain);
    requested.last_known_search_s =
        declare_parameter("last_known_search_s", kDefaultLastKnownSearchS);
    requested.last_known_sweep_deg =
        declare_parameter("last_known_sweep_deg", kDefaultLastKnownSweepDeg);
    requested.last_known_rate_deg_s =
        declare_parameter("last_known_rate_deg_s", kDefaultLastKnownRateDegS);
    requested.max_forward_accel_m_s2 =
        declare_parameter("max_forward_accel_m_s2", kDefaultAccelLimit);
    requested.max_strafe_accel_m_s2 =
        declare_parameter("max_strafe_accel_m_s2", kDefaultAccelLimit);
    requested.max_yaw_accel_deg_s2 = declare_parameter("max_yaw_accel_deg_s2", kDefaultAccelLimit);
    params_ = sanitize(requested);

    OrbitParams orbit;
    orbit.enabled = declare_parameter("orbit_enabled", false);
    orbit.clockwise = declare_parameter("orbit_clockwise", false);
    orbit.period_s = declare_parameter("orbit_period_s", kDefaultOrbitPeriodS);
    orbit.max_speed_m_s = declare_parameter("max_orbit_speed_m_s", kDefaultOrbitSpeedMps);
    orbit_ = sanitize(orbit);

    target_timeout_ms_ = static_cast<int>(declare_parameter("target_timeout_ms",
                                                            static_cast<int64_t>(kDefaultTargetTimeoutMs)));
    person_height_m_ = declare_parameter("person_height_m", kDefaultTargetPersonHeightM);
    auto_arm_takeoff_ = declare_parameter("auto_arm_takeoff", false);
    search_timeout_s_ = declare_parameter("search_timeout_s", 0.0);

    detection_sub_ = create_subscription<perception_msgs::msg::Detection>(
        detection_topic, rclcpp::QoS(10),
        [this](perception_msgs::msg::Detection::SharedPtr msg) { onDetection(*msg, now()); });
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) { onCameraInfo(*msg); });

    control_timer_ = create_wall_timer(kLoopPeriod, [this]() { tick(now()); });

    RCLCPP_INFO(get_logger(),
                "movement: follow %.1f m (%.1f-%.1f), max fwd %.2f / strafe %.2f m/s, yaw %.0f "
                "deg/s, search %.2f m/s %.0f deg/s %.2f Hz (timeout %.0f s), target timeout "
                "%d ms, auto_arm_takeoff=%s, detection=%s, orbit=%s (%s, %.0f s/lap, max %.2f m/s)",
                params_.safe_distance_m, params_.min_distance_m, params_.max_distance_m,
                params_.max_forward_m_s, params_.max_strafe_m_s, params_.max_yaw_rate_deg_s,
                params_.search_forward_m_s, params_.search_yaw_rate_deg_s,
                params_.search_orbit_hz, search_timeout_s_, target_timeout_ms_,
                auto_arm_takeoff_ ? "true" : "false", detection_topic.c_str(),
                orbit_.enabled ? "on" : "off", orbit_.clockwise ? "clockwise" : "counter-clockwise",
                orbit_.period_s, orbit_.max_speed_m_s);
}

MovementNode::~MovementNode() { shutdown(); }

void MovementNode::onDetection(const perception_msgs::msg::Detection& msg,
                               const rclcpp::Time& received) {
    const double x = boxCenterX(msg.x_min, msg.x_max);
    if (!isFinite(x) || !isFinite(msg.y_min) || !isFinite(msg.y_max)) {
        return;  // a broken box is not a sighting; the target goes stale and we scan
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_target_ = {x, msg.y_min, msg.y_max};
    last_detection_time_ = received;
    have_position_ = true;
}

void MovementNode::onCameraInfo(const sensor_msgs::msg::CameraInfo& msg) {
    CameraGeometry c;
    c.center_x_px = msg.k[2];
    c.width_px = msg.width;
    c.height_px = msg.height;
    c.focal_x_px = msg.k[0];
    c.focal_y_px = msg.k[4];
    std::lock_guard<std::mutex> lock(state_mutex_);
    camera_ = c;
    have_camera_info_ = isUsable(c);
}

void MovementNode::armAndTakeoffIfNeeded(const VehicleState& state) {
    if (!state.armed) {
        const Status arm = vehicle_->arm();
        if (!arm.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Arm command failed: %s",
                                 arm.what.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Vehicle armed.");
    }
    if (!state.in_air) {
        const Status takeoff = vehicle_->takeoff();
        if (!takeoff.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Takeoff command failed: %s",
                                 takeoff.what.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Vehicle taking off.");
    }
}

void MovementNode::tick(const rclcpp::Time& now) {
    const std::optional<VehicleState> state = vehicle_->state();
    if (!state) {
        // Link lost: nothing can be sent. Forget offboard so it is restarted on reconnect.
        offboard_started_ = false;
        search_active_ = false;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for the vehicle link.");
        return;
    }

    if (!state->armed || !state->in_air) {
        // The original never reset this, so offboard was never restarted after a landing.
        offboard_started_ = false;
        search_active_ = false;
        orbit_state_ = OrbitState{};
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for an armed, airborne vehicle (armed=%d, in_air=%d).",
                             static_cast<int>(state->armed), static_cast<int>(state->in_air));
        if (auto_arm_takeoff_) {
            armAndTakeoffIfNeeded(*state);
        }
        return;
    }

    CameraGeometry camera;
    Target target;
    bool have_position = false;
    bool have_camera = false;
    rclcpp::Time detected_at(0, 0, RCL_ROS_TIME);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        have_position = have_position_;
        target = latest_target_;
        have_camera = have_camera_info_;
        camera = camera_;
        detected_at = last_detection_time_;
    }
    const bool target_is_fresh =
        have_position && (now - detected_at).nanoseconds() <= static_cast<int64_t>(target_timeout_ms_) * 1000000;

    if (target_is_fresh) {
        search_active_ = false;
        search_gave_up_ = false;
    } else {
        orbit_state_ = OrbitState{};  // the yaw integral must not carry over a lost target
    }
    if (search_gave_up_) {
        return;  // in Hold until the target is seen again
    }

    if (!offboard_started_) {
        const Status start = vehicle_->startOffboard();
        if (!start.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", start.what.c_str());
            return;
        }
        offboard_started_ = true;
        limiter_.reset();  // start from rest, as PX4's zero setpoint did
        RCLCPP_INFO(get_logger(), "Offboard mode active; following target.");
    }

    VelocityCommand cmd;
    if (target_is_fresh && have_camera) {
        // A box cut off by the frame edge gives a wildly long range, so it counts as no range.
        const std::optional<double> distance =
            boxTouchesFrameEdge(target.y_min_px, target.y_max_px, camera)
                ? std::nullopt
                : rangeFromBox(target.y_min_px, target.y_max_px, camera.focal_y_px,
                               person_height_m_);
        cmd = orbit_.enabled
                  ? computeOrbitCommand(target.x_px, distance, camera, params_, orbit_,
                                        orbit_state_,
                                        std::chrono::duration<double>(kLoopPeriod).count())
                  : computeFollowCommand(target.x_px, distance, camera, params_);
        last_offset_ = normalizedOffset(target.x_px, camera);
        if (std::abs(cmd.yawspeed_deg_s) > 1e-6) {
            last_turn_sign_ = cmd.yawspeed_deg_s > 0.0 ? 1.0 : -1.0;
        }
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "%s target x=%.1f px, distance=%.2f m, forward=%.2f m/s, "
                             "strafe=%.2f m/s, yaw=%.1f deg/s",
                             orbit_.enabled ? "orbit" : "follow", target.x_px,
                             distance.value_or(0.0), cmd.forward_m_s, cmd.right_m_s,
                             cmd.yawspeed_deg_s);
    } else if (target_is_fresh) {
        // The original fell into the search spiral here although the target was in view.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Target in view but no camera_info yet; holding position.");
    } else {
        if (!search_active_) {
            search_active_ = true;
            search_start_ = now;
        }
        const double elapsed_s = (now - search_start_).seconds();
        if (search_timeout_s_ > 0.0 && elapsed_s > search_timeout_s_) {
            RCLCPP_WARN(get_logger(), "Target not found in %.0f s; handing back to Hold.",
                        search_timeout_s_);
            search_gave_up_ = true;
            search_active_ = false;
            offboard_started_ = false;
            vehicle_->stopOffboard();
            return;
        }
        if (lastKnownSearchActive(elapsed_s, params_)) {
            // Hold position and sweep the nose around where the person was last seen, toward
            // the side they were drifting to (the offset when it is clear, else the last turn).
            const double side = std::abs(last_offset_) > 0.05 ? last_offset_ : last_turn_sign_;
            cmd = computeLastKnownSearchCommand(elapsed_s, side, params_);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Target lost; sweeping around the last known direction: "
                                 "forward=%.2f m/s, strafe=%.2f m/s, yaw=%.1f deg/s",
                                 cmd.forward_m_s, cmd.right_m_s, cmd.yawspeed_deg_s);
        } else {
            // Out of the last-known window: the original spiral, timed from its own start.
            const double spiral_s = std::max(0.0, elapsed_s - params_.last_known_search_s);
            cmd = computeSearchCommand(spiral_s, params_);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "Target lost; searching with spiral scan: forward=%.2f m/s, "
                                 "strafe=%.2f m/s, yaw=%.1f deg/s",
                                 cmd.forward_m_s, cmd.right_m_s, cmd.yawspeed_deg_s);
        }
    }

    cmd = limiter_.apply(cmd, params_, std::chrono::duration<double>(kLoopPeriod).count());
    const Status sent = vehicle_->setVelocityBody(
        static_cast<float>(cmd.forward_m_s), static_cast<float>(cmd.right_m_s), 0.0F,
        static_cast<float>(cmd.yawspeed_deg_s));
    if (!sent.ok) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", sent.what.c_str());
    }
}

void MovementNode::shutdown() {
    if (control_timer_) {
        control_timer_->cancel();
    }
    if (offboard_started_) {
        offboard_started_ = false;
        const Status stop = vehicle_->stopOffboard();
        if (!stop.ok) {
            RCLCPP_ERROR(get_logger(), "Could not leave offboard: %s", stop.what.c_str());
        }
    }
}

}  // namespace movement
