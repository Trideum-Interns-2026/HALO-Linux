// Web endpoint service (for Kean's site / any phone or laptop on the network).
//   in : everything interesting, over ROS
//   out: HTTP on :8080   GET /state -> JSON snapshot,  GET / -> tiny live page
// Plain POSIX sockets, no extra dependencies. No auth, so keep it on a trusted network.
// To feed QGroundControl instead/as well, MAVLink telemetry already flows to it via PX4.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "drone_msgs/msg/detection.hpp"
#include "drone_msgs/msg/geo_estimate.hpp"
#include "drone_msgs/msg/move_command.hpp"
#include "drone_msgs/msg/poi.hpp"

using drone_msgs::msg::Detection;
using drone_msgs::msg::GeoEstimate;
using drone_msgs::msg::MoveCommand;
using drone_msgs::msg::Poi;
using sensor_msgs::msg::BatteryState;
using sensor_msgs::msg::NavSatFix;

static const char * kPage = R"HTML(<!doctype html><meta charset="utf-8">
<title>Drone</title>
<body style="font-family:monospace;background:#111;color:#cfc">
<h3>Drone state</h3><pre id="o">connecting...</pre>
<script>
async function tick(){
  try{const r=await fetch('/state');document.getElementById('o').textContent=JSON.stringify(await r.json(),null,2);}
  catch(e){document.getElementById('o').textContent='no connection';}
}
setInterval(tick,500);tick();
</script></body>)HTML";

class WebNode : public rclcpp::Node {
public:
  WebNode() : Node("web_node") {
    const int port = declare_parameter("port", 8080);

    gps_sub_ = create_subscription<NavSatFix>("/drone/gps", rclcpp::SensorDataQoS(),
        [this](const NavSatFix & m) { std::lock_guard<std::mutex> l(mu_); gps_ = m; t_gps_ = now().seconds(); });
    bat_sub_ = create_subscription<BatteryState>("/drone/battery", rclcpp::SensorDataQoS(),
        [this](const BatteryState & m) { std::lock_guard<std::mutex> l(mu_); bat_ = m; });
    det_sub_ = create_subscription<Detection>("/perception/detection", 10,
        [this](const Detection &) { std::lock_guard<std::mutex> l(mu_); t_det_ = now().seconds(); });
    geo_sub_ = create_subscription<GeoEstimate>("/geolocation/estimate", 10,
        [this](const GeoEstimate & m) { std::lock_guard<std::mutex> l(mu_); geo_ = m; });
    poi_sub_ = create_subscription<Poi>("/tracking/poi", 10,
        [this](const Poi & m) { std::lock_guard<std::mutex> l(mu_); poi_ = m; t_poi_ = now().seconds(); });
    cmd_sub_ = create_subscription<MoveCommand>("/sim/mavlink_out", 10,
        [this](const MoveCommand & m) { std::lock_guard<std::mutex> l(mu_); cmd_ = m; });

    srv_ = ::socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    ::setsockopt(srv_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(srv_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 || ::listen(srv_, 8) < 0) {
      RCLCPP_ERROR(get_logger(), "Could not listen on port %d", port);
      return;
    }
    running_ = true;
    thread_ = std::thread([this] { serve(); });
    RCLCPP_INFO(get_logger(), "Serving http://0.0.0.0:%d", port);
  }

  ~WebNode() override {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (srv_ >= 0) ::close(srv_);
  }

private:
  void serve() {
    while (running_) {
      pollfd pfd{srv_, POLLIN, 0};
      if (::poll(&pfd, 1, 200) <= 0) continue;
      const int c = ::accept(srv_, nullptr, nullptr);
      if (c < 0) continue;
      char buf[2048];
      const ssize_t n = ::recv(c, buf, sizeof(buf) - 1, 0);
      const std::string req(buf, n > 0 ? static_cast<size_t>(n) : 0);

      std::string status = "200 OK", type = "text/html", body;
      if (req.rfind("GET /state", 0) == 0) { type = "application/json"; body = build_json(); }
      else if (req.rfind("GET / ", 0) == 0) { body = kPage; }
      else { status = "404 Not Found"; body = "not found"; }

      std::ostringstream r;
      r << "HTTP/1.1 " << status << "\r\nContent-Type: " << type
        << "\r\nAccess-Control-Allow-Origin: *\r\nContent-Length: " << body.size()
        << "\r\nConnection: close\r\n\r\n" << body;
      const std::string out = r.str();
      ::send(c, out.data(), out.size(), MSG_NOSIGNAL);
      ::close(c);
    }
  }

  std::string build_json() {
    std::lock_guard<std::mutex> l(mu_);
    const double t = now().seconds();
    auto age = [&](double last) {
      if (last < 0) return std::string("null");
      std::ostringstream a; a << std::fixed << std::setprecision(2) << (t - last); return a.str();
    };
    std::ostringstream o;
    o << std::fixed << std::setprecision(7);

    o << "{\"drone\":";
    if (gps_) o << "{\"lat\":" << gps_->latitude << ",\"lon\":" << gps_->longitude << ",\"alt\":" << gps_->altitude << "}";
    else o << "null";

    o << ",\"poi\":";
    if (poi_) o << "{\"id\":" << poi_->id << ",\"confidence\":" << poi_->confidence << ",\"hits\":" << poi_->hits
                << ",\"lla\":[" << poi_->lla[0] << "," << poi_->lla[1] << "," << poi_->lla[2]
                << "],\"error_ecef\":[" << poi_->error_ecef[0] << "," << poi_->error_ecef[1] << "," << poi_->error_ecef[2] << "]}";
    else o << "null";

    o << ",\"last_geolocation\":";
    if (geo_) o << "{\"lla\":[" << geo_->lla[0] << "," << geo_->lla[1] << "," << geo_->lla[2]
                << "],\"error_ecef\":[" << geo_->error_ecef[0] << "," << geo_->error_ecef[1] << "," << geo_->error_ecef[2] << "]}";
    else o << "null";

    o << ",\"command\":";
    if (cmd_) o << "{\"type\":\"" << (cmd_->type == MoveCommand::ORBIT ? "ORBIT" : "GOTO")
                << "\",\"lat\":" << cmd_->lat << ",\"lon\":" << cmd_->lon << ",\"alt\":" << cmd_->alt
                << ",\"radius\":" << cmd_->radius << "}";
    else o << "null";

    o << ",\"health\":{\"gps_age_s\":" << age(t_gps_) << ",\"detection_age_s\":" << age(t_det_)
      << ",\"poi_age_s\":" << age(t_poi_);
    if (bat_) o << ",\"battery_pct\":" << std::setprecision(1) << bat_->percentage * 100.0
                << ",\"battery_v\":" << bat_->voltage;
    o << "}}";
    return o.str();
  }

  std::mutex mu_;
  std::optional<NavSatFix> gps_;
  std::optional<BatteryState> bat_;
  std::optional<GeoEstimate> geo_;
  std::optional<Poi> poi_;
  std::optional<MoveCommand> cmd_;
  double t_gps_ = -1.0, t_det_ = -1.0, t_poi_ = -1.0;

  int srv_ = -1;
  std::atomic<bool> running_{false};
  std::thread thread_;

  rclcpp::Subscription<NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<BatteryState>::SharedPtr bat_sub_;
  rclcpp::Subscription<Detection>::SharedPtr det_sub_;
  rclcpp::Subscription<GeoEstimate>::SharedPtr geo_sub_;
  rclcpp::Subscription<Poi>::SharedPtr poi_sub_;
  rclcpp::Subscription<MoveCommand>::SharedPtr cmd_sub_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebNode>());
  rclcpp::shutdown();
}
