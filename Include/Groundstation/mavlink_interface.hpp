#ifndef MAVLINK_INTERFACE_HPP
#define MAVLINK_INTERFACE_HPP

#include <mavsdk/mavlink/common/mavlink.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

class MavlinkSender {
public:
    MavlinkSender(const std::string& target_ip, int target_port) {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            std::cerr << "Failed to create socket\n";
            return;
        }

        memset(&addr_, 0, sizeof(addr_));
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(target_port);
        addr_.sin_addr.s_addr = inet_addr(target_ip.c_str());
    }

    ~MavlinkSender() {
        if (sock_ >= 0) close(sock_);
    }

    void sendGlobalPosition(double lat, double lon, float alt,
                             float vx, float vy, float vz) {
        mavlink_message_t msg;
        mavlink_msg_global_position_int_pack(
            1,
            200,
            &msg,
            getTimeBootMs(),
            static_cast<int32_t>(lat * 1e7),
            static_cast<int32_t>(lon * 1e7),
            static_cast<int32_t>(alt * 1000),
            static_cast<int32_t>(alt * 1000),
            static_cast<int16_t>(vx * 100),
            static_cast<int16_t>(vy * 100),
            static_cast<int16_t>(vz * 100),
            0
        );

        sendMessage(msg);
    }

private:
    uint32_t getTimeBootMs() {
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
    }

    void sendMessage(const mavlink_message_t& msg) {
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        int len = mavlink_msg_to_send_buffer(buffer, &msg);

        sendto(sock_, buffer, len, 0,
               reinterpret_cast<struct sockaddr*>(&addr_), sizeof(addr_));
    }

    int sock_;
    struct sockaddr_in addr_;
};

#endif // MAVLINK_INTERFACE_HPP