#include "track_can_control/CanController.hpp"
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

CanController::CanController(const std::string& interface)
    : interface_(interface), socket_fd_(-1), running_(false) {}

CanController::~CanController() {
    shutdown();
}

bool CanController::init() {
    // 创建SocketCAN套接字
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        perror("socket");
        return false;
    }

    // 获取接口索引
    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_.c_str());
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        close(socket_fd_);
        return false;
    }

    // 绑定到接口
    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(socket_fd_);
        return false;
    }

    // 可选：设置接收过滤器以只接收特定ID（本例为了灵活性，不过滤，全部接收后在回调中分发）
    // 这里不过滤，让应用层处理所有标准帧

    running_ = true;
    rx_thread_ = std::thread(&CanController::rxLoop, this);
    tx_thread_ = std::thread(&CanController::txLoop, this);

    std::cout << "CAN interface " << interface_ << " initialized successfully." << std::endl;
    return true;
}

void CanController::shutdown() {
    running_ = false;
    tx_cv_.notify_all();
    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    std::cout << "CAN controller shut down." << std::endl;
}

void CanController::registerRxHandlerExtended(uint32_t can_id, CanRxCallback callback) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    uint32_t key = can_id | CAN_EFF_FLAG;  // 添加扩展帧标志位作为键
    rx_handlers_[key] = callback;
}

void CanController::registerRxHandler(uint32_t can_id, CanRxCallback callback) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    rx_handlers_[can_id] = callback;
}

void CanController::unregisterRxHandler(uint32_t can_id) {
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    rx_handlers_.erase(can_id);
}

//bool CanController::sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
//    if (len > 8) {
//        std::cerr << "CAN data length exceeds 8 bytes" << std::endl;
//        return false;
//    }
//    struct can_frame frame;
//    frame.can_id = can_id ; // 标准帧，清除扩展标志位
//    frame.can_dlc = len;
//    std::memcpy(frame.data, data, len);
//    // 入队异步发送（也可直接调用sendFrameInternal，这里演示队列方式）
//    {
//        std::lock_guard<std::mutex> lock(tx_mutex_);
//        tx_queue_.push(frame);
//    }
//    tx_cv_.notify_one();
//    return true;
//}

bool CanController::sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    if (len > 8) {
        std::cerr << "CAN data length exceeds 8 bytes" << std::endl;
        return false;
    }
    struct can_frame frame;
    // 自动识别扩展帧：如果ID大于0x7FF，则认为是扩展帧，添加标志位
    if (can_id > 0x7FF) {
        frame.can_id = can_id | CAN_EFF_FLAG;
    } else {
        frame.can_id = can_id;
    }
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);
    {
        std::lock_guard<std::mutex> lock(tx_mutex_);
        tx_queue_.push(frame);
    }
    tx_cv_.notify_one();
    return true;
}

void CanController::rxLoop() {
    struct can_frame frame;
    fd_set readSet;
    struct timeval timeout;

    while (running_) {
        FD_ZERO(&readSet);
        FD_SET(socket_fd_, &readSet);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100ms 超时，便于检查running_标志

        int ret = select(socket_fd_ + 1, &readSet, nullptr, nullptr, &timeout);
        if (ret < 0) {
            perror("select");
            break;
        } else if (ret == 0) {
            continue; // 超时
        }

        if (FD_ISSET(socket_fd_, &readSet)) {
            int nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
            if (nbytes < 0) {
                perror("read");
                break;
            } else if (nbytes == sizeof(struct can_frame)) {
                processReceivedFrame(frame);
            }
        }
    }
}


void CanController::processReceivedFrame(const struct can_frame& frame) {
    uint32_t can_id_key = frame.can_id;  // 保留原始标志位（标准帧无标志，扩展帧带CAN_EFF_FLAG）
    std::lock_guard<std::mutex> lock(handlers_mutex_);
    auto it = rx_handlers_.find(can_id_key);
    if (it != rx_handlers_.end() && it->second) {
        it->second(can_id_key & CAN_EFF_MASK, frame.data, frame.can_dlc); // 回调时传递去除标志位的ID
    }
}

void CanController::txLoop() {
    while (running_) {
        std::unique_lock<std::mutex> lock(tx_mutex_);
        tx_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return !tx_queue_.empty() || !running_; });
        if (!running_) break;

        while (!tx_queue_.empty()) {
            struct can_frame frame = tx_queue_.front();
            tx_queue_.pop();
            lock.unlock(); // 发送时解锁
            sendFrameInternal(frame);
            lock.lock();
        }
    }
}

bool CanController::sendFrameInternal(const struct can_frame& frame) {
    if (socket_fd_ < 0) return false;
    int nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != sizeof(frame)) {
        perror("write");
        return false;
    }
    return true;
}
