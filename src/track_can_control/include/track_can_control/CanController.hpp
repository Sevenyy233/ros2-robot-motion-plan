#ifndef CAN_CONTROLLER_HPP
#define CAN_CONTROLLER_HPP

#include <linux/can.h>
#include <functional>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <condition_variable>

// 接收回调函数类型：参数为can_id, can数据指针, 数据长度
using CanRxCallback = std::function<void(uint32_t can_id, const uint8_t* data, uint8_t len)>;

class CanController {
public:
    explicit CanController(const std::string& interface = "can0");
    ~CanController();

    // 初始化CAN socket并启动接收线程
    bool init();
    // 关闭连接并停止线程
    void shutdown();

    // 注册指定ID的接收回调（标准帧）
    void registerRxHandler(uint32_t can_id, CanRxCallback callback);

    void registerRxHandlerExtended(uint32_t can_id, CanRxCallback callback); // can_id 为原始扩展ID（不含标志位）

    // 注销指定ID的回调
    void unregisterRxHandler(uint32_t can_id);

    // 发送CAN帧（标准帧）
    bool sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len);

private:
    std::string interface_;
    int socket_fd_;
    std::atomic<bool> running_;
    std::thread rx_thread_;

    // 接收回调映射表
    std::unordered_map<uint32_t, CanRxCallback> rx_handlers_;
    std::mutex handlers_mutex_;

    // 发送队列（可选，如需异步发送可使用）
    std::queue<struct can_frame> tx_queue_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;
    std::thread tx_thread_;

    // 接收循环
    void rxLoop();
    // 发送循环（如果使用队列）
    void txLoop();
    // 处理接收到的帧
    void processReceivedFrame(const struct can_frame& frame);
    // 实际发送操作
    bool sendFrameInternal(const struct can_frame& frame);
};

#endif // CAN_CONTROLLER_HPP
