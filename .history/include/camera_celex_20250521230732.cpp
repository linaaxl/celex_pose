#ifdef USE_CELEX

#include "camera.hpp"
#include <CeleX5.h>
#include <iostream>
#include <vector>
#include <memory>

// 假定你的缓冲区结构如下，如果不同需自行调整
// #include "buffers.hpp"  // 已包含

class CeleXEventStreamHandler {
public:
    CeleXEventStreamHandler(Buffers* buffers)
        : buffers_(buffers) {}

    // 处理单个事件
    void handleEvent(uint16_t x, uint16_t y, uint64_t ts, bool polarity) {
        Metavision::Event2d ev;
        ev.x = x;
        ev.y = y;
        ev.t = ts;     // 时间戳单位需根据实际SDK，通常是微秒
        ev.p = polarity;
        batch_.push_back(ev);

        // 可批量塞入缓冲区（例如每N个塞一次）
        if (batch_.size() >= 1024) {
            buffers_->pushEvents(batch_.data(), batch_.data() + batch_.size());
            batch_.clear();
        }
    }

    // 剩余事件送入缓冲区
    void flush() {
        if (!batch_.empty()) {
            buffers_->pushEvents(batch_.data(), batch_.data() + batch_.size());
            batch_.clear();
        }
    }

private:
    Buffers* buffers_;
    std::vector<Metavision::Event2d> batch_;
};

// ------------- CeleXCamera实现 -------------

CeleXCamera::CeleXCamera(const Utils::Options::CameraSetup* setup)
    : width(1280), height(800), event_handler_(&reader_buffers)
{
    sensor_ = std::make_shared<CeleX5>();
    // 选用MIPI接口（或更改为USB等你的实际硬件）
    sensor_->openSensor(CeleX5::CeleX5_MIPI);

    // 设置为Event模式
    sensor_->setSensorFixedMode(Event_Off_Pixel_Timestamp_Mode);
    // 你可以进一步配置bias等参数
}

void CeleXCamera::initialize_camera() {
    // 若需bias、极性等参数配置可在这里做
}

void CeleXCamera::start() {
    // 注册回调，每来一个事件帧，拆包为单事件
    sensor_->setEventCallback(CeleX5::EventBinaryFrame, [this](uint8_t* pBuffer, int bufferSize) {
        // SDK结构体参考：struct EventOffPixel
        // 结构: 2字节x, 2字节y, 8字节时间戳, 1字节polarity
        struct EventOffPixel {
            uint16_t x;
            uint16_t y;
            uint64_t t;
            uint8_t  p;
        };
        int eventSize = sizeof(EventOffPixel);
        int numEvents = bufferSize / eventSize;
        auto* evs = reinterpret_cast<EventOffPixel*>(pBuffer);

        for (int i = 0; i < numEvents; ++i) {
            event_handler_.handleEvent(evs[i].x, evs[i].y, evs[i].t, evs[i].p);
        }
        event_handler_.flush();
    });

    // 启动相机
    sensor_->startSensor();
}

#endif
