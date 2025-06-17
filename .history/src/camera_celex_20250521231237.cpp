#ifdef USE_CELEX

#include "camera.hpp"
#include <CeleX5.h>
#include <iostream>
#include <vector>
#include <memory>

// 假定你的算法用 Metavision::Event2d 格式
// 若格式不同请告诉我你的事件结构体
namespace Metavision {
struct Event2d {
    int x, y;
    uint64_t t; // 时间戳（us单位）
    bool p;     // 极性
};
}

// 事件批处理适配器
class CeleXEventStreamHandler {
public:
    CeleXEventStreamHandler(Buffers* buffers)
        : buffers_(buffers) {}

    void handleEvent(uint16_t x, uint16_t y, uint64_t ts, uint8_t polarity) {
        Metavision::Event2d ev;
        ev.x = x;
        ev.y = y;
        ev.t = ts; // 以微秒为单位，若SDK为其他单位需换算
        ev.p = (polarity != 0); // 转为bool
        batch_.push_back(ev);
        if (batch_.size() >= 1024) { // 可根据缓冲优化
            buffers_->pushEvents(batch_.data(), batch_.data() + batch_.size());
            batch_.clear();
        }
    }

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

CeleXCamera::CeleXCamera(const Utils::Options::CameraSetup* setup)
    : width(1280), height(800), reader_buffers(), event_handler_(nullptr)
{
    sensor_ = std::make_shared<CeleX5>();
    event_handler_ = new CeleXEventStreamHandler(&reader_buffers);

    // 打开MIPI接口
    sensor_->openSensor(CeleX5::CeleX5_MIPI);

    // 设置为事件模式
    sensor_->setSensorFixedMode(Event_Off_Pixel_Timestamp_Mode);
    // 若有其他bias参数需要，可在此设置
}

void CeleXCamera::initialize_camera() {
    // 若需设定bias/参数/去噪等可写在此
}

void CeleXCamera::start() {
    // 注册回调，事件帧转为算法格式
    sensor_->setEventCallback(CeleX5::EventBinaryFrame, [this](uint8_t* pBuffer, int bufferSize) {
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
            event_handler_->handleEvent(evs[i].x, evs[i].y, evs[i].t, evs[i].p);
        }
        event_handler_->flush();
    });

    sensor_->startSensor();
}

#endif
