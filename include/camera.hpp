#ifndef FRAMEWORK_MARKERS_CAMERA_HPP
#define FRAMEWORK_MARKERS_CAMERA_HPP

#include "utils/options.hpp"
#include <metavision/sdk/driver/camera.h>
#include <metavision/hal/facilities/i_trigger_in.h>
#include <metavision/hal/facilities/i_hw_register.h>
#include "event_reader.hpp"
#include "utils/buffers.hpp"

#ifdef USE_CELEX
#include <CeleX5.h>
class CeleXCamera {
public:
  CeleXCamera(const Utils::Options::CameraSetup *setup);
  void initialize_camera();
  void start();
  // 宽高与原 Camera 保持一致
  int width, height;
  // 事件缓冲区（复用原 reader.buffers）
  Buffers reader_buffers;
private:
  std::shared_ptr<CeleX5> sensor;
  class CeleXEventStreamHandler* event_handler_; // 事件临时批处理类
};
#endif
class Camera {
public:
    Camera(Utils::Options::CameraSetup *setup);
    int width;
    int height;

    EventBufferReader reader;

    void initialize_camera();
    void start() {camera_object.start();};
    cv::Mat camera_matrix_cv;
    cv::Mat dist_coeffs;
    void printTiming(const Metavision::EventExtTrigger*, const Metavision::EventExtTrigger*);
    
private:
    Metavision::Camera camera_object;
    Metavision::Biases *biases;

    bool is_triggering_active = false;

    Eigen::Matrix3d camera_matrix_eigen;




};


#endif //FRAMEWORK_MARKERS_CAMERA_HPP
