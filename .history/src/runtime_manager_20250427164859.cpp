#include "../include/runtime_manager.hpp"
#include "../include/camera.hpp"
#include "../include/runtime_manager.hpp"
#include "../include/camera.hpp"
RuntimeManager::RuntimeManager(MarkersManager &markers_manager,
#ifdef USE_METAVISION
                                Camera *cam,
#endif
                                Utils::Options::Setup *setup) {
    logger = std::make_shared<Logger>(setup->csv_logging_enabled, setup->parent_tf_name);
    markers = &markers_manager;
    markers->setLogger(logger);

#ifdef USE_METAVISION
    detector = new DetectionAlgorithm(cam->width, cam->height, markers, logger);
#else
    // 没有Camera，默认设定一个分辨率，比如 640x480
    detector = new DetectionAlgorithm(1280, 800, markers, logger);
#endif
}

void RuntimeManager::start() {
    logger->start();

    detector->buffers.setInputBuffer(buffers.getInputBuffer());
    detector->start();
}
