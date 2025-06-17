#include <iostream>
#include "utils/options.hpp"
#include "camera.hpp" // 添加这个
#include <stdexcept> // 为了 std::exception

// 假设 MetavisionSDK 的异常基类或特定异常类
// #include <metavision/sdk/driver/camera_exception.h> // (示例，请确认具体头文件)

int main(int argc, char *argv[]) {
    std::cout << "Stage 1: Basic main reached." << std::endl << std::flush;
    try {
        Utils::Options::Parser parser(argc, argv);
        std::cout << "Stage 2: Parser object created." << std::endl << std::flush;

        // ... (可以把 parser 的 if/else 输出逻辑加回来)

        Camera cam(&parser.current_setup.getCamConfig());
        std::cout << "Stage 3a: Camera object created." << std::endl << std::flush;

        cam.initialize_camera();
        std::cout << "Stage 3b: Camera initialize_camera() called." << std::endl << std::flush;
        // 这里应该有基于异常的错误检查，或者您必须知道 initialize_camera() 如何报告错误

    } /* catch (const Metavision::CameraException &e) { // 使用具体的异常类型
        std::cerr << "Metavision Camera Exception: " << e.what() << std::endl;
        return -1;
    } */ catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception." << std::endl;
        return -1;
    }
    return 0;
}