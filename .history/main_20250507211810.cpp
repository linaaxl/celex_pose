// main.cpp (阶段 3 或更完整版本)
#include <iostream>
#include "utils/options.hpp"
#include "camera.hpp"
#include <stdexcept> // For std::exception

// #include <metavision/sdk/driver/camera_exception.h> // 替换为实际的Metavision异常头文件

void my_handler(int s){ // 确保这个函数在 main 外部或者是一个静态成员等
    std::cout << "EXITING!" << std::endl;
    exit(1);
}

int main(int argc, char *argv[]) {
    std::cout << "Stage 1: Basic main reached." << std::endl << std::flush;

    try {
        std::cout << "Attempting to parse options..." << std::endl << std::flush;
        Utils::Options::Parser parser(argc, argv); // 这一行可能会因为缺少参数而抛出异常
        std::cout << "Stage 2: Parser object created." << std::endl << std::flush;

        if (parser.current_setup.cam_config.is_recording){
            std::cout << "Using recording: " << parser.current_setup.cam_config.file_path << std::endl << std::flush;
        } else {
            std::cout << "Using camera with biases from: " << parser.current_setup.cam_config.biases_file << std::endl << std::flush;
            std::cout << "Using camera config from: " << parser.current_setup.cam_config.config_file_path << std::endl << std::flush;
        }

        std::cout << "Attempting to create Camera object..." << std::endl << std::flush;
        Camera cam(&parser.current_setup.getCamConfig());
        std::cout << "Stage 3a: Camera object created." << std::endl << std::flush;

        std::cout << "Attempting to initialize camera..." << std::endl << std::flush;
        cam.initialize_camera(); // 这一行如果失败，Metavision SDK 应该会抛出异常
        std::cout << "Stage 3b: Camera initialize_camera() called." << std::endl << std::flush;

        // 在这里可以继续逐步添加 MarkersManager, RuntimeManager, VisualizationController 等的创建
        // 以及 cam.start() 和其他 start() 调用
        // 确保它们也都在 try 块内部

        // ... 示例：添加 cam.start() ...
        std::cout << "Attempting to start camera..." << std::endl << std::flush;
        cam.start(); // 如果失败，也应该抛出异常
        std::cout << "Stage 3c: Camera start() called." << std::endl << std::flush;


        // ... 您的其他代码 (signal, main loop etc.) ...


    /* } catch (const Metavision::CameraException &e) { // 使用最具体的 Metavision 异常类型
        std::cerr << "Metavision Camera Exception: " << e.what() << std::endl;
        return -1; */
    } catch (const std::exception& e) { // 捕获 Parser 抛出的异常和其他标准异常
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception caught." << std::endl;
        return -1;
    }

    std::cout << "Program appears to have run its course within try block." << std::endl << std::flush;
    return 0;
}