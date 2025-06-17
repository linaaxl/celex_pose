#include <iostream>
#include "utils/options.hpp"
#include "camera.hpp"
#include "utils/visualization.hpp"
#include "markers.hpp"
#include "runtime_manager.hpp"
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

void my_handler(int s){
    std::cout << "EXITING!" << std::endl;    
    exit(1); 
}

int main(int argc, char *argv[]) {
    std::cerr << "Program started (using cerr)." << std::endl; // 尝试使用 cerr，它通常不被完全缓冲
    std::cout << "Program started (using cout with flush)." << std::endl << std::flush; // 强制刷新 cout
    //Parse optio
    Utils::Options::Parser parser(argc, argv);

    if (parser.current_setup.cam_config.is_recording){
        std::cout << "Using recording: " << parser.current_setup.cam_config.file_path << std::endl;
    }
    else{
        std::cout << "Using camera with biases from: " << parser.current_setup.cam_config.biases_file << std::endl;
        std::cout << "Using camera config from: " << parser.current_setup.cam_config.config_file_path << std::endl;
    }

    try { // 开始 try 块
        Camera cam(&parser.current_setup.getCamConfig());

        cam.initialize_camera(); // 调用初始化
        std::cout << "Camera initialized." << std::endl; // 如果没有异常，则认为成功

        MarkersManager markers(parser.current_setup.getMarkerConfig(), &cam);

        RuntimeManager runtime(markers, &cam, &parser.current_setup);
        runtime.buffers.setInputBuffer(cam.reader.buffers.getOutputBuffer());

        VisualizationController vis(cam.width, cam.height, &cam, &runtime);
        vis.buffers.setInputBuffer(cam.reader.buffers.getOutputBuffer());

        cam.start(); // 调用启动
        std::cout << "Camera started." << std::endl; // 如果没有异常，则认为成功

        auto now_ms = std::chrono::system_clock::now();
        // ... (您的计时代码可以保留)

        vis.start();
        runtime.start();
        cam.reader.start();

        bool recording = false;
        if(parser.current_setup.recording_time > 0){
            recording = true;
        }
        signal (SIGINT, my_handler);

        std::cout << "Entering main loop..." << std::endl;
        while (!vis.isFinished()) {
            if(recording){
            
            }
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        std::cout << "Exiting main loop." << std::endl;

    } catch (const Metavision::CameraException &e) { // 捕获 Metavision 特定的相机异常
        std::cerr << "Metavision Camera Exception: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception &e) { // 捕获其他标准异常
        std::cerr << "Standard Exception: " << e.what() << std::endl;
        return -1;
    } catch (...) { // 捕获所有其他未知类型的异常
        std::cerr << "An unknown error occurred." << std::endl;
        return -1;
    } // 结束 try-catch 块

    std::cout << "Program finished successfully." << std::endl;
    return 0;
}
