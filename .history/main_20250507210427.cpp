#include <iostream>

#include "utils/options.hpp" // 添加这个
int main(int argc, char *argv[]) {
    std::cout << "Stage 1: Basic main reached." << std::endl << std::flush;
    try { // 为了捕获构造函数中可能的异常
        Utils::Options::Parser parser(argc, argv);
        std::cout << "Stage 2: Parser object created." << std::endl << std::flush;
    } catch (const std::exception& e) {
        std::cerr << "Exception during Parser creation: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Unknown exception during Parser creation." << std::endl;
        return -1;
    }
    return 0;
}