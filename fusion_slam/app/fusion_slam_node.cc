/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2025-06-14 02:14:47
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2025-06-14 15:19:18
 * @FilePath: /fusion_slam_ws/src/fusion_slam/app/fusion_slam_node.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <cxxabi.h>    // 用于 demangle
#include <execinfo.h>  // 用于 backtrace
#include <ros/rate.h>
#include <ros/ros.h>
#include <csignal>
#include <memory>
#include <string>
#include "System.hh"
#include "SystemConfig.hh"
#include "common/logger.hpp"

// 平台检测
#ifdef _WIN32
#include <DbgHelp.h>
#include <windows.h>
#pragma comment(lib, "DbgHelp.lib")
#else
#include <dlfcn.h>
#include <unistd.h>
#endif

// 堆栈跟踪函数
std::string capture_stacktrace(int max_frames = 64) {
    std::stringstream ss;
    ss << "Stacktrace:\n";

#ifdef _WIN32
    // Windows 实现
    HANDLE process = GetCurrentProcess();
    SymInitialize(process, NULL, TRUE);

    void* stack[max_frames];
    USHORT frames = CaptureStackBackTrace(0, max_frames, stack, NULL);

    SYMBOL_INFO* symbol = (SYMBOL_INFO*)calloc(sizeof(SYMBOL_INFO) + 256 * sizeof(char), 1);
    symbol->MaxNameLen = 255;
    symbol->SizeOfStruct = sizeof(SYMBOL_INFO);

    for (USHORT i = 0; i < frames; i++) {
        if (SymFromAddr(process, (DWORD64)(stack[i]), 0, symbol)) {
            ss << "  " << i << ": " << symbol->Name << " + " << (DWORD64)(stack[i]) - symbol->Address << "\n";
        }
    }

    free(symbol);
    SymCleanup(process);
#else
    // Linux/macOS 实现
    void* array[max_frames];
    size_t size = backtrace(array, max_frames);
    char** symbols = backtrace_symbols(array, size);

    if (symbols == nullptr) {
        ss << "  (无法解析堆栈符号)\n";
        return ss.str();
    }

    for (size_t i = 0; i < size; i++) {
        char* symbol = symbols[i];
        ss << "  " << i << ": ";

        // 尝试解析符号 (使用 c++filt 风格)
        char* demangled = nullptr;
        int status = 0;

        // 查找括号和加号
        char* begin_name = nullptr;
        char* begin_offset = nullptr;
        char* end_offset = nullptr;

        // 查找 mangled name 的起始位置
        for (char* p = symbol; *p; ++p) {
            if (*p == '(') {
                begin_name = p + 1;
            } else if (*p == '+') {
                begin_offset = p;
            } else if (*p == ')' && begin_name && !begin_offset) {
                *p = '\0';  // 截断符号名
            } else if (*p == ')' && begin_offset) {
                end_offset = p;
                break;
            }
        }

        if (begin_name && begin_offset && end_offset && begin_name < begin_offset) {
            *begin_offset = '\0';
            *end_offset = '\0';

            // 尝试 demangle
            demangled = abi::__cxa_demangle(begin_name, nullptr, nullptr, &status);
            if (status == 0) {
                ss << symbol << "(" << demangled << "+" << begin_offset + 1 << ")\n";
            } else {
                // 无法 demangle，使用原始符号
                ss << symbol << "(" << begin_name << "+" << begin_offset + 1 << ")\n";
            }
            free(demangled);
        } else {
            // 无法解析，直接输出原始符号
            ss << symbol << "\n";
        }
    }

    free(symbols);
#endif

    return ss.str();
}

// 信号处理函数
void signal_handler(int signum) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    // 创建 crash logger，使用控制台 sink
    auto crash_logger = std::make_shared<spdlog::logger>("crash_logger", console_sink);
    crash_logger->set_level(spdlog::level::critical);

    // 记录信号信息和堆栈
    crash_logger->critical("程序崩溃，信号: {}", signum);
    crash_logger->critical("{}", capture_stacktrace());

    // 刷新日志
    crash_logger->flush();

    // 恢复默认信号处理
    signal(signum, SIG_DFL);
    raise(signum);
}

int main(int argc, char** argv) {
    // 注册信号处理函数
    signal(SIGSEGV, signal_handler);  // 段错误
    signal(SIGABRT, signal_handler);  // 断言失败
    signal(SIGFPE, signal_handler);   // 浮点异常
    signal(SIGILL, signal_handler);   // 非法指令
    signal(SIGBUS, signal_handler);   // 总线错误

    ros::init(argc, argv, "fusion_slam_node");
    ros::NodeHandle nh_;
    ros::Rate rate(1000);
    SpdLogger logger;
    LOG_INFO("[Fusion Slam Node Start!]");
    std::string config_path;
    std::shared_ptr<slam::System> system_ptr = std::make_shared<slam::System>(nh_);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }
    LOG_INFO("[Fusion Slam Node Exit!]");
    return 0;
}