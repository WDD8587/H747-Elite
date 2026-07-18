/* RK3566 — 4 pthreads SCHED_FIFO */
#include <iostream>
#include <thread>
#include <atomic>
#include <csignal>
#include <sys/mman.h>
#include "ipc_proto.h"

static std::atomic<bool> gRun{true};
void sig_handler(int) { gRun = false; }

static void slam_thread() {
    while (gRun) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
static void dwa_thread() {
    while (gRun) std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
static void yolo_thread() {
    while (gRun) std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
static void mqtt_thread() {
    while (gRun) std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main() {
    signal(SIGINT, sig_handler); signal(SIGTERM, sig_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);
    std::cout << "[RK3566] 4 threads SCHED_FIFO (80/70/60/50)" << std::endl;

    std::thread t1(slam_thread), t2(dwa_thread), t3(yolo_thread), t4(mqtt_thread);
    t1.join(); t2.join(); t3.join(); t4.join();
    return 0;
}
