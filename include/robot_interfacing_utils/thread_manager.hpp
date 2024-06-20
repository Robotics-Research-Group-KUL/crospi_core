#pragma once

// #include <std>

#include <vector>
#include <mutex>
#include <chrono>
#include <functional>
#include <atomic>
#include <thread>

namespace t_manager {

// Thread data structure
typedef struct thread_s {
    std::chrono::nanoseconds periodicity;   // Thread periodicity
    std::function<void()> update_hook;  // Callback function that updates the robot's control (i.e. send setpoints and get feedback) within a real-time control loop
    std::function<void()> finalize_hook;  // Callback function that stops the robot's control after the real-time control loop stops (i.e. make sure that the robot stops correctly)
}thread_t;



/**
 * It executes a callback function stored in thread structure and sleep until the assigned periodicity is satisfied.
 * When all the registered activities reach the dead state, the loop is interrupted.
 * @param[in] thread thread data structure of type thread_t
 * */
void do_thread_loop(boost::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag);


void setScheduling(std::thread &th, int policy, int priority);

} // namespace t_manager
