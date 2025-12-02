//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

#pragma once

// #include <std>

#include <vector>
#include <mutex>
#include <chrono>
#include <functional>
#include <algorithm>
#include <atomic>
#include <thread>
#include <time.h>

namespace t_manager {

// Thread data structure
typedef struct thread_s {
    std::chrono::nanoseconds periodicity;   // Thread periodicity
    std::function<void()> update_hook;  // Callback function that updates the robot's control (i.e. send setpoints and get feedback) within a real-time control loop
    std::function<void()> finalize_hook;  // Callback function that stops the robot's control after the real-time control loop stops (i.e. make sure that the robot stops correctly)
    std::function<void()> benchmark_hook;  // Callback function that benchmarks the loop by measuring execution times
}thread_t;



/**
 * It executes a callback function stored in thread structure and sleep until the assigned periodicity is satisfied.
 * When all the registered activities reach the dead state, the loop is interrupted.
 * @param[in] thread thread data structure of type thread_t
 * */
void do_thread_loop(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag);

void do_thread_loop_std_sleep_until(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag);
void do_thread_loop_std_sleep_for(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag);
void do_thread_loop_posix_usleep(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag);
void do_thread_loop_posix_clock_nanosleep(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag);


bool sleep_clock_nanosleep(const struct timespec& next_wakeup_time, int& err_int);
void add_ns_to_timespec( const int64_t ns, struct timespec & ts);

void setScheduling(std::thread &th, int policy, int priority);

} // namespace t_manager
