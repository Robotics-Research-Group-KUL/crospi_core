#pragma once

// #include <std>

#include <vector>
#include <mutex>
#include <chrono>
#include <functional>
#include <atomic>
#include <thread>
#include <pthread.h>

#include <iostream>
#include <cstring>
#include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>



#include "robot_interfacing_utils/thread_manager.hpp"

namespace t_manager {
/**
 * It executes a callback function stored in thread structure and sleep until the assigned periodicity is satisfied.
 * When all the registered activities reach the dead state, the loop is interrupted.
 * @param[in] thread thread data structure of type thread_t
 * */
void do_thread_loop(boost::shared_ptr<t_manager::thread_t> thread, volatile std::atomic<bool>& stopFlag){
    // const std::chrono::nanoseconds periodicity = std::chrono::milliseconds(periodicity_param);
    std::chrono::nanoseconds periodicity = thread->periodicity;
    std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;

    std::chrono::steady_clock::time_point  time_start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point  time_end = std::chrono::steady_clock::now();
    std::chrono::duration<double>  elapsed_time = time_end - time_start;

    auto duration_s = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start);

    useconds_t periodicity_us = periodicity.count()/1000.0;

    struct timespec start, end;
    uint64_t elapsed_time_us;


    while (!stopFlag.load()){
    //First attempt
        time_start = std::chrono::steady_clock::now();
        thread->update_hook();
        // Using std library to sleep
        std::this_thread::sleep_until(end_time_sleep);
        while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
            errno = 0;
            std::this_thread::sleep_until(end_time_sleep);
            std::cout << "Needed some extra time"<< std::endl; //Temporarily placed for debugging
        }
        end_time_sleep = std::chrono::steady_clock::now() + periodicity; //adds periodicity

        // //The following evaluates how much time it slept: 
        time_end = std::chrono::steady_clock::now();
        // elapsed_time = time_end - time_start;
        // duration_s = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start);
        // std::cout << "Time spent: "<< duration_s.count()/1000.0<<" milliseconds" << std::endl; //Temporarily placed for debugging



    //Second attempt
        // time_start = std::chrono::steady_clock::now();
        // thread->update_hook();
        // time_end = std::chrono::steady_clock::now();
        // elapsed_time = time_end - time_start;
        // if(periodicity > elapsed_time){
        //     std::this_thread::sleep_for(periodicity - elapsed_time);
        // }

        // //The following evaluates how much time it slept: 
        // time_end = std::chrono::steady_clock::now();
        // // elapsed_time = time_end - time_start;
        // duration_s = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start);
        // std::cout << "Time spent: "<< duration_s.count()/1000.0<<" milliseconds" << std::endl; //Temporarily placed for debugging

    //Third attempt
        // clock_gettime(CLOCK_MONOTONIC_RAW, &start);
        // thread->update_hook();   
        // clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        // elapsed_time_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
        // // Sleep such that the whole loop thread->last periodicity_us
        // // std::cout << periodicity.count() << " cycletime" << std::endl;
        // if ( periodicity_us > elapsed_time_us){
        //     usleep(periodicity_us - elapsed_time_us);
        //     // std::cout << "hellooo" << std::endl;
        // }

        // clock_gettime(CLOCK_MONOTONIC_RAW, &end);
        // elapsed_time_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
        // std::cout << "Time spent: "<< elapsed_time_us/1000.0 <<" milliseconds" << std::endl; //Temporarily placed for debugging

    }

    if ( thread->finalize_hook != nullptr)   {thread->finalize_hook();}
    if(stopFlag.load())    {std::cout << "stopFlag=true" << std::endl;}
    
}


void setScheduling(std::thread &th, int sched_policy, int priority) {
    sched_param sch_params;
    int current_sched_policy;
    pthread_getschedparam(th.native_handle(), &current_sched_policy, &sch_params);

    sch_params.sched_priority = priority;

    if(pthread_setschedparam(th.native_handle(), sched_policy, &sch_params)) {
        std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
    }
}

} // namespace t_manager
