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
void do_thread_loop(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag){

    std::chrono::steady_clock::time_point  start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point  end_time = std::chrono::steady_clock::now();
    // auto duration_s = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    // auto now = std::chrono::steady_clock::now();
    long long period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    long long sum_period = 0ll;
    long long max_period = 0ll;
    long long min_period = 0ll;
    long long num_samples = 0ll;


    // Comment the following lambda out to avoid measuring time:
    
    thread->benchmark_hook = [&end_time, &start_time, &period_ns, &sum_period, &max_period, &min_period, &num_samples]() {
    
        end_time = std::chrono::steady_clock::now();
        period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
        start_time = std::chrono::steady_clock::now();

        sum_period+= period_ns;
        num_samples++;
        if (period_ns > max_period) {
          max_period = period_ns;
        }
        if (min_period == 0 || period_ns < min_period) {
          min_period = period_ns;
        }
    };

    // do_thread_loop_std_sleep_until(thread,  stopFlag);
    // do_thread_loop_std_sleep_for(thread,  stopFlag);
    // do_thread_loop_posix_usleep(thread,  stopFlag);
    do_thread_loop_posix_clock_nanosleep(thread,  stopFlag);



    if ( thread->finalize_hook != nullptr)   {thread->finalize_hook();}
    // if(stopFlag.load(std::memory_order_relaxed))    {std::cout << "stopFlag=true" << std::endl;}

    std::cout << "------------ Periodicity statistics --------------" << std::endl;
    // std::cout << "Sum periodicity: " << sum_period << " ns" << std::endl;
    std::cout << "Max periodicity: " << max_period << " ns" << std::endl;
    std::cout << "Min periodicity: " << min_period << " ns" << std::endl;
    std::cout << "Number of samples: " << num_samples << std::endl;
    std::cout << "Average periodicity: " << (num_samples > 0 ? static_cast<double>(sum_period) / num_samples : 0.0) << " ns" << std::endl;
    
}

void do_thread_loop_std_sleep_until(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag){

   std::chrono::nanoseconds periodicity = thread->periodicity;
   std::chrono::steady_clock::time_point  end_time_sleep = std::chrono::steady_clock::now() + periodicity;

   while (!stopFlag.load(std::memory_order_relaxed)){
        thread->update_hook(); //Function that I want to execute periodically
        // Using std library to sleep
        std::this_thread::sleep_until(end_time_sleep);
        while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
            errno = 0;
            std::this_thread::sleep_until(end_time_sleep);
        }
        end_time_sleep = std::chrono::steady_clock::now() + periodicity; //adds periodicity
        if ( thread->benchmark_hook != nullptr)   {thread->benchmark_hook();}
   }
   
}

void do_thread_loop_std_sleep_for(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag){

   std::chrono::nanoseconds periodicity = thread->periodicity;
   std::chrono::steady_clock::time_point  time_start = std::chrono::steady_clock::now();
   std::chrono::steady_clock::time_point  time_end = std::chrono::steady_clock::now();
   std::chrono::duration<double>  elapsed_time = time_end - time_start;

   while (!stopFlag.load(std::memory_order_relaxed)){
       time_start = std::chrono::steady_clock::now();
       thread->update_hook(); //Function that I want to execute periodically
       time_end = std::chrono::steady_clock::now();
       elapsed_time = time_end - time_start;
       if(periodicity > elapsed_time){
           std::this_thread::sleep_for(periodicity - elapsed_time);
       }
       if ( thread->benchmark_hook != nullptr)   {thread->benchmark_hook();}
   }

}

void do_thread_loop_posix_usleep(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag){

   std::chrono::nanoseconds periodicity = thread->periodicity;
   struct timespec start, end;
   uint64_t elapsed_time_us;
   useconds_t periodicity_us = periodicity.count()/1000.0;

   while (!stopFlag.load(std::memory_order_relaxed)){
       clock_gettime(CLOCK_MONOTONIC_RAW, &start);
       thread->update_hook(); //Function that I want to execute periodically
       clock_gettime(CLOCK_MONOTONIC_RAW, &end);
       elapsed_time_us = (end.tv_sec - start.tv_sec) * 1000000 + (end.tv_nsec - start.tv_nsec) / 1000;
       if ( periodicity_us > elapsed_time_us){
           usleep(periodicity_us - elapsed_time_us);
       }
       if ( thread->benchmark_hook != nullptr)   {thread->benchmark_hook();}
    }
}

void do_thread_loop_posix_clock_nanosleep(std::shared_ptr<thread_t> thread, volatile std::atomic<bool>& stopFlag){
        
    struct timespec next_wakeup_time_;
    std::chrono::nanoseconds periodicity = thread->periodicity;
    uint64_t        period_ns_ = periodicity.count();
    int err_int = 0;
    clock_gettime(CLOCK_MONOTONIC, &next_wakeup_time_);
    
    while (!stopFlag.load(std::memory_order_relaxed)){
        thread->update_hook();
        clock_gettime(CLOCK_MONOTONIC, &next_wakeup_time_);//Remove to compensate for accumulated extra slept time, but we can get times between cyles 
        //... with errors in the order of magnitude of tenths of a millisecond below the specified cycle time. With it we always get erros above the specified cycle time that will accumulate over time. 
        add_ns_to_timespec(static_cast<int64_t>(period_ns_),next_wakeup_time_);

        if(!sleep_clock_nanosleep(next_wakeup_time_, err_int)){
            stopFlag.store(true); //Stops execution of the loop
        }
        if ( thread->benchmark_hook != nullptr)   {thread->benchmark_hook();} //Modify benchmark hook e.g. to measure time between executions
    }
}


bool sleep_clock_nanosleep(const struct timespec& next_wakeup_time, int& err_int){
    

    while ((err_int = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup_time, nullptr)) == EINTR) {
        // Interrupted by a signal handler; sleep for the remaining time
        errno = 0; // errno is thread-local; setting it in one thread does not affect its value in any other thread.
    }

    if (err_int != 0) {
        // Handle errors
        if (err_int == EINVAL) {
            fprintf(stderr, "Either the clockid was invalid or the value in the tv_nsec field was not in the range [0,999999999] or tv_sec was negative.\n");
        } else if (err_int == EFAULT) {
            fprintf(stderr, "EFAULT signal encountered. Memory error with timespec structures request or remain, which were specified an invalid memory address\n");
        } else if(err_int == ENOTSUP){
            fprintf(stderr, "The kernel does not support sleeping against this clockid.\n");
        }
        else {
            fprintf(stderr, "Unknown error with clock_nanosleep: %d\n", err_int);
        }
        return false;
    }
    return true;
}



void setScheduling(std::thread &th, int sched_policy, int priority) {
    sched_param sch_params;
    int current_sched_policy;
    pthread_getschedparam(th.native_handle(), &current_sched_policy, &sch_params);

    sch_params.sched_priority = priority;

    if(pthread_setschedparam(th.native_handle(), sched_policy, &sch_params)) {
        std::cerr << "Failed to set Thread scheduling : " << std::strerror(errno) << std::endl;
    }
    else{
        std::cout << "########################################################### " << std::endl;
        std::cout << "The priority of the thread was changed succesfully to " << priority << std::endl;
    }
}

bool operator <(const timespec& lhs, const timespec& rhs)
{
    if (lhs.tv_sec == rhs.tv_sec)
        return lhs.tv_nsec < rhs.tv_nsec;
    else
        return lhs.tv_sec < rhs.tv_sec;
}

bool operator >(const timespec& lhs, const timespec& rhs)
{
    if (lhs.tv_sec == rhs.tv_sec)
        return lhs.tv_nsec > rhs.tv_nsec;
    else
        return lhs.tv_sec > rhs.tv_sec;
}

 void add_ns_to_timespec( const int64_t ns, struct timespec & ts) {
  ts.tv_nsec += ns;

  while (ts.tv_nsec >= 1'000'000'000) {
    ++ts.tv_sec;
    ts.tv_nsec -= 1'000'000'000;
  }

  while (ts.tv_nsec < 0) {
    --ts.tv_sec;
    ts.tv_nsec += 1'000'000'000;
  }

}

} // namespace t_manager
