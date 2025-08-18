#include "shared_memory_test_new_drivers.hpp"

// For real-time control loop
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>



using namespace std::chrono_literals;
// using namespace etasl;


#define DOF 7

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


RobotControlNode::RobotControlNode(): Node("shared_memory_test_new_drivers")
, time(0.0)
{

  joint_state_msg = sensor_msgs::msg::JointState();
  jointnames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
  joint_state_msg.name = jointnames;
  joint_state_msg.position.resize(jointnames.size(),0.0); 
  joint_state_msg.velocity.resize(jointnames.size(),0.0);
  joint_state_msg.effort.resize(jointnames.size(),0.0);

  initial_joints.resize(DOF,0.0);
  joint_pos.resize(DOF,0.0);
  joint_vel.resize(DOF,0.0);

  this->get_node_base_interface()->get_context()->add_pre_shutdown_callback(std::bind( &RobotControlNode::safe_shutdown, this)); // Adds safe_shutdown as callback before shutting down, e.g. with ctrl+c. This methods returns rclcpp::OnShutdownCallbackHandle shutdown_cb_handle

  

}

void RobotControlNode::safe_shutdown(){
  // RCUTILS_LOG_INFO_NAMED(get_name(), "Program shutting down safely.");

  std::cout << "Program shutting down safely." << std::endl;

  std::cout << "------------ Latency statistics --------------" << std::endl;
  // std::cout << "Sum latency: " << sum_latency << " ns" << std::endl;
  std::cout << "Max latency: " << max_latency << " ns" << std::endl;
  std::cout << "Min latency: " << min_latency << " ns" << std::endl;
  std::cout << "Number of samples: " << num_samples << std::endl;
  std::cout << "Average latency: " << (num_samples > 0 ? static_cast<double>(sum_latency) / num_samples : 0.0) << " ns" << std::endl;
 
  std::cout << "------------ Consumer acquire-release statistics --------------" << std::endl;
  // std::cout << "Sum time acquire-release: " << sum_time_acquire_release << " ns" << std::endl;
  std::cout << "Max time acquire-release: " << max_time_acquire_release << " ns" << std::endl;
  std::cout << "Min time acquire-release: " << min_time_acquire_release << " ns" << std::endl;
  std::cout << "Number of samples acquire-release: " << num_samples_acquire_release << std::endl;
  std::cout << "Average time acquire-release: " << (num_samples_acquire_release > 0 ? static_cast<double>(sum_time_acquire_release) / num_samples_acquire_release : 0.0) << " ns" << std::endl;
  // rclcpp::shutdown(); 
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // rclcpp::shutdown(); 
}

void RobotControlNode::construct(double periodicity_val,  triple_buffer_setpoints_type *triple_buffer_setpoints) {

    triple_buffer_setpoints_ = triple_buffer_setpoints;

    sum_latency = 0ll;
    max_latency = 0ll;
    min_latency = 0ll;
    num_samples = 0ll;

    sum_time_acquire_release = 0ll;
    max_time_acquire_release = 0ll;
    min_time_acquire_release = 0ll;
    num_samples_acquire_release = 0ll;

    periodicity = periodicity_val; //Expressed in seconds
    int periodicity_ms =  static_cast<int>(periodicity_val * 1000); // Convert seconds to milliseconds

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1); //queue_size 1 so that it sends the latest always
    timer_ = this->create_wall_timer(std::chrono::milliseconds(periodicity_ms), std::bind(&RobotControlNode::update, this));
  

    // feedback_ptr->mtx.lock();
    
    // feedback_ptr->joint.pos.data = {0, 0, 0, 0, 0, 0}; // Initialize joint positions to zero
    // feedback_ptr->joint.pos.is_available = true;

    // feedback_ptr->mtx.unlock();
}
void RobotControlNode::consume_data() {
  auto begintime = std::chrono::steady_clock::now();
  // volatile float savepoint = 0.0f; // Dummy variable to ensure the compiler does not optimize away the acquire operation

  

	// if (TRIPLE_BUFFER_ACQUIRE_SUCCEEDED == ret) {
  //   setpoint_ptr = (SetpointMsg *)get_data_chunk_from_port(port_setpoints);
  if(triple_buffer_setpoints_->read(setpoint_local_copy)){
    // for (unsigned int i=0; i<DOF; ++i) {
    //   savepoint = setpoint_local_copy.load1.data[i];
    //   savepoint = setpoint_local_copy.load2.data[i];
    //   savepoint = setpoint_local_copy.load3.data[i];
    //   savepoint = setpoint_local_copy.load4.data[i];
    //   savepoint = setpoint_local_copy.load5.data[i];
    //   savepoint = setpoint_local_copy.load6.data[i];
    //   savepoint = setpoint_local_copy.load7.data[i];
    //   savepoint = setpoint_local_copy.load8.data[i];
    //   savepoint = setpoint_local_copy.load9.data[i];
    //   savepoint = setpoint_local_copy.load10.data[i];
    // }
  
    for (unsigned int i=0; i<DOF; ++i) {
      joint_vel[i] = setpoint_local_copy.data[i];
    }
    // }
    // else{
    //   // std::cout << "Error acquiring from triple buffer consumer port." << std::endl;
    //   // exit(1);
    // }
  
    auto now = std::chrono::steady_clock::now();
    long long latency_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - setpoint_local_copy.timestamp).count();
    
    auto endtime = std::chrono::steady_clock::now();
    long long timediff = std::chrono::duration_cast<std::chrono::nanoseconds>(endtime - begintime).count();
  
    // printf("Latency: %lld ns\n", (long long)latency_ns);
  
    sum_latency+= latency_ns;
    num_samples++;
    if (latency_ns > max_latency) {
      max_latency = latency_ns;
    }
    if (min_latency == 0 || latency_ns < min_latency) {
      min_latency = latency_ns;
    }
  
    sum_time_acquire_release += timediff;
    num_samples_acquire_release++;
    if (timediff > max_time_acquire_release) {
      max_time_acquire_release = timediff;
    }
    if (min_time_acquire_release == 0 || timediff < min_time_acquire_release) {
      min_time_acquire_release = timediff;
    }
  }

  
    



    
  for (unsigned int i=0; i<DOF; ++i) {
      
    // double k2 = joint_pos[i] + 0.5 * (setpoint_ptr->velocity.data[i] - joint_pos[i]); // midpoint
    // double k3 = setpoint_ptr->velocity.data[i] + (setpoint_ptr->velocity.data[i] - joint_pos[i]);       // linear extrapolation
    
    // joint_pos[i] += (periodicity / 6.0) * (setpoint_ptr->velocity.data[i] + 4*k2 + k3);
    
    joint_pos[i] += joint_vel[i]*periodicity; //simple integration

    // std::cout << "periodicity: " << periodicity << std::endl;


    // joint_pos[i] += setpoint_ptr->velocity.data[i]*0.0000005; //simple integration
    // joint_pos[i] += 0.00000001; //test
    // feedback_ptr->joint.pos.data[i] = joint_pos[i];
  }
}

void RobotControlNode::update() {
   // The consumer node acts as the ROBOT DRIVER
    // auto joint_state_msg = std::make_shared<sensor_msgs::msg::JointState>();

    // feedback_ptr->mtx.lock();
    // setpoint_ptr->mtx.lock();

    // for (unsigned int i=0; i<DOF; ++i) {
    //   joint_vel[i] = setpoint_ptr->velocity.data[i];
    //   joint_pos[i] += joint_vel[i]*periodicity; //simple integration
    //   // joint_pos[i] += setpoint_ptr->velocity.data[i]*0.0000005; //simple integration
    //   // joint_pos[i] += 0.00000001; //test
    //   feedback_ptr->joint.pos.data[i] = joint_pos[i];
    // }

    // setpoint_ptr->velocity.fs = etasl::OldData;

    // setpoint_ptr->mtx.unlock();
    // feedback_ptr->mtx.unlock();

    consume_data();



    joint_state_msg.header.stamp = this->now(); // Set the timestamp to the current time
    // joint_state_msg.name = jointnames; //No need to update if defined in configuration
      
    for (unsigned int i=0;i<DOF;++i) {
      // Populate the joint state message according to your robot configuration
      joint_state_msg.position[i]  = joint_pos[i];
      joint_state_msg.velocity[i]  = joint_vel[i];
      joint_state_msg.effort[i]  = 0.0;
    } 

    // joint_positions[DOF - 1] = amplitude*sin(2*3.1416*frequency*time) + initial_joints[DOF - 1];
    // joint_velocities[DOF - 1] = amplitude*2*3.1416*frequency*cos(2*3.1416*frequency*time);
    double amplitude = 1.0;
    double frequency = 0.5;
    joint_state_msg.position[4]  = amplitude*sin(2.0d*3.1416d*frequency*time) + initial_joints[4];
    joint_state_msg.velocity[4]  = amplitude*2.0d*3.1416d*frequency*cos(2.0d*3.1416d*frequency*time);

    time += periodicity; // Update the time for the next iteration
    
    // std::cout << "Time: " << time << std::endl;
    publisher_->publish(joint_state_msg);
}

// Producer::Producer(double periodicity_val,  triple_buffer_setpoints_type *triple_buffer_setpoints):
//  periodicity(periodicity_val) //Expressed in seconds
// , time(0.0)
// , amplitude(1.0) // Amplitude of the sine wave
// , frequency(0.5) // Frequency of the sine wave
// {
//   triple_buffer_setpoints_ = triple_buffer_setpoints;

//   joint_positions.resize(DOF,0.0);
//   joint_velocities.resize(DOF,0.0);
//   initial_joints.resize(DOF,0.0);

//   sum_time_acquire_release = 0ll;
//   max_time_acquire_release = 0ll;
//   min_time_acquire_release = 0ll;
//   num_samples_acquire_release = 0ll;

//   // SetpointMsg initialized_setpoint(DOF); // Assuming DOF is the required constructor argument

      



// }

// void Producer::produce_data(){

//   auto begintime = std::chrono::steady_clock::now();

//   // ret = acquire_from_triple_buffer_port(port_setpoints);
// 	// if (TRIPLE_BUFFER_ACQUIRE_SUCCEEDED == ret) {
//     // setpoint_local_copy.velocity.fs = etasl::NewData;
//     // std::cout << "size: " << setpoint_local_copy.velocity.data.size() << std::endl;
//     for (unsigned int i=0; i<DOF; ++i) {
//       setpoint_local_copy.data[i] = joint_velocities[i];
//     }

//     // for (unsigned int i=0; i<DOF; ++i) {
//     //   setpoint_local_copy.load1.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load2.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load3.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load4.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load5.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load6.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load7.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load8.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load9.data[i] = joint_velocities[i];
//     //   setpoint_local_copy.load10.data[i] = joint_velocities[i];
//     // }

//     setpoint_local_copy.timestamp = std::chrono::steady_clock::now();
    
//     triple_buffer_setpoints_->write(setpoint_local_copy); // Write the data to the triple buffer

//   // }
//   // else{
//   //   std::cout << "Error acquiring from triple buffer producer port." << std::endl;
//   //   // exit(1);
//   // }


//   auto endtime = std::chrono::steady_clock::now();

// 	// assert(TRIPLE_BUFFER_RELEASE_SUCCEEDED == ret);

//   long long timediff = std::chrono::duration_cast<std::chrono::nanoseconds>(endtime - begintime).count();

//   sum_time_acquire_release += timediff;
//   num_samples_acquire_release++;
//   if (timediff > max_time_acquire_release) {
//     max_time_acquire_release = timediff;
//   }
//   if (min_time_acquire_release == 0 || timediff < min_time_acquire_release) {
//     min_time_acquire_release = timediff;
//   }

// }

// void Producer::update(volatile std::atomic<bool>& stopFlag)
// {
//         // The PRODUCER acts as the ETASL
//         joint_positions[DOF - 1] = amplitude*sin(2.0d*3.1416d*frequency*time) + initial_joints[DOF - 1];
//         joint_velocities[DOF - 1] = amplitude*2.0d*3.1416d*frequency*cos(2.0d*3.1416d*frequency*time);

//         produce_data();

//         // jpos_etasl += jvel_etasl*(periodicity_param/1000.0);  // or replace with reading joint positions from real robot
//         // fpos_etasl += fvel_etasl*(periodicity_param/1000.0);  // you always integrate feature variables yourself
        
//         // setpoint_ptr->mtx.lock();
        
//         // setpoint_ptr->velocity.fs = etasl::NewData;
//         // for (unsigned int i=0; i<DOF; ++i) {
//         //   setpoint_ptr->velocity.data[i] = joint_velocities[i];
//         // }
//         // setpoint_ptr->mtx.unlock();

//         time += (periodicity);
//   }

// void Producer::finalize()
// {
//   std::cout << "Producer finalize() called =======================" << std::endl;

//   std::cout << "------------ Producer acquire-release statistics --------------" << std::endl;
//   // std::cout << "Sum time acquire-release: " << sum_time_acquire_release << " ns" << std::endl;
//   std::cout << "Max time acquire-release: " << max_time_acquire_release << " ns" << std::endl;
//   std::cout << "Min time acquire-release: " << min_time_acquire_release << " ns" << std::endl;
//   std::cout << "Number of samples acquire-release: " << num_samples_acquire_release << std::endl;
//   std::cout << "Average time acquire-release: " << (num_samples_acquire_release > 0 ? static_cast<double>(sum_time_acquire_release) / num_samples_acquire_release : 0.0) << " ns" << std::endl;
//         // jpos_etasl += jvel_etasl*(periodicity_param/1000.0);  // or replace with reading joint positions from real robot
//         // fpos_etasl += fvel_etasl*(periodicity_param/1000.0);  // you always integrate feature variables yourself
//         // time += (periodicity_param/1000.0);       // idem.
// }


int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    double periodicity_producer = 0.005; // in seconds
    double periodicity_consumer = 0.01; // in seconds

    // double periodicity_producer = 0.0001; // in seconds 
    // double periodicity_consumer = 0.0001; // in seconds //Apparently ROS2 timer cannot keep such small time, so it goes faster


    // etasl::SetpointMsg initial_value(6);
    // boost::lockfree::spsc_value<etasl::SetpointMsg> triple_buffer_setpoints(initial_value);
    boost::lockfree::spsc_value< robotdrivers::JointData<7> > triple_buffer_setpoints;
    boost::lockfree::spsc_value< std::vector<double> > tb_test;

    // JointData init(6, 0.0); // 6 joints
    // triple_buffer.write(init);
    
    
    // Initialize node
    rclcpp::init(argc, argv);
    std::shared_ptr<RobotControlNode> my_consumer_node = std::make_shared<RobotControlNode>();


    my_consumer_node->construct(periodicity_consumer, &triple_buffer_setpoints);
    // Producer my_producer(feedback_shared_ptr.get(), setpoint_shared_ptr.get(), periodicity_producer);
    // rclcpp::executors::StaticSingleThreadedExecutor executor;

    std::shared_ptr<Producer> my_producer_ptr = std::make_shared<Producer>(periodicity_producer, &triple_buffer_setpoints);

    std::atomic<bool> stopFlag(false);

    std::shared_ptr<t_manager::thread_t> thread_str_driver = std::make_shared<t_manager::thread_t>();

    
    thread_str_driver->periodicity = std::chrono::nanoseconds(static_cast<long long>(periodicity_producer * 1E9)); //*1E9 to convert seconds to nanoseconds
    thread_str_driver->update_hook = std::bind(&Producer::update, my_producer_ptr, std::ref(stopFlag));
    thread_str_driver->finalize_hook = std::bind(&Producer::finalize, my_producer_ptr);
    
  

    std::thread driver_thread(t_manager::do_thread_loop, thread_str_driver, std::ref(stopFlag));
    // t_manager::setScheduling(driver_thread, SCHED_FIFO, 90);
    // driver_thread.detach();// Avoids the main thread to block. See spin() + stopFlag mechanism below.

    rclcpp::ExecutorOptions options;
    options.context = my_consumer_node->get_node_base_interface()->get_context(); //necessary to avoid unexplainable segmentation fault from the ros rclcpp library!!!
    rclcpp::executors::SingleThreadedExecutor executor(options);

    executor.add_node(my_consumer_node);
    executor.spin(); //This method blocks!
    
    stopFlag.store(true); //Stops execution of driver_thread after executor is interrupted with ctr+c signal

    driver_thread.join();//Blocks until driver_thread stops, after the stopFlag is set to true

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); //Needed for the robotdriver thread to stop properly before calling shutdown. Otherwise segmentation fault is observed. This is because shutdown deletes something from the robotdriver as now ros2 pluginlib is being used.
    executor.remove_node(my_consumer_node->get_node_base_interface());

    std::cout << "Program terminated correctly." << std::endl;
    rclcpp::shutdown();

  return 0;
}