#ifndef ETASL_ROS2_SIMPLE_NODE_HPP
#define ETASL_ROS2_SIMPLE_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp" // Include Time message header


#include <expressiongraph/context.hpp>
#include <expressiongraph/context_scripting.hpp>
#include <expressiongraph/solver_registry.hpp>
#include <expressiongraph/solver_factory_qpoases.hpp>
#include <expressiongraph/defaultobserver.hpp>
//#include <expressiongraph/solver_factory_hqp.hpp>
#include <string>
#include <algorithm>
#include <iomanip>
#include <boost/chrono.hpp>
#include "featurevariableinitializer.hpp"

#include "IO_handlers.hpp"


using namespace KDL;
using namespace Eigen;


class etaslNode : public rclcpp::Node
{

    public:
        etaslNode();
        void setAngle(double p_angle);
        void publishJointState();
        void configure_etasl();

        Context::Ptr get_ctx();
        boost::shared_ptr<solver> get_slv();
        boost::shared_ptr<eTaSL_OutputHandler>  get_output_handler();
        boost::shared_ptr<eTaSL_InputHandler> get_input_handler();
        int get_periodicity_param();
        VectorXd get_fpos();
        VectorXd get_jpos();
        double get_time();
        std::string get_outpfilename();
        std::string get_etasl_fname();



    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        size_t count_;
        double angle;
        int periodicity_param; //Expressed in milliseconds
        double time;
        sensor_msgs::msg::JointState joint_state_msg;
        Context::Ptr ctx;
        boost::shared_ptr<solver> slv;

        boost::shared_ptr<eTaSL_OutputHandler> oh;
        boost::shared_ptr<eTaSL_InputHandler> ih;


        // eTaSL_OutputHandler oh;
        // eTaSL_InputHandler ih;

        VectorXd fpos;
        VectorXd jpos;

          
        std::string outpfilename;
        std::string fname;


        





};




#endif