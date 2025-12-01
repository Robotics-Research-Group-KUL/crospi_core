#ifndef crospi_core_SIMPLE_NODE_HPP
#define crospi_core_SIMPLE_NODE_HPP

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

#include "IO_handlers_deleteme.hpp"


using namespace KDL;
using namespace Eigen;


class etaslNode : public rclcpp::Node
{

    public:
        etaslNode();
        void publishJointState();
        void configure_etasl();
        void update();

        // void setJointValues(const std::vector<double>& jval, const std::vector<std::string>& jvalnames);

        Context::Ptr get_ctx();
        std::shared_ptr<solver> get_slv();
        std::shared_ptr<eTaSL_OutputHandler>  get_output_handler();
        std::shared_ptr<eTaSL_InputHandler> get_input_handler();
        int get_periodicity_param();
        VectorXd get_fpos_etasl();
        VectorXd get_jpos_etasl();
        double get_time();
        std::string get_outpfilename();
        std::string get_etasl_fname();

        void update_controller_output(Eigen::VectorXd const& jvalues);
        void update_controller_input(Eigen::VectorXd const& jvalues);

        void solver_configuration();
        void initialize_joints();
        void initialize_feature_variables();
        void configure_jointstate_msg();
        
    


    
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        size_t count_;
        int periodicity_param; //Expressed in milliseconds
        double time;
        sensor_msgs::msg::JointState joint_state_msg;
        Context::Ptr ctx;
        std::shared_ptr<solver> slv;
        SolverRegistry::Ptr solver_registry;

        std::shared_ptr<eTaSL_OutputHandler> oh;
        std::shared_ptr<eTaSL_InputHandler> ih;
        std::shared_ptr<std::ofstream > outpfile_ptr;

        std::vector< std::string > jointnames;
        std::vector<std::string> jnames_in_expr;

        


        // eTaSL_OutputHandler oh;
        // eTaSL_InputHandler ih;

        VectorXd fpos_etasl;
        VectorXd jpos_etasl;
        VectorXd jpos_ros;
        // VectorXd jpos;
        VectorXd jvel_etasl;
        VectorXd fvel_etasl;




          
        std::string outpfilename;
        std::string fname;

        std::map< std::string, int> jindex;
        std::map<std::string,int>  name_ndx;


        





};




#endif