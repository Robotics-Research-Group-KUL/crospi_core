#include "etasl_node_utils/tfinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

namespace etasl {


TFInputHandler::TFInputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    int _nroftries,
    geometry_msgs::msg::Transform  _default_msg,
    double cache_time_,
    const std::string& _when_unpublished,
    const std::string& _target_frame,
    const std::string& _source_frame,
    const std::string& _varname)
    : node(_node)
    , nroftries(_nroftries)
    , default_msg(_default_msg)
    , cache_time(tf2::durationFromSec(cache_time_))
    , tf_buffer(_node->get_clock(), cache_time)
    , tf_listener(tf_buffer)
    , target_frame(_target_frame)
    , source_frame(_source_frame)
    , varname(_varname)
    , when_unpublished(_when_unpublished)
    , frame(KDL::Vector::Zero()) //Identity rotation and zero position
    , counter(0)
    , initialized(false)
    , activated(false)
{
    // using namespace std::chrono_literals;
    // using namespace std::placeholders;
    // auto cb = std::bind(&TFInputHandler::on_new_message, this, _1);
    // auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);
    // sub = node->create_subscription<MsgType>(name, qos, cb);
    // if (!sub) {
    //   RCLCPP_FATAL(node->get_logger(), "Could not create subscriber associated to the TFinputhandler.");
    // }

    

    // cbg = node->create_callback_group()
    // sub = node->create_subscription<MsgType>(_topic_name, rclcpp::QoS(10), cb);

    input_msg.fs = NoData;
    input_msg.data = default_msg;
    // transform_stamped = default_msg;
    
}

bool TFInputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& jpos,
    Eigen::VectorXd& fpos)
{


    if(input_msg.fs == NoData && when_unpublished=="use_default"){
        input_msg.fs = NewData; //Fakes new Data 
        input_msg.data = default_msg;
    }
    else if(input_msg.fs == NoData && when_unpublished=="throw_error"){
        input_msg.data = default_msg;
    }

    initialized = true;
    RCLCPP_INFO(node->get_logger(), "TFinputhandler has been initialized correctly ________________________");

    return true;
}

void TFInputHandler::on_new_message(const TFInputHandler::MsgType& msg)
{

}

void TFInputHandler::update(
    double time,
    const std::vector<std::string>& jnames,
    Eigen::VectorXd& jpos,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& fpos)
{
    // if(activated){
    //     std::cout << "it is activated ////////////////////////////////" << std::endl;
    //     std::cout << "fs: " << input_msg.fs << std::endl;
    //     std::cout << "when_unpublished: " << when_unpublished  << std::endl;
    // }
    input_msg.fs = NewData;
    // input_msg.data = msg;

    if (tf_buffer.canTransform(target_frame, source_frame, tf2::TimePointZero))
    {
        try
        {
            transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            input_msg.data = transform_stamped.transform;
        }
        catch (const tf2::TransformException &ex)
        {
            //Include source_frame in warning:
            RCLCPP_WARN(node->get_logger(), "Could not transform TF from %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        }
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "TF transform from %s to %s not yet available", source_frame.c_str(), target_frame.c_str());
    }



    if(activated && input_msg.fs == NoData && when_unpublished=="throw_error"){

        if (counter > nroftries){
            RCLCPP_ERROR(node->get_logger(), fmt::format("TFinputhandler could not initialize since no TF transformation could be done and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler to use_default if desired.",getName()).c_str());
            rclcpp::shutdown();
        }
        else{
            counter++;
            // std::cout << "Counter +++++: " << counter<< std::endl;
        }
        return;
    }
    else if(activated && input_msg.fs == NewData){
        consume_data(true); 
    }
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHander::update()");
}

void TFInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The TFInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The TFInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    inp = ctx->getInputChannel<KDL::Frame>(varname); 
    if (!inp) {
        RCLCPP_INFO(node->get_logger(), fmt::format("No twist input channel with the name {} was created in the LUA specification, and thus topic {} will be ignored. If desired, this can be created with with createInputChannelTwist function.",varname, getName()).c_str());
        activated = false;
    } 
    else {
        // inps->setJacobian(time_ndx, default_value_kdl );
        consume_data(false); 
        activated = true;
    }
}

void TFInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    inp = nullptr;
    activated=false;
}

const std::string& TFInputHandler::getName() const
{
    return "tfinputhandler";
}

void TFInputHandler::consume_data(const bool& make_old_data)
{
    if (inp) {
        //TODO: Replace with frame (KDL::Frame)
        frame.p[0] = input_msg.data.translation.x;
        frame.p[1] = input_msg.data.translation.y;
        frame.p[2] = input_msg.data.translation.z;
        frame.M = KDL::Rotation::Quaternion(
            input_msg.data.rotation.x,
            input_msg.data.rotation.y,
            input_msg.data.rotation.z,
            input_msg.data.rotation.w);

        // twist.vel[0] = input_msg.data.linear.x;
        // twist.vel[1] = input_msg.data.linear.y;
        // twist.vel[2] = input_msg.data.linear.z;
        // twist.rot[0] = input_msg.data.angular.x;
        // twist.rot[1] = input_msg.data.angular.y;
        // twist.rot[2] = input_msg.data.angular.z;

        // inp->setValue(twist);
        // if (make_old_data){
        //     input_msg.fs = OldData;
        // }
        inp->setValue(frame);
        if (make_old_data){
            input_msg.fs = OldData;
        }
    }
}

TFInputHandler::~TFInputHandler() {

};



} // namespace etasl
