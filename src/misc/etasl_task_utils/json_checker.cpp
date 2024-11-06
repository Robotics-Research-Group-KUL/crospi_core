
#include "etasl_task_utils/json_checker.hpp"
#include <fmt/format.h>

namespace etasl {

    JsonChecker::JsonChecker(
        std::function<void(const std::string&)> error_cb
        ): error_callback(error_cb)
    {

    }


double JsonChecker::asDouble(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    // Split the path by '/' and traverse
    while (std::getline(ss, key, '/')) {
        if (current.isMember(key)) {
            current = current[key]; // Move to the next nested level
        } else {
            // Key not found in the JSON hierarchy
            trigger_error(key_path, key, "double");
            return 0;
        }
    }
    // Check if the final field is a double
    if (!current.isDouble()) {
        trigger_error(key_path, key, "double");
        return 0;
    }

    return current.asDouble();
}

std::string JsonChecker::asString(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    // Split the path by '/' and traverse
    while (std::getline(ss, key, '/')) {
        if (current.isMember(key)) {
            current = current[key]; // Move to the next nested level
        } else {
            // Key not found in the JSON hierarchy
            trigger_error(key_path, key, "string");
            return 0;
        }
    }
    // Check if the final field is a string
    if (!current.isString()) {
        trigger_error(key_path, key, "string");
        return 0;
    }
    return current.asString();
}

void JsonChecker::trigger_error(const std::string& keypath, const std::string& key, const std::string& type){
    std::string message = "The requested parameter `" + key +"` inside the given path `" + keypath + "` was not defined in the provided JSON configuration file OR is not of type `" + type + "`.";
    // RCLCPP_INFO(node->get_logger(), message.c_str());
    error_callback(message);
}




} // namespace etasl




