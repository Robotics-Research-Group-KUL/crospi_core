
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

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a double
    if (!is_key_found || !current.isDouble()) {
        trigger_error(key_path, key, "double");
    }

    return current.asDouble();
}

std::string JsonChecker::asString(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a string
    if (!is_key_found || !current.isString()) {
        trigger_error(key_path, key, "string");
    }
    return current.asString();
}

bool JsonChecker::asBool(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a bool
    if (!is_key_found || !current.isBool()) {
        trigger_error(key_path, key, "bool");
    }

    return current.asBool();
}

int JsonChecker::asInt(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a bool
    if (!is_key_found || !current.isInt()) {
        trigger_error(key_path, key, "int");
    }

    return current.asInt();
}

unsigned int JsonChecker::asUInt(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a bool
    if (!is_key_found || !current.isUInt()) {
        trigger_error(key_path, key, "unsigned int");
    }

    return current.asUInt();
}

size_t JsonChecker::asUInt64(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a bool
    if (!is_key_found || !current.isUInt64()) {
        trigger_error(key_path, key, "UInt64");
    }

    return current.asUInt64();
}

Json::Value JsonChecker::asArray(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    // Check if the current field (end of path) is a array
    if (!is_key_found || !current.isArray()) {
        trigger_error(key_path, key, "array");
    }
    return current;
}

bool JsonChecker::is_member(Json::Value const& json_param, const std::string& key_path){
    std::stringstream ss(key_path);
    std::string key;
    Json::Value current = json_param;

    bool is_key_found = getPath(current, key_path, key);

    return is_key_found;
}

bool JsonChecker::getPath(Json::Value & current, const std::string& key_path, std::string& key){
    std::stringstream ss(key_path);
    while (std::getline(ss, key, '/')) {
        if (current.isMember(key)) {
            current = current[key]; // Move to the next nested level
        } else {
            // Key not found in the JSON hierarchy
            return false;
        }
    }
    return true;
}


void JsonChecker::trigger_error(const std::string& keypath, const std::string& key, const std::string& type){
    std::string message = "The requested parameter `" + key +"` inside the given path `" + keypath + "` was not defined in the provided JSON configuration file OR is not of type `" + type + "`.";
    // RCLCPP_INFO(node->get_logger(), message.c_str());
    error_callback(message);
}




} // namespace etasl




