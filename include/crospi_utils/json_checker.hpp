#pragma once

// #include <vector>
// #include <mutex>
// #include <chrono>
#include <functional>
#include "jsoncpp/json/json.h"
// #include "rclcpp_lifecycle/lifecycle_node.hpp"

// #include <algorithm>
// #include <atomic>
// #include <thread>
// #include <time.h>


namespace etasl {

    class JsonChecker
    {

        public:
            /// @brief Constructs the JSON Checker, providing an error callback that will be triggered when a specific parameter is not part of the JSON
            /// @param error_cb Error callback that is triggered when a specific parameter is not part of the JSON
            JsonChecker(std::function<void(const std::string&)> error_cb);


            /// @brief Gets a JSON double by path, and validates if it exists and is the correct type. If it does not exist or is the wrong type, the defined error_callback will be triggered.
            /// @param json_param JSON::Value from which the parameter will be obtained from
            /// @param key_path std::string that defines a the path of the desired value. Each level should be separated with / (without starting with /).
            /// @return the found value of the parameter as a double
            double asDouble(Json::Value const& json_param, const std::string& key_path);
            
            std::string asString(Json::Value const& json_param, const std::string& key_path);
            bool asBool(Json::Value const& json_param, const std::string& key_path);
            int asInt(Json::Value const& json_param, const std::string& key_path);
            unsigned int asUInt(Json::Value const& json_param, const std::string& key_path);
            size_t asUInt64(Json::Value const& json_param, const std::string& key_path);
            Json::Value asArray(Json::Value const& json_param, const std::string& key_path);
            bool is_member(Json::Value const& json_param, const std::string& key_path);

            bool getPath(Json::Value & current, const std::string& key_path, std::string& key);


        private:
            void trigger_error(const std::string& keypath, const std::string& key, const std::string& type);
            
            std::function<void(const std::string&)> error_callback;

    };

} // namespace etasls
