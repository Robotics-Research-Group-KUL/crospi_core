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
            JsonChecker(std::function<void(const std::string&)> error_cb);

            double asDouble(Json::Value const& json_param, const std::string& key_name);
            std::string asString(Json::Value const& json_param, const std::string& key_name);
            // bool asBool(Json::Value const& json_param, const std::string& key_name);
            // int asInt(Json::Value const& json_param, const std::string& key_name);
            // unsigned int asUInt(Json::Value const& json_param, const std::string& key_name);
            // size_t asUInt64(Json::Value const& json_param, const std::string& key_name);
            // std::vector asVector(Json::Value const& json_param, const std::string& key_name);

        private:
            std::function<void(const std::string&)> error_callback;

            void trigger_error(const std::string& keypath, const std::string& key, const std::string& type);
    };

} // namespace etasls
