//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.

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

    /**
     * @brief Utility class for type-safe extraction and validation of JSON configuration values.
     *
     * JsonChecker provides a convenient interface for reading configuration parameters
     * from a JSON object with automatic type checking and error reporting. It supports
     * nested key paths (separated by `/`) to access values deep within a JSON structure.
     *
     * When a requested parameter is missing or has the wrong type, the configured error
     * callback is invoked with a descriptive message, allowing the caller to handle
     * configuration errors consistently (e.g., logging, throwing exceptions, or
     * preventing node activation).
     *
     * Typical usage in a Crospi plugin's construct() method:
     * @code
     * ip_address = jsonchecker->asString(config, "ip_address");
     * periodicity = jsonchecker->asDouble(config, "periodicity");
     * @endcode
     */
    class JsonChecker
    {

        public:
            /**
             * @brief Construct a JsonChecker with an error callback.
             *
             * The error callback is invoked whenever a requested parameter is missing,
             * has the wrong type, or the path cannot be resolved.
             *
             * @param error_cb A callable that receives a descriptive error message string.
             */
            JsonChecker(std::function<void(const std::string&)> error_cb);


            /**
             * @brief Extract a double value from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of numeric type.
             * If validation fails, the error callback is triggered and 0.0 is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`
             *                 (e.g., "robot/periodicity"). Do not start with `/`.
             * @return The extracted double value, or 0.0 on error.
             */
            double asDouble(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Extract a string value from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of string type.
             * If validation fails, the error callback is triggered and an empty string is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`.
             * @return The extracted string value, or an empty string on error.
             */
            std::string asString(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Extract a boolean value from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of boolean type.
             * If validation fails, the error callback is triggered and false is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`.
             * @return The extracted boolean value, or false on error.
             */
            bool asBool(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Extract an integer value from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of integer type.
             * If validation fails, the error callback is triggered and 0 is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`.
             * @return The extracted integer value, or 0 on error.
             */
            int asInt(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Extract an unsigned integer value from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of unsigned integer type.
             * If validation fails, the error callback is triggered and 0 is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`.
             * @return The extracted unsigned integer value, or 0 on error.
             */
            unsigned int asUInt(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Extract a size_t value from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of unsigned integer type.
             * If validation fails, the error callback is triggered and 0 is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`.
             * @return The extracted size_t value, or 0 on error.
             */
            size_t asUInt64(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Extract a JSON array from the JSON configuration.
             *
             * Validates that the parameter exists at the given path and is of array type.
             * If validation fails, the error callback is triggered and an empty Json::Value is returned.
             *
             * @param json_param The JSON object or sub-object to extract the value from.
             * @param key_path Path to the desired value, with levels separated by `/`.
             * @return The extracted JSON array, or an empty Json::Value on error.
             */
            Json::Value asArray(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Check whether a key path exists in the JSON configuration.
             *
             * Does not trigger the error callback if the key is missing — simply returns false.
             *
             * @param json_param The JSON object or sub-object to check.
             * @param key_path Path to check, with levels separated by `/`.
             * @return true if the key path exists, false otherwise.
             */
            bool is_member(Json::Value const& json_param, const std::string& key_path);

            /**
             * @brief Resolve a key path within the JSON structure.
             *
             * Navigates through the JSON object following the `/`-separated key path.
             * On success, `current` is set to the JSON value at the leaf of the path
             * and `key` contains the final (leaf) key name. This is a helper used
             * internally by the as*() methods.
             *
             * @param[in,out] current The JSON object to start navigation from. On success,
             *                        set to the value at the leaf key.
             * @param key_path Path to resolve, with levels separated by `/`.
             * @param[out] key The final (leaf) key name extracted from the path.
             * @return true if the path was fully resolved, false if any intermediate
             *         key was not found.
             */
            bool getPath(Json::Value & current, const std::string& key_path, std::string& key);


        private:
            /**
             * @brief Invoke the error callback with a formatted message.
             *
             * Constructs a descriptive error message indicating that a parameter
             * at the given path/key was not found or had the wrong expected type.
             *
             * @param keypath The full key path being accessed.
             * @param key The specific key that caused the error.
             * @param type The expected type name (e.g., "double", "string").
             */
            void trigger_error(const std::string& keypath, const std::string& key, const std::string& type);

            /**
             * @brief Callback function invoked on configuration errors.
             */
            std::function<void(const std::string&)> error_callback;

    };

} // namespace etasls
