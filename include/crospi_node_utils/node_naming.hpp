//  Copyright (c) 2026 KU Leuven, Belgium
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

#include <string>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace etasl {

/// Name of the crospi node when no other name is given, e.g. with: ros2 run crospi_core crospi_node --ros-args -r __node:=<node_name>
inline const std::string DEFAULT_NODE_NAME = "crospi_node";

/**
 * @brief Returns the name under which an output topic is published by the given node.
 *
 * If the node uses the default name (DEFAULT_NODE_NAME), the topic is returned unchanged. Otherwise (e.g. to run
 * several crospi nodes simultaneously) it is prefixed with the fully qualified name of the node:
 * /<node_name>/<topic> (for both absolute and relative topic names).
 *
 * @param node node that publishes the topic
 * @param topic topic name as defined by the user
 * @return topic name to publish on
 */
inline std::string node_scoped_topic(const rclcpp_lifecycle::LifecycleNode& node, const std::string& topic)
{
    if (node.get_name() == DEFAULT_NODE_NAME) {
        return topic;
    }
    const std::string separator = (!topic.empty() && topic[0] == '/') ? "" : "/";
    // Fully qualified node name, built from the namespace and name since LifecycleNode::get_fully_qualified_name()
    // is not available in ROS 2 Humble
    const std::string node_namespace = node.get_namespace();
    const std::string fully_qualified_name = (node_namespace == "/" ? "" : node_namespace) + "/" + node.get_name();
    return fully_qualified_name + separator + topic;
}

} // namespace etasl
