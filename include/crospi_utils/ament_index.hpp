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

/**
 * @file ament_index.hpp
 * @brief ament_index_cpp functions that work on every supported ROS 2 distribution.
 *
 * ament_index_cpp replaced its string based API (get_package_share_directory(), get_package_prefix() returning a
 * std::string) by a std::filesystem based one (get_package_share_path(), get_package_prefix(name, path)), available
 * from Jazzy on, and removed the string based API in Rolling (ament_index_cpp 1.14.1), while Humble only has the
 * string based API. Use these functions instead of the ament_index_cpp ones.
 * Both throw ament_index_cpp::PackageNotFoundError when the package is not found.
 */

#include <string>

#if __has_include(<ament_index_cpp/get_package_share_path.hpp>)
#define CROSPI_AMENT_INDEX_FILESYSTEM_API
#include <filesystem>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace etasl {

/// Returns the share directory of a ROS 2 package (<prefix>/share/<package_name>).
inline std::string get_package_share_directory(const std::string& package_name)
{
#ifdef CROSPI_AMENT_INDEX_FILESYSTEM_API
    return ament_index_cpp::get_package_share_path(package_name).string();
#else
    return ament_index_cpp::get_package_share_directory(package_name);
#endif
}

/// Returns the install prefix of a ROS 2 package.
inline std::string get_package_prefix(const std::string& package_name)
{
#ifdef CROSPI_AMENT_INDEX_FILESYSTEM_API
    std::filesystem::path prefix;
    ament_index_cpp::get_package_prefix(package_name, prefix);
    return prefix.string();
#else
    return ament_index_cpp::get_package_prefix(package_name);
#endif
}

} // namespace etasl
