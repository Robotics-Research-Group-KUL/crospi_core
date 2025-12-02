//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Erwin Aertbeliën
//  email: <erwin.aertbelien@kuleuven.be>
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
#include <filesystem>
#include <vector>
#include <fmt/format.h>
#include <regex>
#include <jsoncpp/json/json.h>

namespace etasl {

class URI {
    std::cmatch what;
    const std::string uri_;
    bool result;

public:
    URI(const std::string& uri);
    URI(const URI& obj); // if you copy uri member, references in what are invalidated, so perform the match again
    URI(URI&& obj) = default; // taking over both what and uri members is just fine
    bool matched() const
    {
        return result;
    }
    const std::string_view uri() const;
    const std::string_view protocol() const;
    const std::string_view domain() const;
    const std::string_view port() const;
    const std::filesystem::path path() const;
    const std::string_view query() const;
    const std::string_view fragment() const;
};

/**
 * @brief Encapsulates a search path and a find operation on it.
 *
 */
class SearchPath {

public:
    std::vector<std::filesystem::path> spth;
    std::string expandedspth;
    /**
     * @brief empty constructor
     */
    SearchPath() { }

    /**
     * @brief Constructor
     * @param pth std::string representing  a search path.  The different paths to be checked are
     * by a ":".  Optionally each path can be quoted using "...".  Before processing the
     * string will be interpolated, i.e. ${environment variable}, $(command) and $[ros package] will
     * be understood.
     * @param include_current if true the current directory will be added to the search path.
     */
    SearchPath(const std::string& pth, bool include_current = false);

    /**
     * @brief finds a file back using the search path
     * @param fn file described by a relative or absolute name
     * @return empty if not found, path if file exists in search paths.
     */
    std::filesystem::path find(const std::string& fn);
};

/**
 * @brief finds local alternatives for an uri in the search path
 * @param sp SearchPath object describing the search path
 * @param uri URI to be examined
 * @param path_components_used if -1, all parts of the local path in the URI are returned.  if >=0, then
 *                             the last path_components_used parts are retained. For example, for
 *                              "http://example.com/data/schema/etasl/frame.json" with path_components=0 retains
 *                              "frame.json", =1 retains "etasl/frame.json", =-1 retains "data/schema/etasl/frame.json"
 * @return
 */
std::filesystem::path find_local_alternative(SearchPath& sp, const URI& uri, int path_components_used = -1);



/**
 * @brief keeps the last N parts of a path (file is also counted). If there are less than N parts
 * all available paths are kept.
 * @param path path to analyze
 * @param n number of parts to keep
 * @return adapted path
 */
std::filesystem::path keepLastNPartsOfPath(const std::filesystem::path& path, size_t n);


/**
 * @brief Executes a command and captures the output
 * @param cmd const char* command to be executed in the shell
 * @return captured output, can throws an etasl_error, with etasl_error::STRING_INTERPOLATION_ERROR
 **/
std::string execute_with_captured_output(const char* cmd);

/**
 * @brief trims away space-like characters in front and at the end of the given string.
 * @param s std::string string to trim.
 * @param firstline bool, false by default, determines whether to retain only the first line of the string
 * @return trimmed string
 * @details The space-like characters are " " , "\\t", "\\n"
 */
std::string trim(const std::string& s, bool firstline = false);

/**
 * @brief Interpolates a string.
 *
 * @details Offers a number of substitutions where:
 * "${envvar}" is replaced with the value of
 *      the environment variable envvar if it exists;
 * "$[package]" is replaced by the path the the ros package [package].
 * "$(cmd)" executes a system command (in the shell) and replaces itself with the trimmed output
 * of that system command.
 * @param s std::string
 * @result interpolated string
 *
 **/
std::string string_interpolate(const std::string& s);

} // namespace etasl

template <>
struct fmt::formatter<Json::Value> {
    // parse the format string for this type:
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
        // return position of } (hopefully there)
    }
    // format by always writing its value:
    auto format(const Json::Value& obj, fmt::format_context& ctx) const
    {
        std::stringstream ss;
        ss << obj;
        return fmt::format_to(ctx.out(), "{}", ss.str());
    }
};
template <>
struct fmt::formatter<std::filesystem::path> {
    // parse the format string for this type:
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
        // return position of } (hopefully there)
    }
    // format by always writing its value:
    auto format(const std::filesystem::path& obj, fmt::format_context& ctx) const
    {
        std::stringstream ss;
        ss << obj;
        return fmt::format_to(ctx.out(), "{}", ss.str());
    }
};

template <>
struct fmt::formatter<etasl::URI> {
    // parse the format string for this type:
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
        // return position of } (hopefully there)
    }
    // format by always writing its value:
    auto format(const etasl::URI& urip, fmt::format_context& ctx) const
    {
        return fmt::format_to(ctx.out(),
            R"(uri :                    {}
match :                  {}
protocol :               {}
domain :                 {}
port :                   {}
path :                   {}
query :                  {}
fragment :               {}
)",
            urip.uri(), urip.matched(), urip.protocol(), urip.domain(), urip.port(), urip.path(), urip.query(), urip.fragment());
    }
};

template <>
struct fmt::formatter<etasl::SearchPath> {
    // parse the format string for this type:
    constexpr auto parse(fmt::format_parse_context& ctx)
    {
        return ctx.begin();
        // return position of } (hopefully there)
    }
    // format by always writing its value:
    auto format(const etasl::SearchPath& obj, fmt::format_context& ctx) const
    {
        return fmt::format_to(ctx.out(), "SearchPath(\n{}\n)  ", fmt::join(obj.spth, ",\n"));
    }
};
