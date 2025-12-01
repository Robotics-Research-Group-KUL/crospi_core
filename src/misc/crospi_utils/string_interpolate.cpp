#include "crospi_utils/string_interpolate.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "crospi_utils/etasl_error.hpp"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <regex>
#include <list>

namespace etasl {

std::string basename(const std::string& pth)
{
    std::regex ex("[^/]+");
    auto ex_begin = std::sregex_iterator(pth.begin(), pth.end(), ex);
    auto ex_end = std::sregex_iterator();
    std::smatch p;
    for (auto i = ex_begin; i != ex_end; ++i) {
        p = *i;
    }
    return p.str();
}

// static const std::regex URIregex("(http|https)://([^/ :]+):?([^/ ]*)(/?[^ #?]*)\\x3f?([^ #]*)#?([^ ]*)");
static const std::regex URIregex("(?:(http://|https://)([^/ :]+):?"  "([^/ ]*))?(/?[^ :#?/][^ :#?]*)\\x3f?([^ :#]*)#?([^ :]*)");

// static const std::regex URIregex("(?:(http):|(https):)?//([^/ :]+):?([^/ ]*)(/?[^ #?]*)\\x3f?([^ #]*)#?([^ ]*)");

URI::URI(const std::string& uri)
    : uri_(uri)
{
    result = std::regex_match(uri_.c_str(), what, URIregex);
}
URI::URI(const URI& obj)
    : uri_(obj.uri_)
{
    // redo the match such that the references in what remain correct.
    result = std::regex_match(uri_.c_str(), what, URIregex);
}
const std::string_view URI::uri() const
{
    return uri_;
}
const std::string_view URI::protocol() const
{
    if (result) {
        return std::string_view(what[1].first, what[1].second - what[1].first);
    } else {
        return std::string_view();
    }
}
const std::string_view URI::domain() const
{
    if (result) {
        return std::string_view(what[2].first, what[2].second - what[2].first);
    } else {
        return std::string_view();
    }
}
const std::string_view URI::port() const
{
    if (result) {
        return std::string_view(what[3].first, what[3].second - what[3].first);
    } else {
        return std::string_view();
    }
}
const std::filesystem::path URI::path() const
{
    if (result) {
        //return std::string_view(what[4].first, what[4].second - what[4].first);
        auto p = std::string_view(what[4].first, what[4].second - what[4].first);
        return p;
    } else {
        return std::string_view();
    }
}
const std::string_view URI::query() const
{
    if (result) {
        return std::string_view(what[5].first, what[5].second - what[5].first);
    } else {
        return std::string_view();
    }
}
const std::string_view URI::fragment() const
{
    if (result) {
        return std::string_view(what[6].first, what[6].second - what[6].first);
    } else {
        return std::string_view();
    }
}

static const std::regex searchPathRegex(R"(([^\" :]+)|\"([^\":]*)\")");

SearchPath::SearchPath(const std::string& pth, bool include_current)
{
    expandedspth = etasl::string_interpolate(pth);
    std::smatch sm;
    while (regex_search(expandedspth, sm, searchPathRegex)) {
        if (sm[1].matched) {
            spth.push_back(sm[1].str());
        }
        if (sm[2].matched) {
            spth.push_back(sm[2].str());
        }
        expandedspth = sm.suffix();
    }
    if (include_current) {
        spth.push_back(std::filesystem::current_path());
    }
}

std::filesystem::path SearchPath::find(const std::string& fn)
{
    std::filesystem::path rp(fn);
    for (auto p : spth) {
        auto ap = (p / rp).lexically_normal();
        if (std::filesystem::exists(ap)) {
            return ap;
        }
    }
    return std::filesystem::path("");
}

std::filesystem::path keepLastNPartsOfPath(const std::filesystem::path& path, size_t n)
{
    std::list<std::string> parts;
    for (const auto& part : path)
    {
        parts.push_back(part.string());
        if (parts.size() > n)
            parts.erase(parts.begin());
    }
    std::filesystem::path result;
    for (const auto& part : parts) {
        result = result / part;
    }

    return result;
}


std::string execute_with_captured_output_old(const std::string& cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        throw etasl_error(etasl_error::STRING_INTERPOLATION_ERROR, fmt::format("error while executing $({})", cmd));
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

std::string execute_with_captured_output(const std::string& cmd)
{
    std::string complete_cmd = cmd + " > /tmp/capture_stdout 2> /tmp/capture_stderr";
    int result = std::system(complete_cmd.c_str());
    if (result != 0) {
        throw etasl_error(etasl_error::STRING_INTERPOLATION_ERROR, fmt::format("error while executing $({})", cmd));
    }
    std::stringstream buffer;
    buffer << std::ifstream("/tmp/capture_stdout").rdbuf();
    return buffer.str();
}

std::string trim(const std::string& s, bool firstline)
{
    size_t start = 0;
    while (start < s.size() && (s[start] == ' ' || s[start] == '\t' || s[start] == '\n')) {
        start++;
    }
    size_t stop;
    if (firstline) {
        stop = 0;
        while ((stop < s.size()) && (s[stop] != '\n')) {
            stop++;
        }
        stop--;
    } else {
        stop = s.size() - 1;
    }
    while (stop >= 0 && (s[stop] == ' ' || s[stop] == '\t' || s[stop] == '\n')) {
        stop--;
    }
    if (start <= stop) {
        return s.substr(start, stop - start + 1);
    } else {
        return "";
    }
    return s;
}

std::string string_interpolate(const std::string& s)
{
    enum State {
        normal,
        dollar_encountered,
        envref_encountered,
        packageref_encountered,
        execref_encountered
    };
    size_t loc_opening;
    size_t loc_defv;
    State state = normal;
    std::string result;
    result.reserve(s.size() + 128);
    for (size_t i = 0; i < s.size(); ++i) {
        switch (state) {
        case normal:
            if (s[i] == '$') {
                state = dollar_encountered;
            } else {
                result.push_back(s[i]);
            }
            break;
        case dollar_encountered:
            switch (s[i]) {
            case '{':
                loc_opening = i + 1;
                loc_defv = loc_opening;
                state = envref_encountered;
                break;
            case '[':
                loc_opening = i + 1;
                state = packageref_encountered;
                break;
            case '(':
                loc_opening = i + 1;
                loc_defv = loc_opening;
                state = execref_encountered;
                break;
            default:
                state = normal;
                break;
            }
            break;
        case packageref_encountered:
            if (s[i] == ']') {
                std::string package = s.substr(loc_opening, i - loc_opening);
                try {
                    result += ament_index_cpp::get_package_share_directory(package);
                } catch (ament_index_cpp::PackageNotFoundError& err) {
                    throw etasl::etasl_error(etasl::etasl_error::STRING_INTERPOLATION_ERROR, "Package \"" + package + "\" not found during string interpolation using \"${{{}}\"");
                }
                state = normal;
            }
            break;
        case envref_encountered:
            if (s[i] == ':') {
                loc_defv = i + 1;
            }
            if (s[i] == '}') {
                if (loc_defv == loc_opening) {
                    std::string varname = s.substr(loc_opening, i - loc_opening);
                    auto p = std::getenv(varname.c_str());
                    if (p != nullptr) {
                        result += p;
                    } else {
                        throw etasl::etasl_error(etasl::etasl_error::STRING_INTERPOLATION_ERROR,
                            "Environment variable \"" + varname + "\" not found during string interpolation using \"${{...}}\" ");
                    }
                } else {
                    std::string varname = s.substr(loc_opening, loc_defv - 1 - loc_opening);
                    auto p = std::getenv(varname.c_str());
                    if (p != nullptr) {
                        result += p;
                    } else {
                        result += s.substr(loc_defv, i - loc_defv);
                    }
                }
                state = normal;
            }
            break;
        case execref_encountered:
            if (s[i] == ':') {
                loc_defv = i + 1;
            }
            if (s[i] == ')') {
                if (loc_defv == loc_opening) {
                    std::string execstr = s.substr(loc_opening, i - loc_opening);
                    std::string output = execute_with_captured_output(execstr); // throws if error
                    result += trim(output, true);
                    state = normal;
                } else {
                    std::string execstr = s.substr(loc_opening, loc_defv - 1 - loc_opening);
                    std::string output;
                    try {
                        output = execute_with_captured_output(execstr + " 2> /tmp/captured_stderr"); // throws if error

                    } catch (etasl::etasl_error&) {
                        // std::cout << fmt::format("str='{}' loc_defv={}  i={}, i-loc_defv={}", s, loc_defv, i, i - loc_defv) << std::endl;
                        output = s.substr(loc_defv, i - loc_defv);
                    }
                    result += trim(output, true);
                    state = normal;
                }
            }
            break;
        }
    }
    if (state == envref_encountered) {
        result += s.substr(loc_opening);
        // throw etasl::etasl_error(etasl::etasl_error::STRING_INTERPOLATION_ERROR, "Syntax error during string interpolation: opening bracket '{' never closed");
    }
    if (state == packageref_encountered) {
        result += s.substr(loc_opening);
        // throw etasl::etasl_error(etasl::etasl_error::STRING_INTERPOLATION_ERROR, "Syntax error during string interpolation: opening bracket '[' never closed");
    }
    if (state == execref_encountered) {
        result += s.substr(loc_opening);
        // throw etasl::etasl_error(etasl::etasl_error::STRING_INTERPOLATION_ERROR, "Syntax error during string interpolation: opening bracket '(' never closed");
    }
    return result;
}

std::filesystem::path find_local_alternative(SearchPath& sp, const URI& uri, int path_components_used)
{
    if (!uri.matched()) {
        return std::filesystem::path("");
    }
    if (path_components_used <= -1) {
        return sp.find(uri.path());
    } else {
        std::filesystem::path base = uri.path();
        std::cout << "original path " << base << std::endl;
        while (path_components_used >= 0) {
            base = base.parent_path();
            std::cout << "parent : " << base << std::endl;
            path_components_used--;
        }
        std::cout << base.lexically_normal() << std::endl;
        std::filesystem::path retval = sp.find(uri.path().lexically_relative(base));
        std::cout << retval << std::endl;
        return retval;
    }
}

};