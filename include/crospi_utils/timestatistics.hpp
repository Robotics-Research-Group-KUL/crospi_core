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

#ifndef TIMESTATISTICS_HPP_43EF23
#define TIMESTATISTICS_HPP_43EF23

#include <chrono>
#include <cmath>
#include <fmt/format.h>

namespace etasl {

using namespace std::chrono::_V2;
using namespace std::chrono;
/**
 * auxiliary class to print statics of a loop.
 */
class TimeStatistics {
    high_resolution_clock::time_point prev_timestamp;
    high_resolution_clock::time_point reset_timestamp;

    // for the time between calls of update():
    int n;
    double sum;
    double sum_sq;
    double minval;
    double maxval;
    // for the time between the last update() and computing_finished()
    int comp_n;
    double comp_sum;
    double comp_sum_sq;
    double comp_minval;
    double comp_maxval;
    double reference;

    std::string buffer;

public:
    TimeStatistics() { }

    void reset(double ref)
    {
        prev_timestamp = high_resolution_clock::now();
        reset_timestamp = high_resolution_clock::now();
        sum = 0;
        sum_sq = 0;
        minval = 1E15;
        maxval = -1E15;
        n = 0;
        comp_sum = 0;
        comp_sum_sq = 0;
        comp_minval = 1E15;
        comp_maxval = -1E15;        
        comp_n = 0;
        reference = ref;
        buffer.reserve(200);
    }

    void update()
    {
        high_resolution_clock::time_point cur_timestamp = high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_timestamp - prev_timestamp).count() / 1.E9 - reference;
        prev_timestamp = cur_timestamp;
        sum += duration;
        sum_sq += duration * duration;
        n += 1;
        if (duration < minval) {
            minval = duration;
        }
        if (duration > maxval) {
            maxval = duration;
        }
    }

    void computing_finished()
    {
        high_resolution_clock::time_point cur_timestamp = high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::nanoseconds>(cur_timestamp - prev_timestamp).count() / 1.E9;
        comp_sum += duration;
        comp_sum_sq += duration * duration;
        comp_n += 1;
        if (duration < comp_minval) {
            comp_minval = duration;
        }
        if (duration > maxval) {
            comp_maxval = duration;
        }
    }

    double min() const
    {
        return minval;
    }

    double max() const
    {
        return maxval;
    }

    double std() const
    {
        // double mean = sum / n;
        return sqrt(sum_sq / n);
    }

    double mean() const
    {
        return sum / n;
    }

    double comp_min() const
    {
        return comp_minval;
    }

    double comp_max() const
    {
        return comp_maxval;
    }

    double comp_std() const
    {
        double mean = comp_sum / comp_n;
        return sqrt(comp_sum_sq / comp_n - mean * mean);
    }

    double comp_mean() const
    {
        return comp_sum / comp_n;
    }

    double duration_since_reset()
    {
        high_resolution_clock::time_point cur_timestamp = high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::nanoseconds>(cur_timestamp - reset_timestamp).count() / 1.E9;
    }

    const std::string& getStatistics(double time)
    {
        buffer.clear();
        fmt::format_to(std::back_inserter(buffer), "{:12.3f}s Stats[ms]: cycle(nom={:7.3f} mean={:7.3f}, std={:7.3f}, min={:7.3f}, max {:7.3f}) computing(mean={:7.3f}, std={:7.3f}, min={:7.3f}, max {:7.3f})",
            time, reference, mean() * 1000, std() * 1000, min() * 1000, max() * 1000,
            comp_mean() * 1000, comp_std() * 1000, comp_min() * 1000, comp_max() * 1000);
        return buffer;
    }

}; // TimeStatistics

} // namespace etasl

#endif