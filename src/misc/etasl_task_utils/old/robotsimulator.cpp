#include "etasl_task_utils/robotsimulator.hpp"
#include <unordered_map>

namespace etasl {

RobotSimulator::RobotSimulator(const Json::Value& _param)
    : param(_param)
{
    ignore_non_existing = param["ignore_non_existing_variables"].asBool();
    name = param["name"].asString();
}

const std::string& RobotSimulator::getName() const {
    return name;
}

void etasl::RobotSimulator::initialize()
{
    Json::Value iv = param["initial_values"];
    for (auto n : iv.getMemberNames()) {
        jvalues[n] = iv[n].asDouble();
        jvelocities[n] = 0.0;
    }
}

void etasl::RobotSimulator::setJointState(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel)
{
    // this simulator ignores the jpos (integrated by etasl) and only uses jvel
    assert(jnames.size() == jpos.size());
    assert(jnames.size() == jvel.size());
    for (size_t i = 0; i < jnames.size(); ++i) {
        auto p = jvelocities.find(jnames[i]);
        if ((p == jvelocities.end()) && !ignore_non_existing) {
            throw std::logic_error("RobotSimulator called with non existing joint name and ignore_non_existing_variables==false");
        }
        p->second = jvel[i];
    }
}

void etasl::RobotSimulator::getJointState(const std::vector<std::string>& jnames, Eigen::VectorXd& jpos)
{
    for (size_t i = 0; i < jnames.size(); ++i) {
        auto p = jvalues.find(jnames[i]);
        if ((p == jvalues.end()) && !ignore_non_existing) {
            throw std::logic_error("RobotSimulator called with non existing joint name and ignore_non_existing_variables==false");
        }
        jpos[i] = p->second;
    }
}

void etasl::RobotSimulator::update(double dt)
{
    // robot is simulated as a perfect integrator.
    // can be extended to have a slightly more realistic simulation.
    for (auto p : jvalues) {
        p.second += dt * jvelocities[p.first];
    }
}

} // namespace etasl
