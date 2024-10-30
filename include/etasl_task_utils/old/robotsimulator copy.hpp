#pragma once
#include "etasl_task_utils/robot.hpp"
#include "jsoncpp/json/json.h"

namespace etasl {
typedef std::unordered_map<std::string, double> JointValues;
typedef std::vector<std::string> JointNames;

class RobotSimulator : public Robot {
    std::unordered_map<std::string, double> jvalues;
    std::unordered_map<std::string, double> jvelocities;
    Json::Value param;
    bool ignore_non_existing;
    std::string name;

public:
    RobotSimulator(const Json::Value& _param);
    virtual const std::string& getName() const override;
    virtual void initialize() override;
    virtual void setJointState(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos,
        const Eigen::VectorXd& jvel) override;
    virtual void getJointState(
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos) override;
    virtual void update(double dt) override;
};

} // namespace -