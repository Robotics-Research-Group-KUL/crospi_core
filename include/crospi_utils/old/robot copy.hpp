#pragma once

#include "crospi_utils/outputhandler.hpp"
#include "crospi_utils/inputhandler.hpp"
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <fmt/format.h>

namespace etasl {

class Robot {
public:
    typedef std::shared_ptr<Robot> SharedPtr;
    virtual void initialize() = 0;
    virtual void setJointState(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos,
        const Eigen::VectorXd& jvel)
        = 0;
    virtual void getJointState(
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos)
        = 0;
    virtual void update(double dt) = 0;
    virtual const std::string& getName() const = 0;
    virtual ~Robot() { }
};

class RobotOutputHandler : public OutputHandler {
    Robot::SharedPtr robot;
    std::string name;

public:
    RobotOutputHandler(Robot::SharedPtr _robot)
        : robot(_robot)
    {
        name = fmt::format("RobotOutputHandler({})", robot->getName());
    }

    const std::string& getName() const {
        return name;
    }

    virtual void initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames) override { }

    virtual void update(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos,
        const Eigen::VectorXd& jvel,
        const std::vector<std::string>& fnames,
        const Eigen::VectorXd& fvel,
        const Eigen::VectorXd& fpos) override
    {
        robot->setJointState(jnames, jpos, jvel);
    }
}; // RobotOutputHandler

class RobotInputHandler : public InputHandler {
    Robot::SharedPtr robot;
    std::string name;

public:
    typedef std::shared_ptr<InputHandler> SharedPtr;

    RobotInputHandler(Robot::SharedPtr _robot):robot(_robot) {
        name = fmt::format("RobotInputHandler({})", robot->getName());
    }
    //

    const std::string& getName() const {
        return name;
    }

    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& jpos,
        Eigen::VectorXd& fpos) { return true; }

    virtual void update(
        double time, 
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos, 
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos) {
        robot->getJointState(jnames, jpos);
    }

    virtual void finalize() { }

}; // RobotInputHandler

} // namespace etasl