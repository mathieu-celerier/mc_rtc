#include <mc_tvm/DisturbanceCompensationFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_tvm/Robot.h>

namespace mc_tvm
{

DisturbanceCompensationFunction::DisturbanceCompensationFunction(const mc_rbdyn::Robot & robot, tvm::VariablePtr ddq_dist, tvm::VariablePtr ddq, Eigen::VectorXd & dist)
: LinearFunction(robot.mb().nrDof()), ddq_dist_(*ddq_dist), ddq_(*ddq), dist_(dist)
{
    registerUpdates(Update::Value, &DisturbanceCompensationFunction::updateValue);
    addInputDependency<DisturbanceCompensationFunction>(Update::Value, robot.tvmRobot(), mc_tvm::Robot::Output::ExternalDisturbance);
    addOutputDependency<DisturbanceCompensationFunction>(Output::Value, Update::Value);
    addVariable(ddq_dist, true);
    addVariable(ddq, true);
    jacobian_.at(&ddq_dist_) = Eigen::MatrixXd::Identity(ddq_dist_.size(),ddq_dist_.size());
    jacobian_.at(&ddq_) = -Eigen::MatrixXd::Identity(ddq_.size(),ddq_.size());
}

void DisturbanceCompensationFunction::updateValue() { value_ = ddq_dist_.value() - ddq_.value() - dist_; }

}