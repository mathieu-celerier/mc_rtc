#include <mc_tvm/DisturbanceCompensationFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_tvm/Robot.h>

namespace mc_tvm
{

DisturbanceCompensationFunction::DisturbanceCompensationFunction(mc_tvm::Robot & robot)
: LinearFunction(robot.robot().mb().nrDof()), robot_(robot)
{
    registerUpdates(Update::B, &DisturbanceCompensationFunction::updateB);
    addOutputDependency<DisturbanceCompensationFunction>(Output::B, Update::B);
    addInputDependency<DisturbanceCompensationFunction>(Update::B, robot_.robot().tvmRobot(), mc_tvm::Robot::Output::ExternalDisturbance);
    addVariable(robot.alphaD(), true);
    addVariable(robot.alphaDDisturbed(), true);
    size_t nDof = robot.robot().mb().nrDof();
    jacobian_[robot.alphaDDisturbed().get()] = Eigen::MatrixXd::Identity(nDof,nDof);
    jacobian_[robot.alphaDDisturbed().get()].properties(tvm::internal::MatrixProperties::IDENTITY);
    jacobian_[robot.alphaD().get()] = -Eigen::MatrixXd::Identity(nDof,nDof);
    jacobian_[robot.alphaD().get()].properties(tvm::internal::MatrixProperties::MINUS_IDENTITY);
}

void DisturbanceCompensationFunction::updateB() { b_ = robot_.alphaDDisturbance(); }

}