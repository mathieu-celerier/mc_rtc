#include <mc_tvm/DisturbanceCompensationFunction.h>

#include <mc_rbdyn/Robot.h>
#include <mc_tvm/Robot.h>

namespace mc_tvm
{

DisturbanceCompensationFunction::DisturbanceCompensationFunction(mc_tvm::Robot & robot)
: LinearFunction(robot.robot().mb().nrDof()), robot_(robot)
{
    registerUpdates(Update::Value, &DisturbanceCompensationFunction::updateValue);
    addInputDependency<DisturbanceCompensationFunction>(Update::Value, robot_.robot().tvmRobot(), mc_tvm::Robot::Output::ExternalDisturbance);
    addOutputDependency<DisturbanceCompensationFunction>(Output::Value, Update::Value);
    addVariable(robot.alphaD(), true);
    addVariable(robot.alphaDDisturbed(), true);
    size_t nDof = robot.robot().mb().nrDof();
    jacobian_.at(&*robot.alphaDDisturbed()) = Eigen::MatrixXd::Identity(nDof,nDof);
    jacobian_.at(&*robot.alphaD()) = -Eigen::MatrixXd::Identity(nDof,nDof);
}

void DisturbanceCompensationFunction::updateValue_() { value_ = robot_.alphaDDisturbed()->value() - robot_.alphaD()->value() - robot_.alphaDDisturbance(); }

}