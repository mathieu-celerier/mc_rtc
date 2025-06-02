/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/CompliantOrientationTask.h>

#include <mc_rtc/gui/Checkbox.h>
#include <mc_tvm/Robot.h>
#include <SpaceVecAlg/EigenTypedef.h>
#include "mc_rtc/gui/ArrayInput.h"

namespace mc_tasks
{

CompliantOrientationTask::CompliantOrientationTask(const std::string & bodyName,
                                                   const mc_rbdyn::Robots & robots,
                                                   unsigned int robotIndex,
                                                   double stiffness,
                                                   double weight)
: OrientationTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight),
  compliant_matrix_(Eigen::Matrix3d::Zero()), tvm_robot_(nullptr), rIdx_(robotIndex), bodyName_(bodyName),
  refAccel_(Eigen::Vector3d::Zero())
{
  const mc_rbdyn::RobotFrame & frame = robots.robot(robotIndex).frame(bodyName);

  if(backend_ == Backend::Tasks)
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks] Can't use CompliantOrientationTask with {} backend, please use TVM or TVMHierarchical backend",
        backend_);

  type_ = "compliant_body3d";
  name_ = "compliant_body3d_" + frame.robot().name() + "_" + frame.name();
  OrientationTask::name(name_);
}

void CompliantOrientationTask::refAccel(const Eigen::Vector3d & refAccel) noexcept
{
  refAccel_ = refAccel;
}

void CompliantOrientationTask::makeCompliant(bool compliance)
{
  if(compliance) { compliant_matrix_.diagonal().setOnes(); }
  else { compliant_matrix_.diagonal().setZero(); }
}

void CompliantOrientationTask::setComplianceVector(Eigen::Vector3d gamma)
{
  compliant_matrix_.diagonal() = gamma;
}

bool CompliantOrientationTask::isCompliant(void)
{
  return compliant_matrix_.diagonal().norm() > 0;
}

Eigen::Vector3d CompliantOrientationTask::getComplianceVector(void)
{
  return compliant_matrix_.diagonal();
}

void CompliantOrientationTask::addToSolver(mc_solver::QPSolver & solver)
{
  tvm_robot_ = &solver.robots().robot(rIdx_).tvmRobot();
  jac_ = new rbd::Jacobian(tvm_robot_->robot().mb(), bodyName_);

  OrientationTask::addToSolver(solver);
}

void CompliantOrientationTask::update(mc_solver::QPSolver & solver)
{
  auto J = jac_->jacobian(tvm_robot_->robot().mb(), tvm_robot_->robot().mbc());
  Eigen::Vector6d disturbance = J * tvm_robot_->alphaDExternal();
  Eigen::Vector3d disturbedAccel = refAccel_ + compliant_matrix_ * disturbance.head<3>();

  OrientationTask::refAccel(disturbedAccel);

  // OrientationTask::update(solver);
}

void CompliantOrientationTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("stiffness"))
  {
    auto s = config("stiffness");
    if(s.size())
    {
      Eigen::VectorXd stiff = s;
      OrientationTask::stiffness(stiff);
    }
    else
    {
      double stiff = s;
      OrientationTask::stiffness(stiff);
    }
  }
  if(config.has("damping"))
  {
    auto d = config("damping");
    if(d.size()) { OrientationTask::setGains(OrientationTask::dimStiffness(), d); }
    else { OrientationTask::setGains(OrientationTask::stiffness(), d); }
  }
  if(config.has("compliance"))
  {
    auto g = config("compliance");
    if(g.size()) { setComplianceVector(g); }
    else { makeCompliant((double)g != 0); }
  }
  if(config.has("weight"))
  {
    double w = config("weight");
    OrientationTask::weight(w);
  }
}

void CompliantOrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  gui.addElement({"Tasks", name_, "Compliance"}, mc_rtc::gui::Checkbox(
                                                     "Compliance is active", [this]() { return isCompliant(); },
                                                     [this]() { makeCompliant(!isCompliant()); }));
  gui.addElement({"Tasks", name_, "Compliance"},
                 mc_rtc::gui::ArrayInput(
                     "Compliance parameters", {"rx", "ry", "rz"}, [this]() { return getComplianceVector(); },
                     [this](Eigen::Vector3d v) { setComplianceVector(v); }));

  OrientationTask::addToGUI(gui);
}

} // namespace mc_tasks
