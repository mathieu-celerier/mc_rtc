/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/OrientationTask.h>
#include <RBDyn/MultiBody.h>
#include <Eigen/src/Core/Matrix.h>

namespace mc_tasks
{

/*! \brief Controls an end-effector
 *
 * This task is a thin wrapper around the appropriate tasks in Tasks.
 * The task objective is given in the world frame. For relative control
 * see mc_tasks::RelativeCompliantOrientationTask
 */
struct MC_TASKS_DLLAPI CompliantOrientationTask : public OrientationTask
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  CompliantOrientationTask(const std::string & bodyName,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness = 2.0,
                           double weight = 500.0);

  /** Change acceleration
   *
   * \p refAccel Should be of size 6
   */
  void refAccel(const Eigen::Vector3d & refAccel) noexcept;

  // Set the compliant behavior of the task
  void makeCompliant(bool compliance);
  void setComplianceVector(Eigen::Vector3d gamma);

  // Get compliance state of the task
  bool isCompliant(void);
  Eigen::Vector3d getComplianceVector(void);

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToSolver(mc_solver::QPSolver & solver);

  void update(mc_solver::QPSolver & solver);

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

  Eigen::Matrix3d compliant_matrix_;

  mc_tvm::Robot * tvm_robot_;

  unsigned int rIdx_;

  std::string bodyName_;

  rbd::Jacobian * jac_;

  Eigen::Vector3d refAccel_;
};

} // namespace mc_tasks
