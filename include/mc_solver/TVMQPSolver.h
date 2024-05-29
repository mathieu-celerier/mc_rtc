/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_solver/QPSolver.h>

#include <mc_rtc/clock.h>

#include <tvm/ControlProblem.h>
#include <tvm/LinearizedControlProblem.h>
#include <tvm/scheme/HierarchicalLeastSquares.h>
#include <tvm/scheme/WeightedLeastSquares.h>

namespace mc_solver
{

namespace details
{

/** This implements the QPSolver interface for the TVM backend
 *
 * This solver can accepts tasks and constraints from mc_rtc
 *
 */
template<typename SchemeT>
struct MC_SOLVER_DLLAPI TVMQPSolver final : public QPSolver
{
  TVMQPSolver(mc_rbdyn::RobotsPtr robots, double timeStep);

  TVMQPSolver(double timeStep);

  ~TVMQPSolver() final = default;

  void setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts) final;

  const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & id) const final;

  double solveTime() final;

  double solveAndBuildTime() final;

  /** Access the internal problem */
  inline tvm::LinearizedControlProblem & problem() noexcept { return problem_; }

  /** Access the internal problem (const) */
  inline const tvm::LinearizedControlProblem & problem() const noexcept { return problem_; }

  /** Helper to get a \ref TVMQPSolver from a \ref QPSolver instance
   *
   * The caller should make sure the cast is valid by checking the QPSolver backend.
   *
   * In debug the program will abort otherwise, in release UB abounds
   */
  static inline TVMQPSolver & from_solver(QPSolver & solver) noexcept
  {
    assert(solver.backend() == QPSolver::Backend::TVM || solver.backend() == QPSolver::Backend::TVMHierarchical);
    return static_cast<TVMQPSolver &>(solver);
  }

  /** Helper to get a \ref TVMQPSolver from a \ref QPSolver instance
   *
   * The caller should make sure the cast is valid by checking the QPSolver backend.
   *
   * In debug the program will abort otherwise, in release UB abounds
   */
  static inline const TVMQPSolver & from_solver(const QPSolver & solver) noexcept
  {
    assert(solver.backend() == QPSolver::Backend::TVM || solver.backend() == QPSolver::Backend::TVMHierarchical);
    return static_cast<const TVMQPSolver &>(solver);
  }

private:
  /** Control problem */
  tvm::LinearizedControlProblem problem_;
  /** Solver scheme */
  SchemeT solver_;
  /** Contact data on the solver side */
  struct ContactData
  {
    /** Contact function in the solver */
    tvm::TaskWithRequirementsPtr contactConstraint_;
    /** Force variables on r1 side (if any) */
    tvm::VariableVector f1_;
    /** Constraints on f1 */
    std::vector<tvm::TaskWithRequirementsPtr> f1Constraints_;
    /** Force variables on r2 side (if any) */
    tvm::VariableVector f2_;
    /** Constraints on f2 */
    std::vector<tvm::TaskWithRequirementsPtr> f2Constraints_;
  };
  /** Related contact functions */
  std::vector<ContactData> contactsData_;
  /** Runtime of the latest run call */
  mc_rtc::duration_ms solve_dt_{0};

  /** Common part of control loop */
  bool runCommon();
  /** Run without feedback (open-loop) */
  bool runOpenLoop();
  /** Run with encoders' feedback */
  bool runJointsFeedback(bool wVelocity);

  /**
   * WARNING EXPERIMENTAL
   *
   * Runs the QP on an estimated robot state.
   *
   * Uses the real robot state (mbc.q and mbc.alpha) from realRobots() instances.
   * It is the users responsibility to ensure that the real robot instance is properly estimated
   * and filled. Typically, this will be done through the Observers pipeline.
   * For example, the following pipeline provides a suitable state:
   *
   * \code{.yaml}
   * RunObservers: [Encoder, KinematicInertial]
   * UpdateObservers: [Encoder, KinematicInertial]
   * \endcode
   *
   * @param integrateControlState If true, integration is performed over the control state, otherwise over the observed
   * state
   *
   * @return True if successful, false otherwise
   */
  bool runClosedLoop(bool integrateControlState);

  /** Update a robot from the optimization result */
  void updateRobot(mc_rbdyn::Robot & robot);

  /** Feedback data */
  std::vector<std::vector<double>> prev_encoders_{};
  std::vector<std::vector<double>> encoders_alpha_{};
  std::vector<std::vector<std::vector<double>>> control_q_{};
  std::vector<std::vector<std::vector<double>>> control_alpha_{};

  /** Dynamics constraint currently active for robots in the solver */
  std::unordered_map<std::string, DynamicsConstraint *> dynamics_;

  bool run_impl(FeedbackType fType = FeedbackType::None) final;

  void addDynamicsConstraint(mc_solver::DynamicsConstraint * dynamics) final;

  void removeDynamicsConstraint(mc_solver::ConstraintSet * maybe_dynamics) final;
  void removeDynamicsConstraint(mc_solver::DynamicsConstraint * dyn);

  size_t getContactIdx(const mc_rbdyn::Contact & contact);
  void addContact(const mc_rbdyn::Contact & contact);
  using ContactIterator = std::vector<mc_rbdyn::Contact>::iterator;
  ContactIterator removeContact(size_t idx);
  std::tuple<size_t, bool> addVirtualContactImpl(const mc_rbdyn::Contact & contact);
  void addContactToDynamics(const std::string & robot,
                            const mc_rbdyn::RobotFrame & frame,
                            const std::vector<sva::PTransformd> & points,
                            tvm::VariableVector & forces,
                            std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                            const Eigen::MatrixXd & frictionCone,
                            double dir);
};

extern template struct TVMQPSolver<tvm::scheme::HierarchicalLeastSquares>;
extern template struct TVMQPSolver<tvm::scheme::WeightedLeastSquares>;

} // namespace details

using TVMHQPSolver = details::TVMQPSolver<tvm::scheme::HierarchicalLeastSquares>;
using TVMQPSolver = details::TVMQPSolver<tvm::scheme::WeightedLeastSquares>;

/** Helper to get a \ref TVMQPSolver from a \ref QPSolver instance
 *
 * The caller should make sure the cast is valid by checking the QPSolver backend.
 *
 * In debug the program will abort otherwise, in release UB abounds
 */
inline TVMQPSolver & tvm_solver(QPSolver & solver) noexcept
{
  return TVMQPSolver::from_solver(solver);
}

/* const version */
inline const TVMQPSolver & tvm_solver(const QPSolver & solver) noexcept
{
  return TVMQPSolver::from_solver(solver);
}

/** Helper to get a \ref TVMHQPSolver from a \ref QPSolver instance
 *
 * The caller should make sure the cast is valid by checking the QPSolver backend.
 *
 * In debug the program will abort otherwise, in release UB abounds
 */
inline TVMHQPSolver & tvm_hsolver(QPSolver & solver) noexcept
{
  return TVMHQPSolver::from_solver(solver);
}

/* const version */
inline const TVMHQPSolver & tvm_hsolver(const QPSolver & solver) noexcept
{
  return TVMHQPSolver::from_solver(solver);
}

/** Helper to get a \ref tvm::LinearizedControlProblem from a \ref QPSolver instance
 *
 * This aborts if the solver does not have the right backend
 *
 */
inline tvm::LinearizedControlProblem & tvm_problem(QPSolver & solver) noexcept
{
  switch(solver.backend())
  {
    case QPSolver::Backend::TVM:
      return tvm_solver(solver).problem();
    case QPSolver::Backend::TVMHierarchical:
      return tvm_hsolver(solver).problem();
    default:
      mc_rtc::log::error_and_throw("tvm_problem(solver) called on a solver with the wrong backend");
  }
}

} // namespace mc_solver
