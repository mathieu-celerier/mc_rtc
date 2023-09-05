/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>

#include <mc_solver/TVMQPSolver.h>

namespace mc_control
{

/** An MCController that uses the TVM backend with a hierarchical solver
 *
 * This is simply an helper class to write a TVM-only controller, the key difference with \ref MCController are:
 * - the backend is always set to Backend::TVMHierarchical
 * - solver() returns a \ref mc_solver::TVMHQPSolver
 */
struct MC_CONTROL_DLLAPI TVMHierarchicController
: public details::BackendSpecificController<MCController::Backend::TVMHierarchical, mc_solver::TVMHQPSolver>
{
  using details::BackendSpecificController<MCController::Backend::TVMHierarchical,
                                           mc_solver::TVMHQPSolver>::BackendSpecificController;
};

} // namespace mc_control
