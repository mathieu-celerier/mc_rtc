/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/PostureTask.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI CompliantPostureTask : public PostureTask
{
public:
    CompliantPostureTask(const mc_solver::QPSolver & solver, unsigned int rIndex, double stiffness, double weight);

    /** Change reference acceleration
     *
     * \p refAccel Should be of size nrDof
     */
    void refAccel(const Eigen::VectorXd & refAccel) noexcept;

protected:
    void update(mc_solver::QPSolver & solver);

    void addToGUI(mc_rtc::gui::StateBuilder & gui);

    bool isCompliant_;

    mc_tvm::Robot & tvm_robot_;

    Eigen::VectorXd refAccel_;
};

}