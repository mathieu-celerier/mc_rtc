
/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>

#include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/LinearFunction.h>

namespace mc_tvm
{

/** This class implements an external disturbance compensation function for a given robot */
class MC_TVM_DLLAPI DisturbanceCompensationFunction : public tvm::function::abstract::LinearFunction
{
public:
    SET_UPDATES(DisturbanceCompensationFunction, Value)
    /** Constructor
     *
     */
    DisturbanceCompensationFunction(const mc_rbdyn::Robot & robot, tvm::VariablePtr ddq_dist, tvm::VariablePtr ddq, Eigen::VectorXd & dist);

    void updateValue();

private:
    tvm::Variable & ddq_dist_;
    tvm::Variable & ddq_;
    Eigen::VectorXd & dist_;
};

}