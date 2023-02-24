
/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/Robot.h>

// #include <mc_rbdyn/fwd.h>

#include <tvm/function/abstract/LinearFunction.h>

namespace mc_tvm
{

/** This class implements an external disturbance compensation function for a given robot */
class MC_TVM_DLLAPI DisturbanceCompensationFunction : public tvm::function::abstract::LinearFunction
{
public:
    DISABLE_OUTPUTS(Output::JDot)
    SET_UPDATES(DisturbanceCompensationFunction, B)
    /** Constructor
     *
     */
    DisturbanceCompensationFunction(mc_tvm::Robot & robot);

private:
    void updateB();
    const mc_tvm::Robot & robot_;
};

}