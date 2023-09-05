/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mc_admittance_sample_controller.h"

TRIPLE_CONTROLLERS_CONSTRUCTOR(
    "AdmittanceSample",
    AdmittanceSampleController(rm, dt, config, mc_control::MCController::Backend::Tasks),
    "AdmittanceSample_TVM",
    AdmittanceSampleController(rm, dt, config, mc_control::MCController::Backend::TVM),
    "AdmittanceSample_TVMHierarchical",
    AdmittanceSampleController(rm, dt, config, mc_control::MCController::Backend::TVMHierarchical))
