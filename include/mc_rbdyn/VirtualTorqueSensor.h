/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>

namespace mc_rbdyn
{

struct Robot;

/** This struct is intended to hold static information about a virtual torques sensor that include joint torques and
 * forces/torques at the floating base if necessary, and the current reading of said sensor.
 */
struct MC_RBDYN_DLLAPI VirtualTorqueSensor : public Device
{
public:
  /** Default constructor, this does not represent a valid virtual torques sensor */
  VirtualTorqueSensor();

  /** Construct a valid virtual torques sensor based on static information, this sensor can then be used to provide
   * sensing information to the robot.
   *
   * @param name Name of the sensor
   *
   * @param size Number of DoF
   *
   */
  VirtualTorqueSensor(const std::string & name, const int & size);

  VirtualTorqueSensor(const VirtualTorqueSensor & ets);

  VirtualTorqueSensor & operator=(const VirtualTorqueSensor & ets);

  VirtualTorqueSensor(VirtualTorqueSensor &&) = default;

  /** Destructor */
  ~VirtualTorqueSensor() noexcept override;

  inline const Eigen::VectorXd & torques() const { return virtJointTorques_; }

  inline void torques(Eigen::VectorXd & torques) { virtJointTorques_ = torques; }

  inline const Eigen::VectorXd & equivalentAcc() const { return virtJointAccelerations_; }

  inline void equivalentAcc(Eigen::VectorXd & accelerations) { virtJointAccelerations_ = accelerations; }

  DevicePtr clone() const override;

private:
  Eigen::VectorXd virtJointTorques_;
  Eigen::VectorXd virtJointAccelerations_;
  int size_;
};

} // namespace mc_rbdyn
