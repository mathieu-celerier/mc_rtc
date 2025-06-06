#include <mc_rbdyn/VirtualTorqueSensor.h>

namespace mc_rbdyn
{

VirtualTorqueSensor::VirtualTorqueSensor() : VirtualTorqueSensor("", 0) {}

VirtualTorqueSensor::VirtualTorqueSensor(const std::string & name, const int & size)
: Device(name), virtJointTorques_(Eigen::VectorXd::Zero(size)), virtJointAccelerations_(Eigen::VectorXd::Zero(size)),
  size_(size)
{
}

VirtualTorqueSensor::VirtualTorqueSensor(const VirtualTorqueSensor & ets) : VirtualTorqueSensor(ets.name_, ets.size_)
{
  virtJointTorques_ = ets.virtJointTorques_;
  virtJointAccelerations_ = ets.virtJointAccelerations_;
}

VirtualTorqueSensor & VirtualTorqueSensor::operator=(const VirtualTorqueSensor & ets)
{
  if(&ets == this) { return *this; }
  name_ = ets.name_;
  parent_ = ets.parent_;
  X_p_s_ = ets.X_p_s_;
  virtJointTorques_ = ets.virtJointTorques_;
  virtJointAccelerations_ = ets.virtJointAccelerations_;
  return *this;
}

VirtualTorqueSensor::~VirtualTorqueSensor() noexcept = default;

DevicePtr VirtualTorqueSensor::clone() const
{
  return DevicePtr(new VirtualTorqueSensor(*this));
}

} // namespace mc_rbdyn
