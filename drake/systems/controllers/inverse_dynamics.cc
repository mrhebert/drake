#include "drake/systems/controllers/inverse_dynamics.h"

#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace systems {

template <typename T>
InverseDynamics<T>::InverseDynamics(const RigidBodyTree<T>& tree,
                                    bool pure_gravity_compensation)
    : tree_(tree),
      pure_gravity_compensation_(pure_gravity_compensation),
      q_dim_(tree.get_num_positions()),
      v_dim_(tree.get_num_velocities()),
      act_dim_(tree.get_num_actuators()) {
  input_port_index_state_ =
      this->DeclareInputPort(kVectorValued, q_dim_ + v_dim_).get_index();
  output_port_index_torque_ =
      this->DeclareOutputPort(kVectorValued, act_dim_).get_index();

  // Doesn't declare desired acceleration input port if we are only doing
  // gravity compensation.
  if (!pure_gravity_compensation_) {
    input_port_index_desired_acceleration_ =
        this->DeclareInputPort(kVectorValued, v_dim_).get_index();
  }

  if (v_dim_ != act_dim_) {
    std::stringstream msg;
    msg << "The model is under-actuated!\n"
        << "  - size of gravity vector: " << v_dim_ << "\n"
        << "  - number of actuators: " << act_dim_;
    DRAKE_ABORT_MSG(msg.str().c_str());
  }
}

template <typename T>
void InverseDynamics<T>::DoCalcOutput(const Context<T>& context,
                                      SystemOutput<T>* output) const {
  // State input.
  VectorX<T> x = this->EvalEigenVectorInput(context, input_port_index_state_);

  // Desired acceleration input.
  VectorX<T> desired_vd = VectorX<T>::Zero(v_dim_);

  if (!pure_gravity_compensation_) {
    // Only eval acceleration input port when we are not in pure gravity
    // compensation mode.
    desired_vd = this->EvalEigenVectorInput(
        context, input_port_index_desired_acceleration_);
  } else {
    // Sets velocity to zero in pure gravity compensation.
    x.tail(v_dim_).setZero();
  }

  KinematicsCache<T> cache = tree_.CreateKinematicsCache();
  cache.initialize(x.head(q_dim_), x.tail(v_dim_));
  tree_.doKinematics(cache, true);

  eigen_aligned_std_unordered_map<RigidBody<T> const*, drake::TwistVector<T>>
      f_ext;

  VectorX<T> torque = tree_.inverseDynamics(
      cache, f_ext, desired_vd,
      !pure_gravity_compensation_ /* include v dependent terms */);

  DRAKE_ASSERT(torque.size() ==
               System<T>::get_output_port(output_port_index_torque_).size());
  System<T>::GetMutableOutputVector(output, output_port_index_torque_) = torque;
}

template class InverseDynamics<double>;
// TODO(siyuan): some linking issue on mac.
// template class InverseDynamics<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
