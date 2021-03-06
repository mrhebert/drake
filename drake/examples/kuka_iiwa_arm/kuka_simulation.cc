/// @file
///
/// Implements a simulation of the KUKA iiwa arm.  Like the driver for the
/// physical arm, this simulation communicates over LCM using lcmt_iiwa_status
/// and lcmt_iiwa_command messages. It is intended to be a be a direct
/// replacement for the KUKA iiwa driver and the actual robot hardware.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_path.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/examples/kuka_iiwa_arm/sim_diagram_builder.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/inverse_dynamics_controller.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {
namespace {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;

int DoMain() {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const std::string kModelPath = "/examples/kuka_iiwa_arm/urdf/"
      "iiwa14_estimated_params.urdf";
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    CreateTreedFromFixedModelAtPose(kModelPath, tree.get());
    plant = builder.AddPlant(std::move(tree));
  }
  // Creates and adds LCM publisher for visualization.
  builder.AddVisualizer(&lcm);

  // std::cout << "HI " << std::endl;
  // Adds a iiwa controller
  VectorX<double> iiwa_kp, iiwa_kd, iiwa_ki;
  SetPositionControlledIiwaGains(&iiwa_kp, &iiwa_ki, &iiwa_kd);
  auto controller =
      builder.AddController<systems::InverseDynamicsController<double>>(
          RigidBodyTreeConstants::kFirstNonWorldModelInstanceId,
          GetDrakePath() + kModelPath, nullptr, iiwa_kp, iiwa_ki, iiwa_kd,
          false /* without feedforward acceleration */);
          // std::cout << "HI " << std::endl;

  const RigidBodyTree<double>& tree = plant->get_rigid_body_tree();
  VerifyIiwaTree(tree);
  std::cout << "HI " << std::endl;

  // Create the command subscriber and status publisher.
  systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();
  auto command_sub = base_builder->AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_iiwa_command>(
          "IIWA_COMMAND", &lcm));
          std::cout << "HI " << std::endl;

  auto command_receiver = base_builder->AddSystem<IiwaCommandReceiver>();
  std::cout << "HI " << std::endl;

  auto status_pub = base_builder->AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_iiwa_status>(
          "IIWA_STATUS", &lcm));
  status_pub->set_publish_period(kIiwaLcmStatusPeriod);
  std::cout << "HI " << std::endl;

  auto status_sender = base_builder->AddSystem<IiwaStatusSender>();
  std::cout << "HI " << std::endl;

  base_builder->Connect(command_sub->get_output_port(0),
                  command_receiver->get_input_port(0));
  base_builder->Connect(command_receiver->get_output_port(0),
                  controller->get_input_port_desired_state());
  base_builder->Connect(plant->get_output_port(0),
                  status_sender->get_state_input_port());
  base_builder->Connect(command_receiver->get_output_port(0),
                  status_sender->get_command_input_port());
  base_builder->Connect(status_sender->get_output_port(0),
                  status_pub->get_input_port(0));
                  std::cout << "HI " << std::endl;

  auto sys = builder.Build();
  std::cout << "HI " << std::endl;

  Simulator<double> simulator(*sys);
  std::cout << "HIhgerre " << std::endl;

  lcm.StartReceiveThread();
  std::cout << "HIaaaaaaaasdasdasdasdasdasdasdaaa " << std::endl;

  simulator.Initialize();
  std::cout << "HIaaaaaaaaaa " << std::endl;

  command_receiver->set_initial_position(
      sys->GetMutableSubsystemContext(simulator.get_mutable_context(),
                                      command_receiver),
      VectorX<double>::Zero(tree.get_num_positions()));
      std::cout << "HI " << std::endl;


  // Simulate for a very long time.
  simulator.StepTo(FLAGS_simulation_sec);
  std::cout << "HI " << std::endl;

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::kuka_iiwa_arm::DoMain();
}
