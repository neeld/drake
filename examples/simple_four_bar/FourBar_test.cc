/// @file
///
/// Implements a simulation of the Cassie bipedal robot.

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/common/text_logging.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/frame_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");
DEFINE_string(urdf, "", "Name of urdf to load");
DEFINE_bool(visualize_frames, true, "Visualize end effector frames");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

namespace drake {
namespace examples {
namespace simple_four_bar {
namespace {
using drake::manipulation::util::SimDiagramBuilder;
using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::FrameVisualizer;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
template<typename Scalar>
using VectorX = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

int DoMain() {
  drake::lcm::DrakeLcm lcm;
  SimDiagramBuilder<double> builder;

  // Adds a plant.
  RigidBodyPlant<double>* plant = nullptr;
  const std::string urdf = "drake/examples/simple_four_bar/FourBar_sq.urdf";
  {
    auto tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        FindResourceOrThrow(urdf),
        drake::multibody::joints::kFixed, tree.get());

    auto plantDt = std::make_unique<RigidBodyPlant<double>>(std::move(tree),0.0005);
    plant = builder.AddPlant(std::move(plantDt));
  }
  
  // Creates and adds LCM publisher for visualization.
  builder.AddVisualizer(&lcm);
  builder.get_visualizer()->set_publish_period(1e-2);

  drake::systems::DiagramBuilder<double>* base_builder = builder.get_mutable_builder();

  VectorX<double> constant_vector(plant->get_input_port(0).size());
  constant_vector.setZero();
  auto constant_zero_source = base_builder->AddSystem<ConstantVectorSource<double>>(constant_vector);
  constant_zero_source->set_name("zero input");

  // Connects the blank input command
  base_builder->Connect(constant_zero_source->get_output_port(),
                  plant->get_input_port(0));

  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  lcm.StartReceiveThread();
  simulator.set_publish_every_time_step(true);
  simulator.set_target_realtime_rate(0.1);

  VectorX<double> startPos(6);
  startPos << -1.57080,1.57080,1.57080,0,0,0;
  // VectorX<double> startPos(4);
  // startPos << -1.57080,1.57080,0,0;

  plant->set_state_vector(&simulator.get_mutable_context(),startPos);
  simulator.Initialize();

  simulator.StepTo(1.0);

  return 0;
}

}  // namespace
}  // namespace simple_four_bar
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::logging::HandleSpdlogGflags();
  return drake::examples::simple_four_bar::DoMain();
}