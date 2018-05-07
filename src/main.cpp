#include <iostream>
#include <memory>

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/signal_logger.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/parsers/sdf_parser.h>
#include <drake/automotive/automotive_simulator.h>
#include <drake/lcm/drake_lcm.h>

#include "meta.hpp"
#include "Tractor.hpp"
#include "CarController.hpp"

void simulate(){
    drake::systems::DiagramBuilder<double> builder;
    auto plant = builder.AddSystem<Tractor>();
    auto controller = builder.AddSystem(std::make_unique<CarController>());
    builder.Connect(plant->state_output(), controller->state_input());
    builder.Connect(controller->driving_command_output(), plant->driving_command_input());

    auto visualizer_tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::sdf::AddModelInstancesFromSdfFile(getSrcDir() + "../trailer.sdf", drake::multibody::joints::kFixed, nullptr, visualizer_tree.get());
    auto visualizer_plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(visualizer_tree));
    builder.Connect(plant->visual_output(), visualizer_plant->get_input_port(0));

    drake::lcm::DrakeLcm lcm;
    auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(visualizer_plant->get_rigid_body_tree(), &lcm);
    builder.Connect(visualizer_plant->get_output_port(0), visualizer->get_input_port(0));

    auto diagram = builder.Build();
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.StepTo(5);

}

int main(int argc, char** argv)
{
    simulate();
};
