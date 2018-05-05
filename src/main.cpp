#include <iostream>
#include <memory>

#include <drake/systems/framework/diagram.h>
#include <drake/systems/primitives/signal_logger.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/parsers/sdf_parser.h>
#include "drake/automotive/gen/simple_car_state.h"
#include <drake/automotive/simple_car.h>
#include <drake/automotive/automotive_simulator.h>
#include <drake/lcm/drake_lcm.h>

#include "meta.hpp"
#include "CarController.hpp"

void simulate(){
    drake::systems::DiagramBuilder<double> builder;
    auto plant = builder.AddSystem<drake::automotive::SimpleCar>();
    auto controller = builder.AddSystem(std::make_unique<CarController>());
    builder.Connect(plant->state_output(), controller->state_input());
    builder.Connect(plant->pose_output(), controller->pose_input());
    builder.Connect(plant->velocity_output(), controller->velocity_input());
    builder.Connect(controller->driving_command_output(), plant->get_input_port(0));

    /*** For using AutomotiveSimulator ***/
    //drake::automotive::AutomotiveSimulator<double> simulator;
    //drake::automotive::SimpleCarState<double> initial_state;
    //simulator.AddPriusSimpleCar("test_car", "test_channel", initial_state);
    //simulator.Start();

    auto visualizer_tree = std::make_unique<RigidBodyTree<double>>();
    drake::parsers::sdf::AddModelInstancesFromSdfFile(getSrcDir() + "../car.sdf", drake::multibody::joints::kFixed, nullptr, visualizer_tree.get());
    auto visualizer_plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(visualizer_tree));
    builder.Connect(plant->state_output(), visualizer_plant->get_input_port(0));

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
