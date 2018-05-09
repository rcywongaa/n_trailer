#include <iostream>
#include <memory>

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/signal_logger.h>
#include <drake/systems/rendering/pose_aggregator.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/multibody/rigid_body_tree.h>
#include <drake/multibody/rigid_body_plant/rigid_body_plant.h>
#include <drake/multibody/rigid_body_plant/drake_visualizer.h>
#include <drake/multibody/parsers/sdf_parser.h>
#include <drake/automotive/automotive_simulator.h>
#include <drake/automotive/car_vis_applicator.h>
#include <drake/automotive/prius_vis.h>
#include <drake/automotive/box_car_vis.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/lcmtypes/drake/lcmt_viewer_draw.hpp>
#include <drake/lcmtypes/drake/lcmt_viewer_load_robot.hpp>

#include "meta.hpp"
#include "TrailerSystem.hpp"
#include "CarController.hpp"

const int NUM_TRAILERS = 1;
const int TRAILER_LENGTH = 5;

void simulate(){
    drake::systems::DiagramBuilder<double> builder;
    auto plant = builder.AddSystem<TrailerSystem>(NUM_TRAILERS, TRAILER_LENGTH);
    auto controller = builder.AddSystem(std::make_unique<CarController>());
    builder.Connect(plant->state_output(), controller->state_input());
    builder.Connect(controller->driving_command_output(), plant->driving_command_input());

    // Collect all pose outputs
    auto aggregator = builder.AddSystem<drake::systems::rendering::PoseAggregator<double>>();
    auto car_vis_applicator = builder.AddSystem<drake::automotive::CarVisApplicator<double>>();
    for (int id = 0; id < NUM_TRAILERS+1; id++)
    {
        std::string name;
        if (id == 0) name = "Tractor";
        else name = "Trailer" + std::to_string(id);
        const drake::systems::InputPortDescriptor<double>& ports = aggregator->AddSingleInput(name, id);
        builder.Connect(plant->pose_output(id), ports);
        car_vis_applicator->AddCarVis(std::make_unique<drake::automotive::PriusVis<double>>(id, name));
    }
    builder.Connect(aggregator->get_output_port(0), car_vis_applicator->get_car_poses_input_port());
    // Convert pose outputs to draw messages
    auto bundle_to_draw = builder.AddSystem<drake::systems::rendering::PoseBundleToDrawMessage>();
    builder.Connect(car_vis_applicator->get_visual_geometry_poses_output_port(), bundle_to_draw->get_input_port(0));
    drake::lcm::DrakeLcm lcm;
    auto lcm_publisher = builder.AddSystem(drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_viewer_draw>("DRAKE_VIEWER_DRAW", &lcm));
    builder.Connect(bundle_to_draw->get_output_port(0), lcm_publisher->get_input_port());

    auto diagram = builder.Build();

    // Make viewer load robot
    const drake::lcmt_viewer_load_robot load_car_message = car_vis_applicator->get_load_robot_message();
    drake::lcmt_viewer_load_robot load_message;
    load_message.num_links = load_car_message.num_links;
    for (int i = 0; i < load_car_message.num_links; ++i) {
        load_message.link.push_back(load_car_message.link.at(i));
    }
    drake::lcm::Publish(&lcm, "DRAKE_VIEWER_LOAD_ROBOT", load_message);

    // Start simulation
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1.0);
    simulator.Initialize();
    simulator.StepTo(5);

}

int main(int argc, char** argv)
{
    simulate();
};
