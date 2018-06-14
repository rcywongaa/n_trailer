#include "Tractor.hpp"

#include "drake/automotive/gen/driving_command.h"
#include "drake/automotive/gen/simple_car_state.h"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/rendering/frame_velocity.h>

#include <iostream>

using namespace drake::systems;
using namespace drake::automotive;

class FrameVelocityConverter : public drake::systems::LeafSystem<double>
{
    public:
        FrameVelocityConverter() :
            input_idx(this->DeclareVectorInputPort(rendering::FrameVelocity<double>()).get_index()),
            output_idx(this->DeclareVectorOutputPort(SimpleCarState<double>(), &FrameVelocityConverter::convert).get_index())
        {
            ;
        }
        const drake::systems::InputPortDescriptor<double>& simple_car_state_input() const
        {
            return System<double>::get_input_port(input_idx);
        }
        const drake::systems::OutputPort<double>& simple_car_state_output() const
        {
            return System<double>::get_output_port(output_idx);
        }
    private:
        void convert(const Context<double>& context, SimpleCarState<double>* output) const
        {
            const rendering::FrameVelocity<double>* const velocity = this->EvalVectorInput<rendering::FrameVelocity>(context, input_idx);
            DRAKE_ASSERT(velocity != nullptr);
            // FrameVelocity element order: {wx, wy, wz, vx, vy, vz}
            output->set_x(velocity->GetAtIndex(3));
            output->set_y(velocity->GetAtIndex(4));
            output->set_heading(velocity->GetAtIndex(2));
            output->set_velocity(0.0);
        }
        int input_idx;
        int output_idx;
};

Tractor::Tractor()
{
    DiagramBuilder<double> builder;
    simple_tractor = builder.AddSystem<SimpleCar<double>>();
    // Driving Command input
    driving_command_idx = builder.ExportInput(simple_tractor->get_input_port(0));
    // State output
    state_idx = builder.ExportOutput(simple_tractor->state_output());
    // Velocity output
    auto state_d_converter = builder.AddSystem<FrameVelocityConverter>();
    builder.Connect(simple_tractor->velocity_output(), state_d_converter->simple_car_state_input());
    state_d_idx = builder.ExportOutput(state_d_converter->simple_car_state_output());
    builder.BuildInto(this);

    std::cout << "Tractor driving_command_idx input = " << std::to_string(driving_command_idx) << std::endl;
    std::cout << "Tractor state_idx output = " << std::to_string(state_idx) << std::endl;
    std::cout << "Tractor state_d_idx output = " << std::to_string(state_d_idx) << std::endl;
}

const InputPortDescriptor<double>& Tractor::driving_command_input() const
{
    return this->get_input_port(driving_command_idx);
}

const OutputPort<double>& Tractor::state_output() const
{
    return this->get_output_port(state_idx);
}

const OutputPort<double>& Tractor::state_d_output() const
{
    return this->get_output_port(state_d_idx);
}
