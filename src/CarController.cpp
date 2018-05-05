#include "CarController.hpp"

#include "drake/automotive/gen/simple_car_state.h"
#include "drake/systems/rendering/frame_velocity.h"
#include "drake/systems/rendering/pose_vector.h"

using namespace drake::systems;
using namespace drake::systems::rendering;
using namespace drake::automotive;

CarController::CarController() :
    state_input_idx(this->DeclareVectorInputPort(SimpleCarState<double>()).get_index()),
    pose_input_idx(this->DeclareVectorInputPort(PoseVector<double>()).get_index()),
    velocity_input_idx(this->DeclareVectorInputPort(FrameVelocity<double>()).get_index()),
    driving_command_output_idx(this->DeclareVectorOutputPort(DrivingCommand<double>(), &CarController::create_drive_command).get_index())
{
    ;
}               

const InputPortDescriptor<double>& CarController::state_input() const
{
    return System<double>::get_input_port(state_input_idx);
}

const InputPortDescriptor<double>& CarController::pose_input() const
{
    return System<double>::get_input_port(pose_input_idx);
}

const InputPortDescriptor<double>& CarController::velocity_input() const
{
    return System<double>::get_input_port(velocity_input_idx);
}

const OutputPort<double>& CarController::driving_command_output() const
{
    return System<double>::get_output_port(driving_command_output_idx);
}

void CarController::create_drive_command(const Context<double>& context, DrivingCommand<double>* output) const
{
    const SimpleCarState<double>* const state = this->EvalVectorInput<SimpleCarState>(context, state_input_idx);
    DRAKE_ASSERT(state != nullptr);
    std::cout << "State: " << *state << std::endl;

    const PoseVector<double>* const pose = this->EvalVectorInput<PoseVector>(context, pose_input_idx);
    DRAKE_ASSERT(pose != nullptr);
    std::cout << "Pose: " << *pose << std::endl;

    const FrameVelocity<double>* const velocity = this->EvalVectorInput<FrameVelocity>(context, velocity_input_idx);
    DRAKE_ASSERT(velocity != nullptr);
    std::cout << "Velocity: " << *velocity << std::endl;

    output->set_steering_angle(0.0);
    output->set_acceleration(10);
}
