#include "SimpleCarStateToPose.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

SimpleCarStateToPose::SimpleCarStateToPose() :
    input_idx(this->DeclareVectorInputPort(drake::automotive::SimpleCarState<double>()).get_index()),
    output_idx(this->DeclareVectorOutputPort(drake::systems::rendering::PoseVector<double>(), &SimpleCarStateToPose::convert).get_index())
{
    ;
}

const drake::systems::InputPort<double>& SimpleCarStateToPose::simple_car_state_input() const
{
    return drake::systems::System<double>::get_input_port(input_idx);
}

const drake::systems::OutputPort<double>& SimpleCarStateToPose::pose_output() const
{
    return drake::systems::System<double>::get_output_port(output_idx);
}

void SimpleCarStateToPose::convert(const drake::systems::Context<double>& context, drake::systems::rendering::PoseVector<double>* output) const
{
    const drake::automotive::SimpleCarState<double>* const state = this->EvalVectorInput<drake::automotive::SimpleCarState>(context, input_idx);
    DRAKE_ASSERT(state != nullptr);
    Eigen::Vector3d translation_vec;
    translation_vec << state->x(), state->y(), 0.0;
    output->set_translation(Eigen::Translation<double, 3>(translation_vec));
    output->set_rotation(Eigen::Quaterniond(Eigen::AngleAxisd(state->heading(), Eigen::Vector3d::UnitZ())));
}

