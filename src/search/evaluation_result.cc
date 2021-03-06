#include "evaluation_result.h"

#include <vector>

using namespace std;

const float EvaluationResult::INFINITE = numeric_limits<float>::max();

EvaluationResult::EvaluationResult() : h_value(UNINITIALIZED)
{
}

bool EvaluationResult::is_uninitialized() const
{
    return h_value == UNINITIALIZED;
}

bool EvaluationResult::is_infinite() const
{
    return h_value == INFINITE;
}

float EvaluationResult::get_h_value() const
{
    return h_value;
}

const vector<const GlobalOperator *> &
EvaluationResult::get_preferred_operators() const
{
    return preferred_operators;
}

void EvaluationResult::set_h_value(float value)
{
    h_value = value;
}


void EvaluationResult::set_preferred_operators(
    std::vector<const GlobalOperator *>  &preferred_ops)
{
    preferred_operators.insert(preferred_operators.end(), preferred_ops.begin(), preferred_ops.end());
}

void EvaluationResult::set_preferred_operators(
    std::vector<const GlobalOperator *>  &&preferred_ops)
{
    preferred_operators = move(preferred_ops);
}
