#include "evaluation_result.h"

#include <vector>

using namespace std;

EvaluationResult::EvaluationResult() : h_value(UNINITIALIZED) {
}

bool EvaluationResult::is_uninitialized() const {
    return h_value == UNINITIALIZED;
}

bool EvaluationResult::is_infinite() const {
    return h_value == INFTY;
}

ap_float EvaluationResult::get_h_value() const {
    return h_value;
}

const vector<const GlobalOperator *> &
EvaluationResult::get_preferred_operators() const {
    return preferred_operators;
}

bool EvaluationResult::get_count_evaluation() const {
    return count_evaluation;
}

void EvaluationResult::set_h_value(ap_float value) {
    h_value = value;
}

void EvaluationResult::set_preferred_operators(
    std::vector<const GlobalOperator *> &&preferred_ops) {
    preferred_operators = move(preferred_ops);
}

void EvaluationResult::set_count_evaluation(bool count_eval) {
    count_evaluation = count_eval;
}
