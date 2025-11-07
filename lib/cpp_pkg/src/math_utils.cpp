#include "math_utils.hpp"
#include <stdexcept>

namespace math_utils {

double square(double x) { return x * x; }

double Calculator::add(double a, double b) { return a + b; }

double Calculator::subtract(double a, double b) { return a - b; }

double Calculator::multiply(double a, double b) { return a * b; }

double Calculator::divide(double a, double b) {
  if (b == 0.0) {
    throw std::invalid_argument("Cannot divide by zero.");
  }
  return a / b;
}

} // namespace math_utils
