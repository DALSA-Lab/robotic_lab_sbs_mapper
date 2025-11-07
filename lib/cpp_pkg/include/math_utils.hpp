/**
 * @file math_utils.hpp
 * @brief A dummy math utility module for demonstration.
 */

#pragma once

namespace math_utils {

/// The mathematical constant pi.
constexpr double PI = 3.14159;

/**
 * @brief Calculate the square of a number.
 * @param x The number to be squared.
 * @return The square of the input number.
 */
double square(double x);

/**
 * @brief A simple calculator class for basic arithmetic operations.
 */
class Calculator {
public:
  /**
   * @brief Add two numbers.
   * @param a First number.
   * @param b Second number.
   * @return The sum of a and b.
   */
  double add(double a, double b);

  /**
   * @brief Subtract one number from another.
   * @param a First number.
   * @param b Second number to subtract from the first.
   * @return The result of a - b.
   */
  double subtract(double a, double b);

  /**
   * @brief Multiply two numbers.
   * @param a First number.
   * @param b Second number.
   * @return The product of a and b.
   */
  double multiply(double a, double b);

  /**
   * @brief Divide one number by another.
   * @param a Numerator.
   * @param b Denominator.
   * @return The result of a / b.
   * @throws std::invalid_argument if b is zero.
   */
  double divide(double a, double b);
};

} // namespace math_utils