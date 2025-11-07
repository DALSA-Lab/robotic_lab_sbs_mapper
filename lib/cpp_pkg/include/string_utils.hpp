/**
 * @file string_utils.hpp
 * @brief A dummy string utility module for demonstration.
 */

#pragma once

#include <string>

namespace string_utils {

/// Default greeting string.
const std::string DEFAULT_GREETING = "Hello";

/**
 * @brief Convert text to uppercase.
 * @param text The input string to convert.
 * @return The uppercase version of the input string.
 */
std::string shout(const std::string &text);

/**
 * @brief A class for formatting strings.
 */
class Formatter {
public:
  /**
   * @brief Surround text with a given symbol.
   * @param text The input string to surround.
   * @param symbol The symbol to use for surrounding (default is "*").
   * @return The formatted string with symbols surrounding the text.
   */
  std::string surround(const std::string &text,
                       const std::string &symbol = "*");
};

} // namespace string_utils
