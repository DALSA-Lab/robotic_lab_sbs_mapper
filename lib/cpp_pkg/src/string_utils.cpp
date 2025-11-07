#include "string_utils.hpp"
#include <algorithm>

namespace string_utils {

std::string shout(const std::string &text) {
  std::string result = text;
  std::transform(result.begin(), result.end(), result.begin(), ::toupper);
  return result;
}

std::string Formatter::surround(const std::string &text,
                                const std::string &symbol) {
  return symbol + text + symbol;
}

} // namespace string_utils