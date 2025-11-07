#include <iostream>
#include "math_utils.hpp"
#include "string_utils.hpp"

int main() {
    using namespace math_utils;
    using namespace string_utils;

    Calculator calc;
    std::cout << "Square of 4: " << square(4) << "\n";
    std::cout << "Add 2 + 3: " << calc.add(2, 3) << "\n";
    std::cout << "Divide 10 / 2: " << calc.divide(10, 2) << "\n";

    std::cout << "Shout 'hello': " << shout("hello") << "\n";

    Formatter fmt;
    std::cout << "Surround 'world': " << fmt.surround("world", "#") << "\n";

    return 0;
}