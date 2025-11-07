# Example: Using the Dummy C++ Package

## Demomstration

This example demonstrates how to use the dummy C++ package (see `lib/cpp_pkg`).


```cpp
#include <iostream>
#include "sample.h"

int main() {
	cpppkg::Calculator calc;
	std::cout << "Add: " << calc.add(2, 3) << std::endl;
	std::cout << "Multiply: " << calc.multiply(4, 5) << std::endl;
	return 0;
}
```

## Output Explanation

When you run this program, you will see:

```
Add: 5
Multiply: 20
```

- `Add: 5` is the result of calling `calc.add(2, 3)`, which adds 2 and 3.
- `Multiply: 20` is the result of calling `calc.multiply(4, 5)`, which multiplies 4 and 5.
