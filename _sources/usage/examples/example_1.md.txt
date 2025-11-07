# Example: Using the Dummy Python Package

## Demomstration

This example demonstrates how to use the dummy Python package located in `lib/py_pkg`.

```python
from py_pkg import math_utils, string_utils, module

# Math utilities
print("PI:", math_utils.PI)
print("Square of 4:", math_utils.square(4))

calc = math_utils.Calculator()
print("Add:", calc.add(2, 3))
print("Divide:", calc.divide(10, 2))

# String utilities
print(string_utils.DEFAULT_GREETING)
print(string_utils.shout("hello world"))

fmt = string_utils.Formatter()
print(fmt.surround("example", "#"))
```

## Output Explanation

When you run this code, you will see output similar to:

```
PI: 3.14159
Square of 4: 16
Add: 5
Divide: 5.0
Hello
HELLO WORLD
#example#
```

- `PI: 3.14159` prints the value of `math_utils.PI`.
- `Square of 4: 16` is the result of `math_utils.square(4)`.
- `Add: 5` is the result of `calc.add(2, 3)`.
- `Divide: 5.0` is the result of `calc.divide(10, 2)`.
- `Hello` is the value of `string_utils.DEFAULT_GREETING`.
- `HELLO WORLD` is the result of `string_utils.shout("hello world")`.
- `#example#` is the result of `fmt.surround("example", "#")`.

