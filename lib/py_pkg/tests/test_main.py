from py_pkg.math_utils import Calculator, square
from py_pkg.string_utils import Formatter, shout


def main():
    """Execute the main dummy test."""
    calc = Calculator()
    print("Square of 4:", square(4))
    print("Add 2 + 3:", calc.add(2, 3))
    print("Divide 10 / 2:", calc.divide(10, 2))

    print("Shout 'hello':", shout("hello"))

    fmt = Formatter()
    print("Surround 'world':", fmt.surround("world", "#"))


if __name__ == "__main__":
    main()
