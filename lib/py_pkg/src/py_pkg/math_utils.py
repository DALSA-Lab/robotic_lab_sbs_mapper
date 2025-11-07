"""math_utils.py - A dummy module for demonstration."""

PI = 3.14159


def square(x):
    """Calculate the square of a number.

    Parameters
    ----------
    x : float or int
        The number to be squared.

    Returns
    -------
    float or int
        The square of the input number.

    """
    return x * x


class Calculator:
    """A simple calculator class for basic arithmetic operations."""

    def add(self, a, b):
        """Add two numbers.

        Parameters
        ----------
        a : float or int
            First number.
        b : float or int
            Second number.

        Returns
        -------
        float or int
            The sum of a and b.

        """
        return a + b

    def subtract(self, a, b):
        """Subtract one number from another.

        Parameters
        ----------
        a : float or int
            First number.
        b : float or int
            Second number to subtract from the first.

        Returns
        -------
        float or int
            The result of a - b.

        """
        return a - b

    def multiply(self, a, b):
        """Multiply two numbers.

        Parameters
        ----------
        a : float or int
            First number.
        b : float or int
            Second number.

        Returns
        -------
        float or int
            The product of a and b.

        """
        return a * b

    def divide(self, a, b):
        """Divide one number by another.

        Parameters
        ----------
        a : float or int
            Numerator.
        b : float or int
            Denominator.

        Returns
        -------
        float
            The result of a / b.

        Raises
        ------
        ValueError
            If b is zero.

        """
        if b == 0:
            raise ValueError("Cannot divide by zero.")
        return a / b
