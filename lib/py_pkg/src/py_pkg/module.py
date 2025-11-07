"""module.py - A dummy module for demonstration.

This module contains simple functions with docstrings for Sphinx documentation.
"""


def add(a: int, b: int) -> int:
    """Add two numbers.

    Args:
        a (int or float): First number.
        b (int or float): Second number.

    Returns:
        int or float: The sum of a and b.

    """
    return a + b


def greet(name: str) -> str:
    """Generate a greeting message.

    Args:
        name (str): Name of the person.

    Returns:
        str: Greeting message.

    """
    return f"Hello, {name}!"
