"""string_utils.py - A dummy module for demonstration."""

DEFAULT_GREETING = "Hello"


def shout(text):
    """Convert text to uppercase.

    Parameters
    ----------
    text : str
        The input string to convert.

    Returns
    -------
    str
        The uppercase version of the input string.

    """
    return text.upper()


class Formatter:
    """A class for formatting strings."""

    def surround(self, text, symbol="*"):
        """Surround text with a given symbol.

        Parameters
        ----------
        text : str
            The input string to surround.
        symbol : str, optional
            The symbol to use for surrounding (default is '*').

        Returns
        -------
        str
            The formatted string with symbols surrounding the text.

        """
        return f"{symbol}{text}{symbol}"
