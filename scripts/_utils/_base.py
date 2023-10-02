import shutil
from typing import Any

from colorama import Fore, Style

__all__ = ["title", "info", "any"]


def title(string: Any) -> str:
    terminal_width = shutil.get_terminal_size((80, 20))[0]
    return (
        "\n"
        + "-" * terminal_width
        + "\n"
        + " " * ((terminal_width - len(string)) // 2)
        + f"{Style.BRIGHT}{Fore.BLUE}{string.upper()}{Style.RESET_ALL}"
    )


def info(string: Any) -> str:
    return f"{Fore.GREEN}{string}{Style.RESET_ALL}"


def err(string: Any) -> str:
    return f"{Fore.RED}{string}{Style.RESET_ALL}"
