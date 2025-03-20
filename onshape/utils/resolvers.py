"""Defines a utility class for resolving expression values."""

import logging
import math
import re

logger = logging.getLogger(__name__)


class ExpressionResolver:
    def __init__(self, configuration: str) -> None:
        super().__init__()

        # Parses the configuration string to a dictionary.
        self.configuration_parameters: dict[str, str] = {}
        if configuration != "default":
            for kv in configuration.split(";"):
                key, value = kv.split("=")
                self.configuration_parameters[key] = value

    def read_expression(self, expression: str) -> float:
        """Reads an expression and returns a float value.

        Args:
            expression: The expression to read.

        Returns:
            The float value of the expression.
        """
        if expression[0] == "#":
            expression = self.configuration_parameters[expression[1:]]
        if expression[0:2] == "-#":
            expression = "-" + self.configuration_parameters[expression[2:]]

        # Splitting the expression into value and unit.
        parts = re.split(r"[ +*]", expression)
        if len(parts) != 2:
            raise ValueError(f"Invalid expression: {expression}")
        value, unit = parts

        # Checking the unit, returning values in radians and meters.
        match unit:
            case "deg":
                return math.radians(float(value))
            case "radian" | "rad":
                try:
                    val = float(value)
                except ValueError:
                    if value == "(PI)":
                        val = math.pi
                    else:
                        raise ValueError(f"{value} variable isn't supported")
                return float(val)
            case "mm":
                return float(value) / 1000.0
            case "m":
                return float(value)
            case "cm":
                return float(value) / 100.0
            case "in":
                return float(value) * 0.0254
            case "ft":
                return float(value) * 0.3048
            case _:
                raise ValueError(f"{unit} unit isn't supported")
