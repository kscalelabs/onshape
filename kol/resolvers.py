"""Defines a utility class for resolving expression values."""

import logging
import math

logger = logging.getLogger(__name__)


class ExpressionResolver:
    def __init__(self, configuration_parameters: dict[str, str]) -> None:
        super().__init__()

        self.configuration_parameters = configuration_parameters

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
        parts = expression.split(" ")
        if len(parts) != 2:
            raise ValueError(f"Invalid expression: {expression}")
        value, unit = parts

        # Checking the unit, returning values in radians and meters.
        match unit:
            case "deg":
                return math.radians(float(value))
            case "radian", "rad":
                if isinstance(value, str):
                    if value == "(PI)":
                        val = math.pi
                    else:
                        raise ValueError(f"{value} variable isn't supported")
                else:
                    val = value
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

    def read_parameter_value(self, parameter: dict, name: str) -> float:
        if parameter["typeName"] == "BTMParameterNullableQuantity":
            return self.read_expression(parameter["message"]["expression"])

        if parameter["typeName"] == "BTMParameterConfigured":
            message = parameter["message"]
            parameter_value = self.configuration_parameters[message["configurationParameterId"]]

            for value in message["values"]:
                if value["typeName"] == "BTMConfiguredValueByBoolean":
                    boolean_value = parameter_value == "true"
                    if value["message"]["booleanValue"] == boolean_value:
                        return self.read_expression(value["message"]["value"]["message"]["expression"])
                elif value["typeName"] == "BTMConfiguredValueByEnum":
                    if value["message"]["enumValue"] == parameter_value:
                        return self.read_expression(value["message"]["value"]["message"]["expression"])
                else:
                    raise ValueError(f"Can't read value of parameter {name} configured with {value['typeName']}")
            raise ValueError(f"Could not find the value for {name}")
        raise ValueError(f"Unknown feature type for {name}: {parameter['typeName']}")


class JointLimitResolver:
    def __init__(self, joint_features: dict, expression_resolver: ExpressionResolver) -> None:
        super().__init__()

        self.joint_features = joint_features
        self.expression_resolver = expression_resolver

    def get_limits(self, joint_type: str, name: str) -> tuple[float, float] | None:
        enabled = False
        minimum, maximum = 0.0, 0.0

        if joint_type not in ("revolute", "prismatic", "continuous"):
            logger.warning("Unknown joint type: %s", joint_type)

        for feature in self.joint_features["features"]:
            if name != feature["message"]["name"]:
                continue

            for parameter in feature["message"]["parameters"]:
                if parameter["message"]["parameterId"] == "limitsEnabled":
                    enabled = parameter["message"]["value"]

                match joint_type:
                    case "revolute":
                        if parameter["message"]["parameterId"] == "limitAxialZMin":
                            minimum = self.expression_resolver.read_parameter_value(parameter, name)
                        if parameter["message"]["parameterId"] == "limitAxialZMax":
                            maximum = self.expression_resolver.read_parameter_value(parameter, name)

                    case "prismatic":
                        if parameter["message"]["parameterId"] == "limitZMin":
                            minimum = self.expression_resolver.read_parameter_value(parameter, name)
                        if parameter["message"]["parameterId"] == "limitZMax":
                            maximum = self.expression_resolver.read_parameter_value(parameter, name)

        if enabled:
            return (minimum, maximum)
        if joint_type != "continuous":
            logger.warning("Joint %s of type %s has no limits", name, joint_type)
        return None
