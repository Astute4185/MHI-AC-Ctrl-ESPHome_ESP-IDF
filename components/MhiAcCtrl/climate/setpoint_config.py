"""Resolve the actual MHI setpoint step from current and legacy YAML settings."""


def resolve_setpoint_step(config):
    candidates = []
    if "setpoint_step" in config:
        candidates.append(("setpoint_step", config["setpoint_step"]))
    if "visual_temperature_step" in config:
        candidates.append(("visual_temperature_step", config["visual_temperature_step"]))

    # ESPHome visual settings used in existing user configurations.
    visual_step = config.get("visual", {}).get("temperature_step")
    if isinstance(visual_step, dict):
        visual_step = visual_step.get("target_temperature")
    if visual_step is not None:
        candidates.append(("visual.temperature_step.target_temperature", visual_step))

    for source, value in candidates:
        if float(value) not in (0.5, 1.0):
            raise ValueError(f"{source} must be either 0.5 or 1.0 °C for MHI setpoints")

    if candidates and any(float(value) != float(candidates[0][1]) for _, value in candidates[1:]):
        raise ValueError(
            "Conflicting setpoint steps: setpoint_step, visual_temperature_step and visual.temperature_step must agree"
        )

    return float(candidates[0][1]) if candidates else 1.0
