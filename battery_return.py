LOW_BATTERY_RETURN_THRESHOLD = 10


def should_return_to_charge(voltage, threshold=LOW_BATTERY_RETURN_THRESHOLD):
    if voltage is None:
        return False
    try:
        voltage_value = int(float(str(voltage).strip()))
    except Exception:
        return False
    return voltage_value < threshold
