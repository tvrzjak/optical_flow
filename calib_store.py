"""Uložení/načtení kalibrace (offset výšky, nulový bod flow) do JSON souboru."""
import json
import os


def load_calibration(path: str):
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        return {'dist_offset': data['dist_offset'], 'bias_x': data['bias_x'], 'bias_y': data['bias_y']}
    except (json.JSONDecodeError, KeyError, OSError):
        return None


def save_calibration(path: str, calibration: dict):
    with open(path, 'w') as f:
        json.dump(calibration, f, indent=2)
