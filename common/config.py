from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]

def load_config(path=None):
    """Load the shared YAML config. Pass a path to use another file."""
    with open(path or ROOT / "config.yaml") as f:
        return yaml.safe_load(f)
