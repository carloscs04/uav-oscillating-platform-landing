import os
import yaml


def load_yaml_config(config_path: str, default_values: dict = None) -> dict:
    """
    Generic YAML configuration reader for onboard modules.
    Returns fallback defaults if the YAML file is missing or unreadable.
    """
    config = default_values if default_values is not None else {}
    
    if os.path.exists(config_path):
        try:
            with open(config_path, "r") as file:
                loaded = yaml.safe_load(file)
                if isinstance(loaded, dict):
                    config.update(loaded)
        except Exception as error:
            print(f"[ConfigLoader] Error reading {config_path}: {error}. Using defaults.")
    else:
        print(f"[ConfigLoader] Config file not found at '{config_path}'. Using defaults.")
        
    return config