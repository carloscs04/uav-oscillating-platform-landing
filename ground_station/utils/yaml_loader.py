import os
import yaml

def load_yaml(file_path: str) -> dict:
    """Safely loads and parses a YAML configuration file."""
    if not os.path.exists(file_path):
        print(f"[Utils Warning] Config file not found at: {file_path}")
        return {}
    
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            return data if data is not None else {}
    except yaml.YAMLError as e:
        print(f"[Utils Error] Failed to parse YAML file {file_path}: {e}")
        return {}

def get_profile_data(config_dir: str, mode: str) -> dict:
    """Extracts network profile parameters for a given mode from profiles.yaml."""
    profiles_path = os.path.join(config_dir, "profiles.yaml")
    config = load_yaml(profiles_path)
    
    profiles = config.get("profiles", {})
    default_mode = config.get("default_profile", "wifi")
    
    target_mode = mode if mode in profiles else default_mode
    profile_info = profiles.get(target_mode, {})
    
    return {
        "mode": target_mode,
        "pi_ip": profile_info.get("pi_ip", "10.22.246.2"), #[cite: 4, 8]
        "port_telemetry": profile_info.get("port_telemetry", 5005),
        "port_command": profile_info.get("port_command", 5006),
        "port_video": profile_info.get("port_video", 8080),
        "fallback_sequence": config.get("profiles", {}).get("auto", {}).get("fallback_sequence", ["wifi", "usb", "ap"])
    }