# ground_station/utils/colors.py

import shutil

# ANSI Color Codes
RESET = "\033[0m"
BOLD = "\033[1m"

# Text / Foreground Colors
FG_CYAN = "\033[96m"         # Bright Cyan-Blue Text
FG_BLUE = "\033[94m"         # Light Blue Text
FG_WHITE = "\033[97m"        # High Contrast White
FG_YELLOW = "\033[93m"       # Warning Yellow

# Background Colors
BG_DARK_BLUE = "\033[48;5;18m"  # Deep Dark Blue Canvas
BG_STANDARD_BLUE = "\033[44m"   # Standard Blue

def draw_canvas_row(text: str, fg=FG_CYAN, bg=BG_DARK_BLUE, width: int = 100) -> str:
    """Pads text with spaces to fill the full terminal width in dark blue background."""
    padded_text = text.ljust(width)
    return f"{bg}{fg}{BOLD}{padded_text}{RESET}"