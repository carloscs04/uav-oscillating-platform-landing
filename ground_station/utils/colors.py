# ground_station/utils/colors.py

import shutil

# ANSI Terminal Color Codes (Console Logging)
RESET = "\033[0m"
BOLD = "\033[1m"
FG_CYAN_ANSI = "\033[96m"
FG_BLUE_ANSI = "\033[94m"
FG_WHITE_ANSI = "\033[97m"
FG_YELLOW_ANSI = "\033[93m"
BG_DARK_BLUE = "\033[48;5;18m"
BG_STANDARD_BLUE = "\033[44m"

# Tkinter GUI Hex Color Palette
BG_DARK = "#091428"
BG_CARD = "#102a45"
FG_CYAN = "#00f0ff"
FG_BLUE = "#4a9eff"
FG_WHITE = "#ffffff"
ACCENT_GREEN = "#00e676"
ACCENT_RED = "#ff5252"
ACCENT_YELLOW = "#ffb300"

# MATLAB / Trajectory Plot Color Palette
BG_PLOT_CANVAS = "#050e18"
COLOR_PLOT_REF = ACCENT_RED       # Target trajectory dotted line (#ff5252)
COLOR_PLOT_ACT = ACCENT_GREEN     # Actual feedback solid line (#00e676)
COLOR_PLOT_GRID = "#1a2636"      # Background grid lines
COLOR_PLOT_AXIS = "#37474f"      # Canvas border axis
COLOR_PLOT_ZERO = "#263238"      # Zero baseline grid line
COLOR_PLOT_TEXT = "#546e7a"      # Scale text numbers


def draw_canvas_row(text: str, fg=FG_CYAN_ANSI, bg=BG_DARK_BLUE, width: int = 100) -> str:
    """Pads text with spaces to fill full terminal width."""
    padded_text = text.ljust(width)
    return f"{bg}{fg}{BOLD}{padded_text}{RESET}"