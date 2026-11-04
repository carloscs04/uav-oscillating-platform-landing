# ground_station/panel/views/stage_tracker_view.py

class StageTrackerView:
    def __init__(self, width: int = 50):
        self.width = width

    def render(self, current_stage: str = "STAGE 2: HEALTH CHECK", master_state: str = "STANDBY") -> list[str]:
        """Renders the System Master State Machine terminal header."""
        lines = []
        lines.append("┌" + "─" * (self.width - 2) + "┐")
        lines.append(f"│ SYSTEM MASTER TERMINAL".ljust(self.width - 1) + "│")
        lines.append("├" + "─" * (self.width - 2) + "┤")
        lines.append(f"│ Active Stage : {current_stage}".ljust(self.width - 1) + "│")
        lines.append(f"│ Master State : [{master_state}]".ljust(self.width - 1) + "│")
        lines.append("└" + "─" * (self.width - 2) + "┘")
        return lines