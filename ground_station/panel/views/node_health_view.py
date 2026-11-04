class NodeHealthView:
    def __init__(self, width: int = 50):
        self.width = width

    def render(self, node_states: dict) -> list[str]:
        """Renders status for MAVLink, Distance-Sensor, ArUco, and WiFi nodes."""
        lines = []
        lines.append("┌" + "─" * (self.width - 2) + "┐")
        lines.append(f"│ SUB-NODES STATUS MATRIX".ljust(self.width - 1) + "│")
        lines.append("├" + "─" * (self.width - 2) + "┤")
        
        for node_name, status in node_states.items():
            formatted_line = f"│  * {node_name:<24}: [{status}]"
            lines.append(formatted_line.ljust(self.width - 1) + "│")
            
        lines.append("└" + "─" * (self.width - 2) + "┘")
        return lines