# ground_station/panel/views/camera_view.py

class CameraView:
    def __init__(self, width: int = 38, height: int = 20):
        self.width = width
        self.height = height

    def render(self, stream_active: bool = True, fps: int = 30, res: str = "640x480") -> list[str]:
        """Renders the left video camera streaming display window."""
        lines = []
        status = "STREAMING [LIVE]" if stream_active else "STREAM DISCONNECTED"
        
        lines.append("┌" + "─" * (self.width - 2) + "┐")
        lines.append(f"│ CAMERA FEED ({res} @ {fps}fps)".ljust(self.width - 1) + "│")
        lines.append("├" + "─" * (self.width - 2) + "┤")
        
        # Stream box interior
        inner_height = self.height - 6
        for i in range(inner_height):
            if i == inner_height // 2 - 1:
                content = f"[{status}]".center(self.width - 2)
            elif i == inner_height // 2:
                content = "┌──────────────────────┐".center(self.width - 2)
            elif i == inner_height // 2 + 1:
                content = "│   ArUco Target       │".center(self.width - 2)
            elif i == inner_height // 2 + 2:
                content = "└──────────────────────┘".center(self.width - 2)
            else:
                content = " ".center(self.width - 2)
            lines.append("│" + content + "│")
            
        lines.append("├" + "─" * (self.width - 2) + "┤")
        lines.append(f"│ Network: HTTP MJPEG (Port 8080)".ljust(self.width - 1) + "│")
        lines.append("└" + "─" * (self.width - 2) + "┘")
        return lines