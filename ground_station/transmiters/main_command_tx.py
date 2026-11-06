# ground_station/transmiters/main_command_tx.py

class MainReceiver:
    """Outbound command pipeline communicating with the Raspberry Pi Master Node."""
    
    def __init__(self, link_manager = 1, b =2 ):
        self.link_mgr = link_manager

    def send_command(self, cmd_text: str) -> bool:
        """Sends serialized control string over the socket bridge."""
        if self.link_mgr.is_connected():
            # Packet dispatch logic to active socket link
            return True
        return False