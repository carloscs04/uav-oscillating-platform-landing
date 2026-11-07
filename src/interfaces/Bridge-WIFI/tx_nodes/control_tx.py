# interfaces/Bridge-WIFI/tx_nodes/control_tx.py

import json
import math
import socket
import threading
import time
from typing import Any, Callable, List, Optional, Tuple


class ControlTransmitter(threading.Thread):
    def __init__(
        self,
        target_ip: str = "127.0.0.1",
        port: int = 5007,
        control_node: Optional[Any] = None,
        data_provider: Optional[Callable[[], Tuple[List[float], List[float]]]] = None,
        **kwargs
    ):
        super().__init__(daemon=True)
        self.target_ip = kwargs.get("broadcast_ip", target_ip)
        self.port = port
        self.control_node = control_node
        self.data_provider = data_provider

        self.running = False
        self.streaming = True  # Bandera para pausar/reanudar la transmisión
        self._sock: Optional[socket.socket] = None

    def update_target_ip(self, new_ip: str):
        self.target_ip = new_ip
        print(f"[CONTROL_TX] Nueva IP objetivo asignada: {self.target_ip}")

    def pause_stream(self):
        """Pausa el envío de paquetes UDP."""
        if self.streaming:
            self.streaming = False
            print("[CONTROL_TX] Transmisión PAUSADA por orden del GCS.")

    def resume_stream(self):
        """Reanuda el envío de paquetes UDP."""
        if not self.streaming:
            self.streaming = True
            print("[CONTROL_TX] Transmisión REANUDADA por orden del GCS.")

    def run(self):
        self.running = True
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        print(f"[CONTROL_TX] Transmitiendo telemetría a {self.target_ip}:{self.port}")
        t_start = time.time()

        while self.running:
            # Si no hay IP objetivo o la transmisión está pausada, esperamos sin enviar paquetes
            if not self.target_ip or not self.streaming:
                time.sleep(0.1)
                continue

            ref, act = self._get_telemetry_data(time.time() - t_start)
            pkt = json.dumps({
                "node": "3-Axis Control Node",
                "ref": ref,
                "act": act,
                "timestamp": time.time()
            }).encode("utf-8")

            try:
                self._sock.sendto(pkt, (self.target_ip, self.port))
            except Exception as e:
                print(f"[CONTROL_TX_ERR] Error al enviar paquete a {self.target_ip}: {e}")

            time.sleep(0.05)  # Loop rate a 20 Hz (50 ms)

        if self._sock:
            self._sock.close()

    def _get_telemetry_data(self, elapsed: float) -> Tuple[List[float], List[float]]:
        if self.data_provider is not None:
            try:
                return self.data_provider()
            except Exception as e:
                print(f"[CONTROL_TX_ERR] Error en data_provider: {e}")

        if self.control_node is not None and hasattr(self.control_node, "get_current_state"):
            try:
                return self.control_node.get_current_state()
            except Exception as e:
                print(f"[CONTROL_TX_ERR] Error en control_node.get_current_state(): {e}")

        ref_x = math.sin(elapsed * 0.8)
        ref_y = math.cos(elapsed * 0.8)
        ref_z = 1.5

        act_x = ref_x * 0.95 + 0.02 * math.sin(elapsed * 4.0)
        act_y = ref_y * 0.95 + 0.02 * math.cos(elapsed * 4.0)
        act_z = ref_z - 0.03 + 0.01 * math.sin(elapsed * 2.0)

        return [round(ref_x, 3), round(ref_y, 3), round(ref_z, 3)], \
               [round(act_x, 3), round(act_y, 3), round(act_z, 3)]

    def stop(self):
        self.running = False
        self.streaming = False