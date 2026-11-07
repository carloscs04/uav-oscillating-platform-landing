# interfaces/Bridge-WIFI/master_telecom.py

import os
import sys
import time
from typing import Dict, Any

# Agregar directorio actual al path para importar submódulos
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rx_cmd.pc_bridge import PCBridgeListener
from tx_nodes.control_tx import ControlTransmitter
# Descomentar e importar los demás nodos según corresponda:
# from tx_nodes.camera_tx import CameraTransmitter
# from tx_nodes.distance_sensor_tx import DistanceSensorTransmitter
# from tx_nodes.mavlink_tx import MavlinkTransmitter


class MasterTelecom:
    """Orquestador principal de telecomunicaciones en la Raspberry Pi.

    1. Inicia PCBridgeListener para recibir el handshake de la laptop. 2.
    Obtiene la IP de origen (unicast). 3. Inicia e inyecta la IP objetivo a
    todos los nodos transmisores (TX).
    """

    def __init__(self, handshake_port: int = 5007):
        self.handshake_port = handshake_port
        self.target_ip: str = None
        self.tx_nodes: Dict[str, Any] = {}

        # 1. Instanciar escucha de Handshake
        self.pc_bridge = PCBridgeListener(
            port=self.handshake_port,
            on_ip_discovered=self._on_gcs_ip_discovered
        )

    def start(self):
        print("==================================================")
        print("  INICIANDO RPI MASTER TELECOM (Bridge-WIFI)     ")
        print("==================================================")

        # Paso 1: Iniciar el receptor de Handshake
        self.pc_bridge.start()
        print("[MASTER_TELECOM] Esperando handshake inicial de la Laptop GCS...")

        # Paso 2: Mantener loop principal activo
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[MASTER_TELECOM] Deteniendo servicios...")
            self.stop()

    def _on_gcs_ip_discovered(self, gcs_ip: str):
        """Callback ejecutado cuando pc_bridge.py captura la IP de la laptop."""
        print(f"[MASTER_TELECOM] IP objetivo obtenida: {gcs_ip}")
        self.target_ip = gcs_ip
        self._start_or_update_tx_nodes(gcs_ip)

    def _start_or_update_tx_nodes(self, target_ip: str):
        """Inicializa o actualiza la IP destino en todos los nodos emisores."""
        
        # --- Nodo 3-Axis Control TX ---
        if "control_tx" not in self.tx_nodes:
            print(f"[MASTER_TELECOM] Inicializando control_tx hacia {target_ip}:5007")
            tx = ControlTransmitter(target_ip=target_ip, port=5007)
            tx.start()
            self.tx_nodes["control_tx"] = tx
        else:
            print(f"[MASTER_TELECOM] Actualizando IP objetivo en control_tx: {target_ip}")
            self.tx_nodes["control_tx"].update_target_ip(target_ip)

        # --- Agregar aquí la inicialización para los demás nodos transmisores ---
        # if "camera_tx" not in self.tx_nodes:
        #     cam_tx = CameraTransmitter(target_ip=target_ip, port=8080)
        #     cam_tx.start()
        #     self.tx_nodes["camera_tx"] = cam_tx

    def stop(self):
        self.pc_bridge.stop()
        for name, node in self.tx_nodes.items():
            if hasattr(node, "stop"):
                node.stop()
        print("[MASTER_TELECOM] Todos los servicios detenidos.")


if __name__ == "__main__":
    master = MasterTelecom(handshake_port=5007)
    master.start()