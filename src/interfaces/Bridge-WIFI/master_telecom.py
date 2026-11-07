# interfaces/Bridge-WIFI/master_telecom.py

import os
import sys
import time
from typing import Dict, Any, Set

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from rx_cmd.pc_bridge import PCBridgeListener
from tx_nodes.control_tx import ControlTransmitter


class MasterTelecom:
    """Orquestador principal de telecomunicaciones en la Raspberry Pi.

    Maneja el estado de transmisión global y granular de todos los sub-nodos TX.
    """

    def __init__(self, handshake_port: int = 5007):
        self.handshake_port = handshake_port
        self.target_ip: str = None
        self.tx_nodes: Dict[str, Any] = {}
        self.manually_disabled_nodes: Set[str] = set()  # Toggles deshabilitados desde la GUI

        self.pc_bridge = PCBridgeListener(
            port=self.handshake_port,
            timeout_sec=3.0,
            on_ip_discovered=self._on_gcs_connected,
            on_connection_lost=self._on_gcs_disconnected,
            on_command_received=self._on_command_received
        )

    def start(self):
        print("==================================================")
        print("  INICIANDO RPI MASTER TELECOM (Bridge-WIFI)     ")
        print("==================================================")

        self.pc_bridge.start()
        print("[MASTER_TELECOM] Esperando conexión de Laptop GCS...")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[MASTER_TELECOM] Deteniendo servicios...")
            self.stop()

    def _on_gcs_connected(self, gcs_ip: str):
        """Ejecutado al establecer o reestablecer conexión con la Laptop."""
        print(f"[MASTER_TELECOM] GCS ONLINE -> IP: {gcs_ip}. Reanudando transmisiones activas...")
        self.target_ip = gcs_ip
        self._start_or_update_tx_nodes(gcs_ip)
        self.resume_all_active_nodes()

    def _on_gcs_disconnected(self):
        """Ejecutado al perder la conexión con la Laptop por timeout."""
        print("[MASTER_TELECOM] GCS OFFLINE -> Pausando TODOS los nodos emisores de telemetría...")
        self.pause_all_nodes()

    def pause_all_nodes(self):
        """Pausa la transmisión de absolutamente todos los nodos registrados."""
        for name, node in self.tx_nodes.items():
            if hasattr(node, "pause_stream"):
                node.pause_stream()

    def resume_all_active_nodes(self):
        """Reanuda todos los nodos que NO hayan sido deshabilitados manualmente en la GUI."""
        for name, node in self.tx_nodes.items():
            if name not in self.manually_disabled_nodes and hasattr(node, "resume_stream"):
                node.resume_stream()

    def _on_command_received(self, cmd: str, payload: Dict[str, Any], gcs_ip: str):
        """Maneja comandos individuales enviados desde los botones de la GUI."""
        target_node = payload.get("node")

        if cmd == "STOP_STREAM":
            print(f"[MASTER_TELECOM] Comando 'STOP_STREAM' en nodo: {target_node or 'ALL'}")
            if target_node:
                # Mapear nombre visual del nodo a la llave del diccionario tx_nodes
                node_key = "control_tx" if target_node == "3-Axis Control Node" else target_node
                self.manually_disabled_nodes.add(node_key)
                if node_key in self.tx_nodes:
                    self.tx_nodes[node_key].pause_stream()
            else:
                self.pause_all_nodes()

        elif cmd == "START_STREAM":
            print(f"[MASTER_TELECOM] Comando 'START_STREAM' en nodo: {target_node or 'ALL'}")
            if target_node:
                node_key = "control_tx" if target_node == "3-Axis Control Node" else target_node
                self.manually_disabled_nodes.discard(node_key)
                if node_key in self.tx_nodes and self.pc_bridge.is_gcs_connected:
                    self.tx_nodes[node_key].resume_stream()

    def _start_or_update_tx_nodes(self, target_ip: str):
        if "control_tx" not in self.tx_nodes:
            print(f"[MASTER_TELECOM] Inicializando control_tx hacia {target_ip}:5007")
            tx = ControlTransmitter(target_ip=target_ip, port=5007)
            tx.start()
            self.tx_nodes["control_tx"] = tx
        else:
            self.tx_nodes["control_tx"].update_target_ip(target_ip)

    def stop(self):
        self.pc_bridge.stop()
        for name, node in self.tx_nodes.items():
            if hasattr(node, "stop"):
                node.stop()
        print("[MASTER_TELECOM] Todos los servicios detenidos.")


if __name__ == "__main__":
    master = MasterTelecom(handshake_port=5007)
    master.start()