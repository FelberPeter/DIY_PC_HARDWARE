import os
import json
import threading
import queue
import subprocess
import tkinter as tk
from tkinter import ttk
import shutil
import sys

try:
    import serial
    import serial.tools.list_ports
except Exception:
    serial = None


ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
SENDER_DIR = os.path.join(ROOT_DIR, "esp_sender")
RECEIVER_DIR = os.path.join(ROOT_DIR, "esp_receiver")
PICO_DIR = os.path.join(ROOT_DIR, "pico_bridge")
CONFIG_PATH = os.path.join(ROOT_DIR, "dev_config.json")

DEFAULT_CONFIG = {
    "sender_port": "",
    "receiver_port": "",
    "pico_port": "",
    "pico_mount": "",
}


def load_config():
    if not os.path.exists(CONFIG_PATH):
        return DEFAULT_CONFIG.copy()
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
            cfg = DEFAULT_CONFIG.copy()
            cfg.update({k: v for k, v in data.items() if k in cfg})
            return cfg
    except Exception:
        return DEFAULT_CONFIG.copy()


def save_config(cfg):
    with open(CONFIG_PATH, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


def read_upload_port(ini_path):
    try:
        with open(ini_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line.lower().startswith("upload_port"):
                    parts = line.split("=", 1)
                    if len(parts) == 2:
                        return parts[1].strip()
    except Exception:
        pass
    return ""


def pio_command():
    if shutil.which("pio"):
        return "pio"
    return f"\"{sys.executable}\" -m platformio"


def ensure_subst(path):
    if os.name != "nt" or " " not in path:
        return path
    try:
        out = subprocess.check_output(["cmd", "/c", "subst"], text=True)
    except Exception:
        out = ""
    mappings = {}
    for line in out.splitlines():
        if "=> " in line:
            left, right = line.split("=>", 1)
            drive = left.strip().rstrip("\\")
            target = right.strip()
            mappings[drive.upper()] = target
    for drive, target in mappings.items():
        if os.path.normcase(target) == os.path.normcase(path):
            return drive + "\\"
    for letter in ["S", "T", "U", "V", "W", "X", "Y", "Z"]:
        drive = f"{letter}:"
        if drive not in mappings:
            try:
                subprocess.run(["cmd", "/c", "subst", drive, path], check=True)
                return drive + "\\"
            except Exception:
                continue
    return path


def find_pico_mount():
    for letter in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
        root = f"{letter}:\\"
        info = os.path.join(root, "INFO_UF2.TXT")
        if os.path.exists(info):
            return root
    return ""


def copy_uf2_to_mount(mount):
    uf2_path = os.path.join(PICO_DIR, ".pio", "build", "pico", "firmware.uf2")
    if not os.path.exists(uf2_path):
        return f"UF2 not found: {uf2_path}"
    if not mount:
        return "Pico mount not set"
    dest = os.path.join(mount, "firmware.uf2")
    with open(uf2_path, "rb") as src, open(dest, "wb") as dst:
        dst.write(src.read())
    return f"Copied UF2 to {dest}"


class DevGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DIY PC Hardware Dev GUI")
        self.geometry("920x640")

        self.log_queue = queue.Queue()
        self.serial_queue = queue.Queue()
        self.serial_thread = None
        self.serial_stop = threading.Event()
        self.ser = None

        self.cfg = load_config()

        self._build_ui()
        self.after(50, self._drain_log)
        self.after(50, self._drain_serial)

    def _build_ui(self):
        main = ttk.Frame(self, padding=10)
        main.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(main)
        top.pack(fill=tk.X)

        # Ports
        ports_frame = ttk.Labelframe(top, text="Ports", padding=10)
        ports_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        sender_default = self.cfg["sender_port"] or read_upload_port(os.path.join(SENDER_DIR, "platformio.ini"))
        receiver_default = self.cfg["receiver_port"] or read_upload_port(os.path.join(RECEIVER_DIR, "platformio.ini"))
        pico_default = self.cfg["pico_port"]

        self.sender_port = tk.StringVar(value=sender_default)
        self.receiver_port = tk.StringVar(value=receiver_default)
        self.pico_port = tk.StringVar(value=pico_default)
        self.pico_mount = tk.StringVar(value=self.cfg["pico_mount"])

        ttk.Label(ports_frame, text="Sender (ESP)").grid(row=0, column=0, sticky="w")
        ttk.Entry(ports_frame, textvariable=self.sender_port, width=12).grid(row=0, column=1, padx=5)
        ttk.Label(ports_frame, text="Receiver (ESP)").grid(row=0, column=2, sticky="w")
        ttk.Entry(ports_frame, textvariable=self.receiver_port, width=12).grid(row=0, column=3, padx=5)
        ttk.Label(ports_frame, text="Pico (USB)").grid(row=0, column=4, sticky="w")
        ttk.Entry(ports_frame, textvariable=self.pico_port, width=12).grid(row=0, column=5, padx=5)
        ttk.Button(ports_frame, text="Refresh COMs", command=self._refresh_ports).grid(row=0, column=6, padx=5)

        ttk.Label(ports_frame, text="Pico Mount").grid(row=1, column=0, sticky="w", pady=(5, 0))
        ttk.Entry(ports_frame, textvariable=self.pico_mount, width=12).grid(row=1, column=1, padx=5, pady=(5, 0))
        ttk.Button(ports_frame, text="Detect", command=self._detect_mount).grid(row=1, column=2, padx=5, pady=(5, 0))
        ttk.Button(ports_frame, text="Save", command=self._save_config).grid(row=1, column=3, padx=5, pady=(5, 0))

        # Actions
        actions = ttk.Labelframe(top, text="Build / Flash", padding=10)
        actions.pack(side=tk.RIGHT, fill=tk.X)

        ttk.Button(actions, text="Flash Sender", command=self.flash_sender).grid(row=0, column=0, padx=5, pady=2)
        ttk.Button(actions, text="Flash Receiver", command=self.flash_receiver).grid(row=0, column=1, padx=5, pady=2)
        ttk.Button(actions, text="Build Pico", command=self.build_pico).grid(row=0, column=2, padx=5, pady=2)
        ttk.Button(actions, text="Build+Copy UF2", command=self.build_pico_copy).grid(row=0, column=3, padx=5, pady=2)

        # Serial control
        serial_frame = ttk.Labelframe(main, text="Pico Serial Control", padding=10)
        serial_frame.pack(fill=tk.X, pady=10)

        self.serial_status = tk.StringVar(value="Disconnected")
        ttk.Label(serial_frame, text="Status:").grid(row=0, column=0, sticky="w")
        ttk.Label(serial_frame, textvariable=self.serial_status).grid(row=0, column=1, sticky="w")

        ttk.Button(serial_frame, text="Connect", command=self.serial_connect).grid(row=0, column=2, padx=5)
        ttk.Button(serial_frame, text="Disconnect", command=self.serial_disconnect).grid(row=0, column=3, padx=5)

        self.cmd_entry = tk.StringVar(value="LED:1")
        ttk.Entry(serial_frame, textvariable=self.cmd_entry, width=20).grid(row=0, column=4, padx=5)
        ttk.Button(serial_frame, text="Send", command=self.send_command).grid(row=0, column=5, padx=5)
        ttk.Button(serial_frame, text="LED ON", command=lambda: self._send_quick("LED:1")).grid(row=0, column=6, padx=5)
        ttk.Button(serial_frame, text="LED OFF", command=lambda: self._send_quick("LED:0")).grid(row=0, column=7, padx=5)

        # Log output
        log_frame = ttk.Labelframe(main, text="Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(log_frame, height=20)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _append_log(self, text):
        self.log_text.insert(tk.END, text + "\n")
        self.log_text.see(tk.END)

    def _drain_log(self):
        while True:
            try:
                line = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)
        self.after(50, self._drain_log)

    def _drain_serial(self):
        while True:
            try:
                line = self.serial_queue.get_nowait()
            except queue.Empty:
                break
            self._append_log("[SER] " + line)
        self.after(50, self._drain_serial)

    def _run_cmd(self, cmd, cwd):
        self.log_queue.put(f"$ {cmd}")
        try:
            proc = subprocess.Popen(
                cmd,
                cwd=ensure_subst(cwd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                shell=True,
            )
            for line in proc.stdout:
                self.log_queue.put(line.rstrip())
            proc.wait()
            self.log_queue.put(f"[exit {proc.returncode}]")
        except Exception as exc:
            self.log_queue.put(f"[error] {exc}")

    def _refresh_ports(self):
        if not serial:
            self.log_queue.put("pyserial not available")
            return
        ports = serial.tools.list_ports.comports()
        lines = [f"{p.device} ({p.description})" for p in ports]
        self.log_queue.put("COM ports: " + ", ".join(lines))

    def _detect_mount(self):
        mount = find_pico_mount()
        if mount:
            self.pico_mount.set(mount)
            self.log_queue.put(f"Pico mount: {mount}")
        else:
            self.log_queue.put("Pico mount not found (BOOTSEL mode?)")

    def _save_config(self):
        self.cfg["sender_port"] = self.sender_port.get().strip()
        self.cfg["receiver_port"] = self.receiver_port.get().strip()
        self.cfg["pico_port"] = self.pico_port.get().strip()
        self.cfg["pico_mount"] = self.pico_mount.get().strip()
        save_config(self.cfg)
        self.log_queue.put("Config saved")

    def flash_sender(self):
        port = self.sender_port.get().strip()
        cmd = f"{pio_command()} run -t upload"
        if port:
            cmd += f" --upload-port {port}"
        threading.Thread(target=self._run_cmd, args=(cmd, SENDER_DIR), daemon=True).start()

    def flash_receiver(self):
        port = self.receiver_port.get().strip()
        cmd = f"{pio_command()} run -t upload"
        if port:
            cmd += f" --upload-port {port}"
        threading.Thread(target=self._run_cmd, args=(cmd, RECEIVER_DIR), daemon=True).start()

    def build_pico(self):
        threading.Thread(target=self._run_cmd, args=(f"{pio_command()} run", PICO_DIR), daemon=True).start()
        uf2_path = os.path.join(PICO_DIR, ".pio", "build", "pico", "firmware.uf2")
        self.log_queue.put(f"UF2 output: {uf2_path}")

    def build_pico_copy(self):
        def worker():
            self._run_cmd(f"{pio_command()} run", PICO_DIR)
            mount = self.pico_mount.get().strip() or find_pico_mount()
            msg = copy_uf2_to_mount(mount)
            self.log_queue.put(msg)
        threading.Thread(target=worker, daemon=True).start()

    def serial_connect(self):
        if not serial:
            self.log_queue.put("pyserial not available")
            return
        if self.ser:
            return
        port = self.pico_port.get().strip()
        try:
            self.ser = serial.Serial(port, 115200, timeout=0.1)
            self.serial_status.set(f"Connected: {port}")
            self.serial_stop.clear()
            self.serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
            self.serial_thread.start()
        except Exception as exc:
            self.log_queue.put(f"Serial connect failed: {exc}")

    def serial_disconnect(self):
        self.serial_stop.set()
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.serial_status.set("Disconnected")

    def _serial_loop(self):
        while not self.serial_stop.is_set() and self.ser:
            try:
                data = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if data:
                    self.serial_queue.put(data)
            except Exception:
                break

    def _send_quick(self, text):
        self.cmd_entry.set(text)
        self.send_command()

    def send_command(self):
        if not self.ser:
            self.log_queue.put("Serial not connected")
            return
        line = self.cmd_entry.get().strip()
        if not line:
            return
        try:
            self.ser.write((line + "\n").encode("utf-8"))
        except Exception as exc:
            self.log_queue.put(f"Send failed: {exc}")


if __name__ == "__main__":
    app = DevGUI()
    app.mainloop()
