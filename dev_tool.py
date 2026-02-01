import argparse
import json
import os
import sys
import subprocess
import time
import shutil

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


def run_cmd(cmd, cwd):
    cwd = ensure_subst(cwd)
    print(f"$ {cmd}")
    proc = subprocess.Popen(
        cmd,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        shell=True,
    )
    for line in proc.stdout:
        print(line.rstrip())
    proc.wait()
    return proc.returncode


def list_ports():
    if not serial:
        print("pyserial not available")
        return 1
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = f" ({p.description})" if p.description else ""
        print(f"{p.device}{desc}")
    return 0


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
    # Look for RPI-RP2 UF2 drive by presence of INFO_UF2.TXT
    for letter in "ABCDEFGHIJKLMNOPQRSTUVWXYZ":
        root = f"{letter}:\\"
        info = os.path.join(root, "INFO_UF2.TXT")
        if os.path.exists(info):
            return root
    return ""


def copy_uf2_to_mount(mount):
    uf2_path = os.path.join(PICO_DIR, ".pio", "build", "pico", "firmware.uf2")
    if not os.path.exists(uf2_path):
        print(f"UF2 not found: {uf2_path}")
        return 1
    if not mount:
        print("Pico mount not set")
        return 1
    dest = os.path.join(mount, "firmware.uf2")
    with open(uf2_path, "rb") as src, open(dest, "wb") as dst:
        dst.write(src.read())
    print(f"Copied UF2 to {dest}")
    return 0


def flash_sender(port=None):
    cmd = f"{pio_command()} run -t upload"
    if port:
        cmd += f" --upload-port {port}"
    return run_cmd(cmd, SENDER_DIR)


def flash_receiver(port=None):
    cmd = f"{pio_command()} run -t upload"
    if port:
        cmd += f" --upload-port {port}"
    return run_cmd(cmd, RECEIVER_DIR)


def build_pico(copy=False, mount=""):
    rc = run_cmd(f"{pio_command()} run", PICO_DIR)
    uf2_path = os.path.join(PICO_DIR, ".pio", "build", "pico", "firmware.uf2")
    print(f"UF2 output: {uf2_path}")
    if copy:
        if not mount:
            mount = find_pico_mount()
        if not mount:
            print("Pico mount not found. Put Pico in BOOTSEL mode.")
            return 1
        return copy_uf2_to_mount(mount)
    return rc


def serial_send(port, baud, text):
    if not serial:
        print("pyserial not available")
        return 1
    with serial.Serial(port, baud, timeout=1) as ser:
        ser.write((text + "\n").encode("utf-8"))
    return 0


def serial_monitor(port, baud):
    if not serial:
        print("pyserial not available")
        return 1
    with serial.Serial(port, baud, timeout=0.1) as ser:
        print(f"Connected to {port} @ {baud}. Ctrl+C to exit.")
        try:
            while True:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line:
                    print(line)
        except KeyboardInterrupt:
            return 0


def main():
    cfg = load_config()
    parser = argparse.ArgumentParser(description="DIY PC Hardware Dev Tool")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ports", help="List serial ports")

    p_flash_s = sub.add_parser("flash-sender", help="Flash ESP sender")
    p_flash_s.add_argument("--port", default=cfg["sender_port"] or read_upload_port(os.path.join(SENDER_DIR, "platformio.ini")))

    p_flash_r = sub.add_parser("flash-receiver", help="Flash ESP receiver")
    p_flash_r.add_argument("--port", default=cfg["receiver_port"] or read_upload_port(os.path.join(RECEIVER_DIR, "platformio.ini")))

    p_build = sub.add_parser("build-pico", help="Build Pico firmware")
    p_build.add_argument("--copy", action="store_true", help="Copy UF2 to Pico drive")
    p_build.add_argument("--mount", default=cfg.get("pico_mount", ""))

    p_send = sub.add_parser("send", help="Send a command over serial")
    p_send.add_argument("--port", default=cfg.get("pico_port", ""), required=False)
    p_send.add_argument("--baud", type=int, default=115200)
    p_send.add_argument("text")

    p_mon = sub.add_parser("monitor", help="Serial monitor")
    p_mon.add_argument("--port", default=cfg.get("pico_port", ""), required=False)
    p_mon.add_argument("--baud", type=int, default=115200)

    p_cfg = sub.add_parser("config", help="Show or set config")
    p_cfg.add_argument("--set", nargs=2, metavar=("KEY", "VALUE"))

    args = parser.parse_args()

    if args.cmd == "ports":
        return list_ports()
    if args.cmd == "flash-sender":
        return flash_sender(args.port)
    if args.cmd == "flash-receiver":
        return flash_receiver(args.port)
    if args.cmd == "build-pico":
        return build_pico(copy=args.copy, mount=args.mount)
    if args.cmd == "send":
        if not args.port:
            print("--port required (or set pico_port in config)")
            return 1
        return serial_send(args.port, args.baud, args.text)
    if args.cmd == "monitor":
        if not args.port:
            print("--port required (or set pico_port in config)")
            return 1
        return serial_monitor(args.port, args.baud)
    if args.cmd == "config":
        if args.set:
            key, value = args.set
            if key not in DEFAULT_CONFIG:
                print(f"Unknown key: {key}")
                return 1
            cfg[key] = value
            save_config(cfg)
        print(json.dumps(cfg, indent=2))
        return 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
