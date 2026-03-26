#!/usr/bin/env python3
"""Generate SSL certificates for XR device connections.

Usage:
    python scripts/generate_certs.py [--device pico|quest|avp] [--ip IP_ADDRESS]

Generates a self-signed CA and server certificate for the televuer WebXR server.
The XR device must trust the CA certificate to establish a secure WSS connection.

Output directory: ./certs/
    ca.key, ca.crt       - Certificate Authority (install ca.crt on XR device)
    server.key, server.crt - Server certificate (used by televuer)
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def detect_host_ip() -> str:
    """Detect the host IP address on the robot subnet (192.168.123.x).

    Falls back to the primary IP address if no robot subnet interface is found.

    Returns:
        IP address string.
    """
    try:
        # Try to find an interface on the robot subnet
        result = subprocess.run(
            ["ip", "-4", "addr", "show"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            line = line.strip()
            if "inet 192.168.123." in line:
                # Extract IP: "inet 192.168.123.X/24 ..."
                ip = line.split()[1].split("/")[0]
                return ip
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass

    # Fallback: get primary IP
    try:
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def generate_certs(device: str, host_ip: str, cert_dir: str = "./certs") -> None:
    """Generate CA and server SSL certificates.

    Args:
        device: XR device type ('pico', 'quest', or 'avp').
        host_ip: IP address to include in the certificate SAN.
        cert_dir: Output directory for certificate files.
    """
    cert_path = Path(cert_dir)
    cert_path.mkdir(parents=True, exist_ok=True)

    days_valid = "3650"
    key_size = "2048"

    print(f"[generate_certs] Generating certificates for device: {device}")
    print(f"[generate_certs] Host IP: {host_ip}")
    print(f"[generate_certs] Output directory: {cert_dir}")

    # Step 1: Generate CA
    print("[generate_certs] Step 1/3: Generating CA certificate...")
    subprocess.run([
        "openssl", "genrsa", "-out", str(cert_path / "ca.key"), key_size
    ], check=True, capture_output=True)

    subprocess.run([
        "openssl", "req", "-x509", "-new", "-nodes",
        "-key", str(cert_path / "ca.key"),
        "-sha256", "-days", days_valid,
        "-out", str(cert_path / "ca.crt"),
        "-subj", "/C=US/ST=CA/O=XR-Teleop/CN=XR-Teleop-CA"
    ], check=True, capture_output=True)

    # Step 2: Generate server key and CSR
    print("[generate_certs] Step 2/3: Generating server certificate...")
    subprocess.run([
        "openssl", "genrsa", "-out", str(cert_path / "server.key"), key_size
    ], check=True, capture_output=True)

    # Write SAN config
    san_cnf = cert_path / "san.cnf"
    san_cnf.write_text(f"""[req]
default_bits = {key_size}
prompt = no
default_md = sha256
distinguished_name = dn
req_extensions = v3_req

[dn]
C = US
ST = CA
O = XR-Teleop
CN = xr-teleop-server

[v3_req]
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
IP.2 = {host_ip}
""")

    subprocess.run([
        "openssl", "req", "-new", "-nodes",
        "-key", str(cert_path / "server.key"),
        "-out", str(cert_path / "server.csr"),
        "-config", str(san_cnf)
    ], check=True, capture_output=True)

    # Step 3: Sign server certificate with CA
    print("[generate_certs] Step 3/3: Signing server certificate...")
    subprocess.run([
        "openssl", "x509", "-req",
        "-in", str(cert_path / "server.csr"),
        "-CA", str(cert_path / "ca.crt"),
        "-CAkey", str(cert_path / "ca.key"),
        "-CAcreateserial",
        "-out", str(cert_path / "server.crt"),
        "-days", days_valid,
        "-sha256",
        "-extensions", "v3_req",
        "-extfile", str(san_cnf)
    ], check=True, capture_output=True)

    # Clean up intermediate files
    for f in ["server.csr", "san.cnf", "ca.srl"]:
        (cert_path / f).unlink(missing_ok=True)

    print()
    print("=" * 44)
    print("  SSL certificates generated successfully!")
    print("=" * 44)
    print()
    print(f"Files created in {cert_dir}/:")
    print("  ca.key      - CA private key (keep secret)")
    print("  ca.crt      - CA certificate (install on XR device)")
    print("  server.key  - Server private key")
    print("  server.crt  - Server certificate")
    print()

    # Device-specific instructions
    instructions = {
        "pico": (
            "=== PICO Setup Instructions ===\n"
            f"1. Copy ca.crt to PICO via USB or adb:\n"
            f"   adb push {cert_dir}/ca.crt /sdcard/Download/\n"
            "2. On PICO: Settings -> Security -> Install from storage\n"
            "3. Select ca.crt and install as 'CA certificate'\n"
            f"4. Start televuer with the generated certs:\n"
            f"   export SSL_CERT={cert_dir}/server.crt\n"
            f"   export SSL_KEY={cert_dir}/server.key"
        ),
        "quest": (
            "=== Quest Setup Instructions ===\n"
            f"1. Copy ca.crt to Quest via USB or adb:\n"
            f"   adb push {cert_dir}/ca.crt /sdcard/Download/\n"
            "2. On Quest: Settings -> Security -> Install from storage\n"
            "3. Select ca.crt and install as 'CA certificate'\n"
            "4. Start televuer with the generated certs"
        ),
        "avp": (
            "=== Apple Vision Pro Setup Instructions ===\n"
            "1. Email ca.crt to yourself or host it on a local server\n"
            "2. Open the certificate on Vision Pro and install the profile\n"
            "3. Settings -> General -> About -> Certificate Trust Settings\n"
            "4. Enable full trust for the XR-Teleop-CA certificate"
        ),
    }
    print(instructions.get(device, ""))
    print()
    print(f"Server IP for XR device browser: https://{host_ip}:8012")


def main() -> None:
    """Parse arguments and generate certificates."""
    parser = argparse.ArgumentParser(
        description="Generate SSL certificates for XR device connections."
    )
    parser.add_argument(
        "--device", type=str, choices=["pico", "quest", "avp"],
        default="pico", help="XR device type (default: pico)"
    )
    parser.add_argument(
        "--ip", type=str, default=None,
        help="Host IP address (auto-detected if not specified)"
    )
    parser.add_argument(
        "--output", type=str, default="./certs",
        help="Output directory for certificates (default: ./certs)"
    )
    args = parser.parse_args()

    # Check for openssl
    try:
        subprocess.run(["openssl", "version"], capture_output=True, check=True)
    except (FileNotFoundError, subprocess.CalledProcessError):
        print("Error: openssl is not installed. Please install it first:")
        print("  sudo apt-get install openssl")
        sys.exit(1)

    host_ip = args.ip if args.ip else detect_host_ip()
    generate_certs(args.device, host_ip, args.output)


if __name__ == "__main__":
    main()
