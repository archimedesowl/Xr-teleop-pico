#!/bin/bash
# generate_certs.sh — Generate SSL certificates for XR device connections.
#
# Usage:
#   ./scripts/generate_certs.sh [pico|quest|avp]
#
# This script generates a self-signed CA and server certificate for the
# televuer WebXR server. The XR device must trust the CA certificate to
# establish a secure WebSocket connection.
#
# Output directory: ./certs/
#   - ca.key, ca.crt       — Certificate Authority (install ca.crt on XR device)
#   - server.key, server.crt — Server certificate (used by televuer)

set -euo pipefail

DEVICE="${1:-pico}"
CERT_DIR="./certs"
DAYS_VALID=3650
KEY_SIZE=2048

# Validate device type
case "$DEVICE" in
    pico|quest|avp)
        echo "[generate_certs] Generating certificates for device: $DEVICE"
        ;;
    *)
        echo "Error: Unknown device type '$DEVICE'. Supported: pico, quest, avp"
        echo "Usage: $0 [pico|quest|avp]"
        exit 1
        ;;
esac

# Check for openssl
if ! command -v openssl &>/dev/null; then
    echo "Error: openssl is not installed. Please install it first:"
    echo "  sudo apt-get install openssl"
    exit 1
fi

# Create output directory
mkdir -p "$CERT_DIR"

# Detect host IP on the robot subnet (192.168.123.x)
HOST_IP=""
for iface in $(ls /sys/class/net/ 2>/dev/null); do
    ip_addr=$(ip -4 addr show "$iface" 2>/dev/null | grep -oP '(?<=inet\s)192\.168\.123\.\d+' | head -1)
    if [ -n "$ip_addr" ]; then
        HOST_IP="$ip_addr"
        echo "[generate_certs] Detected host IP on robot subnet: $HOST_IP (interface: $iface)"
        break
    fi
done

if [ -z "$HOST_IP" ]; then
    # Fall back to primary IP
    HOST_IP=$(hostname -I | awk '{print $1}')
    echo "[generate_certs] No interface on 192.168.123.x subnet found."
    echo "[generate_certs] Using primary IP: $HOST_IP"
fi

echo "[generate_certs] Certificates will include SAN for: $HOST_IP, localhost, 127.0.0.1"

# --- Step 1: Generate CA key and certificate ---
echo "[generate_certs] Step 1/3: Generating CA certificate..."
openssl genrsa -out "$CERT_DIR/ca.key" "$KEY_SIZE" 2>/dev/null

openssl req -x509 -new -nodes \
    -key "$CERT_DIR/ca.key" \
    -sha256 \
    -days "$DAYS_VALID" \
    -out "$CERT_DIR/ca.crt" \
    -subj "/C=US/ST=CA/O=XR-Teleop/CN=XR-Teleop-CA"

# --- Step 2: Generate server key and CSR ---
echo "[generate_certs] Step 2/3: Generating server certificate..."
openssl genrsa -out "$CERT_DIR/server.key" "$KEY_SIZE" 2>/dev/null

# Create SAN config
cat > "$CERT_DIR/san.cnf" <<EOF
[req]
default_bits = $KEY_SIZE
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
IP.2 = $HOST_IP
EOF

openssl req -new -nodes \
    -key "$CERT_DIR/server.key" \
    -out "$CERT_DIR/server.csr" \
    -config "$CERT_DIR/san.cnf"

# --- Step 3: Sign server certificate with CA ---
echo "[generate_certs] Step 3/3: Signing server certificate with CA..."
openssl x509 -req \
    -in "$CERT_DIR/server.csr" \
    -CA "$CERT_DIR/ca.crt" \
    -CAkey "$CERT_DIR/ca.key" \
    -CAcreateserial \
    -out "$CERT_DIR/server.crt" \
    -days "$DAYS_VALID" \
    -sha256 \
    -extensions v3_req \
    -extfile "$CERT_DIR/san.cnf" 2>/dev/null

# Clean up intermediate files
rm -f "$CERT_DIR/server.csr" "$CERT_DIR/san.cnf" "$CERT_DIR/ca.srl"

# --- Done ---
echo ""
echo "============================================"
echo "  SSL certificates generated successfully!"
echo "============================================"
echo ""
echo "Files created in $CERT_DIR/:"
echo "  ca.key      — CA private key (keep secret)"
echo "  ca.crt      — CA certificate (install on XR device)"
echo "  server.key  — Server private key"
echo "  server.crt  — Server certificate"
echo ""

# Device-specific instructions
case "$DEVICE" in
    pico)
        echo "=== PICO Setup Instructions ==="
        echo "1. Copy ca.crt to PICO via USB or adb:"
        echo "   adb push $CERT_DIR/ca.crt /sdcard/Download/"
        echo "2. On PICO: Settings → Security → Install from storage"
        echo "3. Select ca.crt and install as 'CA certificate'"
        echo "4. Start televuer with the generated certs:"
        echo "   export SSL_CERT=$CERT_DIR/server.crt"
        echo "   export SSL_KEY=$CERT_DIR/server.key"
        ;;
    quest)
        echo "=== Quest Setup Instructions ==="
        echo "1. Copy ca.crt to Quest via USB or adb:"
        echo "   adb push $CERT_DIR/ca.crt /sdcard/Download/"
        echo "2. On Quest: Settings → Security → Install from storage"
        echo "3. Select ca.crt and install as 'CA certificate'"
        echo "4. Start televuer with the generated certs"
        ;;
    avp)
        echo "=== Apple Vision Pro Setup Instructions ==="
        echo "1. Email ca.crt to yourself or host it on a local server"
        echo "2. Open the certificate on Vision Pro and install the profile"
        echo "3. Settings → General → About → Certificate Trust Settings"
        echo "4. Enable full trust for the XR-Teleop-CA certificate"
        ;;
esac

echo ""
echo "Server IP for XR device browser: https://$HOST_IP:8012"
