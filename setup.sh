#!/bin/bash
set -euo pipefail

# Xr-teleop-pico: One-command bootstrap
# Usage: ./setup.sh [--device pico|quest|avp] [--sim-only] [--skip-certs]

# --- Colors ---
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

info()  { echo -e "${BLUE}[INFO]${NC} $*"; }
ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# --- Parse Arguments ---
DEVICE="pico"
SIM_ONLY=false
SKIP_CERTS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --device)
            DEVICE="$2"
            shift 2
            ;;
        --device=*)
            DEVICE="${1#*=}"
            shift
            ;;
        --sim-only)
            SIM_ONLY=true
            shift
            ;;
        --skip-certs)
            SKIP_CERTS=true
            shift
            ;;
        -h|--help)
            echo "Usage: ./setup.sh [--device pico|quest|avp] [--sim-only] [--skip-certs]"
            echo ""
            echo "Options:"
            echo "  --device DEVICE   XR device type: pico, quest, or avp (default: pico)"
            echo "  --sim-only        Skip network and cert setup (simulation only)"
            echo "  --skip-certs      Skip SSL certificate generation"
            echo "  -h, --help        Show this help"
            exit 0
            ;;
        *)
            err "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Validate device
case "$DEVICE" in
    pico|quest|avp) ;;
    *)
        err "Invalid device: $DEVICE. Must be pico, quest, or avp."
        exit 1
        ;;
esac

REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
info "Repository: $REPO_DIR"
info "Device: $DEVICE"
info "Sim-only: $SIM_ONLY"

# --- Check OS ---
info "Checking OS..."
if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    if [[ "$ID" == "ubuntu" ]]; then
        case "$VERSION_ID" in
            20.04|22.04)
                ok "Ubuntu $VERSION_ID detected"
                ;;
            *)
                warn "Ubuntu $VERSION_ID detected. Tested on 20.04 and 22.04 only — proceed with caution."
                ;;
        esac
    else
        warn "$PRETTY_NAME detected. Tested on Ubuntu 20.04/22.04 only — proceed with caution."
    fi
else
    warn "Cannot detect OS. Tested on Ubuntu 20.04/22.04 only."
fi

# --- Check/Install Conda ---
info "Checking conda..."
if command -v conda &>/dev/null; then
    ok "conda found: $(conda --version)"
else
    warn "conda not found."
    if command -v wget &>/dev/null || command -v curl &>/dev/null; then
        info "Installing Miniforge..."
        INSTALLER="/tmp/miniforge_installer.sh"
        if command -v wget &>/dev/null; then
            wget -q "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh" -O "$INSTALLER"
        else
            curl -fsSL "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh" -o "$INSTALLER"
        fi
        bash "$INSTALLER" -b -p "$HOME/miniforge3"
        rm -f "$INSTALLER"
        eval "$("$HOME/miniforge3/bin/conda" shell.bash hook)"
        ok "Miniforge installed. You may need to restart your shell or run: eval \"\$($HOME/miniforge3/bin/conda shell.bash hook)\""
    else
        err "Neither wget nor curl found. Install conda manually: https://docs.conda.io/en/latest/miniconda.html"
        exit 1
    fi
fi

# Initialize conda for this script
eval "$(conda shell.bash hook 2>/dev/null)" || true

# --- Create Conda Environment ---
ENV_NAME="tv"
info "Setting up conda environment '$ENV_NAME'..."
if conda env list | grep -qw "^$ENV_NAME "; then
    warn "Conda environment '$ENV_NAME' already exists. Skipping creation."
    info "To recreate: conda env remove -n $ENV_NAME && re-run this script"
else
    info "Creating conda environment '$ENV_NAME' with python=3.10, pinocchio=3.1.0, numpy=1.26.4..."
    conda create -n "$ENV_NAME" python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge -y
    ok "Conda environment '$ENV_NAME' created"
fi

conda activate "$ENV_NAME"
ok "Activated conda environment '$ENV_NAME'"

# --- Install unitree_sdk2_python ---
info "Installing unitree_sdk2_python..."
SDK_DIR="/tmp/unitree_sdk2_python"
if python -c "import unitree_sdk2py" 2>/dev/null; then
    ok "unitree_sdk2_python already installed"
else
    if [[ -d "$SDK_DIR" ]]; then
        info "Updating existing clone..."
        git -C "$SDK_DIR" pull --ff-only 2>/dev/null || true
    else
        git clone https://github.com/unitreerobotics/unitree_sdk2_python.git "$SDK_DIR"
    fi
    pip install -e "$SDK_DIR"
    ok "unitree_sdk2_python installed"
fi

# --- Install Submodules ---
info "Initializing git submodules..."
cd "$REPO_DIR"
git submodule update --init --depth 1
ok "Submodules initialized"

# --- Install teleimager ---
info "Installing teleimager submodule..."
pip install -e "$REPO_DIR/teleop/teleimager" --no-deps
ok "teleimager installed"

# --- Install televuer ---
info "Installing televuer submodule..."
pip install -e "$REPO_DIR/teleop/televuer"
ok "televuer installed"

# --- Install requirements.txt ---
info "Installing Python dependencies from requirements.txt..."
pip install -r "$REPO_DIR/requirements.txt"
ok "Python dependencies installed"

# --- Generate SSL Certs ---
if [[ "$SKIP_CERTS" == true ]] || [[ "$SIM_ONLY" == true ]]; then
    info "Skipping SSL certificate generation"
else
    CERT_SCRIPT="$REPO_DIR/scripts/generate_certs.sh"
    if [[ -x "$CERT_SCRIPT" ]]; then
        info "Generating SSL certificates for $DEVICE..."
        bash "$CERT_SCRIPT" "$DEVICE"
        ok "SSL certificates generated"
    else
        warn "Certificate generation script not found at $CERT_SCRIPT"
        info "Generate certificates manually. For PICO/Quest:"
        info "  cd teleop/televuer && openssl req -x509 -nodes -days 365 -newkey rsa:2048 -keyout key.pem -out cert.pem"
        info "See README.md for Apple Vision Pro instructions."
    fi
fi

# --- Auto-detect Network Interface ---
if [[ "$SIM_ONLY" == true ]]; then
    info "Skipping network interface detection (sim-only mode)"
else
    info "Detecting network interface on 192.168.123.x subnet..."
    DETECTED_NIC=""
    for iface in $(ls /sys/class/net/ 2>/dev/null); do
        if ip addr show "$iface" 2>/dev/null | grep -q "192\.168\.123\."; then
            DETECTED_NIC="$iface"
            break
        fi
    done

    if [[ -n "$DETECTED_NIC" ]]; then
        ok "Detected network interface: $DETECTED_NIC (192.168.123.x subnet)"
    else
        warn "No network interface found on 192.168.123.x subnet."
        info "For physical robot, connect an Ethernet cable to the robot's network switch."
        info "For simulation, this is expected — use --sim flag when running."
    fi
fi

# --- Summary ---
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Setup Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "  Device:      $DEVICE"
echo "  Conda env:   $ENV_NAME"
echo "  Python:      $(python --version 2>&1)"
echo ""
echo "Next steps:"
echo ""
echo "  1. Activate the environment:"
echo "     conda activate $ENV_NAME"
echo ""
if [[ "$SIM_ONLY" == true ]]; then
    echo "  2. Start Isaac Sim (in a separate terminal):"
    echo "     See https://github.com/unitreerobotics/unitree_sim_isaaclab"
    echo ""
    echo "  3. Run teleoperation in sim mode:"
    echo "     cd $REPO_DIR/teleop"
    echo "     python teleop_hand_and_arm.py --arm=G1_29 --sim"
else
    echo "  2. Run teleoperation:"
    echo "     cd $REPO_DIR/teleop"
    if [[ -n "${DETECTED_NIC:-}" ]]; then
        echo "     python teleop_hand_and_arm.py --arm=G1_29"
    else
        echo "     python teleop_hand_and_arm.py --arm=G1_29 --sim  # simulation"
        echo "     python teleop_hand_and_arm.py --arm=G1_29        # physical robot (connect to robot network first)"
    fi
fi
echo ""
echo "  For more info: see README.md, AGENTS.md, CODEBASE_INDEX.md"
echo ""
