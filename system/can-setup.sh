#!/bin/bash
# CAN Interface Setup for Waveshare HAT on Ubuntu 22.04
# Configures both CAN0 and CAN1
# Motor requires 1Mbps bitrate

echo "=== Setting up CAN interfaces ==="

# ============================================================
# CAN0 Setup
# ============================================================
echo ""
echo "--- CAN0 ---"

# Bring down if already running
sudo ip link set can0 down 2>/dev/null

# Configure CAN0 at 1Mbps (required by AK60-6 V3.0)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

# Bring up the interface
sudo ip link set can0 up

# Verify
if ip link show can0 | grep -q "UP"; then
    echo "✓ CAN0 is UP and ready"
    ip -details link show can0 | grep -E "can|bitrate"
else
    echo "✗ Failed to bring up CAN0"
fi

# ============================================================
# CAN1 Setup
# ============================================================
echo ""
echo "--- CAN1 ---"

# Bring down if already running
sudo ip link set can1 down 2>/dev/null

# Configure CAN1 at 1Mbps (required by AK60-6 V3.0)
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 txqueuelen 1000

# Bring up the interface
sudo ip link set can1 up

# Verify
if ip link show can1 | grep -q "UP"; then
    echo "✓ CAN1 is UP and ready"
    ip -details link show can1 | grep -E "can|bitrate"
else
    echo "✗ Failed to bring up CAN1"
fi

# ============================================================
# Summary
# ============================================================
echo ""
echo "=== Summary ==="
can0_status=$(ip link show can0 2>/dev/null | grep -q "UP" && echo "✓ UP" || echo "✗ DOWN")
can1_status=$(ip link show can1 2>/dev/null | grep -q "UP" && echo "✓ UP" || echo "✗ DOWN")

echo "CAN0: $can0_status"
echo "CAN1: $can1_status"
echo ""
