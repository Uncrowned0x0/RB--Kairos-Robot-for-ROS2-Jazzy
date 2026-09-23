# RB-KAIROS: Remote Access, Hardware Startup & Power Guide

**Author:** Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)  
**Target Platform:** Robotnik RB-KAIROS Omnidirectional Mobile Base  
**Operating System:** Ubuntu 24.04 LTS (Kernel 6.8)  
**Middleware:** ROS 2 Jazzy Jalisco  
**Document Status:** Production Validated (September 2026)  

---

## 1. Quick Startup Checklist (For Beginners)

Before touching any terminal or computer, perform this quick physical checklist:

```
[ ] 1. Disconnect the 48V battery charger cable from the robot charging socket.
[ ] 2. Ensure robot wheels are unobstructed on flat ground or elevated on safety blocks.
[ ] 3. Twist clockwise to release the red Emergency Stop (E-Stop) mushroom button.
[ ] 4. Turn on the main electrical breaker switch on the rear panel.
[ ] 5. Press the BLUE flashing contactor button on the front panel.
       --> Hear loud physical "CLACK" of the 48V safety power contactors closing.
[ ] 6. Wait ~45 seconds for Ubuntu 24.04 and GNOME Remote Desktop to boot.
[ ] 7. Ensure HDMI Dummy Plug is plugged into the robot HDMI port (for headless mode).
[ ] 8. Connect to Wi-Fi network "Robotica" from your laptop.
[ ] 9. Open Remmina (Linux) or Remote Desktop (Windows/macOS) to 192.168.0.183.
```

---

## 2. Hardware Power-Up & Safety Systems

### 2.1. Main Breaker & Emergency Stop (E-Stop)
The RB-KAIROS mobile base contains high-power 48V DC battery circuitry capable of delivering tens of amperes to the four brushless hub motors. Safety interlocks protect both users and equipment:
- **Red E-Stop Mushroom Button:** Located prominently on the robot top/rear chassis.
  - **Depressed:** Cuts 48V drive power immediately.
  - **Disarm:** Twist clockwise until it pops up.
- **Main Power Switch:** Turn the main rotary breaker on the robot rear panel to `ON`.

### 2.2. Safety Contactor & Blue Reset Pushbutton (CRITICAL)
Even when the main breaker is on and the E-Stop is disarmed, the motor drives and 48V power rails remain mechanically disconnected until the safety relay is armed:
- Look at the **blue pushbutton** located on the front panel.
- If the blue button is flashing or dark, the 48V contactor is open.
- **Firmly press the blue pushbutton once.**
- You must hear a distinct metallic **"CLACK"** sound inside the chassis. This is the main 48V DC solenoid contactor closing.
- If you do not hear this sound or if the battery monitor shows `< 20.0 V` with a `"CONTACTOR OPEN"` warning, the motor drives cannot deliver torque to the wheels.

### 2.3. Charger Disconnection
- **NEVER power up motor drivers or attempt driving while the charging cable is plugged into the wall or robot.**
- Disconnect the charging connector and replace the protective rubber socket cap.
- Check the battery voltage:
  - **53.3 V:** 100% full charge.
  - **48.0 V:** Nominal voltage (~40-50% remaining).
  - **< 45.0 V:** Low battery warning (< 20%). Recharge soon.
  - **44.0 V:** Automatic cutoff threshold to protect LiFePO4 cells from deep discharge.

---

## 3. Network Configuration & Connectivity

The onboard computer communicates across several dedicated network interfaces:

| Network Interface | Type | Role | IP Configuration |
| :--- | :--- | :--- | :--- |
| **`wlp4s0`** | Wi-Fi (Intel Wireless) | Remote Access & Telemetry | **`192.168.0.183`** (SSID: `Robotica`, metric 50) |
| **`enp3s0`** | Gigabit Ethernet (RJ45) | Sensor Network (SICK LiDARs) | **`192.168.0.200`** (Direct switch to LiDARs) |
| **`can0`** | SocketCAN (USB/PCIe CAN) | 48V AMC Motor Drives | Bitrate: **`1,000,000 bps (1 Mbps)`** |

### 3.1. Testing Laptop Connectivity
From your personal laptop, join the **`Robotica`** Wi-Fi network and run:
```bash
ping -c 3 192.168.0.183
```
If packets reply with `< 5 ms` latency, your laptop has network connectivity to the robot.

---

## 4. Remote Desktop (RDP) Access

The robot runs a native GNOME RDP server with TLS encryption, allowing you to access the graphical desktop without attaching an external monitor or keyboard.

### 4.1. Credentials Summary
- **Protocol:** RDP (*Remote Desktop Protocol* - native GNOME)
- **Host / IP:** `192.168.0.183`
- **Port:** `3389` (default RDP port)
- **Username:** `kairos`
- **Password:** `kairos`
- **Security:** TLS self-signed certificate (Accept when prompted)

### 4.2. Connecting from Linux (Remmina - Recommended)
1. Open **Remmina** on your laptop.
2. Click the **"+" (New Connection Profile)** button in the top-left toolbar.
3. Configure the following fields:
   - **Profile Name:** `RB-KAIROS Desktop`
   - **Protocol:** Select `RDP - Remote Desktop Protocol` *(Do NOT select VNC)*
   - **Server:** `192.168.0.183`
   - **User name:** `kairos`
   - **Password:** `kairos`
   - **Color depth:** `True color (32 bpp)` or `High quality (24 bpp)`
   - **Quality:** `Best`
4. Click **"Save and Connect"**.
5. When the certificate popup appears, check **"Trust this certificate"** and click **OK**.
6. The robot Ubuntu desktop will appear in a responsive window with full mouse, keyboard, and shared clipboard support.

### 4.3. Connecting from Windows (Microsoft Remote Desktop / `mstsc`)
1. Press `Win + R`, type `mstsc` and hit Enter.
2. In the **Computer** field, enter: `192.168.0.183`
3. Click **Connect**.
4. Enter username `kairos` and password `kairos`.
5. Check **"Don't ask me again for connections to this computer"** on the certificate warning and click **Yes**.
6. The GNOME desktop will display in full screen.

### 4.4. Connecting from macOS (Microsoft Remote Desktop)
1. Install **Microsoft Remote Desktop** from the Mac App Store.
2. Click **"Add PC"**:
   - **PC name:** `192.168.0.183`
   - **User account:** Add account with user `kairos` and password `kairos`.
3. Double-click the saved connection card and accept the certificate.

### 4.5. Headless HDMI Dummy Plug (Required for Detached Operation)
When operating without a physical display cable attached to the robot:
- Plug the small **HDMI Dummy Plug** (*headless display emulator*) into any HDMI port on the robot graphics card.
- This forces Ubuntu to render a virtual 1920x1080 (or 2560x1600) display buffer with hardware acceleration.
- If the dummy plug is missing, RDP may connect to a black or frozen 0x0 resolution screen.

---

## 5. Terminal Access via SSH (Lightweight Fallback)

If you only need a shell or have poor Wi-Fi bandwidth, SSH into the robot directly:
```bash
ssh kairos@192.168.0.183
# Password: kairos
```

To enable X11 graphical forwarding for individual tools:
```bash
ssh -X kairos@192.168.0.183
```

---

## 6. Pre-Flight Hardware Diagnostics

Once logged into a terminal on the robot, run these checks to ensure all peripherals are operational:

### 6.1. Verify CAN Bus
```bash
ip link show can0
```
- If the output shows `state UP` at `bitrate 1000000`, CAN is ready.
- If it shows `state DOWN`, activate it:
  ```bash
  sudo ip link set can0 up type can bitrate 1000000
  ```

### 6.2. Verify SICK LiDAR Network Routes
The two SICK TiM5xx LiDAR scanners are mounted on the static IP addresses `192.168.0.10` (Front) and `192.168.0.11` (Rear). Ensure routing table entries exist:
```bash
ping -c 2 192.168.0.10
ping -c 2 192.168.0.11
```
If pings fail, re-add the host routes:
```bash
sudo ip route add 192.168.0.10 dev enp3s0
sudo ip route add 192.168.0.11 dev enp3s0
```

### 6.3. Verify Serial Devices (IMU & LEDs)
Check that udev rules created persistent symlinks:
```bash
ls -l /dev/ttyUSB_IMU /dev/ttyUSB_LEDS
```
- `/dev/ttyUSB_IMU` -> Points to FTDI chip of VectorNav VN-100.
- `/dev/ttyUSB_LEDS` -> Points to Teensyduino LED controller.

---

## 7. Troubleshooting Common Startup Issues

| Symptom | Probable Cause | Exact Solution |
| :--- | :--- | :--- |
| **Pings to `192.168.0.183` fail** | Laptop not on `Robotica` Wi-Fi | Reconnect laptop to `Robotica` network. Check robot Wi-Fi with `ip -4 addr show wlp4s0`. |
| **RDP connects to a black screen** | HDMI dummy plug missing or loose | Plug the HDMI dummy plug firmly into the robot HDMI port and reconnect. |
| **Motors do not move; no error output** | 48V contactor open | Disengage E-Stop, then firmly press the front **blue button**. Listen for the "CLACK". |
| **`can0: Network is down` error** | Interface uninitialized after reboot | Run `sudo ip link set can0 up type can bitrate 1000000`. |
| **GNOME RDP service unresponsive** | Stale user session lock | Run `systemctl --user restart gnome-remote-desktop`. |
| **Low battery buzzer or yellow LEDs** | Battery below 45V | Plug in the 48V charger immediately. Do not discharge below 44V. |

---

*Author: Kamil BENMADI (<kamil.benmadi@sigma-clermont.fr>) — [GitHub](https://github.com/Uncrowned0x0) | [LinkedIn](https://www.linkedin.com/in/kamilb-)*
