# PICO VR Teleoperation Setup and Usage

[Chinese version](pico_setup_zh.md)

This guide covers the one-time hardware and software setup required for GRIT
whole-body teleoperation with PICO VR, including PICO hardware preparation,
XRoboToolkit installation, Motion Tracker calibration, and network setup.

After completing this guide, return to the main README to continue with:

- [PICO sim2sim](../README.md#pico-sim2sim)
- [G1 hardware deployment](../README.md#g1-hardware-deployment)

## Required hardware

- a PICO 4 or PICO 4 Pro headset
- two PICO controllers
- two PICO Motion Trackers, one attached to each ankle
- a fast, low-latency Wi-Fi network
- an x86_64 laptop or workstation running Ubuntu 22.04

> [!IMPORTANT]
> Teleoperation stability and latency depend heavily on wireless network
> quality. Connect the workstation to the router over Ethernet when possible,
> and connect the PICO to the same router over 5 GHz Wi-Fi. Avoid congested
> public networks and weak hotspots.

## Step 1: Install XRoboToolkit

XRoboToolkit consists of a PC Service and a PICO application. The PC Service
runs on the workstation, while the PICO application runs on the headset. They
work together to transmit body-tracking data.

### 1. Install the PC Service

The PC Service must be installed and running on the workstation before the PICO
can connect.

On an Ubuntu 22.04 x86_64 workstation, run:

```bash
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

For other platforms or newer versions, see the
[XRoboToolkit PC Service releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases).

After installation, the PC Service is located at
`/opt/apps/roboticsservice` by default. Start it from the desktop application
menu or run:

```bash
bash /opt/apps/roboticsservice/runService.sh
```

GRIT also requires the XRoboToolkit Python binding. From the repository root,
run:

```bash
cd sim2real
bash install_xrobottoolkit_sdk.sh
```

### 2. Install the PICO application

1. Put on the PICO headset, complete the initial PICO setup, and connect the
   headset to Wi-Fi.
2. Enable PICO developer options and select `File transfer` in the USB
   connection settings.
3. Download
   [XRoboToolkit-PICO-1.1.1.apk](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases/download/v1.1.1/XRoboToolkit-PICO-1.1.1.apk)
   on the workstation. Other versions are available from the
   [XRoboToolkit Unity Client releases](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases).
4. Copy the APK to removable storage connected to the PICO, or transfer it to
   the headset over USB.
5. Select `XRoboToolkit-PICO-1.1.1.apk` in the PICO file manager and install it.
6. Open XRoboToolkit from the `Unknown` sources section of the application
   library.

If ADB is installed on the workstation, you can install the application over
USB instead:

```bash
adb devices
adb install -g XRoboToolkit-PICO-1.1.1.apk
```

After running `adb devices`, accept the USB debugging prompt in the headset if
one appears. A successful connection is listed as `device`, not
`unauthorized`.

## Step 2: Configure the PICO teleoperation environment

### 1. Calibrate the Motion Trackers

1. Attach one PICO Motion Tracker to each ankle. Check their orientation and
   make sure the left and right trackers are not reversed.
2. Press and hold the button on each tracker for about six seconds, until its
   indicator lights up.
3. Open the Motion Tracker application on the PICO and verify that both
   trackers are paired correctly.
4. Follow the instructions in the headset to calibrate the Motion Trackers.
5. Check whether the virtual character's feet are resting on the floor. If they
   are floating or below the floor, select `Calibrate Floor` in the upper-left
   corner until both feet are correctly grounded.

During calibration, stand naturally with your feet shoulder-width apart. Make
sure the headset, controllers, and ankle trackers have sufficient battery. Run
calibration again after changing the user, tracker placement, or operating
area.

### 2. Connect the PICO to the workstation

1. Open the Wi-Fi settings on the workstation and PICO, and verify that both
   devices are connected to the same local network.
2. Find and record the workstation's Wi-Fi IPv4 address. On Ubuntu, run:

   ```bash
   hostname -I
   ```

   If multiple addresses are shown, use the local address on the same subnet as
   the PICO. Do not use `127.0.0.1`, a container address, or the dedicated robot
   network interface address.

3. Start the XRoboToolkit PC Service on the workstation.
4. Open XRoboToolkit on the PICO. Select `Enter` next to `PC Service:` and enter
   the workstation's IPv4 address.
5. When the connection succeeds, `WORKING` appears next to `Status:`. If an IP
   address is already saved, select `Reconnect` next to `Status:` in the
   `Network` section.

### 3. Check the data transmission options

Verify the following settings in the XRoboToolkit PICO application:

| Section | Setting |
| --- | --- |
| `Tracking` | enable `Head` and `Controller` |
| `Data/Control` | select `Send` |
| `Pico Motion Tracker` | select `Full body` |

After configuration, start the GRIT retargeting process on the workstation:

```bash
cd sim2real
taskset -c 1 uv run teleop/serve_xrobot_teleop.py --robot g1
```

The live pipeline uses TCP ports `28701`, `28702`, and `28703`. Confirm that the
terminal reports no connection errors before continuing with the MuJoCo or
hardware control workflow.

## Connection checklist

Before deployment, verify that:

- the headset, both controllers, and both ankle trackers are connected and have
  sufficient battery
- the virtual character's feet are correctly grounded in the PICO view
- the PICO and workstation are on the same local network
- XRoboToolkit displays `WORKING`
- `Head`, `Controller`, `Send`, and `Full body` are configured as described
- the PC Service and GRIT retargeting process are both running
- the firewall does not block TCP ports `28701`, `28702`, or `28703`

## Troubleshooting

### XRoboToolkit does not display `WORKING`

- Confirm that the PC Service is running on the workstation.
- Confirm that the configured IPv4 address belongs to the workstation on the
  same local network as the PICO.
- Confirm that the PICO and workstation can reach each other and that wireless
  client isolation is disabled.
- If the workstation address changes, update it in the PICO application and
  select `Reconnect`.

### Full-body pose data is unavailable

- Confirm that both ankle trackers are powered on, paired, and calibrated.
- Confirm that `Pico Motion Tracker` is set to `Full body`.
- Repeat the floor calibration and make sure the left and right trackers are not
  reversed.

### ADB displays `unauthorized`

Reconnect the USB cable, accept the USB debugging authorization in the headset,
and run `adb devices` again.

### Network latency or motion jitter is high

Use 5 GHz Wi-Fi, reduce the distance between the PICO and access point, and
limit video streaming or large file transfers on the same wireless network. For
hardware deployment, configure the dedicated G1 network interface separately
from the local network interface used by the PICO to avoid routing conflicts.

