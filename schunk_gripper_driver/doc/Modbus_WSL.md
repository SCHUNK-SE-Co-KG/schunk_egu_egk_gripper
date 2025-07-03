# Using a Modbus gripper with Windows WSL2 + Ubuntu

1. Use a USB to RS485 converter
2. Install the required Windows drivers for it if needed
3. Check that the device appears active in Windows' device manager (e.g. as `USB Serial Converter`)
4. Install the `usbipd` app in Windows
5. Open a terminal with root permissions on Windows and
    ```bash
    usbipd list
    ```
    This gives a list of available devices.
    check the _BUSID_ of the device (e.g. `1-12`) and bind it with
    ```bash
    usbpid bind -b 1-12
    ```
    Check `usbipd list` again and verify that the device now appears as _shared_.
6. List your WSL instances with
    ```bash
    wsl -l
    ```
    and choose one to which to attach the device (e.g. `Ubuntu`).
7. Attach the device with
    ```bash
    usbipd attach -b 1-12 -w Ubuntu
    ```
8. Check `usbipd list` again and verify that the device now appears as _Attached_.
9. You should now reach the device in WSL as a tty device, such as `/dev/ttyUSB0`.
10. Repeat step `7.` every time you restart Windows. The attachment is not persistent.
