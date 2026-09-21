// test_firmware_cmd.cpp
// Standalone test for FirmwareCmd — requires real Astra Pro hardware.
// Usage: ./test_firmware_cmd (needs udev rules or root for USB access)

#include "src/UsbDevice.h"
#include "src/FirmwareCmd.h"

#include <cstdio>
#include <cstdlib>
#include <unistd.h>

static int test_firmware_cmd()
{
    // 1. Open device
    UsbDevice usb;
    if (!usb.openByVidPid(0x2bc5, 0x0403)) {
        fprintf(stderr, "FAIL: could not open Astra Pro (0x2bc5:0x0403)\n");
        return 1;
    }
    printf("PASS: USB device opened\n");

    // 2. Create FirmwareCmd and init
    FirmwareCmd fw(&usb);
    if (!fw.init()) {
        fprintf(stderr, "FAIL: FirmwareCmd::init() returned false\n");
        usb.close();
        return 1;
    }
    printf("PASS: FirmwareCmd::init() succeeded\n");

    // 3. Print device info
    printf("\n--- Device Info ---\n");
    printf("Firmware version: 0x%08x\n", fw.firmwareVersion());
    printf("Serial number:    %s\n", fw.serialNumber().c_str());

    // 4. Test laser off -> on -> off
    printf("\n--- Laser Test ---\n");

    // Read current state
    bool laserOn = false;
    if (fw.getLaserState(laserOn)) {
        printf("Current laser state: %s\n", laserOn ? "ON" : "OFF");
    } else {
        printf("Could not read laser state (non-fatal)\n");
    }

    // Laser ON
    if (fw.setLaser(true)) {
        printf("PASS: laser ON command succeeded\n");
    } else {
        fprintf(stderr, "FAIL: laser ON command failed\n");
        usb.close();
        return 1;
    }

    // Brief delay for firmware to apply change, then verify
    usleep(100000);  // 100ms
    if (fw.getLaserState(laserOn)) {
        printf("Laser state after ON: %s\n", laserOn ? "ON" : "OFF");
    }

    // Laser OFF
    if (fw.setLaser(false)) {
        printf("PASS: laser OFF command succeeded\n");
    } else {
        fprintf(stderr, "FAIL: laser OFF command failed\n");
        usb.close();
        return 1;
    }

    // Verify it's off
    usleep(100000);
    if (fw.getLaserState(laserOn)) {
        printf("Laser state after OFF: %s\n", laserOn ? "ON" : "OFF");
    }

    // 5. Test flash read (camera params at 0x10000, safe read-only)
    printf("\n--- Flash Read Test ---\n");
    uint8_t flashData[64];
    int flashSize = 0;
    if (fw.readFlash(0x10000, 16, flashData, &flashSize)) {
        printf("PASS: flash read at 0x10000 (%d bytes)\n", flashSize);
        for (int i = 0; i < flashSize && i < 16; i++) {
            printf(" %02x", flashData[i]);
        }
        printf("\n");
    } else {
        printf("WARN: flash read failed (non-fatal)\n");
    }

    // 6. Test getParam (GetMode via XnHostProtocol)
    printf("\n--- GetParam Test ---\n");
    uint32_t modeVal = 0;
    if (fw.getParam(0x0005, modeVal)) {  // GetMode
        printf("PASS: GetMode = 0x%08x\n", modeVal);
    } else {
        printf("WARN: GetMode failed (non-fatal)\n");
    }

    // 7. Test getParam with a standard param ID (0x0002 = GetParam, param 0)
    printf("\n--- ReadFlash at serial (0x50000) ---\n");
    flashSize = 0;
    if (fw.readFlash(0x50000, 36, flashData, &flashSize)) {
        printf("PASS: flash read at 0x50000 (%d bytes)\n", flashSize);
        if (flashSize > 0) {
            // Show as ASCII (customer serial)
            int len = std::min(flashSize, 36);
            while (len > 0 && flashData[len-1] == 0) len--;
            printf("Customer serial: '%.*s'\n", len, reinterpret_cast<char*>(flashData));
        }
    } else {
        printf("WARN: flash read at 0x50000 failed (non-fatal)\n");
    }

    printf("\n--- All tests complete ---\n");
    usb.close();
    return 0;
}

int main()
{
    return test_firmware_cmd();
}
