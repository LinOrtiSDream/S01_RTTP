# Short Example of DLL interfacing:

import ctypes

flasher_lib = ctypes.windll.SiUtil
# flasher_lib = ctypes.CDLL("SiUtil.dll")

# extern “C” __declspec(dllimport) HRESULT__stdcall Download(
# char * sDownloadFile, int nDeviceErase=0, int nDisableDialogBoxes=0,
# int nDownloadScratchPadSFLE=0, int nBankSelect=-1, int nLockFlash=0),
# BOOL bPersistFlash=1);
# Parameters: 1. sDownloadFile—A character pointer to the beginning of a character array (string) containing
# the full path and filename of the file to be downloaded.
# 2. nDevice Erase—When set to 1, performs a device erase before the download initiates. If set
# to 0, the part will not be erased. A device erase will erase the entire contents of the device’s
# Flash. The default is 0.
# 3. nDisableDialogBoxes—Disable (1) or enable (0) dialogs boxes within the DLL. The default is 0.
# 4. nDownloadScratchPadSFLE—This parameter is only for use with devices that have a
# Scratchpad Flash memory block. Currently, this includes the C8051F02x, C8051F04x,
# C8051F06x, and C8051F12x devices. For all other devices, this parameter should be left in
# the default state. Set this parameter to 1 in order to download to Scratchpad memory. When
# accessing and downloading to Scratchpad memory, the only valid address range is
# 0x0000 to 0x007F. The default is 0.
# 5. nBankSelect—This parameter is only for use with C8051F12x devices. For all other devices,
# this parameter should be left in the default state. When using a C8051F12x derivative, set this
# parameter to 1, 2, or 3 in order to download to a specific bank. The default is –1.
# 6. nLockFlash—Set this parameter to 1 to lock the Flash following the download. If Flash is locked,
# the DLL will no longer be able to connect to the device.
# 7. bPersistFlash—If set to 1, the contents of Flash will be read prior to programming. Flash pages
# are erased prior to programming. If the pages to be programmed contain any data in Flash that
# need to be preserved, then set this parameter to 1.


if __name__ == '__main__':
    print("Alive!")
    flasher_lib.ConnectUSB("", 1, 0, 0)
    print("Connected:{}".format(flasher_lib.Connected()))
    flasher_lib.Download(ctypes.c_char_p(b"hexforflash\\Iteration2Project.hex"), 0, 0, 0, -1, 0, 0)
    flasher_lib.DisconnectUSB()
    print("Stil Alive!")
