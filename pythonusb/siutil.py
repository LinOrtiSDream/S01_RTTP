import ctypes
import ctypes.wintypes


class SiUtil:
    def __init__(self):
        self.dll = ctypes.windll.SiUtil

    def __del__(self):
        if self.is_connected():
            self.usb_disconnect()   # TODO when serial connection will be added, disconnect properly

    def get_usb_current_status(self):
        ret = {'number_of_debug_devices': self.get_usb_number_of_debug_devices(),
               'device_sn': None, 'firmware_version': None,
               'is_connected': self.is_connected()}
        if ret['number_of_debug_devices'] > 0:
            ret['device_sn'] = self.get_usb_device_sn()
            ret['firmware_version'] = self.get_usb_firmware_version()
        return ret

    def get_dll_version(self):
        self.dll.GetDLLVersion.restype = ctypes.c_char_p
        return self.dll.GetDLLVersion().decode('utf-8')

    # GetDLLVersion()
    # Description: This function returns the current version of the Utilities DLL.
    # Supported Debug Adapters: Serial Adapter, USB Debug Adapter
    # C++ Prototype: extern "C" __declspec(dllimport) char* __stdcall GetDLLVersion();
    # Parameters: none
    # Return Value: A string containing the Utilities DLL version

    def get_usb_dll_version(self):
        ret = ctypes.c_char_p()
        self.dll.GetUSBDLLVersion(ctypes.byref(ret))
        return ret.value.decode('utf-8')

    def get_usb_number_of_debug_devices(self):
        ret = ctypes.wintypes.DWORD()
        self.dll.USBDebugDevices(ctypes.byref(ret))
        return ret.value

    def get_usb_device_sn(self, number=0):
        ret = ctypes.c_char_p()
        self.dll.GetUSBDeviceSN(ctypes.wintypes.DWORD(number), ctypes.byref(ret))
        return ret.value.decode('utf-8') if ret.value is not None else None

    def get_usb_firmware_version(self):
        return self.dll.GetUSBFirmwareVersion()

    def get_device_name(self):
        ret = ctypes.c_char_p()
        self.dll.GetDeviceName(ctypes.byref(ret))
        return ret.value.decode('utf-8') if ret.value is not None else None

    # 4.6. GetDeviceName()
    # Description: This function returns the name of the target C8051Fxxx device that is currently supported.
    # Supported Debug Adapters: Serial Adapter, USB Debug Adapter
    # C++ Prototype: extern "C" __declspec(dllimport) HRESULT__stdcall GetDeviceName(
    # const char **psDeviceName);
    # Parameters: 1. psDeviceName - A pointer to a character string location where the device name will be copied.
    # Return Value: See Section 12 on page 16 for error return information.

    def is_connected(self):
        return self.dll.Connected()

    def usb_connect(self, serial_number='', protocol=1, power_target=False, disable_dialog_boxes=False):
        self.dll.ConnectUSB.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        self.dll.ConnectUSB(serial_number.encode(), protocol, power_target, disable_dialog_boxes)
# 3.3. ConnectUSB()
# Description: This function is used to connect to a target C8051Fxxx device using a USB Debug Adapter.
# Establishing a valid connection is necessary for all memory operations to succeed.
# Supported Debug Adapters: USB Debug Adapter
# C++ Prototype: extern “C” __declspec(dllimport) HRESULT__stdcall ConnectUSB(
# const char * sSerialNum=””, int nECprotocol=0, int nPowerTarget=0,
# int nDisableDialogBoxes=0);
# Parameters: 1. sSerialNumber—The serial number of the USB Debug Adapter. See Section 8 for information
# on obtaining the serial number of each USB Debug Adapter connected. If only one USB
# Debug Adapter is connected, an empty string can be used. The default is an empty string.
# 2. nECprotocol—Connection protocol used by the target device; JTAG (0) or Silicon Laboratories
# 2-Wire (C2) (1). The default is 0. The C2 interface is used with C8051F3xx derivative devices
# and the JTAG interface is used with C8051F0xx, C8051F1xx, and C8051F2xx derivative
# devices.
# 3. nPowerTarget—If this parameter is set to 1, the USB Debug Adapter will be configured to
# continue supplying power after it has been disconnected from the target device. The default is
# 0, configuring the adapter to discontinue supplying power when disconnected.
# 4. nDisableDialogBoxes—Disable (1) or enable (0) dialogs boxes within the DLL. The default is 0.
# Return Value: See Section 12 on page 16 for error return information.

    def usb_disconnect(self):
        self.dll.DisconnectUSB()

# 3.4. DisconnectUSB()
# Description: This function is used to disconnect from a target C8051Fxxx device using a USB Debug Adapter.
# Supported Debug Adapters: USB Debug Adapter
# C++ Prototype: extern “C” __declspec(dllimport) HRESULT__stdcall DisconnectUSB();
# Parameters: none
# Return Value: See Section 12 on page 16 for error return information.

    def download(self, file_name, device_erase=False, disable_dialog_boxes=False, download_scratchpad_sfle=False,
                 bank_select=-1, lock_flash=False, persist_flash=True):
        if not isinstance(file_name, str):
            file_name = str(file_name)
        if download_scratchpad_sfle or bank_select != -1 or lock_flash:
            raise Exception("DON'T DO IT!!!")   # TODO some meaningful checks needed
        self.dll.Download.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_int, ctypes.wintypes.BOOL]
        self.dll.Download(file_name.encode(), device_erase, disable_dialog_boxes, download_scratchpad_sfle,
                          bank_select, lock_flash, persist_flash)
# 4.1. Download()
# Description: This function is used to download a hex file to a target C8051Fxxx device. After a successful exit
# from the Download() function, the target C8051xxx will be in a “Halt” state. If the device is left in
# the “Halt” state, it will not begin code execution until the device is reset by a Power-On reset or
# by a SetTargetGo() DLL function call.
# Supported Debug Adapters: Serial Adapter, USB Debug Adapter
# C++ Prototype: extern “C” __declspec(dllimport) HRESULT__stdcall Download(
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
# Return Value: See Section 12 on page 16 for error return information

    def set_target_go(self):
        self.dll.SetTargetGo()
# 7.1. SetTargetGo()
# Description: After a successful exit from the SetTargetGo() function, the target C8051xxx will be in a “Run” state.
# Supported Debug Adapters: Serial Adapter, USB Debug Adapter
# C++ Prototype: extern “C” __declspec(dllimport) HRESULT__stdcall SetTargetGo();
# Parameters: none
# Return Value: See Section 12 on page 16 for error return information.

    def set_target_halt(self):
        self.dll.SetTargetHalt()
# 7.2. SetTargetHalt()
# Description: After a successful exit from the SetTargetHalt() function, the target C8051xxx will be in a “Halt” state.
# Supported Debug Adapters: Serial Adapter, USB Debug Adapter
# C++ Prototype: extern “C” __declspec(dllimport) HRESULT__stdcall SetTargetHalt();
# Parameters: none
# Return Value: See Section "12. Test Results" on page 16 for error return information.

    def usb_flash_run_and_disconnect(self, *args, **kwargs):
        self.download(*args, **kwargs)
        self.set_target_go()
        self.usb_disconnect()