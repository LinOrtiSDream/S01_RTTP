#!/usr/bin/env python
# -------------------------------------------------------------------------------
# Name:        PyUsbXpresse  pyusbxp.py
# Purpose:     Python wrapper for UsbXpresse API
#
# Author:      nkom@rocketmail.com
#
# Ref:         http://www.silabs.com/Support%20Documents/TechnicalDocs/an169.pdf
#
# Created:     06/04/2013
# Copyright:   (c) nkom 2013
# Licence:     LGPL
#
#
# Installation: Simply put this module and SiUSBxp.dll in anywherer python can access.
#               SiUSBxp.dll can be found in
#                    C:\Program Files\Polimaster\PM PRD PoliIdentify Software
#               or   C:\Program Files (x86)\Polimaster\PM PRD PoliIdentify Software
#
#               You can get it from Silicon lab's USBXpress developement kit
#                   http://www.silabs.com/products/mcu/Pages/USBXpress.aspx
#
#
# ver. 0.22 2013-04-08
#       Some functions added. Few functions are still missing.
#
# ver. 0.1  2013-04-06
#       Initial release
# -------------------------------------------------------------------------------


import time
from ctypes import *
from ctypes.wintypes import *
from pprint import pprint, pformat
import struct
import sys

return_code_name = {
    0x00: "SI_SUCCESS",
    0xFF: "SI_DEVICE_NOT_FOUND",
    0x01: "SI_INVALID_HANDLE",
    0x02: "SI_READ_ERROR",
    0x03: "SI_RX_QUEUE_NOT_READY",
    0x04: "SI_WRITE_ERROR",
    0x05: "SI_RESET_ERROR",
    0x06: "SI_INVALID_PARAMETER",
    0x07: "SI_INVALID_REQUEST_LENGTH",
    0x08: "SI_DEVICE_IO_FAILED",
    0x09: "SI_INVALID_BAUDRATE",
    0x0a: "SI_FUNCTION_NOT_SUPPORTED",
    0x0b: "SI_GLOBAL_DATA_ERROR",
    0x0c: "SI_SYSTEM_ERROR_CODE",
    0x0d: "SI_READ_TIMED_OUT",
    0x0e: "SI_WRITE_TIMED_OUT",
    0x0f: "SI_IO_PENDING",

    0x1f: "waitRX_time_out"
}


# TODO using exceptions for return codes is strange possibly rewrite
class UsbxpError(Exception):
    def __init__(self, *args, error_id=None):
        Exception.__init__(self, *args)
        self.error_id = error_id
ex = UsbxpError


class Usbxp(object):
    dll = WinDLL("SiUSBxp_{architecture}{oldness}.dll".format(architecture=struct.calcsize('P') * 8,
                                                              oldness='' if sys.getwindowsversion()[0] > 5 else '_old'))
    vh = DWORD()
    vl = DWORD()
    # SI_STATUS SI_GetDLLVersion (DWORD* HighVersion,DWORD* LowVersion)
    dll.SI_GetDLLVersion(byref(vh), byref(vl))
    dll_version = (vh.value >> 16 & 0xFFFF, vh.value & 0xFFFF, vl.value >> 16 & 0xFFFF, vl.value & 0xFFFF)
    dll_version_string = "%d.%d.%d.%d" % dll_version
    # SI_STATUS SI_GetDriverVersion (DWORD* HighVersion,DWORD* LowVersion)
    if return_code_name[dll.SI_GetDriverVersion(byref(vh), byref(vl))] == "SI_FUNCTION_NOT_SUPPORTED":
        driver_version = None
        driver_version_string = '<Not Supported>'
    else:
        driver_version = (vh.value >> 16 & 0xFFFF, vh.value & 0xFFFF, vl.value >> 16 & 0xFFFF, vl.value & 0xFFFF)
        driver_version_string = "%d.%d.%d.%d" % driver_version

    @property
    def number_of_devices(self):
        # SI_STATUS SI_GetNumDevices (LPDWORD NumDevices)
        num_dev = DWORD()
        self.dll.SI_GetNumDevices(byref(num_dev))
        return num_dev.value

    @property
    def list_of_devices(self):
        # SI_GetProductString (DWORD DeviceNum, LPVOID DeviceString, DWORD Options)
        dsp = c_char_p(b'\0' * 256)
        # define SI_RETURN_SERIAL_NUMBER 0x00
        # define SI_RETURN_DESCRIPTION 0x01
        # define SI_RETURN_LINK_NAME 0x02
        # define SI_RETURN_VID 0x03
        # define SI_RETURN_PID 0x04
        lst = []
        for dn in range(self.number_of_devices):
            ll = {'Serial Number': 0x00, 'Description': 0x01, 'Link Name': 0x02, 'VID': 0x03, 'PID': 0x04}
            for key in ll:
                self.dll.SI_GetProductString(DWORD(dn), dsp, DWORD(ll[key]))
                ll[key] = dsp.value
            lst.append(ll)
        return lst

    def __init__(self):
        self._handle = HANDLE()
        self.is_open = False
        self.device_number = None

    def __del__(self, return_code=0):
        # Destructer and error habdling
        if self.is_open:
            self.close()
        if return_code:
            raise ex(return_code_name[return_code] + "\n" + pformat(self, width=100),
                     error_id=return_code_name[return_code])

    def die_on_error(self, return_code):
        if return_code:
            self.__del__(return_code)

    def __repr__(self):
        return "<{self.__class__.__name__} instance at {id}>:" \
               "\n    Using DLL v. {self.dll_version_string}" \
               "\n        driver v. {self.driver_version_string}" \
               "\n    currently found {self.number_of_devices} compatible device{s}:"\
                   .format(self=self, id=id(self), s='s'[self.number_of_devices < 2:]) \
               + "\n" + pformat(self.list_of_devices, indent=4, width=100) \
               + "\n" + pformat(vars(self), indent=4, width=100)

    def get_driver_version(self):
        # SI_STATUS SI_GetDriverVersion (DWORD* HighVersion,DWORD* LowVersion)
        vh = DWORD()
        vl = DWORD()
        return_code = self.dll.SI_GetDriverVersion(byref(vh), byref(vl))
        if return_code_name[return_code] == "SI_FUNCTION_NOT_SUPPORTED":
            return None
        else:
            self.die_on_error(return_code)
        return vh.value >> 16 & 0xFFFF, vh.value & 0xFFFF, vl.value >> 16 & 0xFFFF, vl.value & 0xFFFF

    def open(self, device_number=0):
        # SI_STATUS SI_Open (DWORD DeviceNum, HANDLE *Handle)
        self.device_number = device_number
        if self.device_number > self.number_of_devices - 1:
            self.device_number = 0  # !!!TODO some error message needed
        if self.is_open:
            self.__del__()
            time.sleep(0.5)
        self.die_on_error(
            self.dll.SI_Open(DWORD(self.device_number), byref(self._handle)))
        self.is_open = True
        return self._handle

    def close(self):
        # SI_Close (HANDLE Handle)
        return_code = self.dll.SI_Close(self._handle)
        if return_code:
            raise ex(return_code_name[return_code])
        self.is_open = False

    def read(self, nb, fmt=""):
        # SI_STATUS SI_Read (HANDLE Handle, LPVOID Buffer, DWORD NumBytesToRead,
        # DWORD *NumBytesReturned, OVERLAPPED* o = NULL)
        buf = create_string_buffer(4096)
#        o = c_char_p('\0' * 4096)
        nr = DWORD()
        self.die_on_error(
            self.dll.SI_Read(self._handle, buf, DWORD(nb), byref(nr), None))
        if fmt:
            return struct.unpack_from(fmt, buf.raw)
        else:   # !!!TODO possibly wrong
            s = buf.value[:nr.value]
            return s

    def write(self, s=""):
        # SI_STATUS SI_Write (HANDLE Handle, LPVOID Buffer, DWORD NumBytesToWrite,
        # DWORD *NumBytesWritten, OVERLAPPED* o = NULL)
        buf = c_char_p(s)
        nb = DWORD()
        self.die_on_error(
            self.dll.SI_Write(self._handle, buf, DWORD(len(s)), byref(nb), 0))
#        if self.dbg: print("Usbxp.write(): '%s' %d %s" % (s, nb.value, return_code_name[r]))
        return nb.value

    def writeln(self, s=""):
        return self.write(s + "\r\n")

    def wait_so(self):
        # WaitForSingleObject()
        pass

    def settimeout(self, rt=00, wt=100):
        # SI_SetTimeouts (DWORD ReadTimeout, DWORD WriteTimeout)
        self.die_on_error(
            self.dll.SI_SetTimeouts(DWORD(rt), DWORD(wt)))
        return self.gettimeout()

    def gettimeout(self):
        # SI_STATUS SI_GetTimeouts (LPDWORD ReadTimeout, LPDWORD WriteTimeout)
        rt = DWORD()
        wt = DWORD()
        self.die_on_error(
            self.dll.SI_GetTimeouts(byref(rt), byref(wt)))
        return rt.value, wt.value

    def check_rx(self):
        # SI_CheckRXQueue (HANDLE Handle, LPDWORD NumBytesInQueue, LPDWORD QueueStatus)
        nb = DWORD()
        qs = DWORD()
        self.die_on_error(
            self.dll.SI_CheckRXQueue(self._handle, byref(nb), byref(qs)))
        return nb.value, qs.value

    def wait_rx(self, tout=15000):
        iv = 0.1
        x = int(tout / 1000 / iv)
        for i in range(x):
            r = self.check_rx()
            if r[1] == 2:
                return r
            time.sleep(iv)
#        if self.dbg: print("waitRX:", x)
        self.__del__(0x1f)

    def flush(self):
        self.die_on_error(
            self.dll.SI_FlushBuffers(self._handle, BYTE(1), BYTE(1)))

    def status(self):
        b = BYTE(0)
        self.die_on_error(
            self.dll.SI_GetModemStatus(self._handle, byref(b)))
        return b.value

    def setbr(self, br=9600):
        self.die_on_error(
            self.dll.SI_SetBaudRate(self._handle, DWORD(br)))

    def setlc(self, lc):
        self.die_on_error(
            self.dll.SI_SetLineControl(self._handle, WORD(lc)))

    def setbreak(self, bs=0):
        # bs = 1:  Set break.    bs = 0 reset break
        self.die_on_error(
            self.dll.SI_SetBreak(self._handle, WORD(bs)))

    def getpn(self):
        b = BYTE(0)
        self.die_on_error(
            self.dll.SI_GetPartNumber(self._handle, byref(b)))
        return b.value


def main():
    # List USBXpress devices
    pprint(Usbxp())

if __name__ == '__main__':
    main()
