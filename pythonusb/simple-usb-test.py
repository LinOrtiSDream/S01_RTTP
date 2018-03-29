# Short Example of DLL interfacing:

import ctypes

# ERROR CODES (non-success ones removed for the example)
SI_SUCCESS                      = 0x00

# GetProductString() function flags
SI_RETURN_SERIAL_NUMBER         = 0x00
SI_RETURN_DESCRIPTION           = 0x01
SI_RETURN_LINK_NAME             = 0x02
SI_RETURN_VID                   = 0x03
SI_RETURN_PID                   = 0x04

# Buffer size limits
SI_MAX_DEVICE_STRLEN            = 256

# if hasattr(ctypes, 'windll'):
#     usbxlib = ctypes.windll.SiUSBXp
# elif hasattr(ctypes, 'cdll'):
#     ctypes.cdll.LoadLibrary("libc.so.6")
#     usbxlib = ctypes.windll.SiUSBXp
#     print (ctypes.cdll)
# else:
#     print ("NOT DEFINED")
usbxlib = ctypes.windll.SiUSBXp

#
# N.B. All the DLL calls return a status code 
#


# SI_GetNumDevices(LPDWORD lpdwNumDevices);
def can_get_num_devices():
    num_devices = ctypes.c_ulong()
    status = usbxlib.SI_GetNumDevices(ctypes.pointer(num_devices))
    if status != SI_SUCCESS:
        print('Get devices error code ', status)
    return num_devices.value
    
  
# SI_GetProductString(DWORD dwDeviceNum, LPVOID lpvDeviceString, DWORD dwFlags);
def can_get_product_string(device_num, flags=SI_RETURN_DESCRIPTION):
    sbuf = ctypes.create_string_buffer(SI_MAX_DEVICE_STRLEN)
    status = usbxlib.SI_GetProductString(device_num, ctypes.pointer(sbuf), flags)
    if status != SI_SUCCESS:
        print('Get Product string status ', status)
    return sbuf.value
    
 
if __name__ == '__main__':
  
    print('USBXpress Python interfacing sample')
    print('Num devices = ', can_get_num_devices())
    print('Device 0: ')
    print('SI_RETURN_SERIAL_NUMBER = ', can_get_product_string(0, SI_RETURN_SERIAL_NUMBER))
    print('SI_RETURN_DESCRIPTION = ', can_get_product_string(0, SI_RETURN_DESCRIPTION))
    print('SI_RETURN_LINK_NAME = ', can_get_product_string(0, SI_RETURN_LINK_NAME))
    print('SI_RETURN_VID = ', can_get_product_string(0, SI_RETURN_VID))
    print('SI_RETURN_PID = ', can_get_product_string(0, SI_RETURN_PID))