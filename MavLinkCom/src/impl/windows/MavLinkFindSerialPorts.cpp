// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "MavLinkConnection.hpp"
#include <Windows.h>
#include <ObjIdl.h>
#include <SetupAPI.h>
#include <Cfgmgr32.h>
#define INITGUID
#include <propkey.h>
#include <devpkey.h>
#include <string>

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

// {4d36e978-e325-11ce-bfc1-08002be10318}
const GUID serialDeviceClass = { 0x4d36e978, 0xe325, 0x11ce, 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 };


bool parseVidPid(std::wstring deviceId, int* vid, int* pid)
{
    const wchar_t* ptr = deviceId.c_str();
    const wchar_t* end = ptr + deviceId.size();
    // parse out the VID number
    const wchar_t* pos = wcsstr(ptr, L"\\VID_");
    if (pos == NULL) {
        return false;
    }
    wchar_t* numberEnd = NULL;
    long c = wcstol(pos + 5, &numberEnd, 16);
    *vid = (int)c;

    // now the PID 
    pos = wcsstr(numberEnd, L"PID_");
    if (pos == NULL) {
        return false;
    }

    numberEnd = NULL;
    c = wcstol(pos + 4, &numberEnd, 16);
    *pid = (int)c;

    return true;
}

void parseDisplayName(std::wstring displayName, SerialPortInfo* info)
{
    info->displayName = displayName;
    const wchar_t* ptr = displayName.c_str();
    // parse out the VID number
    const wchar_t* pos = wcsrchr(ptr, '(');
    if (pos == NULL) {
        return;
    }
    pos++; // skip '('
    const wchar_t* end = wcschr(pos, ')');
    if (end != NULL) {
        info->portName = std::wstring(pos, (size_t)(end - pos));
    }
}


std::vector<SerialPortInfo> MavLinkConnection::findSerialPorts(int vid, int pid)
{
    bool debug = false;
    std::vector<SerialPortInfo> result;
    DEVINSTID pDeviceId = nullptr;
    PWSTR DeviceInterfaceList = nullptr;
    ULONG DeviceInterfaceListLength;

    CONFIGRET cr = CM_Get_Device_Interface_List_Size(&DeviceInterfaceListLength, const_cast<GUID*>(&GUID_DEVINTERFACE_COMPORT), pDeviceId, CM_GET_DEVICE_INTERFACE_LIST_ALL_DEVICES);

    if (CR_SUCCESS != cr) {
        return result;
    }

    DeviceInterfaceList = new WCHAR[DeviceInterfaceListLength];

    cr = CM_Get_Device_Interface_List(const_cast<GUID*>(&GUID_DEVINTERFACE_COMPORT), pDeviceId, DeviceInterfaceList, DeviceInterfaceListLength, CM_GET_DEVICE_INTERFACE_LIST_ALL_DEVICES);
    if (CR_SUCCESS != cr) {
        delete[] DeviceInterfaceList;
        return result;
    }

    for (PWSTR CurrentInterface = DeviceInterfaceList; *CurrentInterface; CurrentInterface += wcslen(CurrentInterface) + 1) {
        ULONG size = 0;
        DEVINST Devinst;
        ULONG PropertySize;
        DEVPROPTYPE PropertyType;
        WCHAR CurrentDevice[MAX_DEVICE_ID_LEN];

        PropertySize = sizeof(CurrentDevice);
        cr = CM_Get_Device_Interface_Property(CurrentInterface,
            &DEVPKEY_Device_InstanceId,
            &PropertyType,
            (PBYTE)CurrentDevice,
            &PropertySize,
            0);

        if (cr != CR_SUCCESS) {
            continue;
        }

        // Since the list of interfaces includes all interfaces, enabled or not, the
        // device that exposed that interface may currently be non-present, so
        // CM_LOCATE_DEVNODE_PHANTOM should be used.
        cr = CM_Locate_DevNode(&Devinst,
            CurrentDevice,
            CM_LOCATE_DEVNODE_PHANTOM);

        cr = CM_Get_Device_ID_Size(&size, Devinst, 0);
        if (cr == CR_SUCCESS) {
            std::wstring buffer(size + 1, '\0');
            cr = CM_Get_Device_ID(Devinst, (PWSTR)buffer.c_str(), size + 1, 0);

            // examples:
            // PX4: USB\VID_26AC&PID_0011\0
            // FTDI cable: FTDIBUS\VID_0403+PID_6001+FTUAN9UJA\0000"
            //printf("Found: %S\n", buffer.c_str());

			// Also support ACPI defined UART ports
			// "ACPI\\MSFT8000\\1"
			SerialPortInfo portInfo;
			DEVPROPTYPE propertyType;

#if 0
			// if you need the short port name
			const DEVPROPKEY propkeyPortName = {
				PKEY_DeviceInterface_Serial_PortName.fmtid,
				PKEY_DeviceInterface_Serial_PortName.pid
			};
			WCHAR portName[512] = { 0 };
			ULONG portNameBufferSize = sizeof(portName);
			cr = CM_Get_Device_Interface_PropertyW(
				CurrentInterface,
				&propkeyPortName,
				&propertyType,
				reinterpret_cast<BYTE*>(&portName),
				&portNameBufferSize,
				0); // ulFlags
#endif

			portInfo.portName = std::wstring(CurrentInterface);

			WCHAR displayName[512] = { 0 };
			ULONG displayNameBufferSize = sizeof(displayName);
			const DEVPROPKEY propkeyDisplayName = {
				PKEY_ItemNameDisplay.fmtid,
				PKEY_ItemNameDisplay.pid
			};
			cr = CM_Get_Device_Interface_PropertyW(
				CurrentInterface,
				&propkeyDisplayName,
				&propertyType,
				reinterpret_cast<BYTE*>(&displayName),
				&displayNameBufferSize,
				0); // ulFlags

			portInfo.displayName = std::wstring(displayName);

			uint16_t vid = 0;
			ULONG vidBufferSize = sizeof(vid);
			const DEVPROPKEY propkeyVid = {
				PKEY_DeviceInterface_Serial_UsbVendorId.fmtid,
				PKEY_DeviceInterface_Serial_UsbVendorId.pid
			};
			cr = CM_Get_Device_Interface_PropertyW(
				CurrentInterface,
				&propkeyVid,
				&propertyType,
				reinterpret_cast<BYTE*>(&vid),
				&vidBufferSize,
				0); // ulFlags

			portInfo.vid = vid;

			uint16_t pid = 0;
			ULONG pidBufferSize = sizeof(pid);
			const DEVPROPKEY propkeyPid = {
				PKEY_DeviceInterface_Serial_UsbProductId.fmtid,
				PKEY_DeviceInterface_Serial_UsbProductId.pid
			};
			cr = CM_Get_Device_Interface_PropertyW(
				CurrentInterface,
				&propkeyPid,
				&propertyType,
				reinterpret_cast<BYTE*>(&pid),
				&pidBufferSize,
				0); // ulFlags

			portInfo.pid = pid;
			
			result.push_back(portInfo);
        }
    }

    delete[] DeviceInterfaceList;

    return result;
}
