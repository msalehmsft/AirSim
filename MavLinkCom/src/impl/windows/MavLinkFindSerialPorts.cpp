// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "../MavLinkConnectionImpl.hpp"
#include <Windows.h>
#include <ObjIdl.h>
#include <SetupAPI.h>
#include <Cfgmgr32.h>
#define INITGUID
#include <propkey.h>
#include <Devpkey.h>

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

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


std::vector<SerialPortInfo> MavLinkConnectionImpl::findSerialPorts(int vid, int pid)
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

            int dvid = 0, dpid = 0;
            if (parseVidPid(buffer, &dvid, &dpid) &&
                ((dvid == vid && dpid == pid) || (vid == 0 && pid == 0)))
            {
                ULONG keyCount = 0;
                cr = CM_Get_DevNode_Property_Keys(Devinst, NULL, &keyCount, 0);
                if (cr != CR_BUFFER_SMALL) {
                    continue;
                }

                SerialPortInfo portInfo;
                portInfo.pid = dpid;
                portInfo.vid = dvid;

                DEVPROPKEY* keyArray = new DEVPROPKEY[keyCount];

                if (CR_SUCCESS == CM_Get_DevNode_Property_Keys(Devinst, keyArray, &keyCount, 0)) {

                    for (DWORD j = 0; j < keyCount; j++)
                    {
                        DEVPROPKEY* key = &keyArray[j];
                        bool isItemNameProperty = (key->fmtid == PKEY_ItemNameDisplay.fmtid && key->pid == PKEY_ItemNameDisplay.pid);
                        if (isItemNameProperty) {
                            ULONG bufferSize = 0;
                            DEVPROPTYPE propertyType;
                            cr = CM_Get_DevNode_Property(Devinst, &keyArray[j], &propertyType, NULL, &bufferSize, 0);
                            if (cr == CR_BUFFER_SMALL && bufferSize > 0) {
                                BYTE* propertyBuffer = new BYTE[bufferSize];
                                cr = CM_Get_DevNode_Property(Devinst, &keyArray[j], &propertyType, propertyBuffer, &bufferSize, 0);
                                if (cr == CR_SUCCESS) {
                                    std::wstring displayName((WCHAR*)propertyBuffer);
                                    parseDisplayName(displayName, &portInfo);
                                }
                                delete[] propertyBuffer;
                            }
                        }
                    }
                }

                result.push_back(portInfo);

                delete[] keyArray;
            }
        }
    }

    delete[] DeviceInterfaceList;

    return result;
}
