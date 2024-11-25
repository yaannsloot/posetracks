/*
Copyright (C) 2024 Ian Sloat

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

----------------------------------------------------------------------

DllMain for windows builds. Adds bin to the list of DLL directories.
*/

#include <windows.h>
#include <cstdio>
#include <string>
#include <iostream>

BOOL WINAPI DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
    std::cout << "OWOWOWOWWO" << std::endl;
    switch (ul_reason_for_call) {
    case DLL_PROCESS_ATTACH:
    {
        char path[MAX_PATH];
        HMODULE hm = NULL;
        if (GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
            GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
            (LPCSTR)&DllMain, &hm) == 0)
        {
            int ret = GetLastError();
            fprintf(stderr, "GetModuleHandle failed, error = %d\n", ret);
            return false;
        }
        if (GetModuleFileName(hm, path, sizeof(path)) == 0)
        {
            int ret = GetLastError();
            fprintf(stderr, "GetModuleFileName failed, error = %d\n", ret);
            return false;
        }
        if (!SetDefaultDllDirectories(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS)) {
            int ret = GetLastError();
            fprintf(stderr, "SetDefaultDllDirectories failed, error = %d\n", ret);
            return false;
        }
        std::string fullpath(path);
        fullpath += "\\bin";
        std::wstring wfullpath = std::wstring(fullpath.begin(), fullpath.end());
        if (!AddDllDirectory(wfullpath.c_str())) {
            int ret = GetLastError();
            fprintf(stderr, "AddDllDirectory failed, error = %d\n", ret);
            return false;
        }
        std::cout << fullpath << std::endl;
        break;
    }
    default:
        break;
    }
    return true;
}
