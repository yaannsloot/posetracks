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
#include <filesystem>
#include <iostream>

BOOL WINAPI DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) {
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
        std::filesystem::path module_path(path);
        std::filesystem::path bin_path = module_path.parent_path() / "bin";
        if (!AddDllDirectory(bin_path.c_str())) {
            int ret = GetLastError();
            fprintf(stderr, "AddDllDirectory failed, error = %d\n", ret);
            return false;
        }
        break;
    }
    default:
        break;
    }
    return true;
}
