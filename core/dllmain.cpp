// dllmain.cpp : Defines the entry point for the DLL application.
#include <windows.h>
#include <pch.h>
#include <cstdio>
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <motion_engine.hpp>

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
#ifndef WITH_CONSOLE
    std::stringstream ss;
    if (lpReserved == NULL)
        ss << "MEComputeLibdll_" << 0;
    else
        ss << "MEComputeLibdll_" << lpReserved;
    std::string stdout_file = ss.str();
    std::string stderr_file = ss.str();
    stdout_file += "_stdout.txt";
    stderr_file += "_stderr.txt";
#endif
    switch (ul_reason_for_call)
    {
    case DLL_PROCESS_ATTACH:
#ifdef WITH_CONSOLE
        FILE* pConsole;
        AllocConsole();
        freopen_s(&pConsole, "CONOUT$", "wb", stdout);
        std::cout << "Loaded MotionEngine API" << std::endl;
#else
        FILE* f_stdout;
        FILE* f_stderr;
        freopen_s(&f_stdout, stdout_file.c_str(), "w", stdout);
        freopen_s(&f_stderr, stderr_file.c_str(), "w", stderr);
#endif
        break;
    case DLL_THREAD_ATTACH:
        break;
    case DLL_THREAD_DETACH:
        break;
    case DLL_PROCESS_DETACH:
        _fcloseall();
        break;
}
    return TRUE;
}

