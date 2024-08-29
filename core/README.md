# PoseTracks Core

PoseTracks Core is a compiled python module responsible for executing pre-trained models and performing computer vision related tasks. It is required for the add-on to function.

## Building

### Using build.py

The build.py script is the easiest way to build PoseTracks Core. It currently only works for Windows and Linux.

The following must be installed and available on the system PATH:

* Windows: VisualStudio BuildTools or a VS IDE with the desktop C++ development package installed.
* Linux: GCC
* CMake 3.16 or later
* Python 3.9, 3.10, and/or 3.11

Additionally, GPU builds require:
* CUDA 12
* CUDA 12 Toolkit
* cuDNN 8.9.2.26

If building as part of the add-on, use this table as a guide to see which python version is required.

| Python | Blender |
| ------ | ------- |
| 3.9    | 2.93 to < 3.11 |
| 3.10   | 3.1 to < 4.1 |
| 3.11   | 4.1 or later |

Once these requirements are met, run:

```
python build.py
```

All 3rd party libraries except for ONNXRuntime will be built from source. This can potentially take a **LONG** time.
Built libraries will be stored in the `3rd_party` directory, while the module itself can be found in the `build/redis` directory. 
