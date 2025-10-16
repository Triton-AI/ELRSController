# ELRSController — Debug logging

This project uses a compile-time debug flag to enable Serial debug output.

How to enable debug logging with PlatformIO

Add the following to the environment you build for in `platformio.ini`:

> **Note:** Replace `your_env` with the actual environment name you use in your `platformio.ini` file (e.g., `[env:leonardo]`).
```ini
[env:your_env]
build_flags = -DDEBUG_LOG
```

When `DEBUG_LOG` is defined the code will initialize Serial and DEBUG_PRINT/DEBUG_PRINTLN
macros will output to Serial. When `DEBUG_LOG` is not defined the macros compile to
no-ops and there will be no Serial output.

Caution

Some boards (especially those that emulate USB HID like a joystick) have conflicts
between the Serial USB interface and HID. Only enable debug Serial during testing.
