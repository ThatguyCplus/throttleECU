C2000 throttle ECU — self-contained source folder
==================================================

Why CCS shows “zero projects”
------------------------------
`c2000_f280049_throttle` is only **source code** (.c / .h). A **CCS project** is a
separate thing: a folder CCS creates that contains hidden project files (e.g.
`.cproject`, `.project`) plus build settings.

Opening this folder as a “workspace” does **not** create a project. You must use
**File → New → CCS Project** once, then attach these sources to that project.

All .c and .h for this app live in THIS ONE FOLDER (flat layout).
Quoted includes like #include "board.h" resolve to this same directory — you do NOT add
this folder to the compiler’s #include search path. Only add C2000Ware paths (below).

CCS setup (do this in order)
-----------------------------
1. **Create the project (this is the step you were missing)**
   - In CCS or CCS Theia: **File → New → CCS Project** (or **New Project**).
   - **Target**: TMS320F280049C (or LAUNCHXL-F280049C if offered).
   - **Project name**: e.g. `throttle_f280049`.
   - **Location**: your CCS workspace folder (e.g. `…/workspace_ccstheia/`).
   - **Template**: **Empty Project** or **Empty Project with DriverLib** for F28004x.
     Do **not** pick a random “Library” or PMBus sample as the base.
   - Finish the wizard. You should now see **one project** in the Project Explorer.

2. **Remove the template `main.c` (if the wizard created one)**
   - Our code already provides `main.c`. Delete or exclude the template’s `main.c`
     so you only have **one** `main.c` in the project.

3. **Add our source files (pick ONE approach)**

   **A — Link files (keeps this repo folder as-is)**
   - Right‑click the new project → **Add Files…** (or **Import**).
   - Browse to `c2000_f280049_throttle`.
   - Select **all nine** `.c` files:  
     `main.c`, `board.c`, `encoder_gpio.c`, `motor_epwm.c`, `adc_sense.c`,
     `sci_io.c`, `pid.c`, `safety.c`, `throttle_ecu.c`.
   - If CCS asks **Copy** vs **Link**: choose **Link** so the `.c` files stay here;
     headers in the same folder still resolve correctly.

   **B — Copy files (everything lives inside the CCS project folder)**
   - Copy **all** `.c` and `.h` files from `c2000_f280049_throttle` into the new
     project’s source folder (same directory as the project’s other sources).
   - In CCS: **Refresh** the project so the files appear.

4. **C2000Ware include paths** (still required)
   - Right‑click the project → **Properties** → **Build** → **C2000 Compiler** → **Include Options**.
   - Add ONLY the TI paths, for example:
     C:/ti/C2000Ware_xxx/device_support/f28004x/headers/include
     C:/ti/C2000Ware_xxx/driverlib/f28004x/driverlib
   - Use your real C2000Ware folder name (e.g. `C2000Ware_26_00_00_00`).

5. **Linker**: keep the F280049 **Flash** `.cmd` file from the empty project template
   (Project Properties → Build → C2000 Linker → File Search Path if you need to check).

6. **device.c / startup**: keep what the empty template added (`device.c`, codestartup, etc.).
   Do not duplicate them. Only one `main.c` (ours).

7. **Build** (hammer icon), then **Debug** (bug icon).

If anything still fails, the first error line tells you what’s missing (usually device.h path).
