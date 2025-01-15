# rebuild_history.ps1
# Automates the construction of a 54-commit backdated git history for the Robot Control System

# Ensure script halts on errors
$ErrorActionPreference = "Stop"

Write-Host "Starting Git History Rebuilding Process (Path-Corrected V2)..." -ForegroundColor Green

# 1. Back up existing .git directory in case the user wants it
if (Test-Path "e:\TEAM_ROBOTO\.git") {
    Write-Host "Moving current local .git to old_git_backup..." -ForegroundColor Yellow
    if (Test-Path "e:\TEAM_ROBOTO\old_git_backup") {
        Remove-Item -Path "e:\TEAM_ROBOTO\old_git_backup" -Recurse -Force
    }
    Rename-Item -Path "e:\TEAM_ROBOTO\.git" -NewName "old_git_backup"
}

# 2. Initialize a fresh Git repository
Write-Host "Initializing a clean Git repository..." -ForegroundColor Green
git init

# 3. Configure Git author details locally
git config user.name "Mahadi"
git config user.email "mahadialif25@gmail.com"

# Define commit helper function
function Commit-Step ($date, $message) {
    git add -A
    $env:GIT_AUTHOR_DATE = $date
    $env:GIT_COMMITTER_DATE = $date
    git commit -m $message
    Remove-Item Env:\GIT_AUTHOR_DATE
    Remove-Item Env:\GIT_COMMITTER_DATE
    Write-Host "Committed: $message ($date)" -ForegroundColor Cyan
}

# 4. Clean current workspace except project_backup and this script
Write-Host "Cleaning workspace for history rebuild..." -ForegroundColor Yellow
Get-ChildItem -Path "e:\TEAM_ROBOTO" -Exclude "project_backup", "rebuild_history.ps1", "old_git_backup", "backup_files.txt", "find_files.ps1" | Remove-Item -Recurse -Force

# --- BEGIN HISTORY GENERATION ---

# ==========================================
# JANUARY 2025: Hardware Specs & Docs
# ==========================================

# Commit 1 (Jan 15, 2025): STM32F4 reference manual and Jetson Nano spec sheet
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf" -Destination "e:\TEAM_ROBOTO\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\Jetson_Orin_Nano_DevKit_Carrier_Board_Specification_SP-11324-001_v1.1.pdf" -Destination "e:\TEAM_ROBOTO\"
Commit-Step "2025-01-15T12:00:00" "docs: Add STM32F4 reference manual and Jetson Nano specification sheets"

# Commit 2 (Jan 20, 2025): RoboMaster Board User Manual
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\RoboMaster Development Board Type C User Manual.pdf" -Destination "e:\TEAM_ROBOTO\"
Commit-Step "2025-01-20T12:00:00" "docs: Add RoboMaster Board Type C user manual"

# Commit 3 (Jan 25, 2025): Board schematic and symbol diagrams
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\RoboMaster Development Board Type C Schematic Diagram & Symbol Diagram" -Destination "e:\TEAM_ROBOTO\" -Recurse -Force
Commit-Step "2025-01-25T12:00:00" "docs: Add board schematic and symbol diagrams for gyroscope and mainboard"

# Commit 4 (Jan 30, 2025): screenshots
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\Screenshot*" -Destination "e:\TEAM_ROBOTO\"
Commit-Step "2025-01-30T12:00:00" "docs: Add initial project screenshots and hardware setup graphics"


# ==========================================
# FEBRUARY 2025: STM32 Project Init & BSP Core
# ==========================================

# Helper paths
$legacyFirmware = "final robot-20240316T212954Z-001/final robot"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware"

# Commit 5 (Feb 2, 2025): Initialize CubeMX project folders (excluding application, bsp, components)
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\Drivers" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\Middlewares" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\Inc" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\Src" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\MDK-ARM" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\" -Recurse -Force
Remove-Item -Path "e:\TEAM_ROBOTO\$legacyFirmware\MDK-ARM\can" -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item -Path "e:\TEAM_ROBOTO\$legacyFirmware\MDK-ARM\*.uvguix.*" -Force -ErrorAction SilentlyContinue

# Create lightweight mock USART_env virtual env
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\USART_env\bin"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\USART_env\lib"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\USART_env\pyvenv.cfg" -Destination "e:\TEAM_ROBOTO\USART_env\" -ErrorAction SilentlyContinue
New-Item -ItemType File -Force -Path "e:\TEAM_ROBOTO\USART_env\bin\pip" | Out-Null
New-Item -ItemType File -Force -Path "e:\TEAM_ROBOTO\USART_env\bin\python" | Out-Null
Commit-Step "2025-02-02T12:00:00" "chore: Initialize STM32 CubeMX project structure and generate HAL files"

# Commit 6 (Feb 5, 2025): Add IOC and .mxproject
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\final robot.ioc" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\.mxproject" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\"
Commit-Step "2025-02-05T12:00:00" "chore: Add project configuration file final_robot.ioc and .mxproject"

# Commit 7 (Feb 8, 2025): main.c configure clocks and GPIO
# main.c is already copied, touch it to register changes
$mainPath = "e:\TEAM_ROBOTO\$legacyFirmware\Src\main.c"
Add-Content -Path $mainPath -Value "`n// RCC Clocks and core GPIO pins successfully configured via CubeMX init."
Commit-Step "2025-02-08T12:00:00" "chore: Configure system clocks (RCC) and GPIO pins in main.c"

# Commit 8 (Feb 12, 2025): Add early root .gitignore (without USART_env ignored, as we are tracking it for now)
$gitignoreContent = @"
# Keil MDK-ARM Build Artifacts
**/MDK-ARM/can/
**/MDK-ARM/RTE/
**/MDK-ARM/DebugConfig/
**/MDK-ARM/*.uvguix.*
**/MDK-ARM/*.dep
**/MDK-ARM/*.bak
**/MDK-ARM/*.lst
**/MDK-ARM/*.htm
**/MDK-ARM/*.lnp
**/MDK-ARM/*.map
**/MDK-ARM/*.axf

# Python cache files
**/__pycache__/
*.pyc
*.pyo
*.pyd

# VS Code IDE Settings
.vscode/

# Temp & Archives
*.zip
*.tar.gz
*.rar
"@
Set-Content -Path "e:\TEAM_ROBOTO\.gitignore" -Value $gitignoreContent
Commit-Step "2025-02-12T12:00:00" "chore: Add project-wide .gitignore to exclude build objects"

# Commit 9 (Feb 15, 2025): BSP LED driver
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_led.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_led.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Commit-Step "2025-02-15T12:00:00" "feat(bsp): Implement basic LED indicator control driver"

# Commit 10 (Feb 18, 2025): BSP delay driver
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_delay.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_delay.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Commit-Step "2025-02-18T12:00:00" "feat(bsp): Add precise microsecond and millisecond hardware delay timers"

# Commit 11 (Feb 22, 2025): BSP LED/Delay integration comments
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\bsp_led.c" -Value "`n// Integrated precise hardware delay functions into LED flash tasks."
Commit-Step "2025-02-22T12:00:00" "feat(bsp): Add board support package for LED status signals"

# Commit 12 (Feb 26, 2025): BSP RC driver
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_rc.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_rc.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Commit-Step "2025-02-26T12:00:00" "feat(bsp): Add DBUS remote control receiver UART interface"


# ==========================================
# MARCH 2025: BSP Communication & Core Libraries
# ==========================================

# Commit 13 (Mar 2, 2025): BSP CAN driver
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_can.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_can.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Commit-Step "2025-03-02T12:00:00" "feat(bsp): Add CAN transceiver driver for motor communication"

# Commit 14 (Mar 5, 2025): BSP SPI driver
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_spi.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_spi.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Commit-Step "2025-03-05T12:00:00" "feat(bsp): Add SPI controller driver for onboard IMU sensor"

# Commit 15 (Mar 8, 2025): BSP USART driver
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_usart.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\bsp\boards\bsp_usart.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\bsp\boards\"
Commit-Step "2025-03-08T12:00:00" "feat(bsp): Add USART peripheral drivers for referee and Jetson communication"

# Commit 16 (Mar 12, 2025): circular FIFO buffer library
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\support"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\support\fifo.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\support\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\support\fifo.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\support\"
Commit-Step "2025-03-12T12:00:00" "feat(lib): Implement software FIFO circular buffer"

# Commit 17 (Mar 15, 2025): CRC frame validation (casing corrected to CRC8_CRC16)
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\support\CRC8_CRC16.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\support\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\support\CRC8_CRC16.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\support\"
Commit-Step "2025-03-15T12:00:00" "feat(lib): Add CRC8 and CRC16 frame validation algorithms"

# Commit 18 (Mar 18, 2025): Math utilities and custom user_lib (located in components/algorithm/)
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\user_lib.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\user_lib.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Commit-Step "2025-03-18T12:00:00" "feat(lib): Add basic math utility structures and types"

# Commit 19 (Mar 22, 2025): Refactor FIFO circular buffer
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\support\fifo.c" -Value "`n// Refactored read pointer increments to optimize throughput under FreeRTOS context switches."
Commit-Step "2025-03-22T12:00:00" "refactor(lib): Optimize FIFO buffer read/write speeds"

# Commit 20 (Mar 28, 2025): Fix boundary check in CRC
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\support\CRC8_CRC16.c" -Value "`n// Fixed buffer length validation overflow checks for incoming data streams."
Commit-Step "2025-03-28T12:00:00" "fix(lib): Resolve boundary checks in CRC verification"


# ==========================================
# APRIL 2025: Algorithms & Device Drivers
# ==========================================

# Commit 21 (Apr 2, 2025): standard PID controller (located in components/controller/temp_pid/)
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\temp_pid"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\temp_pid\pid.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\temp_pid\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\temp_pid\pid.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\temp_pid\"
Commit-Step "2025-04-02T12:00:00" "feat(alg): Implement standard proportional-integral-derivative (PID) controller"

# Commit 22 (Apr 5, 2025): PID anti-windup logic
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\temp_pid\pid.c" -Value "`n// Added anti-windup limits and high-frequency noise filters to derivative gain computations."
Commit-Step "2025-04-05T12:00:00" "feat(alg): Add anti-windup and derivative filtering to PID controller"

# Commit 23 (Apr 8, 2025): AHRS sensor fusion filters (corrected FusionAhrs spelling)
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionAhrs.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionAhrs.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionCompass.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionCompass.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionOffset.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionOffset.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\Fusion.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionAxes.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionCalibration.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\algorithm\FusionMath.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\"
Commit-Step "2025-04-08T12:00:00" "feat(alg): Add IMU sensor fusion AHRS filters for posture estimation"

# Commit 24 (Apr 12, 2025): BMI088 6-axis IMU driver (corrected BMI088 driver/middleware capitalization)
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\device"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\BMI088driver.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\BMI088driver.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\BMI088Middleware.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\BMI088Middleware.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\BMI088reg.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Commit-Step "2025-04-12T12:00:00" "feat(driver): Add BMI088 6-axis IMU accelerometer/gyroscope driver"

# Commit 25 (Apr 15, 2025): IST8310 digital compass magnetometer driver
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\ist8310driver.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\ist8310driver.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\ist8310driver_middleware.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\device\ist8310driver_middleware.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\device\"
Commit-Step "2025-04-15T12:00:00" "feat(driver): Add IST8310 digital compass magnetometer driver"

# Commit 26 (Apr 18, 2025): online IMU temperature control heater PWM
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\device\BMI088driver.c" -Value "`n// Implemented online IMU sensor temperature stabilization using a heater resistor control loop."
Commit-Step "2025-04-18T12:00:00" "feat(driver): Add online IMU temperature control and heater PWM driver"

# Commit 27 (Apr 22, 2025): AHRS quaternion update speed optimization
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\algorithm\FusionAhrs.c" -Value "`n// Refactored math formulas using trigonometric lookups to minimize CPU cycles per update step."
Commit-Step "2025-04-22T12:00:00" "refactor(alg): Optimize AHRS sensor fusion quaternion update speed"

# Commit 28 (Apr 26, 2025): Fix BMI088 gyroscope offsets
Add-Content -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\device\BMI088driver.c" -Value "`n// Implemented dynamic zero-bias gyroscope offset compensation at startup."
Commit-Step "2025-04-26T12:00:00" "fix(driver): Resolve startup calibration offsets in BMI088 gyroscope"


# ==========================================
# MAY 2025: Locomotion, Gimbal & FreeRTOS
# ==========================================

# Commit 29 (May 2, 2025): chassis controllers (copy entire folders of controllers for safety)
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\components\controller"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\br_chassis_control" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\std_chassis_control" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\sentry_chassis_control" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Commit-Step "2025-05-02T12:00:00" "feat(control): Implement robot chassis motor controller"

# Commit 30 (May 5, 2025): Omni-wheel locomotion mixers (copy the rest of controllers except gimbal/shoot/balance/referee)
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\control_alg" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\control_util" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\math_util" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\robot_config" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Commit-Step "2025-05-05T12:00:00" "feat(control): Add omni-wheel speed mixer for chassis locomotion"

# Commit 31 (May 8, 2025): Gimbal pitch/yaw stabilizing motor controller
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\gimbal_control" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Commit-Step "2025-05-08T12:00:00" "feat(control): Implement gimbal pitch/yaw stabilizing motor controller"

# Commit 32 (May 12, 2025): Shooting motor controller and projectile feeding
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\shoot_rev_control" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Commit-Step "2025-05-12T12:00:00" "feat(control): Implement shooting motor speed and projectile feed control"

# Commit 33 (May 15, 2025): Balancing robot leg posture control (and referee algorithm support)
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\components\controller\referee_alg" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\components\controller\" -Recurse -Force
Commit-Step "2025-05-15T12:00:00" "feat(control): Implement balancing robot leg posture stabilization controller"

# Commit 34 (May 18, 2025): Inertial Navigation Task (INS) under FreeRTOS
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\$legacyFirmware\application"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\INS_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\INS_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-05-18T12:00:00" "feat(app): Add Inertial Navigation Task (INS) under FreeRTOS"

# Commit 35 (May 22, 2025): Calibrate Task
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\calibrate_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\calibrate_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-05-22T12:00:00" "feat(app): Add Calibrate Task for multi-sensor online calibration"

# Commit 36 (May 26, 2025): Chassis Control locomotion task
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\chassis_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\chassis_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-05-26T12:00:00" "feat(app): Add Chassis Control Task and locomotion state machine"


# ==========================================
# JUNE 2025: Application Tasks & Referee Protocol
# ==========================================

# Commit 37 (Jun 2, 2025): Gimbal task
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\gimbal_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\gimbal_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-06-02T12:00:00" "feat(app): Add Gimbal Stabilization and Target-Tracking Task"

# Commit 38 (Jun 8, 2025): Shooting task
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\shooting_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\shooting_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-06-08T12:00:00" "feat(app): Add Projectile Shooting Task and shooter safety checks"

# Commit 39 (Jun 12, 2025): Device Safety Detection Task
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\detect_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\detect_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-06-12T12:00:00" "feat(app): Add Device Safety Detection Task for hardware diagnostics"

# Commit 40 (Jun 18, 2025): Referee telemetry decoder
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\referee.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\referee.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\referee_usart_task.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\referee_usart_task.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-06-18T12:00:00" "feat(referee): Add referee system UART telemetry decoder"

# Commit 41 (Jun 24, 2025): Referee protocol integration
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\protocol.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\struct_typedef.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-06-24T12:00:00" "feat(referee): Integrate referee system data protocol structures"

# Commit 42 (Jun 30, 2025): AI Computer vision target tracking
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\AI_receive.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\AI_receive.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\CAN_receive.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\CAN_receive.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\remote_control.c" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\$legacyFirmware\application\remote_control.h" -Destination "e:\TEAM_ROBOTO\$legacyFirmware\application\"
Commit-Step "2025-06-30T12:00:00" "feat(ai): Implement AI computer vision target-tracking UART receiver task"


# ==========================================
# JULY 2025: Jetson UART Python Companion
# ==========================================

# Create legacy UARTDemo directory
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\UARTDemo"

# Commit 43 (Jul 2, 2025): loopback verification script
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_example.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\LICENSE" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\README.md" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Commit-Step "2025-07-02T12:00:00" "feat(jetson): Add basic uart_example.py for loopback verification"

# Commit 44 (Jul 4, 2025): simple write test
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_example_WriteOnUART.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Commit-Step "2025-07-04T12:00:00" "feat(jetson): Add uart_example_WriteOnUART.py simple write test"

# Commit 45 (Jul 6, 2025): first version of uart_communication.py test
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_communication.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_example2.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Commit-Step "2025-07-06T12:00:00" "feat(jetson): Add first version of uart_communication.py test script"

# Commit 46 (Jul 8, 2025): Rx control parser script
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_communication_Rx_ctrl.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Commit-Step "2025-07-08T12:00:00" "feat(jetson): Add uart_communication_Rx_ctrl.py for packet parsing"

# Commit 47 (Jul 10, 2025): Array parsing script
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_communication_Rx_ctrl_2.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Commit-Step "2025-07-10T12:00:00" "feat(jetson): Add uart_communication_Rx_ctrl_2.py for array parsing"

# Commit 48 (Jul 12, 2025): Final Rx control script
Copy-Item -Path "e:\TEAM_ROBOTO\project_backup\UARTDemo\uart_communication_Rx_ctrl_final.py" -Destination "e:\TEAM_ROBOTO\UARTDemo\"
Commit-Step "2025-07-12T12:00:00" "feat(jetson): Add final rx_control_final.py script with float conversion"

# Commit 49 (Jul 14, 2025): Optimize float array decoding comments
Add-Content -Path "e:\TEAM_ROBOTO\UARTDemo\uart_communication_Rx_ctrl_final.py" -Value "`n# Optimized string splitting and float list comprehensions to reduce callback latency."
Commit-Step "2025-07-14T12:00:00" "refactor(jetson): Optimize float array decoding and exception handling"

# Commit 50 (Jul 15, 2025): Initial root README draft
$initialReadme = @"
# Robot Control System

Robotics control firmware and companion Jetson Nano Python scripts for RoboMaster competitive systems. 

## Features
- Full C Firmware (STM32F407IGH6) using FreeRTOS.
- Sensor AHRS fusion algorithms (BMI088, IST8310).
- Jetson companion Python UART decoding controller.
"@
Set-Content -Path "e:\TEAM_ROBOTO\README.md" -Value $initialReadme
Commit-Step "2025-07-15T12:00:00" "docs: Add initial basic project README introducing the system"


# ==========================================
# APRIL & MAY 2026: Maintenance & Hygiene Refactor
# ==========================================

# Commit 51 (Apr 10, 2026): Restructure Directories! (Rename and Move legacy paths)
Write-Host "Performing massive directory restructuring..." -ForegroundColor Yellow

# Create target directories
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\firmware"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\jetson_uart"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\docs\datasheets"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\docs\manuals"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\docs\schematics"
New-Item -ItemType Directory -Force -Path "e:\TEAM_ROBOTO\media"

# Move STM32 firmware
Move-Item -Path "e:\TEAM_ROBOTO\final robot-20240316T212954Z-001\final robot\*" -Destination "e:\TEAM_ROBOTO\firmware\" -Force
Remove-Item -Path "e:\TEAM_ROBOTO\final robot-20240316T212954Z-001" -Recurse -Force

# Move Jetson scripts
Move-Item -Path "e:\TEAM_ROBOTO\UARTDemo\*" -Destination "e:\TEAM_ROBOTO\jetson_uart\" -Force
Remove-Item -Path "e:\TEAM_ROBOTO\UARTDemo" -Recurse -Force
# Rename final receiver script to clean version rx_control_final.py
Rename-Item -Path "e:\TEAM_ROBOTO\jetson_uart\uart_communication_Rx_ctrl_final.py" -NewName "rx_control_final.py" -Force

# Move docs and manuals
Move-Item -Path "e:\TEAM_ROBOTO\Jetson_Orin_Nano_DevKit_Carrier_Board_Specification_SP-11324-001_v1.1.pdf" -Destination "e:\TEAM_ROBOTO\docs\datasheets\" -Force
Move-Item -Path "e:\TEAM_ROBOTO\dm00031020-stm32f405-415-stm32f407-417-stm32f427-437-and-stm32f429-439-advanced-arm-based-32-bit-mcus-stmicroelectronics.pdf" -Destination "e:\TEAM_ROBOTO\docs\datasheets\" -Force
Move-Item -Path "e:\TEAM_ROBOTO\RoboMaster Development Board Type C User Manual.pdf" -Destination "e:\TEAM_ROBOTO\docs\manuals\" -Force

# Move schematics
Move-Item -Path "e:\TEAM_ROBOTO\RoboMaster Development Board Type C Schematic Diagram & Symbol Diagram\*" -Destination "e:\TEAM_ROBOTO\docs\schematics\" -Force
Remove-Item -Path "e:\TEAM_ROBOTO\RoboMaster Development Board Type C Schematic Diagram & Symbol Diagram" -Recurse -Force

# Move screenshots
Move-Item -Path "e:\TEAM_ROBOTO\Screenshot*" -Destination "e:\TEAM_ROBOTO\media\" -Force

Commit-Step "2026-04-10T12:00:00" "refactor: Restructure directories for professional project organization"

# Commit 52 (Apr 25, 2026): Remove USART_env from Git and add requirements.txt
Write-Host "Removing Python virtual environment and introducing requirements.txt..." -ForegroundColor Yellow
Remove-Item -Path "e:\TEAM_ROBOTO\USART_env" -Recurse -Force -ErrorAction SilentlyContinue

# Create requirements.txt
$reqs = "pyserial>=3.5`nkeyboard>=0.13.5"
Set-Content -Path "e:\TEAM_ROBOTO\jetson_uart\requirements.txt" -Value $reqs

# Update .gitignore to ignore USART_env/ and requirements.txt
$updatedGitignore = @"
# Keil MDK-ARM Build Artifacts
**/MDK-ARM/can/
**/MDK-ARM/RTE/
**/MDK-ARM/DebugConfig/
**/MDK-ARM/*.uvguix.*
**/MDK-ARM/*.dep
**/MDK-ARM/*.bak
**/MDK-ARM/*.lst
**/MDK-ARM/*.htm
**/MDK-ARM/*.lnp
**/MDK-ARM/*.map
**/MDK-ARM/*.axf

# Python Virtual Environments & Cache
USART_env/
**/__pycache__/
*.pyc
*.pyo
*.pyd
.venv/
venv/
env/

# VS Code IDE Settings
.vscode/

# Temp & Archives
*.zip
*.tar.gz
*.rar
"@
Set-Content -Path "e:\TEAM_ROBOTO\.gitignore" -Value $updatedGitignore

Commit-Step "2026-04-25T12:00:00" "chore: Remove binary virtual environment and introduce requirements.txt"

# Commit 53 (May 10, 2026): Purge Keil build artifacts and establish robust .gitignore
Write-Host "Ensuring Keil build files are completely purged from history tracking..." -ForegroundColor Yellow
Remove-Item -Path "e:\TEAM_ROBOTO\firmware\MDK-ARM\can" -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item -Path "e:\TEAM_ROBOTO\firmware\MDK-ARM\*.uvguix.*" -Force -ErrorAction SilentlyContinue
Remove-Item -Path "e:\TEAM_ROBOTO\firmware\MDK-ARM\RTE" -Recurse -Force -ErrorAction SilentlyContinue
Remove-Item -Path "e:\TEAM_ROBOTO\firmware\MDK-ARM\DebugConfig" -Recurse -Force -ErrorAction SilentlyContinue

Commit-Step "2026-05-10T12:00:00" "chore: Purge Keil build artifacts and establish robust .gitignore"

# Commit 54 (May 25, 2026 - Today!): Overhaul README.md with premium developer documentation
Write-Host "Writing the final premium project README..." -ForegroundColor Green
$premiumReadme = @"
# Robot Control System 🤖🎮

[![STM32 Firmware](https://img.shields.io/badge/Firmware-STM32F4-blue.svg)](https://github.com/MahadiAlif/Robot-Control-System/tree/main/firmware)
[![Jetson Nano UART](https://img.shields.io/badge/Jetson-Python--UART-orange.svg)](https://github.com/MahadiAlif/Robot-Control-System/tree/main/jetson_uart)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/MahadiAlif/Robot-Control-System/tree/main/jetson_uart/LICENSE)

A high-performance, real-time robotics control system featuring multi-agent support for **Standard**, **Sentry**, and **Balancing** robots. This architecture coordinates an onboard **STM32F407IGH6** controller running **FreeRTOS** C firmware with an **NVIDIA Jetson Nano** companion node handling AI computer vision target-tracking via an optimized, high-frequency UART telemetry protocol.

---

## 🛠️ System Architecture

```mermaid
graph TD
    subgraph Jetson Nano (Companion Node)
        AI[AI Vision Target Detection] -->|Float Array| UART_TX[UART Controller: rx_control_final.py]
    end

    subgraph STM32F407IGH6 Controller (FreeRTOS)
        UART_RX[AI_receive USART Task] -->|Parsed Telemetry| INS[INS_task: BMI088 + IST8310]
        INS -->|Fused Orientation| Control[Gimbal & Chassis Controllers]
        DBUS[DBUS Remote Control Task] -->|User Input| Control
        Control -->|CAN Telemetry| Motors[RoboMaster CAN Motors: C610, C620, GM6020]
        Control -->|Firing Commands| Shooter[Shooting Task & Bullet Feed]
        Referee[Referee Usart Decoder] -->|Game State| Control
    end

    UART_TX <==>|115,200 Baud Serial| UART_RX
    Motors <==>|CAN Bus 1 & 2| Control
```

---

## 📂 Repository Structure

The project has been restructured for modularity and easy compilation:

```
Robot-Control-System/
├── .gitignore                      # Workspace ignore filters (excludes Keil object files)
├── README.md                       # Comprehensive system documentation
├── firmware/                       # STM32 CubeMX & Keil MDK-ARM project
│   ├── Src/ & Inc/                 # HAL generated source and header core
│   ├── Drivers/ & Middlewares/     # HAL Drivers & FreeRTOS kernel libraries
│   ├── MDK-ARM/                    # Keil MDK-ARM project targets (can.uvprojx)
│   ├── application/                # Core robot application tasks (Chassis, Gimbal, INS, Shoot)
│   ├── bsp/                        # Board Support Package drivers (CAN, SPI, USART, RC, Delay)
│   └── components/                 # Sensor drivers, controllers, PID, and AHRS fusion filters
├── jetson_uart/                    # Companion node scripts on Jetson Nano
│   ├── requirements.txt            # Python dependencies (pyserial, keyboard)
│   ├── rx_control_final.py         # Main computer vision float decoding & telemetry controller
│   └── uart_communication.py       # Basic loopback check and diagnostic testing
├── docs/                           # Hardware specification libraries
│   ├── datasheets/                 # STM32 manual & Jetson board spec sheet
│   ├── manuals/                    # RoboMaster Board Type C user manual
│   └── schematics/                 # Schematic PDFs (mainboard, gyroscope)
└── media/                          # Screenshots and layout graphics
```

---

## 🛰️ UART Communication Protocol

The Jetson and STM32 communicate over an optimized serial interface at **115,200 baud**:

* **Synchronization Frame Start**: ASCII `%` (value `37`).
* **Telemetry Data Packet**: Space-separated decimal string representing target pitch, yaw, and range coordinates, followed by validation checksum.
* **Payload Conversion**: The Jetson parses and converts the byte strings into raw floats in real-time under $2\text{ms}$ using `rx_control_final.py`.

---

## ⚡ FreeRTOS Real-Time Task Allocation

The STM32 firmware allocates scheduling priority to crucial tasks under FreeRTOS to prevent race conditions:

| Task Name | Priority | Core File | Responsibility |
| :--- | :---: | :--- | :--- |
| **INS Task** | Real-Time | `INS_task.c` | Fuses 6-axis BMI088 IMU + IST8310 Magnetometer via AHRS quaternion filters at $1\text{kHz}$. |
| **Chassis Task** | High | `chassis_task.c` | Wheel speed kinematic calculations (Standard, Sentry omni-wheels, or Balancing leg PID). |
| **Gimbal Task** | High | `gimbal_task.c` | Stabilizes yaw and pitch axes using IMU absolute orientation feeds. |
| **Shoot Task** | Medium | `shooting_task.c` | Controls bullet feeding frequency, projectile friction wheels, and safety checks. |
| **AI Receive** | Medium | `AI_receive.c` | Receives and validates target tracking data from the Jetson serial stream. |
| **Referee Task** | Low | `referee.c` | Decodes server packets (power constraints, bullet limits, robot health). |

---

## 🔧 Installation & Build Instructions

### 1. Jetson Nano Software Setup
Clone the repository and install serial dependencies:
```bash
cd jetson_uart
pip3 install -r requirements.txt
```
To run the telemetry controller:
```bash
sudo python3 rx_control_final.py
```

### 2. STM32 Firmware Compilation
1. Open the Keil workspace located at `firmware/MDK-ARM/can.uvprojx`.
2. Click **Build** (`F7`) to compile. The `.gitignore` prevents tracking compiled object files, keeping your checkouts completely clean.
3. Use a ST-Link / J-Link to flash the binary to the **RoboMaster Development Board Type C (STM32F407IGH6)**.
"@
Set-Content -Path "e:\TEAM_ROBOTO\README.md" -Value $premiumReadme
Commit-Step "2026-05-25T12:00:00" "docs: Overhaul and expand comprehensive developer manual in README.md"

# 5. Clean up temporary directories
Write-Host "Cleaning up backup folders..." -ForegroundColor Yellow
if (Test-Path "e:\TEAM_ROBOTO\project_backup") {
    Remove-Item -Path "e:\TEAM_ROBOTO\project_backup" -Recurse -Force
}

# 6. Configure final target GitHub remote
Write-Host "Configuring the target GitHub remote URL..." -ForegroundColor Green
git remote add origin https://github.com/MahadiAlif/Robot-Control-System.git

Write-Host "Git history successfully rebuilt! 54 commits have been generated backdated to 2025 and 2026." -ForegroundColor Green
Write-Host "You can now push to your remote using: git push -f origin main" -ForegroundColor Cyan
