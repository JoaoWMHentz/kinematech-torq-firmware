# KINEMATECH TORQ ESC - AI Coding Agent Guide

## System Architecture

**Target:** STM32G473CBT6-based ESC with Field-Oriented Control (FOC) for BLDC motors  
**Paradigm:** Bare-metal embedded C (no RTOS) with STM32 HAL  
**Toolchain:** STM32CubeIDE (Eclipse-based) + STM32CubeMX code generator

### Core Components
```
Core/Src/main.c              → Main loop: Hall_ProcessData() + USB telemetry @ 5Hz
Core/Src/hall_sensor.c       → TIM8 Hall Interface driver (ISR + state machine)
Core/Src/usb_communication.c → USB CDC telemetry (CSV format)
Core/Inc/config.h            → Motor params & math constants (6 decimal places)
```

### Hardware Timers
- **TIM1:** (Reserved) FOC PWM generation @ 20kHz - NOT YET IMPLEMENTED
- **TIM8:** Hall Sensor Interface (prescaler=169 → 1μs ticks, XOR mode)
- **TIM2:** SysTick replacement (HAL timebase, priority 15)

### Data Flow: Hall Sensor → Angle Estimation
1. **ISR (TIM8):** `HAL_TIM_IC_CaptureCallback()` → `Hall_TIM_CaptureCallback()` (< 5μs)
   - Reads GPIO state (`GPIOB->IDR` & pins) → physical state (1-7)
   - Captures timer value → period measurement
   - Sets `new_capture_flag = 1`
2. **Main Loop:** `Hall_ProcessData()` if flag set
   - Maps physical state (1→5→4→6→2→3) to logical sector (1→2→3→4→5→6) via `HALL_STATE_TO_SECTOR[]`
   - Validates transition via `HALL_TRANSITION_TABLE` (detects glitches/direction)
   - Calculates velocity (eRPM) from period
   - Updates `angle_electrical` from `HALL_ANGLE_TABLE` (0°, 60°, 120°...)

### Critical Conventions

#### STM32CubeMX Integration
- **NEVER edit outside `/* USER CODE BEGIN */.../* USER CODE END */` blocks** in auto-generated files
- Safe files: `main.c`, `hall_sensor.c/h`, `usb_communication.c/h`, `config.h`
- After `.ioc` regeneration, user code is preserved ONLY inside markers

#### Code Style
```c
// Functions: snake_case
void Hall_ProcessData(HallSensor_t* hall);

// Structs: PascalCase_t
typedef struct { volatile uint8_t hall_state; } HallSensor_t;

// Math constants: 6 decimals (float precision balance)
#define PI_DIV_3  1.047198f  // NOT 1.047197551...

// Comments: Single-line "//" preferred, explain "what" not "how"
hall->new_capture_flag = 0;  // Clear flag (NOT "Set flag to zero")

// ISR-shared vars: ALWAYS volatile
volatile uint8_t new_capture_flag;
```

#### ISR Performance Rules
- Target: < 5μs execution time
- Do: Capture data, set flags, read hardware registers
- Don't: Float math, loops, USB/UART transmit
- Example: `Hall_TIM_CaptureCallback()` → read GPIO + capture counter + flag = ~2μs

## Development Workflows

### Build & Flash (STM32CubeIDE)
1. **Build:** `Project → Build All` (Ctrl+B) → generates `Debug/kinematech-torq-firmware.elf`
2. **Flash/Debug:** Use `kinematech-torq-firmware Debug.launch` (Run → Debug As)
3. **Monitor USB:** `minicom -D /dev/ttyACM0` or equivalent serial terminal

### Modifying Hardware Config
1. Open `kinematech-torq-firmware.ioc` in STM32CubeMX
2. Change peripherals/pins
3. **Generate Code** → overwrites files EXCEPT `/* USER CODE */` sections
4. Verify user sections intact before committing

### Adding New Modules (Pattern)
1. Create `Core/Src/my_module.c` and `Core/Inc/my_module.h`
2. Add global instance in `main.c` `/* USER CODE BEGIN PV */`
3. Initialize in `main()` `/* USER CODE BEGIN 2 */` after HAL_Init()
4. Process in `while(1)` `/* USER CODE BEGIN 3 */`
5. Register ISR callbacks in `/* USER CODE BEGIN 4 */` (see `HAL_TIM_IC_CaptureCallback`)

## Motor Control Specifics

### Hall State Mapping (CRITICAL)
Physical sequence (CW rotation): `1→5→4→6→2→3→1`  
Logical sequence (normalized): `1→2→3→4→5→6→1`  

```c
// Physical → Logical mapping
HALL_STATE_TO_SECTOR[1]=1, [5]=2, [4]=3, [6]=4, [2]=5, [3]=6

// Angle lookup (electrical)
HALL_ANGLE_TABLE[1]=0°, [5]=60°, [4]=120°, [6]=180°, [2]=240°, [3]=300°
```

### Transition Validation
`HALL_TRANSITION_TABLE[prev][next]` returns:
- `+1`: Valid clockwise transition
- `-1`: Valid counter-clockwise
- `0`: Invalid (sensor glitch or wiring error)

Example: From state 1 → next valid states: 5 (CW, returns +1) or 3 (CCW, returns -1)

### Telemetry Format
```
H:1,Ang:1,Vel:1234,ISR:567,Time:890\r\n
  ↑    ↑      ↑       ↑       ↑
  |    |      |       |       └─ Uptime (ms)
  |    |      |       └───────── ISR call counter
  |    |      └───────────────── Velocity (eRPM × 10, int32)
  |    └──────────────────────── Logical sector (1-6)
  └───────────────────────────── Logical sector (duplicate for compatibility)
```

## Future Expansion Checklist

When implementing FOC (TIM1 PWM):
- [ ] Add ISR priority 0 for `TIM1_UP_IRQHandler`
- [ ] Keep FOC loop < 50μs (20kHz = one cycle every 50μs)
- [ ] Use DMA for ADC current sensing (INA240) to avoid blocking
- [ ] Update `config.h` with `FOC_LOOP_FREQUENCY_HZ` validation

When adding USB command parser:
- [ ] Buffer in `usb_communication.c` → parse in main loop (not CDC callback)
- [ ] Commands format: ASCII `CMD:PARAM\n` (e.g., `CAL:START`, `VEL:1500`)
- [ ] Echo response: `OK:CMD` or `ERR:reason`

## Common Pitfalls

1. **Modifying auto-generated code outside markers** → Lost on next `.ioc` regeneration
2. **Forgetting `volatile` on ISR variables** → Compiler optimizations break data flow
3. **Heavy processing in ISRs** → Causes jitter, missed Hall transitions
4. **Using floats in tight loops** → STM32G4 has FPU but still ~10x slower than int math
5. **USB transmit in ISR** → Can block 100μs+, destroying real-time performance

## Quick Reference

**Key Files:**
- Motor constants: `Core/Inc/config.h`
- Hall logic: `Core/Src/hall_sensor.c` (lines 14-38 for tables)
- ISR routing: `Core/Src/main.c` `HAL_TIM_IC_CaptureCallback()`
- Telemetry: `Core/Src/usb_communication.c` `USB_Comm_SendTelemetry()`

**Debug Strategy:**
- Use `USB_Comm_Printf()` sparingly (slow, can drop data)
- Monitor `isr_counter` for Hall transition rate (should match motor RPM × pole pairs)
- Check `HALL_TRANSITION_TABLE` validation failures → wiring issues

**NVIC Priorities (0 = highest):**
- 0: TIM1 (FOC) - reserved
- 2: TIM8 (Hall) - must preempt USB
- 5: USB
- 15: SysTick (TIM2)