# SmoothStepper Mach3 Plugin DLL - Complete Reverse Engineering

**File:** SmoothStepper_v17fe.dll | **Compiler:** MSVC 7.10 | **Arch:** x86 | **Base:** 0x10000000

---

## Table of Contents
1. [Export Table](#1-export-table)
2. [Global Variables](#2-global-variables)
3. [SS Object Layout](#3-ss-object-layout)
4. [Engine Field Access Map](#4-engine-field-access-map)
5. [MainPlanner Field Access Map](#5-mainplanner-field-access-map)
6. [Function Analysis](#6-function-analysis)
   - [InitControl](#61-initcontrol)
   - [Update (SS::Update)](#62-update--ssupdate)
   - [PostInitControl](#63-postinitcontrol)
   - [Home](#64-home)
   - [Probe](#65-probe)
   - [Reset](#66-reset)
   - [Config](#67-config)
   - [Notify](#68-notify)
   - [DoDwell](#69-dodwell)
   - [Purge](#610-purge)
   - [JogOff](#611-jogoff)
7. [SS Command Protocol](#7-ss-command-protocol)
8. [HW Response Packet Format](#8-hw-response-packet-format)
9. [Data Flow](#9-data-flow)
10. [Key Observations](#10-key-observations)

---

## 1. Export Table

25 exported functions from the DLL:

| Export Name       | RVA      | VA           | Notes                          |
|-------------------|----------|--------------|--------------------------------|
| Config            | 0x3FE0   | 0x10003FE0   | Wrapper -> SS->Config          |
| DoDwell           | 0x3F70   | 0x10003F70   | Wrapper -> SS->DoDwell         |
| Home              | 0x40E0   | 0x100040E0   | Wrapper -> SS->Home            |
| InitControl       | 0x3FA0   | 0x10003FA0   | Stores Mach3 pointers          |
| JogOff            | 0x4090   | 0x10004090   | Wrapper -> SS->JogOff          |
| JogOn             | 0x4070   | 0x10004070   | Wrapper -> SS->JogOn           |
| Notify            | 0x3D20   | 0x10003D20   | Wrapper -> SS->Notify          |
| PostInitControl   | 0x3F90   | 0x10003F90   | Wrapper -> SS->PostInitControl |
| Probe             | 0x40D0   | 0x100040D0   | Wrapper -> SS->Probe           |
| Purge             | 0x40B0   | 0x100040B0   | Wrapper -> SS->Purge           |
| Reset             | 0x4060   | 0x10004060   | Wrapper -> SS->Reset           |
| SetCode           | 0x3DA0   | 0x10003DA0   | Stores Code callback ptr       |
| SetDoButton       | 0x3D40   | 0x10003D40   | Stores DoButton callback ptr   |
| SetGetDRO         | 0x3D70   | 0x10003D70   | Stores GetDRO callback ptr     |
| SetGetMenuRange   | 0x3D50   | 0x10003D50   | Stores GetMenuRange ptr        |
| SetSetDRO         | 0x3D60   | 0x10003D60   | Stores SetDRO callback ptr     |
| StopPlug          | 0x3F00   | 0x10003F00   | Wrapper -> SS->StopPlug        |
| Update            | 0x4180   | 0x10004180   | Wrapper -> SS->Update          |

Plus 7 additional internal/utility exports.

---

## 2. Global Variables

| Address      | Name             | Type        | Set By           | Notes                                  |
|-------------|------------------|-------------|------------------|----------------------------------------|
| 0x10078C44  | g_Engine         | Engine*     | InitControl      | Engine pointer from Mach3              |
| 0x10078C40  | g_Setup          | Setup*      | InitControl      | Setup pointer from Mach3              |
| 0x10078C48  | g_View           | View*       | InitControl      | View pointer from Mach3               |
| 0x10078C4C  | g_MainPlanner    | MainPlanner*| InitControl      | MainPlanner pointer from Mach3        |
| 0x10078DEC  | g_SS             | SS*         | PostInitControl  | The SmoothStepper object (heap alloc) |
| 0x10078C7C  | g_UpdateActive   | byte        | Update wrapper   | Set to 1 during Update                |
| 0x10078C7D  | g_TimerCreated   | byte        | Update wrapper   | Timer thread creation flag            |
| 0x10078C50  | g_pfnDoButton    | fn*         | SetDoButton      | DoButton callback pointer             |
| 0x10078C54  | g_pfnGetMenuRange| fn*         | SetGetMenuRange  | GetMenuRange callback pointer         |
| 0x10078C58  | g_pfnSetDRO      | fn*         | SetSetDRO        | SetDRO callback pointer               |
| 0x10078C5C  | g_pfnGetDRO      | fn*         | SetGetDRO        | GetDRO callback pointer               |
| 0x10078C60  | g_pfnCode        | fn*         | SetCode          | Code execution callback pointer       |
| 0x10078C64  | g_InitFlags      | DWORD       | Various          | Bitfield for first-run init stages    |
| 0x10078C68  | g_ConfigDialog   | HWND/ptr    | Config           | Config dialog handle                  |
| 0x10078C70  | g_PluginVersion  | DWORD       | Static           | Version identifier                    |
| 0x10078C74  | g_PluginBuild    | DWORD       | Static           | Build number                          |
| 0x10078C78  | g_DebugFlags     | DWORD       | Various          | Debug/trace flags                     |

---

## 3. SS Object Layout

The SS object is allocated on the heap during PostInitControl. Approximate size: **0x42780 bytes (~272 KB)**.

| Offset        | Size     | Name                    | Description                                      |
|--------------|----------|-------------------------|--------------------------------------------------|
| +0x00000     | 4        | vtable                  | Virtual function table pointer                   |
| +0x00004     | varies   | (config fields)         | Configuration / setup fields                     |
| +0x00068     | 4        | USBDevHandle            | USB device handle                                |
| +0x0006C     | 4        | USBReadPipe             | USB read pipe handle                             |
| +0x00070     | 4        | USBWritePipe            | USB write pipe handle                            |
| +0x20028     | 8x6      | MachinePosSteps[6]      | double - Machine position in steps per axis      |
| +0x20058     | 1x6      | AxisDirtyFlag[6]        | byte - Position update needed flags              |
| +0x20060     | 4        | USBBulkHandle           | Bulk transfer handle for SendToHW                |
| +0x20090     | 4        | OperatingMode           | State machine: 0-4 (see state table)             |
| +0x20094     | 4        | OperatingSubState       | Sub-state within operating mode                  |
| +0x200A0     | varies   | (operating state data)  | State-specific scratch data                      |
| +0x20140     | 4x6      | HWStepPosition[6]       | int32 - Hardware-reported step positions          |
| +0x20158     | 4x6      | EncoderPosition[6]      | int32 - Encoder positions from HW                |
| +0x20170     | 8x6      | AxisVelocity[6]         | double - Current velocity per axis               |
| +0x201A0     | 4        | HWInputBits             | Hardware input port bits                         |
| +0x201A4     | 4        | HWInputBits2            | Additional input bits                            |
| +0x201A8     | 4        | IndexPulseState         | Index pulse capture                              |
| +0x201B0     | 4x6      | LimitSwitchState[6]     | Limit switch states per axis                     |
| +0x201D0     | 4        | ProbeInputState         | Probe input state                                |
| +0x20200     | varies   | (spindle data)          | Spindle control fields                           |
| +0x301B4     | varies   | TxBuffer[~64KB]         | USB transmit buffer                              |
| +0x401B4     | 4        | TxWritePtr              | Current write position in TxBuffer               |
| +0x401B8     | 4        | TxBasePtr               | Base of current packet being built               |
| +0x401BC     | 4        | TxByteCount             | Bytes written in current packet                  |
| +0x40200     | varies   | RxBuffer[~8KB]          | USB receive buffer                               |
| +0x40A00     | 4        | RxBytesAvail            | Bytes available in RxBuffer                      |
| +0x40A04     | 4        | RxReadPtr               | Current read position in RxBuffer                |
| +0x4231C     | 4        | OperatingSubState14     | 14-state sub-state machine                       |
| +0x42320     | 4        | SeqNumber               | Sequence number for USB packets (6-bit)          |
| +0x42324     | 4        | AckedSeqNumber          | Last acknowledged sequence from HW               |
| +0x42330     | 4        | SyncErrorCount          | Count of sync word mismatches                    |
| +0x42340     | 4        | PacketCount             | Total valid packets received                     |
| +0x42350     | 4        | HeartbeatCounter        | Heartbeat timer counter                          |
| +0x42400     | varies   | (homing state)          | Homing-specific state machine data               |
| +0x42500     | varies   | (probe state)           | Probing-specific state data                      |
| +0x42700     | varies   | (jog state)             | Jogging-specific state data                      |
| +0x42760     | 4        | MotionState             | Motion state machine: 0-4                        |
| +0x42764     | 4        | FeedHoldActive          | Feed hold flag                                   |
| +0x42768     | 4        | StatusFlags             | Bitfield: 0x100=StatusCmd, 0x4000=USBErr, etc.   |
| +0x4276C     | 4        | ChargePumpEnabled       | Charge pump enable flag                          |
| +0x42770     | 4        | EStopEnabled            | E-stop circuit enable flag                       |
| +0x42774     | 4        | SpindleRunning          | Spindle active flag                              |
| +0x42778     | 4        | MistOn                  | Mist coolant flag                                |
| +0x4277C     | 4        | FloodOn                 | Flood coolant flag                               |

---

## 4. Engine Field Access Map

sizeof(AxisInfo) = 0x60 = 96 bytes. Axis index `ax` ranges 0..5 (X,Y,Z,A,B,C).

| Engine Offset          | Formula                       | R/W | Description                         | Used In                    |
|-----------------------|-------------------------------|-----|-------------------------------------|----------------------------|
| +0x0000               | Engine+0x00                   | R   | EngineState / Mode                  | Update, Reset              |
| +0x0004               | Engine+0x04                   | R   | EStopState                          | Update (Phase 2)           |
| +0x0008               | Engine+0x08                   | R/W | StatusWord                          | Update, Reset              |
| +0x000C               | Engine+0x0C                   | R   | ControlFlags                        | Update (Phase 3)           |
| +0x0010               | Engine+0x10                   | R   | TrajIndex (write head)              | Update (Phase 9)           |
| +0x0014               | Engine+0x14                   | R/W | TrajHead (read head)                | Update (Phase 9)           |
| +0x001C               | Engine+0x1C                   | R   | FeedHoldRequest                     | Update                     |
| +0x0020               | Engine+0x20                   | R   | FeedRate                            | Update, FeedTraj           |
| +0x0028               | Engine+0x28                   | R   | RapidRate                           | FeedTraj                   |
| +0x0060               | Engine+0x60                   | R   | SpindleDirection                    | Update (Phase 5)           |
| +0x0064               | Engine+0x64                   | R   | SpindleSpeed                        | Update (Phase 5)           |
| +0x0068               | Engine+0x68                   | R   | SpindleState                        | Update                     |
| +0x0118               | Engine+0x118                  | R/W | OutputBits                          | Update                     |
| +0x011C               | Engine+0x11C                  | R/W | InputBits                           | ReadInputs                 |
| +0x0120               | Engine+0x120                  | R/W | InputBits2                          | ReadInputs                 |
| +0x1C0                | Engine+0x1C0                  | R   | MistCoolant                         | Update                     |
| +0x1C4                | Engine+0x1C4                  | R   | FloodCoolant                        | Update                     |
| +0x28050+ax*0x60      | Engine->Axis[ax].StepPos      | W   | Step position feedback              | ProcessDROs_F570           |
| +0x28054+ax*0x60      | Engine->Axis[ax].MachPos      | W   | Machine position (units)            | ProcessDROs_F570           |
| +0x28058+ax*0x60      | Engine->Axis[ax].Enabled      | R   | Axis enabled flag                   | Home, JogOff               |
| +0x2805C+ax*0x60      | Engine->Axis[ax].Homing       | R/W | Homing active flag                  | Home                       |
| +0x28060+ax*0x60      | Engine->Axis[ax].Homed        | W   | Homing complete flag                | Home                       |
| +0x28064+ax*0x60      | Engine->Axis[ax].SoftLimitEn  | R   | Soft limits enabled                 | FeedTraj                   |
| +0x28C4C+ax*0x60      | Engine->Axis[ax].MinLimit     | R   | Minimum soft limit                  | FeedTraj                   |
| +0x28C64+ax*0x60      | Engine->Axis[ax].MaxLimit     | R   | Maximum soft limit                  | FeedTraj                   |
| +0x28068+ax*0x60      | Engine->Axis[ax].Velocity     | R   | Axis velocity setting               | FeedTraj, Home             |
| +0x28070+ax*0x60      | Engine->Axis[ax].Accel        | R   | Axis acceleration                   | Home                       |
| +0x28078+ax*0x60      | Engine->Axis[ax].StepsPerUnit | R   | Steps per unit conversion           | ProcessDROs                |
| +0x2807C+ax*0x60      | Engine->Axis[ax].Backlash     | R   | Backlash compensation               | Update                     |
| +0x28080+ax*0x60      | Engine->Axis[ax].Direction    | R/W | Current direction                   | Home, JogOff               |
| +0x28084+ax*0x60      | Engine->Axis[ax].LimitTripped | R/W | Limit switch tripped flag           | Home, ReadInputs           |
| +0x30000              | Engine+0x30000                | R   | TrajBuffer base                     | ProcessTrajectory          |
| +0x30000+idx*TrajSize | Engine->TrajPt[idx]           | R   | Individual trajectory point         | ProcessTrajectory          |

---

## 5. MainPlanner Field Access Map

| MainPlanner Offset | Formula                            | R/W | Description                    | Used In                    |
|-------------------|------------------------------------|-----|--------------------------------|----------------------------|
| +0x00000          | MainPlanner+0x00                   | R   | PlannerState                   | Update                     |
| +0x00004          | MainPlanner+0x04                   | R/W | FeedHoldState                  | Update (Phase 10)          |
| +0x00008          | MainPlanner+0x08                   | R   | QueueDepth                     | Update                     |
| +0x00010          | MainPlanner+0x10                   | R   | MotionMode                     | Update, FeedTraj           |
| +0x8F498+ax*8    | MainPlanner->StepsPerAxis[ax]      | R   | Steps per axis                 | FeedTraj                   |
| +0x8F4D0+ax*8    | MainPlanner->AxisMaxVel[ax]        | R   | Maximum velocity per axis      | FeedTraj                   |
| +0x8F508+ax*8    | MainPlanner->AxisAccel[ax]         | R   | Acceleration per axis          | Home, FeedTraj             |
| +0x11DDD8+ax*8   | MainPlanner->Velocity[ax]          | R   | Current velocity command       | FeedTraj                   |
| +0x11DE10+ax*8   | MainPlanner->Position[ax]          | R   | Current position command       | FeedTraj                   |
| +0x11DE48         | MainPlanner+0x11DE48               | R   | FeedOverride                   | Update                     |
| +0x11DE50         | MainPlanner+0x11DE50               | R   | SpindleOverride                | Update                     |
| +0x11DE58         | MainPlanner+0x11DE58               | R   | RapidOverride                  | Update                     |
| +0x11DE60         | MainPlanner+0x11DE60               | R/W | PositionChanged flag           | Update (Phase 10)          |
| +0x11DE68+ax*8   | MainPlanner->TargetPos[ax]         | R   | Target position                | FeedTraj                   |
| +0x11DEA0         | MainPlanner+0x11DEA0               | R   | ExactStopMode                  | Update                     |
---

## 6. Function Analysis

### 6.1 InitControl

**Export:** `InitControl` @ 0x10003FA0
**Signature:** `void __cdecl InitControl(Engine* eng, Setup* setup, MainPlanner* mp, View* view)`

```c
void InitControl(Engine* eng, Setup* setup, MainPlanner* mp, View* view) {
    g_Engine      = eng;       // [10078C44]
    g_Setup       = setup;     // [10078C40]
    g_View        = view;      // [10078C48]
    g_MainPlanner = mp;        // [10078C4C]
    g_TimerCreated = 0;        // [10078C7D]
    g_UpdateActive = 0;        // [10078C7C]
}
```

**Purpose:** Called by Mach3 during plugin load. Stores the four core Mach3 object pointers into global variables. No SS object exists yet.

---

### 6.2 Update / SS::Update

**Export wrapper:** `Update` @ 0x10004180
**Real implementation:** `SS::Update` @ 0x10010C90 (thiscall, ~1451 lines)

#### Update Wrapper

```c
void __cdecl Update() {
    g_UpdateActive = 1;                    // [10078C7C]

    if (!g_TimerCreated) {                 // [10078C7D]
        g_TimerCreated = 1;
        CreateTimerThread(25);             // 25ms = 40Hz timer
    }

    if (g_SS != NULL) {                    // [10078DEC]
        g_SS->Update();                    // thiscall to 0x10010C90
    }

    g_UpdateActive = 0;
}
```

#### SS::Update - Phase-by-Phase Pseudocode

**Phase 1: First-run initialization**
```c
void SS::Update() {
    // Phase 1 - One-time init on first few calls
    if (!(g_InitFlags & 0x01)) {
        g_InitFlags |= 0x01;
        // Initialize axis mapping, load config from registry
    }
    if (!(g_InitFlags & 0x02)) {
        g_InitFlags |= 0x02;
        // Initialize USB communication
    }
```

**Phase 2: E-stop state change detection**
```c
    // Phase 2 - E-stop transition handling
    int currentEStop = Engine->EStopState;           // Engine+0x04
    if (currentEStop != this->prevEStopState) {
        this->prevEStopState = currentEStop;
        if (currentEStop) {
            for (int ax = 0; ax < 6; ax++)
                this->savedStepPos[ax] = this->HWStepPosition[ax];
        } else {
            for (int ax = 0; ax < 6; ax++)
                this->MachinePosSteps[ax] = (double)this->HWStepPosition[ax];
        }
    }
```

**Phase 3: Control flags - charge pump and E-stop enable**
```c
    // Phase 3 - Control outputs
    int controlFlags = Engine->ControlFlags;          // Engine+0x0C
    if (controlFlags & CTRL_CHARGE_PUMP) {
        if (!this->ChargePumpEnabled) {
            this->ChargePumpEnabled = 1;
            BuildPacketCmd(this, CMD_CHARGE_PUMP_ON, 0x01);
        }
    } else {
        if (this->ChargePumpEnabled) {
            this->ChargePumpEnabled = 0;
            BuildPacketCmd(this, CMD_CHARGE_PUMP_OFF, 0x00);
        }
    }
    if (controlFlags & CTRL_ESTOP_ENABLE) {
        if (!this->EStopEnabled) {
            this->EStopEnabled = 1;
            BuildPacketCmd(this, CMD_ESTOP_ENABLE, 0x01);
        }
    }
```

**Phase 4: Read hardware inputs**
```c
    // Phase 4 - USB Read and Parse
    ReadInputs(this);                                 // 0x10012F20
    // Reads USB bulk data, searches for 0x5599AA66 sync word
    // Validates checksum, extracts step positions, encoders,
    // input bits, limit switches, probe state, acked seq number
```

**Phase 5: Spindle control**
```c
    // Phase 5 - Spindle
    int spindleDir = Engine->SpindleDirection;        // Engine+0x60
    int spindleSpd = Engine->SpindleSpeed;            // Engine+0x64
    int spindleSt  = Engine->SpindleState;            // Engine+0x68
    if (spindleSt != this->prevSpindleState ||
        spindleSpd != this->prevSpindleSpeed) {
        this->prevSpindleState = spindleSt;
        this->prevSpindleSpeed = spindleSpd;
        if (spindleSt) {
            BuildPacketCmd(this, CMD_SPINDLE_SPEED, spindleSpd & 0xFF);
            BuildPacketCmd(this, CMD_SPINDLE_DIR, spindleDir);
            BuildPacketCmd(this, CMD_SPINDLE_ON, 0x01);
        } else {
            BuildPacketCmd(this, CMD_SPINDLE_OFF, 0x00);
        }
    }
```

**Phase 6: Operating Mode state machine (5 states)**
```c
    // Phase 6 - OperatingMode at SS+0x20090
    switch (this->OperatingMode) {
        case 0: // IDLE
            if (Engine->TrajIndex != Engine->TrajHead)
                this->OperatingMode = 1;
            break;
        case 1: // RUNNING
            ProcessTrajectory(this);       // 0x10012E40
            break;
        case 2: // HOMING
            break;
        case 3: // PROBING
            break;
        case 4: // ERROR
            break;
    }
```

**Phase 7-8: USB error and status commands**
```c
    if (this->StatusFlags & 0x4000)
        this->StatusFlags &= ~0x4000;
    if (this->StatusFlags & 0x100) {
        BuildPacketCmd(this, CMD_STATUS_QUERY, 0x00);
        this->StatusFlags &= ~0x100;
    }
```

**Phase 9: Trajectory feeding - the core motion pipeline**
```c
    // Phase 9 - Feed trajectory data to hardware
    int trajIndex = Engine->TrajIndex;                // Engine+0x10
    int trajHead  = Engine->TrajHead;                 // Engine+0x14
    if (trajIndex != trajHead) {
        if (!this->FeedHoldActive) {
            int pending = (this->SeqNumber - this->AckedSeqNumber) & 0x3F;
            if (pending < MAX_PENDING_PACKETS) {
                FeedTraj(this);                       // 0x10014650
                Engine->TrajHead = (trajHead + 1) & TRAJ_MASK;
            }
        }
    }
```

**Phase 10: MainPlanner feed hold and position change**
```c
    int mpFeedHold = MainPlanner->FeedHoldState;      // MainPlanner+0x04
    if (mpFeedHold != this->prevFeedHoldState) {
        this->prevFeedHoldState = mpFeedHold;
        this->FeedHoldActive = mpFeedHold;
        BuildPacketCmd(this, CMD_FEED_HOLD, mpFeedHold ? 0x01 : 0x00);
    }
    if (MainPlanner->PositionChanged) {               // MainPlanner+0x11DE60
        MainPlanner->PositionChanged = 0;
        ProcessDROs_F570(this);
        ProcessDROs_F2B0(this);
    }
```

**Phase 11: Output bits and coolant**
```c
    int outputBits = Engine->OutputBits;              // Engine+0x118
    if (outputBits != this->prevOutputBits) {
        this->prevOutputBits = outputBits;
        BuildPacketCmd(this, CMD_OUTPUT_BITS, outputBits & 0xFF);
        BuildPacketCmd(this, CMD_OUTPUT_BITS_HI, (outputBits >> 8) & 0xFF);
    }
    int mist = Engine->MistCoolant;                   // Engine+0x1C0
    int flood = Engine->FloodCoolant;                 // Engine+0x1C4
    if (mist != this->MistOn) {
        this->MistOn = mist;
        BuildPacketCmd(this, CMD_MIST, mist ? 1 : 0);
    }
    if (flood != this->FloodOn) {
        this->FloodOn = flood;
        BuildPacketCmd(this, CMD_FLOOD, flood ? 1 : 0);
    }
```

**Phase 12: Motion State machine (5 states)**
```c
    // Phase 12 - MotionState at SS+0x42760
    switch (this->MotionState) {
        case 0: // IDLE
            ProcessDROs_F570(this); ProcessDROs_F2B0(this); break;
        case 1: // MOVING
            ProcessDROs_F570(this); ProcessDROs_F3B0(this); break;
        case 2: // DECEL
            ProcessDROs_F570(this); ProcessDROs_F7D0(this); break;
        case 3: // STOPPED
            ProcessDROs_F570(this); ProcessDROs_F2B0(this);
            this->MotionState = 0; break;
        case 4: // ERROR
            break;
    }
```

**Phase 13: Housekeeping**
```c
    this->HeartbeatCounter++;
    if (this->HeartbeatCounter >= HEARTBEAT_INTERVAL) {
        this->HeartbeatCounter = 0;
        BuildPacketCmd(this, 0x80, 0x00);  // Heartbeat / NOP
    }
    this->SeqNumber = (this->SeqNumber + 1) & 0x3F;
    SendToHW(this);                                   // 0x10014110
}
```

---

### 6.3 PostInitControl

**Export wrapper:** `PostInitControl` @ 0x10003F90
**Real implementation:** SS::PostInitControl @ 0x100061D0 (801 lines)

```c
void SS::PostInitControl() {
    // Allocate the SS object (~0x42780 bytes)
    SS* ss = new SS();
    g_SS = ss;                                        // [10078DEC]

    // Zero-initialize the object
    memset(ss, 0, sizeof(SS));

    // Read configuration from Windows registry
    // Key: HKCU\Software\SmoothStepper or similar
    // Values: motor tuning, pin mappings, USB device, kernel freq, charge pump

    // Open USB device
    // Enumerate FTDI/USB devices, find SmoothStepper by VID/PID
    ss->USBDevHandle = OpenDevice(...);
    ss->USBReadPipe  = OpenPipe(READ);
    ss->USBWritePipe = OpenPipe(WRITE);

    // Initialize FPGA - send initial configuration commands
    // Set kernel frequency, configure step/dir pins, charge pump

    // Read motor tuning from Engine for each axis
    for (int ax = 0; ax < 6; ax++) {
        double stepsPerUnit = Engine->Axis[ax].StepsPerUnit;  // +0x28078
        double velocity     = Engine->Axis[ax].Velocity;      // +0x28068
        double accel        = Engine->Axis[ax].Accel;          // +0x28070
        // Convert to FPGA timing, send axis config
        BuildPacketCmd(ss, CMD_AXIS_CONFIG, ...);
    }

    // Cache MainPlanner steps-per-axis for FeedTraj
    for (int ax = 0; ax < 6; ax++) {
        double spa = MainPlanner->StepsPerAxis[ax];   // +0x8F498
    }

    // Set initial state
    ss->OperatingMode = 0;   // IDLE
    ss->MotionState = 0;     // IDLE
    ss->SeqNumber = 0;

    // Send initial status query
    BuildPacketCmd(ss, CMD_STATUS_QUERY, 0);
    SendToHW(ss);
}
```

**Key observations:**
- This is where the SS object is heap-allocated and stored in g_SS
- USB device discovery and connection happens here
- FPGA is configured with motor tuning from Engine->Axis[] fields
- No GetDRO/SetDRO/DoButton calls

---

### 6.4 Home

**Export wrapper:** `Home` @ 0x100040E0
**Real implementation:** SS::Home @ 0x10009B80 (744 lines)
**Signature:** `void __thiscall SS::Home(short axis)`

```c
void SS::Home(short axis) {
    if (axis < 0 || axis >= 6) return;
    if (!Engine->Axis[axis].Enabled) return;          // +0x28058+ax*0x60

    Engine->Axis[axis].Homing = 1;                    // +0x2805C+ax*0x60

    // Read homing parameters
    double velocity     = Engine->Axis[axis].Velocity;     // +0x28068
    double accel        = Engine->Axis[axis].Accel;        // +0x28070
    double stepsPerUnit = Engine->Axis[axis].StepsPerUnit; // +0x28078

    // Calculate homing speed (25% of max)
    double homeSpeed = velocity * stepsPerUnit * 0.25;

    // Set operating mode to HOMING
    this->OperatingMode = 2;

    // Homing state machine (within SS+0x42400 area):
    // State 0: Fast approach - move toward home switch at 25% speed
    //          Monitor limit switch via HWInputBits
    //          On limit trigger -> State 1
    // State 1: Back off - reverse slowly at ~5% speed
    //          On limit release -> State 2
    // State 2: Slow approach - creep at ~2% speed
    //          On limit trigger -> State 3
    // State 3: Set home position
    //          Zero axis, set Homed=1, Homing=0
    //          OperatingMode = 0 (IDLE)

    BuildPacketCmd(this, CMD_HOME_START, axis);

    // On completion:
    // Engine->Axis[axis].Homed = 1;             // +0x28060
    // Engine->Axis[axis].Homing = 0;            // +0x2805C
    // this->MachinePosSteps[axis] = 0.0;
    // this->HWStepPosition[axis] = 0;
    // Engine->Axis[axis].StepPos = 0;           // +0x28050
}
```

**Key observations:**
- 744 lines = complex multi-phase homing state machine
- Three-phase approach: fast approach, back-off, slow approach
- Limit switch state from hardware via ReadInputs/USB packets
- Positions zeroed on completion

---

### 6.5 Probe

**Export wrapper:** `Probe` @ 0x100040D0
**Real implementation:** SS::Probe @ 0x1000A7E0 (228 lines)

```c
void SS::Probe() {
    this->OperatingMode = 3;

    // Probe state machine (SS+0x42500 area):
    // State 0: Begin probe move - slow feed in probe direction
    // State 1: Moving - monitor ProbeInputState from HW each Update
    //          On trigger: record position, stop motion -> State 2
    //          On move complete without trigger -> State 3
    // State 2: Probe triggered
    //          Store trip position in Engine probe result fields
    //          Engine->ProbeTripped = 1
    //          OperatingMode = 0
    // State 3: Probe failed
    //          Engine->ProbeTripped = 0
    //          OperatingMode = 0

    BuildPacketCmd(this, CMD_PROBE_START, 0);
    // Probe input at SS+0x201D0, updated by ReadInputs()
}
```

---

### 6.6 Reset

**Export wrapper:** `Reset` @ 0x10004060
**Real implementation:** SS::Reset @ 0x10008B00 (283 lines)

```c
void SS::Reset() {
    // Stop all motion immediately
    BuildPacketCmd(this, CMD_STOP_ALL, 0x00);
    SendToHW(this);

    // Reset state machines
    this->OperatingMode = 0;
    this->MotionState = 0;
    this->FeedHoldActive = 0;
    this->StatusFlags = 0;
    this->SeqNumber = 0;
    this->AckedSeqNumber = 0;

    // Reset TX buffer
    this->TxWritePtr = &this->TxBuffer[0];
    this->TxByteCount = 0;

    // Synchronize positions from hardware
    for (int ax = 0; ax < 6; ax++) {
        this->MachinePosSteps[ax] = (double)this->HWStepPosition[ax];
        double pos = this->MachinePosSteps[ax];
        double spu = Engine->Axis[ax].StepsPerUnit;
        Engine->Axis[ax].StepPos = (int)pos;
        if (spu != 0.0)
            Engine->Axis[ax].MachPos = pos / spu;
    }

    // Cancel active homing
    for (int ax = 0; ax < 6; ax++)
        Engine->Axis[ax].Homing = 0;

    // Reset peripherals
    this->ChargePumpEnabled = 0;
    this->SpindleRunning = 0;
    BuildPacketCmd(this, CMD_SPINDLE_OFF, 0);
    BuildPacketCmd(this, CMD_RESET, 0x00);
    SendToHW(this);

    this->SyncErrorCount = 0;
    Engine->StatusWord &= ~STATUS_MOVING;
}
```

---

### 6.7 Config

**Export wrapper:** `Config` @ 0x10003FE0
**Real implementation:** Dialog-based, creates a Win32 dialog

```c
void SS::Config() {
    // Creates a modal Win32 dialog (DialogBoxParam)
    // Dialog resource embedded in DLL resources
    // Handles:
    //   - Axis configuration (steps/unit, velocity, acceleration)
    //   - Pin mapping (step, dir, enable per axis)
    //   - Input pin assignments (limits, home, probe, E-stop)
    //   - USB device selection
    //   - Kernel frequency (25kHz, 50kHz, 100kHz)
    //   - Charge pump settings
    //   - Encoder configuration
    //   - Spindle PWM settings
    // On OK: save to registry, update SS config, send new config to FPGA
    // On Cancel: no changes
}
```

---

### 6.8 Notify

**Export wrapper:** `Notify` @ 0x10003D20
**Real implementation:** SS::Notify @ 0x10007E90 (501 lines)
**Signature:** `void __thiscall SS::Notify(int msg)`

```c
void SS::Notify(int msg) {
    switch (msg) {
        case 0:  // Plugin loaded
            break;
        case 1:  // Configuration changed
            // Re-read motor tuning from Engine, update FPGA
            for (int ax = 0; ax < 6; ax++) {
                // Re-read Engine->Axis[ax] parameters
                // Recalculate timing, send to FPGA
            }
            break;
        case 2:  // Units changed (inch/mm)
            break;
        case 3:  // E-stop state changed
            if (Engine->EStopState) {
                BuildPacketCmd(this, CMD_ESTOP_ASSERT, 0x01);
                SendToHW(this);
            } else {
                BuildPacketCmd(this, CMD_ESTOP_RELEASE, 0x00);
                SendToHW(this);
            }
            break;
        case 4:  // Axis parameters changed
            break;
        case 5:  // DRO update request
            ProcessDROs_F570(this);
            ProcessDROs_F2B0(this);
            break;
        case 6:  // G-code mode change
            break;
        // Additional cases up to ~15
    }
}
```

**Key observations:**
- 501 lines, large switch/case
- Handles immediate E-stop notification (supplementing Update polling)
- Can trigger immediate USB sends

---

### 6.9 DoDwell

**Export wrapper:** `DoDwell` @ 0x10003F70
**Real implementation:** SS::DoDwell @ 0x10008AB0 (18 lines)
**Signature:** `void __thiscall SS::DoDwell(double seconds)`

```c
void SS::DoDwell(double seconds) {
    int ms = (int)(seconds * 1000.0);
    BuildPacketCmd(this, CMD_DWELL_LO, ms & 0xFF);
    BuildPacketCmd(this, CMD_DWELL_HI, (ms >> 8) & 0xFF);
    // FPGA handles timing internally - no busy-wait
}
```

**Key observations:**
- Only 18 lines of assembly - extremely simple
- Delegates timing entirely to FPGA
- Does NOT block the Update loop

---

### 6.10 Purge

**Export wrapper:** `Purge` @ 0x100040B0
**Real implementation:** SS::Purge @ 0x1000AC20 (62 lines)
**Signature:** `void __thiscall SS::Purge(short flags)`

```c
void SS::Purge(short flags) {
    BuildPacketCmd(this, CMD_STOP_ALL, 0x00);
    SendToHW(this);

    // Reset TX buffer
    this->TxWritePtr = &this->TxBuffer[0];
    this->TxByteCount = 0;

    // Drain trajectory queue if requested
    if (flags & 0x01)
        Engine->TrajHead = Engine->TrajIndex;

    this->MotionState = 0;

    // Position sync if requested
    if (flags & 0x02) {
        ProcessDROs_F570(this);
        ProcessDROs_F2B0(this);
    }

    BuildPacketCmd(this, CMD_PURGE, flags & 0xFF);
    SendToHW(this);
}
```

---

### 6.11 JogOff

**Export wrapper:** `JogOff` @ 0x10004090
**Real implementation:** SS::JogOff @ 0x10009720 (137 lines)
**Signature:** `void __thiscall SS::JogOff(short axis)`

```c
void SS::JogOff(short axis) {
    if (axis < 0 || axis >= 6) return;
    if (!Engine->Axis[axis].Enabled) return;          // +0x28058

    double currentVel   = this->AxisVelocity[axis];   // SS+0x20170+ax*8
    double accel        = Engine->Axis[axis].Accel;    // +0x28070
    double stepsPerUnit = Engine->Axis[axis].StepsPerUnit; // +0x28078

    if (currentVel == 0.0) return;  // Already stopped

    // Calculate deceleration distance: d = v^2 / (2*a)
    double decelSteps = (currentVel * currentVel) / (2.0 * accel * stepsPerUnit);

    // Command FPGA to decelerate this axis
    BuildPacketCmd(this, CMD_JOG_STOP, axis);

    // Clear jog velocity
    this->AxisVelocity[axis] = 0.0;
    // Actual decel executed by FPGA, position via ReadInputs/ProcessDROs
}
```

**Key observations:**
- Smooth deceleration, not abrupt stop
- Delegates decel profile to FPGA
- 137 lines includes parameter validation and encoding

---

## 7. SS Command Protocol

Commands are encoded as 2-byte pairs via `BuildPacketCmd` (0x10014330): `[cmdId][param]`

| Cmd ID  | Name               | Param           | Description                        |
|---------|--------------------|-----------------|------------------------------------|
| 0x10    | CHARGE_PUMP_ON     | 0x01            | Enable charge pump output          |
| 0x11    | CHARGE_PUMP_OFF    | 0x00            | Disable charge pump output         |
| 0x12    | ESTOP_ENABLE       | 0x01            | Enable E-stop circuit              |
| 0x13    | ESTOP_DISABLE      | 0x00            | Disable E-stop circuit             |
| 0x14    | ESTOP_ASSERT       | 0x01            | Assert E-stop to FPGA              |
| 0x15    | ESTOP_RELEASE      | 0x00            | Release E-stop                     |
| 0x20    | SPINDLE_ON         | speed_lo        | Turn spindle on                    |
| 0x21    | SPINDLE_OFF        | 0x00            | Turn spindle off                   |
| 0x22    | SPINDLE_SPEED      | speed_byte      | Set spindle speed                  |
| 0x23    | SPINDLE_DIR        | 0/1             | Set spindle direction              |
| 0x30    | STATUS_QUERY       | 0x00            | Request status packet from HW      |
| 0x40    | OUTPUT_BITS        | bits_lo         | Set output bits (low byte)         |
| 0x41    | OUTPUT_BITS_HI     | bits_hi         | Set output bits (high byte)        |
| 0x50    | STOP_ALL           | 0x00            | Emergency stop all axes            |
| 0x51    | FEED_HOLD          | 0/1             | Feed hold on/off                   |
| 0x52    | PURGE              | flags           | Purge motion buffers               |
| 0x60    | HOME_START         | axis            | Begin homing sequence              |
| 0x61    | PROBE_START        | 0x00            | Begin probing                      |
| 0x62    | JOG_STOP           | axis            | Stop jog on axis                   |
| 0x70    | AXIS_CONFIG        | axis            | Configure axis (followed by data)  |
| 0x71    | DWELL_LO           | ms_lo           | Dwell low byte                     |
| 0x72    | DWELL_HI           | ms_hi           | Dwell high byte                    |
| 0x80    | NOP/HEARTBEAT      | 0x00            | Heartbeat / padding                |
| 0xA5    | ACCUM_STEPS        | (complex)       | Absolute position command          |
| 0xFE    | RESET              | 0x00            | FPGA reset command                 |

### Step Delta Encoding (FeedTraj)

Trajectory step deltas are encoded in 6-bit groups for compact transmission:

```
For each axis (0-5):
  velocity_steps = MainPlanner->Velocity[axis] * MainPlanner->StepsPerAxis[axis]
  delta = ftol(velocity_steps)

Deltas are packed into bytes:
  - Each axis delta is split into 6-bit groups
  - Groups are OR'd together with axis identification
  - Packed sequentially into the TX buffer
```

### AccumSteps Encoding (0x10014BF0)

Absolute position command with 6-bit group encoding:

```
Byte 0: 0xA5 (header)
For groups 5 down to 0 (3 bytes each):
  shift = group * 6
  byte = (position >> shift) & 0x3F
  Write byte to TX buffer
Final: axis index byte
Total: 1 + 18 + 1 = 20 bytes per position command
```

---

## 8. HW Response Packet Format

Hardware response packets are 0x7C (124) bytes, read by `ReadInputs` (0x10012F20).

```
Offset  Size  Description
------  ----  -----------
0x00    4     Sync word: 0x5599AA66 (little-endian)
0x04    88    Payload data:
  0x04  4*6   Step positions per axis (int32 * 6 = 24 bytes)
  0x1C  4*6   Encoder positions per axis (int32 * 6 = 24 bytes)
  0x34  4     Input bits (port 1)
  0x38  4     Input bits (port 2)
  0x3C  4     Limit switch states (bitfield)
  0x40  4     Probe input state
  0x44  4     Index pulse state
  0x48  4     Acknowledged sequence number (& 0x3F)
  0x4C  4     FPGA status flags
  0x50  4     Spindle feedback / RPM
  0x54  4     Error code
  0x58  4     Reserved
0x5C    28    (additional fields / padding)
0x78    2     Checksum (sum of bytes 0x04..0x5B)
0x7A    2     Inverted checksum (~checksum[0x78])
```

### Validation (from ReadInputs):
```c
// Search for sync word
if (*(uint32_t*)ptr != 0x5599AA66)
    continue;  // skip 1 byte, try again

// Compute checksum
uint16_t sum = 0;
for (int i = 4; i < 0x5C; i++)
    sum += packet[i];

// Validate
if (packet_u16[0x78/2] == sum && packet_u16[0x7A/2] == (uint16_t)~sum) {
    ParseHWPacket(this, packet);  // 0x10013560
    this->PacketCount++;
} else {
    this->SyncErrorCount++;
}
```

---

## 9. Data Flow

### Complete trajectory-to-position pipeline:

```
Mach3 MainPlanner
    |
    | Writes TrajPoints to Engine->TrajBuffer
    | Increments Engine->TrajIndex
    |
    v
SS::Update() - Phase 9 (Trajectory Feed)
    |
    | Reads Engine->TrajIndex vs Engine->TrajHead
    | Checks sequence number flow control (pending < MAX)
    |
    v
SS::ProcessTrajectory() [0x10012E40]
    |
    | Reads TrajPoint from Engine->TrajBuffer[TrajHead]
    | Extracts per-axis velocity/position data
    |
    v
SS::FeedTraj() [0x10014650]
    |
    | For each axis 0-5:
    |   steps = MainPlanner->Velocity[ax] * MainPlanner->StepsPerAxis[ax]
    |   Reads Engine->Axis[ax].MinLimit / MaxLimit for bounds
    | Encodes step deltas in 6-bit groups
    | Appends to TX buffer
    |
    v
SS::BuildPacketCmd() [0x10014330]
    |
    | Appends 2-byte command pairs to TX buffer
    | Multiple commands accumulated per Update cycle
    |
    v
SS::SendToHW() [0x10014110]
    |
    | Pads TX buffer to 64-byte alignment (0x80 NOP fill)
    | USB bulk write to FPGA
    |
    v
[USB Bus -> FPGA Hardware]
    |
    | FPGA executes step pulses
    | FPGA counts encoder feedback
    | FPGA monitors inputs
    |
    v
[FPGA -> USB Bus -> PC]
    |
    v
SS::ReadInputs() [0x10012F20]
    |
    | USB bulk read
    | Search for 0x5599AA66 sync word
    | Validate checksum (sum + inverted sum)
    | Parse into:
    |   HWStepPosition[6]    (SS+0x20140)
    |   EncoderPosition[6]   (SS+0x20158)
    |   HWInputBits          (SS+0x201A0)
    |   LimitSwitchState[6]  (SS+0x201B0)
    |   ProbeInputState      (SS+0x201D0)
    |   AckedSeqNumber       (SS+0x42324)
    |
    v
SS::ProcessDROs_F570() [0x1000F570]
    |
    | For each axis with dirty flag:
    |   delta = HWStepPosition[ax] - MachinePosSteps[ax]
    |   MachinePosSteps[ax] = HWStepPosition[ax]  (update)
    |   Write to Engine->Axis[ax].StepPos  (+0x28050)
    |
    v
SS::ProcessDROs_F2B0() [0x1000F2B0]
    |
    | Convert step positions to machine units
    | Engine->Axis[ax].MachPos = StepPos / StepsPerUnit
    |
    v
SS::ProcessDROs_F3B0() [0x1000F3B0]   (during active motion)
    |
    | Update velocity-related DRO fields
    |
    v
SS::ProcessDROs_F7D0() [0x1000F7D0]   (during deceleration, 301 lines)
    |
    | Complex position processing during deceleration
    | Handles overshoot compensation
    | Final position settling
    |
    v
Engine->Axis[ax].StepPos / MachPos
    |
    | Mach3 reads these to update DRO displays
    | and for G-code position tracking
    v
[Mach3 UI / DRO Displays]
```

### Sequence Number Flow Control:

```
PC Side:                    FPGA Side:
---------                   ----------
SeqNumber = 0               AckedSeq = 0

Send packet (seq=0)  -----> Receive, execute
SeqNumber = 1               Send ack (seq=0) -----> AckedSeqNumber = 0

Send packet (seq=1)  -----> Receive, execute
SeqNumber = 2               Send ack (seq=1) -----> AckedSeqNumber = 1

...

Pending = (SeqNumber - AckedSeqNumber) & 0x3F
If pending >= MAX_PENDING: stall, do not send more
If pending < MAX_PENDING: safe to send next batch

Sequence wraps at 6 bits: & 0x3F (0-63)
```

---

## 10. Key Observations

### 1. No GetDRO/SetDRO/DoButton calls in motion-critical paths
The SmoothStepper plugin does **NOT** call GetDRO, SetDRO, or DoButton during normal operation. While these callback pointers are stored (via SetGetDRO, SetSetDRO, SetDoButton exports), the motion control path operates entirely through direct Engine and MainPlanner field access. It communicates with Mach3 exclusively through shared memory structures, not through the callback API.

### 2. No Engine->Axis[].MaxVelocity writes
The plugin **reads** velocity and acceleration from Engine->Axis[] but does **not write** MaxVelocity. Velocity limits are enforced by the FPGA firmware, not by the plugin. The plugin passes through velocity commands from MainPlanner->Velocity[] to the FPGA without modification.

### 3. Ring buffer mechanism
The trajectory ring buffer uses Engine->TrajIndex (write head, set by Mach3/MainPlanner) and Engine->TrajHead (read head, advanced by the plugin). The plugin consumes points when TrajIndex != TrajHead, and flow control is managed by the 6-bit sequence number handshake with the FPGA.

### 4. 6-bit step delta encoding
Step deltas are encoded in 6-bit groups for bandwidth-efficient USB transmission. This allows up to 63 steps per group, with multiple groups per axis per packet. The encoding is compact enough to maintain real-time trajectory streaming over USB 2.0 bulk transfers.

### 5. 64-byte USB packet alignment
All USB transmissions are padded to 64-byte boundaries using 0x80 (NOP/heartbeat) bytes. This aligns with USB bulk transfer packet sizes for optimal throughput.

### 6. FPGA-delegated timing
Time-critical operations (step pulse generation, dwell timing, deceleration profiles) are handled by the FPGA, not the PC. The plugin operates as a trajectory streamer and status monitor, not a real-time step generator.

### 7. Dual checksum validation
Hardware response packets use a dual checksum: a 16-bit sum of the payload, plus the bitwise inverse of that sum. Both must validate for a packet to be accepted. Invalid packets increment SyncErrorCount and the parser attempts to resync by scanning for the next 0x5599AA66 sync word.

### 8. Position feedback path
Position updates flow: FPGA step counters -> USB packet -> HWStepPosition[] -> MachinePosSteps[] -> Engine->Axis[].StepPos -> Engine->Axis[].MachPos. The dirty flag mechanism (SS+0x20058+axis) ensures positions are only updated when new data arrives from the FPGA, preventing stale reads.

### 9. E-stop snapshot/restore
On E-stop entry, the plugin snapshots all axis positions. On E-stop exit, it restores positions from HWStepPosition (which reflects the actual stopped position). This prevents position loss across E-stop cycles.

### 10. Timer thread at 25ms (40Hz)
The Update wrapper creates a timer thread with a 25ms period (40Hz). Combined with the Mach3 Update call frequency, the effective update rate is approximately 25-50Hz. This is sufficient for trajectory streaming since the FPGA handles the microsecond-level step timing.

---

## Appendix A: Sub-function Address Table

| Address      | Name                   | Lines | Called From              |
|-------------|------------------------|-------|--------------------------|
| 0x10010C90  | SS::Update             | 1451  | Update wrapper           |
| 0x100061D0  | SS::PostInitControl    | 801   | PostInitControl wrapper  |
| 0x10007E90  | SS::Notify             | 501   | Notify wrapper           |
| 0x10008AB0  | SS::DoDwell            | 18    | DoDwell wrapper          |
| 0x10008B00  | SS::Reset              | 283   | Reset wrapper            |
| 0x10009720  | SS::JogOff             | 137   | JogOff wrapper           |
| 0x10009B80  | SS::Home               | 744   | Home wrapper             |
| 0x1000A7E0  | SS::Probe              | 228   | Probe wrapper            |
| 0x1000AC20  | SS::Purge              | 62    | Purge wrapper            |
| 0x10012E40  | SS::ProcessTrajectory  | 66    | Update (Phase 6/9)       |
| 0x10012F20  | SS::ReadInputs         | 401   | Update (Phase 4)         |
| 0x10013560  | SS::ParseHWPacket      | ~200  | ReadInputs               |
| 0x10012160  | SS::ConvertAndSendTraj | ~150  | ProcessTrajectory        |
| 0x10014110  | SS::SendToHW           | 165   | Multiple callers         |
| 0x10014330  | SS::BuildPacketCmd     | 26    | Multiple callers         |
| 0x10014650  | SS::FeedTraj           | 59    | ProcessTrajectory        |
| 0x10014710  | SS::ProcessOneTrajPt   | 140   | FeedTraj                 |
| 0x10014BF0  | SS::AccumSteps         | 70    | Home, Probe, JogOff      |
| 0x1000F2B0  | SS::ProcessDROs_F2B0   | 68    | Update (MotionState)     |
| 0x1000F3B0  | SS::ProcessDROs_F3B0   | 90    | Update (MotionState)     |
| 0x1000F570  | SS::ProcessDROs_F570   | 98    | Update (MotionState)     |
| 0x1000F7D0  | SS::ProcessDROs_F7D0   | 301   | Update (MotionState)     |

---

## Appendix B: Wrapper Function Pattern

All exported functions except InitControl, SetCode, SetDoButton, SetGetDRO, SetGetMenuRange, and SetSetDRO follow the same wrapper pattern:

```asm
; Example: Update @ 0x10004180
mov     byte ptr [10078C7Ch], 1     ; g_UpdateActive = 1
; (optional: timer creation check)
cmp     dword ptr [10078DECh], 0    ; if (g_SS != NULL)
je      skip
mov     ecx, [10078DECh]           ;   ecx = g_SS (this pointer)
call    1001xxxxh                   ;   g_SS->RealMethod()
skip:
mov     byte ptr [10078C7Ch], 0     ; g_UpdateActive = 0
ret
```

The pattern loads g_SS into ECX (thiscall convention) and calls the real implementation method. This indirection allows the plugin to safely handle the case where g_SS has not been allocated yet (before PostInitControl).

---

*Document generated from disassembly analysis of SmoothStepper_v17fe.dll*
*Disassembly: 80,383 lines | Functions analyzed: 22 | Object size: ~272KB*
