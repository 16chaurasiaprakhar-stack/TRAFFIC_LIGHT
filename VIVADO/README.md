# ⚡ AMD Vivado FPGA Implementation Flow

This directory contains the commercial FPGA design, synthesis, implementation, and simulation environment for the **Intelligent Traffic Light Controller**, implemented and tested in **AMD Vivado 2025.2** targeting the **Xilinx Artix-7 FPGA (`xc7a35tcpg236-1`)**.

---

## 🎯 Why AMD Vivado is Required
While behavioral simulation verifies logical functionality, physical implementation on real FPGA silicon is essential for real-world deployment:
1. **Physical Synthesis:** Maps high-level Verilog RTL constructs into vendor-specific FPGA primitives (LUTs, Flip-Flops, Multiplexers, and I/O buffers).
2. **Static Timing Analysis (STA):** Validates that setup ($T_{su}$) and hold ($T_h$) constraints are satisfied across worst-case process, voltage, and temperature (PVT) corners with positive timing slack.
3. **Power Dissipation Profiling:** Measures on-chip power consumption (dynamic switching vs. static device leakage) to ensure reliable thermal operation inside an unventilated traffic control cabinet.
4. **Physical Placement & Routing (P&R):** Determines the exact slice and pin placement on the Artix-7 die to avoid clock skew and route congestion.

---

## 📁 Directory Structure

```
VIVADO/
├── README.md                      # This documentation
├── TRAFFIC_LIGHT/                 # Complete Vivado Project Directory
│   ├── TRAFFIX__NEW_LIGHT.xpr     # Main project file
│   ├── tb_traffic_fsm_behav.wcfg  # Waveform viewer configuration
│   ├── TRAFFIX__NEW_LIGHT.srcs/   # RTL sources, testbenches, constraints (.xdc)
│   ├── TRAFFIX__NEW_LIGHT.runs/   # Synthesis and implementation run data
│   └── TRAFFIX__NEW_LIGHT.sim/    # Simulation artifacts (XSim)
└── OUTPUTS/                       # Implementation Results & Visual Artifacts
    ├── SCHMETIC.png               # Synthesized RTL gate-level schematic
    ├── DEVICE.png                 # Artix-7 FPGA package floorplan
    ├── DEVICE(1).png              # Zoomed-in slice placement and routing map
    ├── SIMULATION2.png            # Behavioral simulation output waveforms
    ├── TFXX.png                   # Waveform timing analysis
    ├── POWER.png / POWER (2).png  # Vivado Power Analyzer summary
    └── TIMING.png                 # Timing summary and setup/hold slack
```

---

## 📊 Visual Artifacts & Implementation Reports

The `OUTPUTS/` folder contains verification evidence captured from the Vivado design flow:

| Image File | Description | Engineering Significance |
| :--- | :--- | :--- |
| **`SCHMETIC.png`** | Synthesized Gate Schematic | Displays how the Moore FSM, timer down-counter, and combinatorial output logic are mapped to logic gates. |
| **`DEVICE.png`** | Artix-7 Device View | Illustrates overall logic placement across the FPGA die, confirming very low logic utilization. |
| **`DEVICE(1).png`** | Detailed Slice Routing | Close-up view of LUT-to-Flip-Flop routing interconnects within the Configurable Logic Blocks (CLBs). |
| **`POWER.png`** / **`POWER (2).png`** | On-Chip Power Analysis | Comprehensive power summary showing dynamic switching vs. static leakage power under nominal supply voltages. |
| **`TIMING.png`** | Timing Summary Report | Proves that all paths meet timing with zero negative slack ($\text{WNS} \ge 0\,\text{ns}$, $\text{TNS} = 0\,\text{ns}$). |
| **`SIMULATION2.png`** / **`TFXX.png`** | XSim Simulation Waveforms | Verifies phase dwell intervals, round-robin priority transitions, and fail-safe all-red clearance. |

---

## 🚀 How to Run in AMD Vivado

### Method 1: Using the Vivado GUI
1. Launch **AMD Vivado 2025.2**.
2. Select **Open Project** $\to$ navigate to and select:
   ```
   VIVADO/TRAFFIC_LIGHT/TRAFFIX__NEW_LIGHT.xpr
   ```
3. **To Run Behavioral Simulation:**
   * In the **Flow Navigator** (left panel), click **Simulation $\to$ Run Behavioral Simulation**.
   * In the waveform window, load the saved configuration:
     `File $\to$ Open Waveform Configuration $\to$ select tb_traffic_fsm_behav.wcfg`.
   * Click **Run All** ($F3$) to observe the self-checking assertions.
4. **To Run Synthesis & Implementation:**
   * In the Flow Navigator, click **Run Synthesis** $\to$ view the synthesized schematic via **Open Synthesized Design**.
   * Click **Run Implementation** $\to$ view the device floorplan, timing summary, and power utilization.
5. **To Generate Bitstream (for Basys 3 / Artix-7 boards):**
   * Click **Generate Bitstream** $\to$ program the FPGA via the **Hardware Manager**.

### Method 2: Using Vivado Tcl Shell (Batch / Automation)
```tcl
# Open project
open_project VIVADO/TRAFFIC_LIGHT/TRAFFIX__NEW_LIGHT.xpr

# Run behavioral simulation
launch_simulation
run all

# Run synthesis and implementation
launch_runs synth_1 -jobs 4
wait_on_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1

# Generate summary reports
open_run impl_1
report_timing_summary -file timing_summary.rpt
report_power -file power_analysis.rpt
report_utilization -file utilization.rpt
```

---

## 💡 Key Results
* **Target FPGA:** Xilinx Artix-7 `xc7a35tcpg236-1`
* **Synthesis Result:** Clean mapping with zero inferred latches and zero DRC errors.
* **Timing Closure:** Zero setup and hold timing violations.
* **Power Envelope:** Ultra-low on-chip power footprint, making it suitable for solar-powered or battery-backed traffic control units.
