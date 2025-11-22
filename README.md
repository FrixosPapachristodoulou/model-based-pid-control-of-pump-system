# Model-Based PID Control of a Pump System

This repository contains the full workflow for designing, validating, and deploying a **model-based PID controller** for a Variable Frequency Drive (VFD)-driven centrifugal pump.  
It includes:

- **MATLAB System Identification** using experimental pump data  
- **Closed-loop PID tuning and disturbance simulation**  
- **Embedded PID implementation** on a microcontroller  
- **Real-time Modbus communication with a VFD**  
- **Pressure-based flow estimation** and safety logic  

This project completes the entire digital-twin → controller-design → hardware-deployment cycle for a real pump system.

---

## Project Overview

### **Goal**
Maintain a **constant flow rate of 20 m³/h**, despite:

- VFD speed variations  
- Valve-closing / clogging disturbances  
- Sensor noise  
- Non-linear flow–frequency relationships  

### **Approach**
1. Collect experimental measurements from pump + VFD system  
2. Identify a **dynamic transfer function model** using `tfest`  
3. Tune a robust PID controller (≥60° PM, disturbance-rejection focus)  
4. Validate controller response in MATLAB under clogging disturbance  
5. Deploy the final gains to microcontroller firmware  
6. Control the real pump via Modbus while estimating flow from pressure  

---
```

+---------------------------+
| MATLAB Model |
+---------------------------+
| - Import datasets |
| - System ID (tfest) |
| - PID tuning (pidtune) |
| - Clogging simulation |
| - Closed-loop validation |
+-------------+-------------+
|
v
+---------------------------+
| Embedded Controller |
+---------------------------+
| - I2C pressure sensor |
| - EMA smoothing filter |
| - Flow estimation model |
| - PID loop (2 s update) |
| - 30–60 Hz VFD command |
| - Modbus communication |
| - Safety buzzer logic |
+-------------+-------------+
|
v
+---------------------------+
| Physical Pump |
+---------------------------+
| - Centrifugal pump |
| - Driven by VFD |
| - Flow disturbances |
+---------------------------+

```


---

# MATLAB System Identification & Simulation  
**File:** `pump_system_identification_and_pid_simulation.m`

### **1. Load experimental datasets**
- Gradual frequency sweep  
- Step change response  
- Valve-closing/clogging disturbance  

### **2. Fit clogging disturbance**
A 4th-order polynomial models the gradual flow reduction caused by partial valve closure:

```matlab
p = polyfit(time_clogging, flow_rate_clogging, 4);
```

### 3. System Identification

Two datasets are merged and fitted to a **2-pole / 0-zero** transfer function:

```matlab
sys_combined = tfest(combined_data, 2, 0);
```


### 4. PID Tuning

`pidtune` was configured with:

- **DesignFocus:** disturbance-rejection  
- **PhaseMargin:** ≥ 60°

The deployed PID gains used in both MATLAB and the embedded controller are:

Kp = 0.054
Ki = 0.0000001
Kd = 1.3

---

### 5. Closed-Loop Simulation

The simulation includes:

- **Frequency saturation** (30–60 Hz)  
- **Clogging disturbance** injected after **480 s**  
- **PID activation** after **240 s**  
- **1-second discrete update** of the controller  

Plots generated:

- Controlled flow rate  
- Clogging effect  
- VFD frequency evolution  
- Error relative to the **±2% band**


## ⚙️ Embedded Firmware (Arduino)

**File:** `pump_pid_controller_arduino.ino`

---

### **Sensors & Interfaces**

- **I²C pressure sensor** (`0x28`)
- **EMA smoothing filter** (`alpha = 0.045`)
- **Flow estimation model:**

flow = 15.079 * pressure^0.5921

yaml
Copy code

- **Modbus (Serial1)** communication with VFD  
- **Switch input** for ON/OFF  
- **Active-low buzzer** for saturation warnings  

---

### **Control Loop Details**

- Sensor read: **every 1 s**  
- PID update: **every 2 s**  
- PID activated after **240 s**  
- Frequency limited to **30–60 Hz**  

---

### **PID Law**

```c
error = targetFlowRate - currentFlowRate;
integralError += error * dt;
derivative = (error - lastFlowRateError) / dt;

adjustment = Kp*error + Ki*integralError + Kd*derivative;
frequencyHz = constrain(frequencyHz + adjustment, 30, 60);
```

---

### **Deadband**

Flow within **94.25% – 105.75%** of target temporarily disables control action  
to prevent oscillation and unnecessary actuator movement.

---

### **Buzzer Safety Logic**

The buzzer activates if:

- The frequency reaches **60 Hz**, **and**
- It remains at 60 Hz for **> 3 seconds**

---

## Why This Approach Works

### **Model-driven controller design**
The controller is tuned using a mathematically identified pump model.

### **Disturbance rejection**
PID is configured to handle slow nonlinear clogging disturbances.

### **Realistic actuator limits**
Frequency saturation (**30–60 Hz**) is included in both simulation and firmware.

### **Noise suppression**
EMA filtering stabilizes noisy pressure measurements for a cleaner flow estimate.

### **One-to-one match between MATLAB and firmware**
Same gains, same constraints, same timings → accurate digital twin behavior.

---

## Requirements

### **MATLAB**
- System Identification Toolbox  
- Control System Toolbox  
- Curve Fitting Toolbox  

### **Embedded**
- Arduino-compatible microcontroller  
- `ModbusMaster` library  
- I²C sensor support (400 kHz clock)  
- VFD supporting Modbus registers `0x2000` and `0x2001`  

---

## Results Summary

The PID controller successfully:

- Maintains **20 m³/h ±2%**  
- Recovers from **clogging disturbances**  
- Avoids overshoot and oscillations  
- Respects **VFD actuator limits**  
- Operates robustly in real hardware tests  

---


## Acknowledgements

Developed as part of **DMT Group 16C** (2023-24), Imperial College London.



