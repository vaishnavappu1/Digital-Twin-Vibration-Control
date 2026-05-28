# Intelligent Vibration Fault Detection and Active Suppression System
### Based on Digital Twin and Adaptive Control | ESP32

## Overview
A real-time vibration monitoring and active suppression system implemented 
on ESP32. Integrates automatic digital twin construction, priority-based 
multi-fault detection, and bounded adaptive PWM sweep-based active 
vibration suppression using a secondary motor with eccentric mass.

## Key Results
- Best-case vibration reduction: 23.6% to 53.3% (mean 38.8%)
- Average session reduction: 12.2% across 9 experimental sessions
- 4-priority fault detection with real-time serial monitoring
- Self-calibrating digital twin — no prior system model required

## Hardware Components
| Component | Role |
|-----------|------|
| ESP32 | Central controller |
| MPU6050 | 3-axis vibration sensing |
| IBT-2 (BTS7960) | Motor driver for Motor A and B |
| ACS712 | Current sensing |
| IR Reflective Sensor | RPM measurement |
| Motor A + eccentric mass | Disturbance source |
| Motor B + eccentric mass | Active suppression actuator |
## Hardware Setup
<img width="861" height="312" alt="image" src="https://github.com/user-attachments/assets/0a01a2a4-a9e7-403b-b031-53a2008d6492" />


## System Architecture
### 1. Auto-Commissioning & Digital Twin
- Sweeps Motor A across PWM 80–170 on startup
- Fits 3 linear regression models (vibration, RPM, current vs PWM)
- No external model required — fully self-calibrating
- <img width="1035" height="493" alt="image" src="https://github.com/user-attachments/assets/db3b8900-5394-4bfc-8925-4f4ee2eaf913" />


### 2. Fault Detection (Priority 1–4)
| Priority | Fault | Trigger |
|----------|-------|---------|
| 1 | Resonance | vibRMS > 1.5× normal in resonance zone |
| 2 | Structural Change | vibRMS < 0.35× expected |
| 3 | Electrical | RPM drop or current deviation |
| 4 | IR Sensor Fault | No RPM signal at PWM > 100 for 5s |

### 3. Adaptive PWM Sweep (Active Suppression)
- Motor B sweeps PWM 80–160 searching for minimum vibration
- Band narrows ±20 PWM around best point after 15 seconds
- 3 fallback mechanisms prevent entrapment in poor phase regions

### 4. FFT Analysis
- 128-point FFT at 100 Hz during commissioning
- 0.78 Hz frequency resolution
- Cross-validated against IR RPM for resonance confirmation

## Control Modes
| Mode | Command | Behaviour |
|------|---------|-----------|
| Manual | M | Start Motor B sweep immediately |
| Resonance | P | Start on resonance fault |
| Zone | Z | Auto start/stop at zone boundary |
| Stop | S | Stop Motor B, print session report |
| Recalibrate | R | Re-run full commissioning |

## Mathematical Models
- V_exp = VIB_SLOPE × PWM + VIB_OFFSET
- RPM_exp = RPM_SLOPE × PWM + RPM_OFFSET (PWM ≥ 155 only)
- I_exp = I_SLOPE × PWM + I_OFFSET

## Results Summary (9 Sessions)
| Metric | Value |
|--------|-------|
| Mean best-case reduction | 38.8% |
| Mean average reduction | 12.2% |
| Max reduction achieved | 53.3% (Session 3) |
| Optimal Motor B PWM range | 85–111 |
## Results

### PWM vs Vibration
<img width="938" height="249" alt="image" src="https://github.com/user-attachments/assets/bed12397-b439-4ae5-92bd-9ccbaa4f81f8" />


### PWM vs RPM
<img width="936" height="200" alt="image" src="https://github.com/user-attachments/assets/782d5145-88ac-4a63-9e43-384deea9415c" />


### PWM vs Current
<img width="936" height="226" alt="image" src="https://github.com/user-attachments/assets/a0c06adb-804a-4772-98d0-c868d46aed25" />


### Digital Twin Validation
<img width="938" height="625" alt="image" src="https://github.com/user-attachments/assets/70da4ae1-c2f0-4016-87dd-bdcd47db774c" />


## Future Improvements
- Dual IR sensors for direct phase measurement
- Filtered-x LMS adaptive control
- Wireless MQTT telemetry dashboard
- Machine learning anomaly detection

## Tools & Libraries
- Arduino IDE / ESP32
- arduinoFFT library
- MPU6050 I2C library

## Team
Vaishnav Suresh | Nasheeth K | Sreerag N | Mayoogh Manohar  
BTech — Applied Electronics and Instrumentation  
College of Engineering Trivandrum, Kerala | May 2026
