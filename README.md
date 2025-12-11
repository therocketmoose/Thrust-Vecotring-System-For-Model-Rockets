Thrust Vectoring & Landing Control System for Model Rockets

Lead Engineer: Xavier Vimard
Project Start: 27/09/2025
Expected Completion: 20/05/2026

Project Overview:

This repository contains the full development of a thrust-controlled landing system (TCLS) for a model rocket platform. The project aims to design, build, and test a reusable model rocket capable of performing controlled ascent, attitude stabilization, descent management, and soft landing. The objective is to create a working proof-of-concept that integrates thrust vectoring, aerodynamic control, reaction wheel stabilization, and deployable landing legs into a unified flight system. This project is expected to operate over a one-year development window, with several intermediate prototypes and test campaigns.

The rocket platform itself is not the primary focus; the emphasis is the TCLS hardware, control algorithms, and validation testing necessary to achieve reliable vertical landing.

Project Goals:
- Develop a fully functional thrust vectoring system capable of four-axis gimbal control for ascent and landing.

- Implement a stability assist system using grid fins to manage minor deviations during ascent.

- Design a reaction wheel to counter roll and maintain thrust vector orientation.
as roll stabilization is critical, as uncontrolled roll makes TVC ineffective.

- Engineer deployable landing legs to ensure stable recovery after touchdown.

- Build a complete electronics and software stack integrating sensors, control logic, and closed-loop PID stabilization.

- Validate feasibility via simulation, static fire testing, and full-scale launch tests.

Development Timeline:

My project follows a structured month-by-month roadmap:

Nov 2025: Design, print and stress test the TVC mount, the reaction wheel and the payload bay.
Dec 2025: Design and simulate and prepare the code for the PCB.
Jan 2026: Test the TVC with real motors, sssemble prototype rocket (without parachute), refine TVC mount, begin full system dry runs.
Feb 2026: First flight test of the prototype, collect IMU/GPS data, evaluate control authority.
Mar 2026: Analyze flight performance, improve PID tuning, begin landing system integration.
Apr 2026: Integrate final landing mechanism, conduct ground validations.
May 2026: Perform full integration test and finalize launch preparation for main flight.
Jun 2026: Full-system launch and documentation of results.

System Architecture
Core Components:

- MCU: Teensy 4.1 — selected for deterministic real-time control performance.

- IMU: Bosch BMI088 — optimized for high-vibration flight profiles.

- Barometer: MS5611 — high-resolution altitude tracking (10 cm).

- Ground Sation: ESP32 and Lora reciever — supplementary system for communication and logging.

- Structure: Carbon fiber body, TVC gimbal, electronics bay, reaction wheel assembly.

CAD & Mechanical: 
The CAD design includes

- A 4-axis servo-driven TVC module.

- A compact reaction-wheel chamber below the nose cone.

- A payload bay positioned directly above the TVC to avoid cable latency.

Simulation Work

To validate feasibility, ascent and landing behavior is simulated using Xcos (SciLab).
Simulations include gyro stabilization behavior, ascent velocity and altitude curves, and TVC response testing. Early results indicate stable linear altitude growth and adequate thrust-to-weight margin, confirming the motor’s suitability for controlled ascent.
