# Robotic Arm Control System

This repository contains the design, simulation, and embedded implementation
of a 4-degree-of-freedom serial robotic arm, developed with a simulation-first
and hardware-aware workflow.

The project progresses through multiple stages:
- MATLAB-based modeling and control simulation
- Hardware-in-the-loop (HIL) validation
- Real-time embedded firmware on STM32
- Physical hardware bring-up and testing

## Repository Structure

- simulation/   MATLAB modeling, control, and trajectory planning
- hil/          Hardware-in-the-loop validation framework
- firmware/     Embedded STM32 control firmware
- hardware/     Schematics, PCB, and bring-up notes
- docs/         System-level architecture and assumptions

## Current Status

- Simulation: Completed
- HIL: In progress
- Firmware: In progress
- Hardware: In progress

## Roadmap

- Validate Cartesian trajectory tracking in simulation
- Introduce timing and numerical constraints via HIL
- Port control logic to STM32 firmware
- Integrate encoder feedback and motor drivers
 robotic-arm
