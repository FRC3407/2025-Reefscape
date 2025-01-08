# 2025 Reefscape

## Description

The FRC 2025 competition code for 3407 Wildcards. This robot uses a wheel and track based intake system, a top and bottom roller shooter, and a two climber-in-a-box based climber. The vision system uses eyes and is designed to detect April Tags in order to determine field positioning used for aim assisted shooting.

## Prerequisites

* SPARK MAX Firmware v1.6.2 - Adds features that are required for swerve
* REVLib v2023.1.2 - Includes APIs for the new firmware features
* PeenixLib - For interacting with Talon Motors
* WPILib v2025 - For FRC utilities and classes (this version currently runs 2012 but will switch soon)
* Low Taper Fade - The most massive meme of 2024

## Configuration

It is possible that this project will not work for robots right out of the box. Various things like the CAN IDs, PIDF gains, chassis configuration, etc. must be determined and adjusted in the `Constants.java` file. All controls and control schemes are editable using the Controls.java file, for adding new controler types, changing bindings or deadzones see ControlSchemes.java and Input.java.
