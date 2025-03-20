# Team 1710 2025 Robot Code <div style="text-align: right">[![Build](https://github.com/FRC-Team-1710/2024-Robot/actions/workflows/main.yml/badge.svg)](https://github.com/FRC-Team-1710/2024-Robot/actions/workflows/main.yml) [![Spotless](https://github.com/FRC-Team-1710/2024-Robot/actions/workflows/spotless.yml/badge.svg)](https://github.com/FRC-Team-1710/2024-Robot/actions/workflows/spotless.yml)</div>
We use PhotonVision with 3 Orange Pi 5s, SDS Mk4i swerve modules, and PathPlanner for autos. Special thanks to 5712 for their [template](https://github.com/Hemlock5712/2025SwerveTemplate).

```
F  I  R  S  T   R  O  B  O  T  I  C  S   T  E  A  M
______________  _  _____   _  _____  ______________
\_____________|/ ||___  | / ||  _  ||_____________/
 \_ _ _ _ _ _ || |   / /  | || | | || _ _ _ _ _ _/
  \ _ _ _ _ _ || |  / /   | || |_| || _ _ _ _ _ /
   \__________||_|_/_/___ |_||_____||__________/
    \___________________/ \___________________/
                     ___.^.___
                     '.     .'
                      /.' '.\
```
___
![AdvantageKit Swerve Base Logo](assets/logo.png)


<!--- Version badges. Will automatically pull the latest version from main branch. --->
<p align="center">
<img src="https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/Hemlock5712/2025SwerveTemplate/refs/heads/main/vendordeps/AdvantageKit.json&query=$.version&label=AdvantageKit&color=fbc30c&style=for-the-badge" alt="AdvantageKit Version">
<img src="https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/Hemlock5712/2025SwerveTemplate/refs/heads/main/vendordeps/PathplannerLib.json&query=$.version&label=PathPlanner&color=3F56CE&style=for-the-badge" alt="PathPlanner Version">
<img src="https://img.shields.io/badge/dynamic/json?url=https://raw.githubusercontent.com/Hemlock5712/2025SwerveTemplate/refs/heads/main/vendordeps/Phoenix6-frc2025-latest.json&query=$.version&label=Phoenix%206&color=97d700&style=for-the-badge" alt="Phoenix 6 Version">
</p>

This is a full featured template repository designed to make setting up a new robot as easy as possible. We use the Phoenix 6 Swerve library to provide top of the line drivetrain responsiveness, along with the new [PathPlanner setpoint generation API](https://pathplanner.dev/pplib-swerve-setpoint-generator.html) (based on a version created by team 254) to prevent skidding and wheel slipping.

This template comes with full simulation and replay support built in, which allow you to program the robot, without even having the robot finished yet. More information on replay usage can be found on the [AdvantageKit documentation website](https://docs.advantagekit.org/getting-started/traditional-replay).
