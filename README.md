-----

# Project-Robot: Yellow

## FRC 2025: Team 10213 Balta Robotics Robot Code

This repository contains the source code for the **Yellow** robot, developed by **Balta Robotics (Team 10213)** for the 2025 FRC (FIRST Robotics Competition) season.

-----

## Overview

**Yellow-Robot** is a robot specifically designed and optimized for the FRC 2025 game theme. The primary goal of this project is to enable the robot to efficiently execute tasks in both autonomous and teleoperated modes. To achieve this, advanced **sensor fusion**, precise **motion control algorithms**, and game-specific **mechanism controls** have been integrated.

-----

## Key Features

Yellow-Robot boasts the following core features:

  * **Advanced Sensor Fusion:** Image processing data is combined with odometry data from the chassis using a sensor fusion algorithm. This enables the robot to estimate its position and orientation with high accuracy, which is crucial for precise autonomous movements.
  * **PID-Assisted Target Alignment:** Leveraging the consistent odometry data provided by sensor fusion, the robot can accurately approach specific targets using a **PID (Proportional-Integral-Derivative) control algorithm**. This offers a significant advantage in tasks such as accurate scoring of game objects or precise positioning in designated areas.
  * **Game Task Integration:** The robot is capable of performing functions tied to the FRC 2025 game theme, including **elevator mechanism movement** and **scoring of game objects**.

-----

## Setup

To set up and run the Yellow-Robot code locally, please follow these steps:

1.  **Install WPILib VS Code:** You'll need Visual Studio Code with the WPILib extension to compile and deploy the robot code. Visit the link below for installation instructions:

      * [WPILib VS Code Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/index.html)

2.  **Install NI Package Manager:** To facilitate communication, control, and relevant software uploads with the **NI roboRIO** used in FRC competitions, you must install the NI Package Manager. Visit the link below for installation instructions:

      * [NI Package Manager Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)

-----

## Usage

After cloning the code to your local environment via Git version control, you can use it by **updating the team number according to your scenario**.

```bash
git clone https://github.com/BaltaRobotics/Yellow-Robot.git
cd Yellow-Robot
```

  * **Updating Team Number:** To run the project on your team's robot, you may need to change the team number setting within the code (typically found in `build.gradle` or a similar configuration file) to your own FRC team number.

If you encounter any problems or wish to report a bug, please don't hesitate to open an **'Issue'** in this repository.

-----

## Contact

Should you have any questions about the project or need to consult regarding the code, you can reach me via the following contact information:

  * **Email:** alptng72@gmail.com
  * **LinkedIn:** [Burak Talha Sümer](https://www.linkedin.com/in/burak-talha-s%C3%BCmer-b3a339205/)

-----

## Contributing

This project was developed for an FRC season, and as such, the active contribution process may be limited. However, if you have suggestions for improvement or encounter any bugs, please feel free to share them with us by **opening an Issue**.
