// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//other example repos: 4607, 3457
//this code is based on team frc3512 SwerveBot-2022

package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.subsystems.lighting.Lighting;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.controls.Controls;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    Limelight.getInstance();

    Controls.getInstance().configureControls();

    // Initialize the lighting by calling getInstance() because nothing else in the code does.
    Lighting.getInstance();
    
    LiveWindow.disableAllTelemetry();
  }

  public void teleopEnable() {}
}