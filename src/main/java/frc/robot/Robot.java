// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ISubsystem;
import frc.robot.subsystems.Superstructure;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private Timer autoTimer;

  private ISubsystem everything;
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    autoTimer = new Timer();
    everything = Superstructure.getInstance();
    robotContainer = new RobotContainer();
  } 

  @Override
  public void robotPeriodic(){
    everything.onLoop();
  }

  @Override
  public void autonomousInit() {
    autoTimer.restart();
    robotContainer.getAutonomousCommand();
  }

  @Override
  public void autonomousPeriodic(){
    robotContainer.getAutonomousCommand();
  }

  @Override
  public void teleopInit(){}

  @Override
  public void teleopPeriodic(){}
}