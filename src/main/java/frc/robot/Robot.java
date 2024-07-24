// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
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
  PS4Controller driver = new PS4Controller(0);
  //PS4Controller operator = new PS4Controller(1);
  PS4Controller operator = driver;

  Timer autoTimer = new Timer();

  private ISubsystem everything;

  @Override
  public void robotInit() {
    everything = Superstructure.getInstance();
  } 

  @Override
  public void robotPeriodic(){
    everything.onLoop();
  }

  @Override
  public void autonomousInit() {
    autoTimer.restart();
  }

  @Override
  public void autonomousPeriodic(){}

  @Override
  public void teleopInit(){}

  @Override
  public void teleopPeriodic(){}
}