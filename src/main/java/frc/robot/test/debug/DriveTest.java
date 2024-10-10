package frc.robot.test.debug;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Control;

import com.kauailabs.navx.frc.AHRS;

public class DriveTest extends TimedRobot{
    private int i_top_left = 1, i_bottom_left = 3, i_top_right = 2, i_bottom_right = 4;

    private CANSparkMax top_left = new CANSparkMax(i_top_left, MotorType.kBrushless);
    private CANSparkMax bottom_left = new CANSparkMax(i_bottom_left, MotorType.kBrushless);
    private CANSparkMax top_right = new CANSparkMax(i_top_right, MotorType.kBrushless);
    private CANSparkMax bottom_right = new CANSparkMax(i_bottom_right, MotorType.kBrushless);

    private MecanumDrive dt;

    private AHRS gyro;

    private PS5Controller driver = new PS5Controller(0);

    private Timer timer = new Timer();

    private double volts = 0;

    @Override
    public void robotInit(){
        top_left.restoreFactoryDefaults();
        bottom_left.restoreFactoryDefaults();
        top_right.restoreFactoryDefaults();
        bottom_right.restoreFactoryDefaults();

        top_left.setInverted(false);
        bottom_left.setInverted(false);
        top_right.setInverted(true);
        bottom_right.setInverted(true);

        //top_left.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        //bottom_left.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        //top_right.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        //bottom_right.getEncoder().setVelocityConversionFactor(Control.drivetrain.kVelocityConversionFactor);
        top_left.getPIDController().setP(0.1);
        bottom_left.getPIDController().setP(0.1);
        top_right.getPIDController().setP(0.1);
        bottom_right.getPIDController().setP(0.1);
        top_left.getPIDController().setD(1);
        bottom_left.getPIDController().setD(1);
        top_right.getPIDController().setD(1);
        bottom_right.getPIDController().setD(1);
        //top_left.getEncoder().setPositionConversionFactor(1 / 5.95);
        //bottom_left.getEncoder().setPositionConversionFactor(1 / 5.95);
        //top_right.getEncoder().setPositionConversionFactor(1 / 5.95);
        //bottom_right.getEncoder().setPositionConversionFactor(1 / 5.95);

        dt = new MecanumDrive(top_left, bottom_left, top_right, bottom_right);
        top_left.setIdleMode(IdleMode.kBrake);
        bottom_left.setIdleMode(IdleMode.kBrake);
        top_right.setIdleMode(IdleMode.kBrake);
        bottom_right.setIdleMode(IdleMode.kBrake);


        gyro = new AHRS(SPI.Port.kMXP);
    }

    private void breakIn(){
        timer.restart();
        //if (timer.get() < 1800){
            top_left.set(0.1);
            bottom_left.set(0.1);
            top_right.set(0.1);
            bottom_right.set(0.1);
        //}
    }
    private void accelTest(){
        top_left.set(1);
        bottom_left.set(1);
        top_right.set(1);
        bottom_right.set(1);
    }
    private void conversionTest(){
        top_left.getPIDController().setReference(0.1, ControlType.kDutyCycle);
        bottom_left.getPIDController().setReference(0.1, ControlType.kDutyCycle);
        top_right.getPIDController().setReference(0.1, ControlType.kDutyCycle);
        bottom_right.getPIDController().setReference(0.1, ControlType.kDutyCycle);
    }

    @Override
    public void autonomousInit(){
        timer.restart();
    }

    @Override
    public void autonomousPeriodic(){ //test m/s
        //if (timer.get() < 1){
            top_left.getPIDController().setReference(0.1, ControlType.kSmartMotion);
            bottom_left.getPIDController().setReference(0.1, ControlType.kSmartMotion);
            top_right.getPIDController().setReference(0.1, ControlType.kSmartMotion);
            bottom_right.getPIDController().setReference(0.1, ControlType.kSmartMotion);
        //}
    }

    @Override
    public void teleopInit(){
        top_left.getEncoder().setPosition(0);
        bottom_left.getEncoder().setPosition(0);
        top_right.getEncoder().setPosition(0);
        bottom_right.getEncoder().setPosition(0);
    }

    @Override
    public void teleopPeriodic(){
        //dt.driveCartesian( - driver.getLeftY(), driver.getLeftX(), driver.getRightX());
        //dt.driveCartesian( - driver.getLeftX(), driver.getLeftY(), driver.getRightX(), gyro.getRotation2d());
        //breakIn();  //use to break in motors for 30 minutes
        //accelTest();
        //conversionTest();

        //volts = driver.getLeftY() * 12;
        volts = 0.13;
        top_right.setVoltage(volts);

        SmartDashboard.putNumber("flVelocity", top_left.getEncoder().getVelocity());
        SmartDashboard.putNumber("rlVelocity", bottom_left.getEncoder().getVelocity());
        SmartDashboard.putNumber("frVelocity", top_right.getEncoder().getVelocity());
        SmartDashboard.putNumber("rrVelocity", top_left.getEncoder().getVelocity());
        SmartDashboard.putNumber("volts", volts);
        SmartDashboard.putNumber("something", (volts - 0.0) / top_right.getEncoder().getVelocity());
    }
}