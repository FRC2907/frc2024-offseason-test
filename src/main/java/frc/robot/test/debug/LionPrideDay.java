package frc.robot.test.debug;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class LionPrideDay extends TimedRobot{
    int i_front_left = 1, i_rear_left = 2, i_front_right = 3, i_rear_right = 4,
        i_intake_slow = 5, i_intake_fast = 6;

    CANSparkMax frontLeft = new CANSparkMax(i_front_left, MotorType.kBrushless);
    CANSparkMax rearLeft = new CANSparkMax(i_rear_left, MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(i_front_right, MotorType.kBrushless);
    CANSparkMax rearRight = new CANSparkMax(i_rear_right, MotorType.kBrushless);

    CANSparkMax intakeSlow = new CANSparkMax(i_intake_slow, MotorType.kBrushless);
    CANSparkMax intakeFast = new CANSparkMax(i_intake_fast, MotorType.kBrushless);

    MecanumDrive dt;

    PS5Controller driver = new PS5Controller(0);

    @Override
    public void robotInit(){
        frontLeft.restoreFactoryDefaults();
        rearLeft.restoreFactoryDefaults();
        frontRight.restoreFactoryDefaults();
        rearRight.restoreFactoryDefaults();

        frontLeft.setInverted(false);
        rearLeft.setInverted(false);
        frontRight.setInverted(true);
        rearRight.setInverted(true);

        intakeSlow.restoreFactoryDefaults();
        intakeFast.restoreFactoryDefaults();

        intakeFast.follow(intakeFast);
        intakeFast.getEncoder().setVelocityConversionFactor(2);

        dt = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);
    }

    @Override
    public void autonomousInit(){}

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void teleopPeriodic(){
        dt.driveCartesian(driver.getLeftY(), driver.getLeftX(), driver.getRightX());

        if (driver.getR2Button())
            intakeSlow.getPIDController().setReference(600, ControlType.kVelocity);
    }

}
