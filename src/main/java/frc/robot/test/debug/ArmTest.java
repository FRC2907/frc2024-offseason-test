package frc.robot.test.debug;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Control;

public class ArmTest extends TimedRobot{
    private int i_arm = 14;

    CANSparkMax arm = new CANSparkMax(i_arm, MotorType.kBrushless);

    @Override
    public void robotInit(){
        this.arm.restoreFactoryDefaults();
        this.arm.burnFlash();
        this.arm.getEncoder().setPosition(0);
        this.arm.getPIDController().setP(Control.arm.kP);
        this.arm.getPIDController().setD(Control.arm.kD);
    }
    @Override
    public void robotPeriodic(){
        SmartDashboard.putNumber("position", arm.getEncoder().getPosition());
    }

    @Override
    public void teleopInit(){
        this.arm.getEncoder().setPosition(0);
        this.arm.getPIDController().setOutputRange(-0.4, 0.4);
    }
    @Override
    public void teleopPeriodic(){
        this.arm.getPIDController().setReference(100, ControlType.kPosition);
    }
}