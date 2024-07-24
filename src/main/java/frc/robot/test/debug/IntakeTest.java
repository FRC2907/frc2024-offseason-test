package frc.robot.test.debug;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;

public class IntakeTest extends TimedRobot {
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private PS4Controller driver;

    @Override
    public void robotInit(){
        leftMotor = new CANSparkMax(0, MotorType.kBrushless);
        rightMotor = new CANSparkMax(1, MotorType.kBrushless);
        driver = new PS4Controller(0);

        leftMotor.setInverted(true);
    }
    
    @Override
    public void autonomousInit(){}

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void teleopPeriodic(){
        if (driver.getCrossButton()){
           rightMotor.set(1);
        }
        leftMotor.set(rightMotor.get() / 2);
    }
}