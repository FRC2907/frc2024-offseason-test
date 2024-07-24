package frc.robot.test.debug;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class LEDTest extends TimedRobot {
    private PWMSparkMax led;
    private PS4Controller driver;

    @Override
    public void robotInit(){
        led = new PWMSparkMax(0);
        driver = new PS4Controller(0);
    }
    
    @Override
    public void autonomousInit(){}

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void teleopPeriodic(){
        if (driver.getCrossButton()){
            led.set(0.61);
        }
        if (driver.getSquareButton()){
            led.set(0.65);
        }
        if (driver.getCircleButton()){
            led.set(0.69);
        }
        if (driver.getR1Button()){
            led.set(0.81);
        }
        if (driver.getR2Button()){
            led.set(0.71);
        }
    }
}