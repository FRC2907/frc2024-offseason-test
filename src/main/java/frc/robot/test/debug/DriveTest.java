package frc.robot.test.debug;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveTest extends TimedRobot{
    int i_top_left = 1, i_bottom_left = 2, i_top_right = 3, i_bottom_right = 4;

    CANSparkMax top_left = new CANSparkMax(i_top_left, MotorType.kBrushless);
    CANSparkMax bottom_left = new CANSparkMax(i_bottom_left, MotorType.kBrushless);
    CANSparkMax top_right = new CANSparkMax(i_top_right, MotorType.kBrushless);
    CANSparkMax bottom_right = new CANSparkMax(i_bottom_right, MotorType.kBrushless);

    MecanumDrive dt;

    PS5Controller driver = new PS5Controller(0);

    Timer timer = new Timer();

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

        dt = new MecanumDrive(top_left, bottom_left, top_right, bottom_right);
    }

    private void breakIn(){
        timer.restart();
        if (timer.get() < 1800){
            top_left.set(0.1);
            bottom_left.set(0.1);
            top_right.set(0.1);
            bottom_right.set(0.1);
        }
    }

    @Override
    public void teleopPeriodic(){
        dt.driveCartesian(driver.getLeftX(), driver.getLeftY(), driver.getRightX());
        //breakIn();  //use to break in motors for 30 minutes
    }
}