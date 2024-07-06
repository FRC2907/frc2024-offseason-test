package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.util.Util;




public class Drivetrain extends DifferentialDrive implements ISubsystem{
    /*int i_left_leader = 1, i_left_follower_1 = 2, i_left_follower_2 = 3,
     i_right_leader = 14, i_right_follower_1 = 15, i_right_follower_2 = 16;

    TalonFX left_leader = new TalonFX(i_left_leader);
    TalonFX left_follower_1 = new TalonFX(i_left_follower_1);
    TalonFX left_follower_2 = new TalonFX(i_left_follower_2);

    TalonFX right_leader = new TalonFX(i_right_leader);
    TalonFX right_follower_1 = new TalonFX(i_right_follower_1);
    TalonFX right_follower_2 = new TalonFX(i_right_follower_2); */

    public TalonFX leftMotor;
    public TalonFX rightMotor;

    public static DriveMode mode;

    private Drivetrain(TalonFX left, TalonFX right){
      super(left, right);
      this.leftMotor = left;
      this.rightMotor = right;

      mode = DriveMode.LOCAL_FORWARD; 
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
      TalonFX left, right;
      if (instance == null){
        left = Util.createTalonFXGroup(frc.robot.constants.Ports.can.drivetrain.LEFTS, true, false);
        right = Util.createTalonFXGroup(frc.robot.constants.Ports.can.drivetrain.RIGHTS, false, false);
        instance = new Drivetrain(left, right);
      }
      return instance;
    }

    public enum DriveMode{
        AUTO, LOCAL_FORWARD, LOCAL_REVERSED
    }
    
    public static void setDriveMode(DriveMode newMode){
        mode = newMode;
    }
  
    public DriveMode getDriveMode(){
        return mode;
    }

    public void reverse(){
      if (mode == DriveMode.LOCAL_FORWARD){
        setDriveMode(DriveMode.LOCAL_REVERSED);
      } else if (mode == DriveMode.LOCAL_REVERSED){
        setDriveMode(DriveMode.LOCAL_FORWARD);
      }
    }
    
    /*private void handleDriving(){
        double speed = driver.getLeftY();
        double rotation = -driver.getRightX();
    
        switch(mode){
          case AUTO:
          case LOCAL_FORWARD:
            break;
          case LOCAL_REVERSED:
            speed = -speed;
            break;
        }
    
        double left = speed + rotation;
        double right = speed - rotation;
    
        if (left > 1.0 || left < -1.0) { right = right / Math.abs(left); left = left / Math.abs(left); }
        if (right > 1.0 || right < -1.0) { left = left / Math.abs(right); right = right / Math.abs(right); }
    
        left = left / 4;
        right = right / 4;
    
        left_leader.set(left);
        right_leader.set(right);
      } 
    }*///superstructure

    @Override
    public void onLoop(){}

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
  }
