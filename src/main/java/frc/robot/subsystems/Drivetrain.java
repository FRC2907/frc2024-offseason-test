package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.constants.Ports;
import frc.robot.util.Util;




public class Drivetrain extends DifferentialDrive implements ISubsystem{
    public CANSparkMax leftMotor;
    public CANSparkMax rightMotor;

    public static DriveMode mode;

    private Drivetrain(CANSparkMax left, CANSparkMax right){
      super(left, right);
      this.leftMotor = left;
      this.rightMotor = right;

      mode = DriveMode.LOCAL_FORWARD; 
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance(){
      CANSparkMax left, right;
      if (instance == null){
        left = Util.createSparkGroup(Ports.can.drivetrain.LEFTS, true, false);
        right = Util.createSparkGroup(Ports.can.drivetrain.RIGHTS, false, false);
        instance = new Drivetrain(left, right);
      }
      return instance;
    }

    public enum DriveMode {
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
    
    private Pose2d pose;

    public Pose2d getPose(){
      return this.pose;
    }

    private void setPose(Pose2d _pose){
      this.pose = _pose;
    }

    @Override
    public void onLoop(){}

    

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
  }
