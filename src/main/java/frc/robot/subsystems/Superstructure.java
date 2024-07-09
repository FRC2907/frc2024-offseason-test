package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PS5Controller;



public class Superstructure implements ISubsystem{
    private Arm arm;
    private Drivetrain drivetrain;
    private Intake intake;
    private LED led;
    private Shooter shooter;
    private ISubsystem[] subsystems;
    
    PS5Controller driver = new PS5Controller(frc.robot.constants.Ports.HID.DRIVER);
    PS5Controller operator = new PS5Controller(frc.robot.constants.Ports.HID.OPERATOR);

    private RobotState state;
    private boolean automateScoring;



    public enum RobotState {
        MOVING_TO_START,

        START, NEUTRAL,

        MOVING_TO_INTAKING, INTAKING, HOLDING_BALL, 

        MOVING_TO_SCORE, READY_TO_SCORE, SCORING
    }

    private Superstructure(RobotState _state, boolean _automation){
        this.state = _state;
        //this.automateScoring(_automation);
        this.arm = Arm.getInstance();
        this.drivetrain = Drivetrain.getInstance();
        this.intake = Intake.getInstance();
        this.led = LED.getInstance();
        this.shooter = Shooter.getInstance();
        this.subsystems = new ISubsystem[] { arm, drivetrain, intake, led, shooter };
    } 

    private static Superstructure instance;

    public static Superstructure getInstance(){
      if (instance == null){
        instance = new Superstructure(RobotState.NEUTRAL, true);
      }
      return instance;
    }

    private void handleDriving(CANSparkMax leftMotor, CANSparkMax rightMotor){
        double left;
        double right;
        double speed = driver.getLeftY();
        double rotation = -driver.getRightX();
    
        switch(drivetrain.getDriveMode()){
          case AUTO:
          case LOCAL_FORWARD:
            break;
          case LOCAL_REVERSED:
            speed = -speed;
            break;
        }
    
        left = speed + rotation;
        right = speed - rotation;
    
        if (left > 1.0 || left < -1.0) { right = right / Math.abs(left); left = left / Math.abs(left); }
        if (right > 1.0 || right < -1.0) { left = left / Math.abs(right); right = right / Math.abs(right); }
    
        left = left / 4;
        right = right / 4;

        leftMotor.set(left);
        rightMotor.set(right);
      } 

    private void handleInputs(){ //TODO change to reference states
      if (driver.getR1ButtonPressed()){
      drivetrain.reverse();
      }
  }

    @Override
    public void onLoop(){
      handleDriving(drivetrain.leftMotor, drivetrain.rightMotor);
      handleInputs();

      for (ISubsystem s : this.subsystems){
        s.onLoop();
      }
    }

    @Override
    public void submitTelemetry(){}

    @Override
    public void receiveOptions(){}
    
}
