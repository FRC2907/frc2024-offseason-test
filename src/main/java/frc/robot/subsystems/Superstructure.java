package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PS5Controller;



public class Superstructure implements ISubsystem{
    private Arm arm;
    private Drivetrain drivetrain;
    private Intake intake;
    private Led led;
    private Shooter shooter;
    private ISubsystem[] subsystems;
    
    PS5Controller driver = new PS5Controller(frc.robot.constants.Ports.HID.DRIVER);
    PS5Controller operator = new PS5Controller(frc.robot.constants.Ports.HID.OPERATOR);

    private RobotState state;
    private boolean automateScoring;

    private Trajectory trajectory;



    public enum RobotState {
      MOVING_TO_START
      , START, NEUTRAL

      , MOVING_TO_INTAKING, INTAKING, OUTAKING
      , MOVING_TO_HOLDING_NOTE, HOLDING_NOTE

      , MOVING_TO_AMP, READY_TO_SCORE_AMP, SCORING_AMP

      , MOVING_TO_SPEAKER, READY_TO_SCORE_SPEAKER, SCORING_SPEAKER

      , FOLLOWING_TRAJECTORY
    }

    private Superstructure(RobotState _state, boolean _automation){
        this.state = _state;
        //this.automateScoring(_automation);
        this.arm = Arm.getInstance();
        this.drivetrain = Drivetrain.getInstance();
        this.intake = Intake.getInstance();
        this.led = Led.getInstance();
        this.shooter = Shooter.getInstance();
        this.subsystems = new ISubsystem[] { arm, drivetrain, intake, led, shooter };
    } 

    private Superstructure(){
      this(RobotState.START, true);
    }

    private static Superstructure instance;

    public static Superstructure getInstance(){
      if (instance == null){
        instance = new Superstructure(RobotState.NEUTRAL, true);
      }
      return instance;
    }

    public RobotState getState() { return this.state; }


    public void moveToStart(){
      this.state = RobotState.MOVING_TO_START;
    }
    public void start(){
      this.state = RobotState.START;
    }
    public void moveToIntaking(){
      this.state = RobotState.MOVING_TO_INTAKING;
    }
    public void intakeNote(){
      this.state = RobotState.INTAKING;
    }
    public void outakeNote(){
      this.state = RobotState.OUTAKING;
    }
    public void moveToAmp(){
      this.state = RobotState.MOVING_TO_AMP;
    }
    public void scoreInAmp(){
      this.state = RobotState.SCORING_AMP;
    }
    public void moveToSpeaker(){
      this.state = RobotState.MOVING_TO_SPEAKER;
    }
    public void scoreInSpeaker(){
      this.state = RobotState.SCORING_SPEAKER;
    }
    public void followingTrajectory(){
      state = RobotState.FOLLOWING_TRAJECTORY;
    }
    public void neutralPosition() {
        if (intake.hasNote()){
            this.state = RobotState.HOLDING_NOTE;
            operator.setRumble(RumbleType.kBothRumble, 0.3);
        } else {
            this.state = RobotState.NEUTRAL;
        }
    }

    public void cancelAction(){
      neutralPosition();
    }
    public void automateScoring(boolean _automation){
      this.automateScoring = _automation;
    }
    public boolean isScoringAutomated(){
      return this.automateScoring;
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
          case FIELD_FORWARD:
            break;
          case FIELD_REVERSED:
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
      switch (this.state) {
        case MOVING_TO_START:
            arm.start();
            intake.off();
            shooter.off();
            if (arm.reachedSetPoint()){
                this.state = RobotState.START;
            }
            break;
        case START:
            break;

        case MOVING_TO_INTAKING:
            intake.intake();
            if (intake.reachedSetPoint()){
              this.state = RobotState.INTAKING;
            }
            break;
        case INTAKING:
            intake.intake();
            if (intake.hasNote()){
              shooter.noteScored = false;
              this.state = RobotState.HOLDING_NOTE;
            }
            break;

        case OUTAKING:
            intake.outake();
            if (!intake.hasNote()){
              this.state = RobotState.NEUTRAL;
            }
            break;
        case HOLDING_NOTE:
            arm.holdingPosition();
            intake.off();
            break;

        case MOVING_TO_AMP:
            arm.ampPosition();
            // TODO automatically drive up to the Amp
            if (arm.reachedSetPoint()) { // TODO add drivetrain reached set point
              this.state = RobotState.READY_TO_SCORE_AMP;
            }
            break;
        case READY_TO_SCORE_AMP:
            if (this.isScoringAutomated()){
              this.state = RobotState.SCORING_AMP;
            }
            break;
        case SCORING_AMP:
            shooter.amp();
            if (shooter.noteScored()){
              intake.hasNote = false;
              this.state = RobotState.NEUTRAL;
            }
            break;

        case MOVING_TO_SPEAKER:
            arm.speakerPosition();
            // TODO automatically drive up to the Speaker
            if (arm.reachedSetPoint()) { // TODO add drivetrain reached set point
              // TODO do we also want to get the shooter wheels up to speed first? or no?
              this.state = RobotState.READY_TO_SCORE_SPEAKER;
            }
            break;
        case READY_TO_SCORE_SPEAKER:
            if (this.isScoringAutomated())
              this.state = RobotState.SCORING_SPEAKER;
            break;
        case SCORING_SPEAKER:
            shooter.speaker();
            if (shooter.noteScored()) { 
              intake.hasNote = false;
              this.state = RobotState.NEUTRAL;
            }
            break;

        case NEUTRAL:
            arm.neutralPosition();
            intake.off();
            shooter.off();
            break;
        default:
            break;
      }


      //handleDriving(drivetrain.leftMotor, drivetrain.rightMotor);
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