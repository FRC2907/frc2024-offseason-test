package frc.robot.subsystems;

public interface ISubsystem {
    public default void onLoop(){
        receiveOptions();
        submitTelemetry();
    }
    public default void submitTelemetry(){}
    public default void receiveOptions(){}
}