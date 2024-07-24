package frc.robot.subsystems;

public interface ISubsystem {
    public void onLoop();
    public void submitTelemetry();
    public void receiveOptions();
}