package frc.robot.subsystems;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

public class TrajectoryFollower { //just delete this, i don't want do
    private Trajectory trajectory;
    private Timer timer;
    private MecanumDriveKinematics kinematics;

    private static RamseteController controller = new RamseteController();


    public TrajectoryFollower(Trajectory t, MecanumDriveKinematics k){}

    public void startTrajectory(Trajectory t){
        timer.reset();
        this.trajectory = t;
        timer.start();
    }

    public double[] getWheelSpeeds(Pose2d current){
        Trajectory.State goal = trajectory.sample(timer.get());
        ChassisSpeeds adjustedSpeeds = controller.calculate(current, goal);
        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(adjustedSpeeds);
        double frontLeft  = wheelSpeeds.frontLeftMetersPerSecond;
        double rearLeft   = wheelSpeeds.rearLeftMetersPerSecond;
        double frontRight = wheelSpeeds.frontRightMetersPerSecond;
        double rearRight  = wheelSpeeds.rearRightMetersPerSecond;

        return new double[] { frontLeft, rearLeft, frontRight, rearRight };
    }

    public boolean isDone(){
        return trajectory.getTotalTimeSeconds() < timer.get();
    }
}
