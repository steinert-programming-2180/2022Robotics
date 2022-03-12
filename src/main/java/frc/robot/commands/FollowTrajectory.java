package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends RamseteCommand {
    public FollowTrajectory(Trajectory trajectory, Drivetrain drivetrain){
        super(
            trajectory,
            drivetrain::getPose, 
            new RamseteController(Drive.b, Drive.zeta), 
            new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA),
            new DifferentialDriveKinematics(Drive.trackWidth), 
            drivetrain::getWheelSpeeds, 
            new PIDController(Drive.leftKp, Drive.leftKi, Drive.leftKd), 
            new PIDController(Drive.rightKp, Drive.rightKi, Drive.rightKd), 
            drivetrain::driveByVoltage, 
            drivetrain
        );
    }
}
