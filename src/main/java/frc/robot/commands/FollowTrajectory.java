package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends RamseteCommand {
    public FollowTrajectory(Trajectory trajectory, Drivetrain drivetrain){
        super(
            trajectory,
            drivetrain::getPose, 
            new RamseteController(), 
            new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
            new DifferentialDriveKinematics(DriveConstants.trackWidth), 
            drivetrain::getWheelSpeeds, 
            new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
            new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD), 
            drivetrain::driveByVoltage, 
            drivetrain
        );
    }
}
