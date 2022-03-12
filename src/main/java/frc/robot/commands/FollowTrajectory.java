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
            new RamseteController(DriveConstants.b, DriveConstants.zeta), 
            new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA),
            new DifferentialDriveKinematics(DriveConstants.trackWidth), 
            drivetrain::getWheelSpeeds, 
            new PIDController(DriveConstants.leftKp, DriveConstants.leftKi, DriveConstants.leftKd), 
            new PIDController(DriveConstants.rightKp, DriveConstants.rightKi, DriveConstants.rightKd), 
            drivetrain::driveByVoltage, 
            drivetrain
        );
    }
}
