package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleopSwerve extends Command {
        private final DrivetrainSubsystem swerveSubsystem;

        public TeleopSwerve(DrivetrainSubsystem swerveSubsystem) {
                this.swerveSubsystem = swerveSubsystem;
                addRequirements(this.swerveSubsystem);
        }

        @Override
        public void execute() {
                //swerveSubsystem.getSwerveDrive().setGyro(new Rotation3d(0, 0, swerveSubsystem.getSwerveDrive().getPitch().getRadians()));

                double xVelocity   = Math.pow(-RobotContainer.driverJoystick.getRawAxis(DriverConstants.forwardAxis), 3);
                double yVelocity   = Math.pow(-RobotContainer.driverJoystick.getRawAxis(DriverConstants.strafeAxis), 3);
                double angVelocity = Math.pow(-RobotContainer.driverJoystick.getRawAxis(DriverConstants.turnAxis), 3);

                swerveSubsystem.getSwerveDrive().drive(new Translation2d(xVelocity * DriverConstants.speedMetersPerSecond, yVelocity * DriverConstants.speedMetersPerSecond),
                 angVelocity * DriverConstants.speedRadiansPerSecond, true, false);

                /*double vX = -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.forwardAxis) * DriverConstants.speedMetersPerSecond;
                double vY = -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.strafeAxis) * DriverConstants.speedMetersPerSecond;
                double heading = -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.turnAxis);

                ChassisSpeeds desiredSpeeds = swerveSubsystem.getTargetSpeeds(vX, vY,
                                new Rotation2d(heading * Math.PI));

                Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
                
                //add in if robot is tipping
                 * translation = SwerveMath.limitVelocity(translation,
                 * swerve.getFieldVelocity(), swerve.getPose(),
                 * Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
                 * swerve.getSwerveDriveConfiguration());

                // Make the robot move
                swerveSubsystem.getSwerveDrive().drive(translation, desiredSpeeds.omegaRadiansPerSecond, true, false);
                */

                //robot relative drive, probably wont ever use
                /*
                 * swerveSubsystem.getSwerveDrive().drive(
                 * new Translation2d(-RobotContainer.driverJoystick.getRawAxis(Constants.
                 * DriverConstants.forwardAxis) * DriverConstants.speedMetersPerSecond,
                 * -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.
                 * strafeAxis) * DriverConstants.speedMetersPerSecond),
                 * -RobotContainer.driverJoystick.getRawAxis(Constants.DriverConstants.turnAxis)
                 * * DriverConstants.speedRadiansPerSecond,
                 * false,
                 * false
                 * );
                 */
        }
}
