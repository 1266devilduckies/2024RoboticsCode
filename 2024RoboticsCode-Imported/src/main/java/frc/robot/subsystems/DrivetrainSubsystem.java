package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleopSwerve;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DrivetrainSubsystem extends SubsystemBase {
        private final double maximumSpeed = Units.feetToMeters(14.5);
        private File swerveJsonDirectory;
        private SwerveDrive swerveDrive;

        public DrivetrainSubsystem(RobotContainer robotContainer) {
                this.swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");

                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.MACHINE;
                try {
                        this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
                } catch (IOException e) {
                        e.printStackTrace();
                }
                swerveDrive.setHeadingCorrection(false);
                this.setDefaultCommand(new TeleopSwerve(this));
        }

        @Override
        public void periodic() {

                // This method will be called once per scheduler run
                SmartDashboard.putNumber(swerveDrive.getModules()[0].configuration.name,
                                swerveDrive.getModules()[0].getAbsolutePosition());
                SmartDashboard.putNumber(swerveDrive.getModules()[3].configuration.name,
                                swerveDrive.getModules()[3].getAbsolutePosition());
                SmartDashboard.putNumber(swerveDrive.getModules()[1].configuration.name,
                                swerveDrive.getModules()[1].getAbsolutePosition());
                SmartDashboard.putNumber(swerveDrive.getModules()[2].configuration.name,
                                swerveDrive.getModules()[2].getAbsolutePosition());

        }

        /**
         * Setup AutoBuilder for PathPlanner.
         */
        public void setupPathPlanner() {
                AutoBuilder.configureHolonomic(
                                this::getPose, // Robot pose supplier
                                this::resetOdometry, // Method to reset odometry (will be called if your auto has a
                                                     // starting pose)
                                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                                                        // ChassisSpeeds
                                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                                                                 // in your Constants class
                                                new PIDConstants(5.0, 0.0, 0.0),
                                                // Translation PID constants
                                                new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p,
                                                                swerveDrive.swerveController.config.headingPIDF.i,
                                                                swerveDrive.swerveController.config.headingPIDF.d),
                                                // Rotation PID constants
                                                4.5,
                                                // Max module speed, in m/s
                                                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                                // Drive base radius in meters. Distance from robot center to furthest
                                                // module.
                                                new ReplanningConfig()
                                // Default path replanning config. See the API for the options here
                                ),
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                                        var alliance = DriverStation.getAlliance();
                                        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red
                                                        : false;
                                },
                                this // Reference to this subsystem to set requirements
                );
        }

        public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
                // Load the path you want to follow using its name in the GUI
                PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

                if (setOdomToStart) {
                        resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
                }

                // Create a path following command using AutoBuilder. This will also trigger
                // event markers.
                return AutoBuilder.followPath(path);
        }

        /**
         * Get the chassis speeds based on controller input of 2 joysticks. One for
         * speeds in which direction. The other for
         * the angle of the robot.
         *
         * @param xInput   X joystick input for the robot to move in the X direction.
         * @param yInput   Y joystick input for the robot to move in the Y direction.
         * @param headingX X joystick which controls the angle of the robot.
         * @param headingY Y joystick which controls the angle of the robot.
         * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
         */
        public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
                xInput = Math.pow(xInput, 3);
                yInput = Math.pow(yInput, 3);
                return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                yInput,
                                headingX,
                                headingY,
                                getHeading().getRadians(),
                                maximumSpeed);
        }

        /**
         * Get the chassis speeds based on controller input of 1 joystick and one angle.
         *
         * @param xInput X joystick input for the robot to move in the X direction.
         * @param yInput Y joystick input for the robot to move in the Y direction.
         * @param angle  The angle in as a {@link Rotation2d}.
         * @return {@link ChassisSpeeds} which can be sent to th Swerve Drive.
         */
        public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
                xInput = Math.pow(xInput, 3);
                yInput = Math.pow(yInput, 3);
                return swerveDrive.swerveController.getTargetSpeeds(xInput,
                                yInput,
                                angle.getRadians(),
                                getHeading().getRadians(),
                                maximumSpeed);
        }

        /**
         * Gets the current yaw angle of the robot, as reported by the imu. CCW
         * positive, not wrapped.
         *
         * @return The yaw angle
         */
        public Rotation2d getHeading() {
                return swerveDrive.getYaw();
        }

        public Pose2d getPose() {
                return swerveDrive.getPose();
        }

        public ChassisSpeeds getRobotVelocity() {
                return swerveDrive.getRobotVelocity();
        }

        public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
                swerveDrive.setChassisSpeeds(chassisSpeeds);
        }

        public void zeroGyro() {
                swerveDrive.zeroGyro();
        }

        public void resetOdometry(Pose2d initialHolonomicPose) {
                swerveDrive.resetOdometry(initialHolonomicPose);
        }

        public SwerveDrive getSwerveDrive() {
                return swerveDrive;
        }
}