// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);
  private final int leftX = 3;
  private final int leftY = 2;
  private final int rightX = 0;
  private final int rightY = 1;
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
                                                                                
  private final Elevator elevator;
  private final Intake intake;
  private final Wrist wrist;

  // Buttons for controlling the elevator
  
  private final Trigger elevatorL4Button = new Trigger(() -> operatorController.getRawButton(8));
  private final Trigger elevatorL3Button = new Trigger(() -> operatorController.getRawButton(5));
  private final Trigger elevatorL2Button = new Trigger(() -> operatorController.getRawButton(20));
  private final Trigger elevatorL1Button = new Trigger(() -> operatorController.getRawButton(7));
  private final Trigger elevatorDownButton = new Trigger(() -> operatorController.getRawButton(10));
  private final Trigger elevatorCollectButton = new Trigger(() -> operatorController.getRawButton(11));

  // Buttons for controlling the intake
  private final Trigger intakeInButton = new Trigger(() -> operatorController.getRawButton(6));
  private final Trigger intakeOutButton = new Trigger(() -> operatorController.getRawButton(13));

  // Buttons for controlling the wrist
  
  private final Trigger wristCollectButton = new Trigger(() -> operatorController.getRawButton(19));
  private final Trigger wristUpButton = new Trigger(() -> operatorController.getRawButton(50));
  private final Trigger wristScoreButton = new Trigger(() -> operatorController.getRawButton(17));

  // Buttons for controlling ratchet mode
  private final Trigger ratchetOpenButton = new Trigger(() -> operatorController.getRawButton(12));
  private final Trigger ratchetCloseButton = new Trigger(() -> operatorController.getRawButton(18));

  private final Trigger climbButton = new Trigger(() -> driverController.getRawButton(1));
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getRawAxis(rightY) * -1,
                                                                () -> driverController.getRawAxis(rightX) * 1)
                                                            .withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverController.getRawAxis(leftX),
  () -> driverController.getRawAxis(leftY) * -1)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake).inverted(true);



    elevator = new Elevator(elevatorConfig);
    intake = new Intake(intakeConfig);
    wrist = new Wrist(config);
    // AbsoluteDriveAdv closAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase, driverController.getRawAxis(1), driverController.getRawAxis(0), driverController.getRawAxis(3), null, null, null, null, null)
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    elevatorL4Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE)));
    elevatorL3Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE)));
    elevatorL2Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE)));
    elevatorL1Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L1_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE)));
    elevatorCollectButton.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_COLLECT_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_COLLECT_ANGLE)));
    elevatorDownButton.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(0), wrist.setPositionCommand(Constants.WristConstants.WRIST_MAX_ANGLE)));
    
    intakeInButton.whileTrue(new StartEndCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_IN_SPEED), () -> intake.setSpeed(0), intake));
    intakeOutButton.whileTrue(new StartEndCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_OUT_SPEED), () -> intake.setSpeed(0), intake));
    intakeOutButton.onFalse(wrist.setPositionCommand(Constants.WristConstants.WRIST_MAX_ANGLE));
    
    //ratchetOpenButton.whileTrue(new InstantCommand(()-> elevator.setPosition(2500), elevator));
    ratchetCloseButton.whileTrue(new InstantCommand(()-> elevator.toggleServo(), elevator));
    climbButton.whileTrue(new StartEndCommand(() -> elevator.setPower(Constants.ElevatorConstants.ELEVATOR_CLIMB_BUTTON_POWER), () -> elevator.setPower(0), elevator));

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);

    if (RobotBase.isSimulation())
    {
     // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    if (Robot.isSimulation())
    {
      // driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      // driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().onTrue(Commands.none());
      // driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      // driverXbox.start().whileTrue(Commands.none());
      // driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      // driverXbox.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand()
  // {
  //   // An example command will be run in autonomous
  //   //return drivebase.getAutonomousCommand("New Auto");
  // }

  // public void setMotorBrake(boolean brake)
  // {
  //   drivebase.setMotorBrake(brake);
  // }
}
