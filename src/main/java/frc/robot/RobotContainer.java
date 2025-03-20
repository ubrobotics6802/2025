// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
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

import org.photonvision.PhotonCamera;

import swervelib.SwerveDrive;
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
  
  public PhotonCamera camera = new PhotonCamera("FrontCam");

  SendableChooser<Command> autoChooser;


  int requestedAngle = 0;

  private boolean fieldOriented = true;

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
  private final Trigger elevatorCollectButton = new Trigger(() -> operatorController.getRawButton(11));

  // Buttons for controlling the intake
  private final Trigger intakeInButton = new Trigger(() -> operatorController.getRawButton(6));
  private final Trigger intakeOutButton = new Trigger(() -> operatorController.getRawButton(13));

  // Buttons for controlling the wrist
  
  private final Trigger coralRightButton = new Trigger(() -> operatorController.getRawButton(19));
  private final Trigger strafeRightButton = new Trigger(() -> driverController.getRawButton(2));
  private final Trigger coralLeftButton = new Trigger(() -> operatorController.getRawButton(17));

  // Buttons for controlling ratchet mode
  private final Trigger ratchetCloseButton = new Trigger(() -> operatorController.getRawButton(18));

  private final Trigger strafeLeftButton = new Trigger(() -> driverController.getRawButton(1));

  private final Trigger driveModeButton = new Trigger(() -> driverController.getRawButton(4));
  private final Trigger robotModeButton = new Trigger(() -> driverController.getRawButton(3));

  Trigger frontReefButton = new Trigger(() -> operatorController.getRawButton(15));
  Trigger backReefButton = new Trigger(() -> operatorController.getRawButton(12));
  Trigger frontLeftReefButton = new Trigger(() -> operatorController.getRawButton(16));
  Trigger backLeftReefButton = new Trigger(() -> operatorController.getRawButton(4));
  Trigger frontRightReefButton = new Trigger(() -> operatorController.getRawButton(14));
  Trigger backRightReefButton = new Trigger(() -> operatorController.getRawButton(9));

  Trigger visionTestButton = new Trigger(() -> operatorController.getRawButton(10));

  Trigger algaeMode = new Trigger(this::triggerAlgae);





  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getRawAxis(rightY) * 1,
                                                                () -> driverController.getRawAxis(rightX) * -1)
                                                            .withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> driverController.getRawAxis(leftX) * -1,
  () -> driverController.getRawAxis(leftY))
                                                           .headingWhile(true);

SwerveInputStream driveDirectAngleToDestinationFront = driveAngularVelocity.copy().withControllerHeadingAxis(() -> 0, () -> 1)
                                                           .headingWhile(true);
SwerveInputStream driveDirectAngleToDestinationBack = driveAngularVelocity.copy().withControllerHeadingAxis(() -> 0, () -> -1)
                                                           .headingWhile(true);
                                                           
SwerveInputStream driveDirectAngleToDestinationFrontLeft = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -.866, () -> .5)
                                                           .headingWhile(true);
SwerveInputStream driveDirectAngleToDestinationBackLeft = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -.866, () -> -.5)
                                                           .headingWhile(true);
SwerveInputStream driveDirectAngleToDestinationFrontRight = driveAngularVelocity.copy().withControllerHeadingAxis(() -> .866, () -> .5)
                                                           .headingWhile(true);
                                                           
SwerveInputStream driveDirectAngleToDestinationBackRight = driveAngularVelocity.copy().withControllerHeadingAxis(() -> .866, () -> -.5)
                                                           .headingWhile(true);
                                                        

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverController.getRawAxis(rightY) * 1,
                                                                () -> driverController.getRawAxis(rightX) * -1)
                                                            .withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                          
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false)
                                                            .robotRelative(true);

    SwerveInputStream strafeRobotLeft = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverController.getRawAxis(rightY) * 1,
    () -> .2)
.withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(0.8)
.allianceRelativeControl(false)
.robotRelative(true);

SwerveInputStream strafeRobotRight = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> driverController.getRawAxis(rightY) * 1,
    () -> -.2)
.withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(0.8)
.allianceRelativeControl(false)
.robotRelative(true);

SwerveInputStream robotForward = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> -.3,
    () -> driverController.getRawAxis(rightX) * -1)
.withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(0.8)
.allianceRelativeControl(false)
.robotRelative(true);

SwerveInputStream robotBackward = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> .3,
    () -> driverController.getRawAxis(rightX) * -1)
.withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
.deadband(OperatorConstants.DEADBAND)
.scaleTranslation(0.8)
.allianceRelativeControl(false)
.robotRelative(true);
  
  

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

    
    NamedCommands.registerCommand("raiseElevator", elevator.setElevatorPositionCommand(()->15).alongWith(wrist.setPositionCommand(() -> Constants.WristConstants.WRIST_MAX_ANGLE)));
    NamedCommands.registerCommand("raisel4", Commands.parallel(elevator.setElevatorPositionCommand(()->Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT), wrist.setPositionCommand(()->Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE)));
    NamedCommands.registerCommand("scoreCoral", new StartEndCommand(() -> intake.setSpeed(elevator.getElevatorPosition() == Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT ? .5 : .5), () ->wrist.setPosition(Constants.WristConstants.WRIST_MAX_ANGLE)));
    NamedCommands.registerCommand("centerWheels", drivebase.centerModulesCommand());
    
    
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);

    
    // AbsoluteDriveAdv closAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase, driverController.getRawAxis(1), driverController.getRawAxis(0), driverController.getRawAxis(3), null, null, null, null, null)
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
        
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  public double getYHeadingValue(){
    double value = 0;
    if(requestedAngle == 0){
      value = 1;
    } else if(requestedAngle == 60){
      value = .866;
    } else if(requestedAngle == 180){
      value = -1;
    } else if(requestedAngle == 120){
      value = .866;
    } else if(requestedAngle == 240){
      value = -.866;
    } else if(requestedAngle == 300){
      value = .866;
    }
    return value;
  }
  
  public double getXHeadingValue(){
    double value = 0;
    if(requestedAngle == 0){
      value = 0;
    } else if(requestedAngle == 60){
      value = .5;
    } else if(requestedAngle == 180){
      value = 0;
    } else if(requestedAngle == 120){
      value = -.5;
    } else if(requestedAngle == 240){
      value = -.5;
    } else if(requestedAngle == 300){
      value = -.5;
    }
    return value;
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

    //Setup drive commands
    Command driveDestinationAngleFront = drivebase.driveFieldOriented(driveDirectAngleToDestinationFront);
    Command driveDestinationAngleBack = drivebase.driveFieldOriented(driveDirectAngleToDestinationBack);
    Command driveDestinationAngleFrontLeft = drivebase.driveFieldOriented(driveDirectAngleToDestinationFrontLeft);
    Command driveDestinationAngleBackLeft = drivebase.driveFieldOriented(driveDirectAngleToDestinationBackLeft);
    Command driveDestinationAngleFrontRight = drivebase.driveFieldOriented(driveDirectAngleToDestinationFrontRight);
    Command driveDestinationAngleBackRight = drivebase.driveFieldOriented(driveDirectAngleToDestinationBackRight);
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command strafeLeftCommand = drivebase.driveFieldOriented(strafeRobotLeft);
    Command strafeRightCommand = drivebase.driveFieldOriented(strafeRobotRight);
    Command driveStraightCommand = drivebase.driveFieldOriented(robotForward);
    Command driveStraightCommandAuto = drivebase.driveFieldOriented(robotForward);
    Command driveStraightCommandRight = drivebase.driveFieldOriented(robotForward);
    Command driveBackwardCommand = drivebase.driveFieldOriented(robotBackward);

    // NamedCommands.registerCommand("LeftAutoAlign", strafeLeftCommand.until(drivebase::leftLidarClear)
    // .andThen(driveStraightCommandAuto.withTimeout(.3))
    // .andThen(getManipulatorScoringCommand().withTimeout(1))
    // .andThen(driveBackwardCommand.withTimeout(.5))
    // .andThen(new InstantCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_OUT_SLOW))));

    frontLeftReefButton.onTrue((driveDestinationAngleFrontLeft));
    frontRightReefButton.onTrue((driveDestinationAngleFrontRight));
    backRightReefButton.onTrue((driveDestinationAngleBackRight));
    backLeftReefButton.onTrue((driveDestinationAngleBackLeft));
    backReefButton.onTrue((driveDestinationAngleBack));
    frontReefButton.onTrue((driveDestinationAngleFront));    
    coralLeftButton.onTrue(driveDestinationAngleBackRight);
    coralRightButton.onTrue(driveDestinationAngleBackLeft);

    driveModeButton.onTrue(driveFieldOrientedDirectAngle);
    robotModeButton.onTrue(driveRobotOrientedAngularVelocity);

    /**
     * Elevator Button Flow:
     * 1. Pushing cooresponding branch button will set the elevator's target height (setTargetPosition(position)).
     * 2. Elevator stays in its current position until told to transition to target height (setElevatorPositionCommand())
     * 3. If instant transition is needed in the case of reseting the height or an emergency, setElevatorPositionCommand(height) can still be used
     */

    // elevatorL4Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE)));
    // elevatorL3Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE)));
    // elevatorL2Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE)));
    // elevatorL1Button.onTrue(Commands.parallel(elevator.setElevatorPositionCommand(Constants.ElevatorConstants.ELEVATOR_L1_HEIGHT), wrist.setPositionCommand(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE)));
    
    elevatorL4Button.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE);}));
    elevatorL3Button.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE);}));
    elevatorL2Button.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE);}));
    elevatorL1Button.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L1_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE);}));
    elevatorCollectButton.onTrue(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_COLLECT_HEIGHT, Constants.WristConstants.WRIST_COLLECT_ANGLE));
    
    //Intake Buttons
    intakeInButton.whileTrue(new StartEndCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_IN_SPEED), () -> intake.setSpeed(0), intake).until(intake::shouldStop));
    intakeInButton.onTrue(wrist.setPositionCommand(()->Constants.WristConstants.WRIST_COLLECT_ANGLE));
    intakeInButton.onFalse(wrist.setPositionCommand(()->Constants.WristConstants.WRIST_MAX_ANGLE));
    intakeOutButton.whileTrue(new StartEndCommand(() -> intake.setSpeed(elevator.getElevatorPosition() == Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT ? Constants.IntakeConstants.INTAKE_OUT_SLOW : Constants.IntakeConstants.INTAKE_OUT_SPEED), () -> intake.setSpeed(0), intake));
    intakeOutButton.onFalse(wrist.setPositionCommand(()->Constants.WristConstants.WRIST_MAX_ANGLE));
    algaeMode.onTrue(new InstantCommand(()-> {wrist.setAlgaeMode(true); intake.setAlgaeMode(true);}));
    algaeMode.onFalse(new InstantCommand(()-> {wrist.setAlgaeMode(false); intake.setAlgaeMode(false);}));

    //Climbing Buttons    
    ratchetCloseButton.whileTrue(new InstantCommand(()-> elevator.toggleServo(), elevator));
    visionTestButton.whileTrue(new StartEndCommand(() -> {elevator.setPower(Constants.ElevatorConstants.ELEVATOR_CLIMB_BUTTON_POWER); wrist.setPosition(Constants.WristConstants.WRIST_MAX_ANGLE);}, () -> elevator.setPower(0), elevator));


    //Auto Score Buttons
    strafeLeftButton.onTrue(strafeLeftCommand.until(drivebase::leftLidarClear).andThen(driveStraightCommand.withTimeout(.3)).andThen(getManipulatorScoringCommand().withTimeout(.5)));
    strafeRightButton.onTrue(strafeRightCommand.until(drivebase::rightLidarClear).andThen(drivebase.centerModulesCommand().withTimeout(.5)).andThen(driveStraightCommandRight.withTimeout(.4)).andThen(getManipulatorScoringCommand().withTimeout(.5)));
    //strafeRightButton.onTrue(drivebase.centerModulesCommand());
    if (RobotBase.isSimulation())
    {
     // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
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

  public Command getManipulatorScoringCommand(double height, double angle){
    return Commands.parallel(elevator.setElevatorPositionCommand(()->height), wrist.setPositionCommand(()->angle));
  }
  
  public Command getManipulatorScoringCommand(){
    return Commands.parallel(drivebase.centerModulesCommand(), elevator.setElevatorPositionCommand(), wrist.setPositionCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public boolean triggerAlgae(){
    return driverController.getRawAxis(5) > 0;
  }

  // public void setMotorBrake(boolean brake)
  // {
  //   drivebase.setMotorBrake(brake);
  // }
}
