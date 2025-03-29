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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  public final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
                                                                                
  public final Elevator elevator;
  private final Intake intake;
  private final Wrist wrist;
  

  // Buttons for controlling the elevator
  
 

  private final Trigger elevatorL4Button = new Trigger(() -> operatorController.getRawButton(8));
  private final Trigger elevatorL3Button = new Trigger(() -> operatorController.getRawButton(5));
  private final Trigger elevatorL2Button = new Trigger(() -> operatorController.getRawButton(20));
  private final Trigger elevatorL1Button = new Trigger(() -> operatorController.getRawButton(7));
  
  Trigger algaeMode = new Trigger(this::triggerAlgae);
  Trigger notAlgaeMode = new Trigger(this::notTriggerAlgae);

  Trigger algaeL4 = new Trigger(this::triggerAlgae).and(()->elevatorL4Button.getAsBoolean());
  Trigger algaeL3 = new Trigger(this::triggerAlgae).and(()->elevatorL3Button.getAsBoolean());
  Trigger algaeL2 = new Trigger(this::triggerAlgae).and(()->elevatorL2Button.getAsBoolean());
  Trigger algaeFloor = new Trigger(this::triggerAlgae).and(()->elevatorL1Button.getAsBoolean());  

  private final Trigger coralL4 = new Trigger(()-> elevatorL4Button.getAsBoolean()).and(this::notTriggerAlgae);
  private final Trigger coralL3 = new Trigger(()-> elevatorL3Button.getAsBoolean()).and(this::notTriggerAlgae);
  private final Trigger coralL2 = new Trigger(()-> elevatorL2Button.getAsBoolean()).and(this::notTriggerAlgae);
  private final Trigger coralL1 = new Trigger(()-> elevatorL1Button.getAsBoolean()).and(this::notTriggerAlgae);

  private final Trigger elevatorCollectButton = new Trigger(() -> operatorController.getRawButton(11));

  // Buttons for controlling the intake
  private final Trigger intakeInButton = new Trigger(() -> operatorController.getRawButton(6));
  private final Trigger intakeOutButton = new Trigger(() -> operatorController.getRawButton(13));

  // Buttons for controlling the wrist
  
  private final Trigger coralRightButton = new Trigger(() -> operatorController.getRawButton(19));

  
  private final Trigger strafeLeftButton = new Trigger(() -> driverController.getRawButton(1));
  private final Trigger strafeRightButton = new Trigger(() -> driverController.getRawButton(2));

  private final Trigger lidarStrafeLeft = strafeLeftButton.and(this::triggerLidar);
  private final Trigger lidarStafeRight = strafeRightButton.and(this::triggerLidar);

  private final Trigger tagStrafeLeft = strafeLeftButton.and(this::triggerAprilTags);
  private final Trigger tagStrafeRight = strafeRightButton.and(this::triggerAprilTags);

  private final Trigger coralLeftButton = new Trigger(() -> operatorController.getRawButton(17));

  // Buttons for controlling ratchet mode
  private final Trigger ratchetCloseButton = new Trigger(() -> operatorController.getRawButton(18));


  private final Trigger driveModeButton = new Trigger(() -> driverController.getRawButton(4));
  private final Trigger robotModeButton = new Trigger(() -> driverController.getRawButton(3));

  Trigger frontReefButton = new Trigger(() -> operatorController.getRawButton(15));
  Trigger backReefButton = new Trigger(() -> operatorController.getRawButton(12));
  Trigger frontLeftReefButton = new Trigger(() -> operatorController.getRawButton(16));
  Trigger backLeftReefButton = new Trigger(() -> operatorController.getRawButton(4));
  Trigger frontRightReefButton = new Trigger(() -> operatorController.getRawButton(14));
  Trigger backRightReefButton = new Trigger(() -> operatorController.getRawButton(9));

  Trigger visionTestButton = new Trigger(() -> operatorController.getRawButton(10));

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

    SwerveInputStream driveRobotOrientedAprilTag = SwerveInputStream.of(drivebase.getSwerveDrive(),
    () -> drivebase.getXValueForTag(),
    () -> drivebase.getYValueForTag())
.withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
.deadband(.01)

.scaleTranslation(0.8)
.allianceRelativeControl(false)
.robotRelative(true);



  //Right is negative left is positive
    public SwerveInputStream getHorizonatalInputStream(double speed){
      return SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> driverController.getRawAxis(rightY) * 1,
      () -> speed)
    .withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(false)
    .robotRelative(true);
  }
  public SwerveInputStream getVerticalInputStream(double speed){
    return SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> speed,
      () -> driverController.getRawAxis(rightX) * -1)
  .withControllerRotationAxis(()-> driverController.getRawAxis(leftX) * -1)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(0.8)
  .allianceRelativeControl(false)
  .robotRelative(true);
  }

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

    
    NamedCommands.registerCommand("raiseElevator", getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_COLLECT_HEIGHT, Constants.WristConstants.WRIST_MAX_ANGLE));
    NamedCommands.registerCommand("raisel4", Commands.parallel(elevator.setElevatorPositionCommand(()->Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT), wrist.setPositionCommand(()->Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE)));
    NamedCommands.registerCommand("scoreCoral", new StartEndCommand(() -> intake.setSpeed(elevator.getElevatorPosition() == Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT ? .5 : .5), () ->wrist.setPosition(Constants.WristConstants.WRIST_MAX_ANGLE)));
    NamedCommands.registerCommand("centerWheels", drivebase.centerModulesCommand());
    NamedCommands.registerCommand("engageServo", new InstantCommand(()-> elevator.setPosition(2500)));
    NamedCommands.registerCommand("autoIntake", new StartEndCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_IN_SPEED), () -> intake.setSpeed(0), intake).until(intake::shouldStop));
    
    
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

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    //Setup strafe buttons
    strafeLeftButton.onTrue(new InstantCommand(()-> drivebase.isLeft = true));
    strafeRightButton.onTrue(new InstantCommand(()-> drivebase.isLeft = false));
    //Setup drive commands
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);

    frontLeftReefButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationFrontLeft));
    frontRightReefButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationFrontRight));
    backRightReefButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationBackRight));
    backLeftReefButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationBackLeft));
    backReefButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationBack));
    frontReefButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationFront));    
    coralLeftButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationBackRight));
    coralRightButton.onTrue(drivebase.driveFieldOriented(driveDirectAngleToDestinationBackLeft));

    driveModeButton.onTrue(driveFieldOrientedAnglularVelocity);
    robotModeButton.onTrue(driveRobotOrientedAngularVelocity);

    /**
     * Elevator Button Flow:
     * 1. Pushing cooresponding branch button will set the elevator's target height (setTargetPosition(position)).
     * 2. Elevator stays in its current position until told to transition to target height (setElevatorPositionCommand())
     * 3. If instant transition is needed in the case of reseting the height or an emergency, setElevatorPositionCommand(height) can still be used
     */
    
    coralL4.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE);}));
    coralL3.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE);}));
    coralL2.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE);}));
    coralL1.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L1_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE);}));
    elevatorCollectButton.onTrue(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_COLLECT_HEIGHT, Constants.WristConstants.WRIST_COLLECT_ANGLE));
    
    //Intake Buttons
    intakeInButton.whileTrue(new StartEndCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_IN_SPEED), () -> intake.setSpeed(0), intake).until(intake::shouldStop));
    intakeInButton.onTrue(wrist.setPositionCommand(()->Constants.WristConstants.WRIST_COLLECT_ANGLE));
    intakeInButton.onFalse(wrist.setPositionCommand(()->Constants.WristConstants.WRIST_MAX_ANGLE));
    intakeOutButton.whileTrue(new StartEndCommand(() -> intake.setSpeed(elevator.getElevatorPosition() == Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT ? Constants.IntakeConstants.INTAKE_OUT_SLOW : Constants.IntakeConstants.INTAKE_OUT_SPEED), () -> intake.setSpeed(0), intake));
    intakeOutButton.onFalse(wrist.setPositionCommand(()->Constants.WristConstants.WRIST_MAX_ANGLE));
    //algaeMode.onTrue(new InstantCommand(()-> {wrist.setAlgaeMode(true); intake.setAlgaeMode(true);}));
    //algaeMode.onFalse(new InstantCommand(()-> {wrist.setAlgaeMode(false); intake.setAlgaeMode(false);}));

     //algaeL3.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE);}));
    // algaeL2.onTrue(new InstantCommand(()-> {elevator.setTargetElevatorPosition(Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT); wrist.setPosition(Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE);}));
    algaeFloor.onTrue(getManipulatorScoringCommand(0, Constants.WristConstants.WRIST_LOWER_SCORING_ANGLE));
    algaeL2.onTrue(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT - 5, Constants.WristConstants.WRIST_ALGAE_COLLECT_ANGLE));
    algaeL3.onTrue(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_L3_HEIGHT - 5, Constants.WristConstants.WRIST_ALGAE_COLLECT_ANGLE));
    algaeL4.onTrue(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT, Constants.WristConstants.WRIST_BARGE_ANGLE));

    //Climbing Buttons    
    ratchetCloseButton.whileTrue(elevator.setElevatorPositionCommand(()->Constants.ElevatorConstants.ELEVATOR_L2_HEIGHT));
    ratchetCloseButton.onFalse(new InstantCommand(()-> elevator.toggleServo()));
    visionTestButton.whileTrue(new StartEndCommand(() -> {elevator.setPower(Constants.ElevatorConstants.ELEVATOR_CLIMB_BUTTON_POWER); wrist.setPosition(Constants.WristConstants.WRIST_MAX_ANGLE);}, () -> elevator.setPower(0), elevator));

    NamedCommands.registerCommand("LeftAutoAlign", (drivebase.driveFieldOriented(driveRobotOrientedAprilTag)
       .alongWith(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT, Constants.WristConstants.WRIST_HIGHER_SCORING_ANGLE)))
     .andThen(drivebase.driveFieldOriented(getVerticalInputStream(.3)).withTimeout(.5))
     .andThen(new RunCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_OUT_SPEED)).withTimeout(.5))
     .andThen(wrist.setPositionCommand(()-> Constants.WristConstants.WRIST_MAX_ANGLE).withTimeout(.5))
     .andThen(getManipulatorScoringCommand(Constants.ElevatorConstants.ELEVATOR_COLLECT_HEIGHT, Constants.WristConstants.WRIST_COLLECT_ANGLE).withTimeout(.25))
     .andThen(new InstantCommand(() -> intake.setSpeed(Constants.IntakeConstants.INTAKE_IN_SPEED)).withTimeout(.1)));

    //Auto Score Buttons
    lidarStrafeLeft.onTrue(drivebase.driveFieldOriented(getHorizonatalInputStream(.2)).until(drivebase::leftLidarClear)
    .andThen(drivebase.driveFieldOriented(getVerticalInputStream(-.3)).withTimeout(.3))
    .andThen(getManipulatorScoringCommand().withTimeout(.5)));
    
    lidarStafeRight.onTrue(drivebase.driveFieldOriented(getHorizonatalInputStream(-.2)).until(drivebase::rightLidarClear)
    .andThen(drivebase.driveFieldOriented(getVerticalInputStream(-.3)).withTimeout(.4))
    .andThen(getManipulatorScoringCommand().withTimeout(.5)));

    tagStrafeLeft.onTrue(drivebase.driveFieldOriented(driveRobotOrientedAprilTag)
    .alongWith(getManipulatorScoringCommand()).until(drivebase::hasStopped));
    tagStrafeRight.onTrue(drivebase.driveFieldOriented(driveRobotOrientedAprilTag)
    .alongWith(getManipulatorScoringCommand()).until(drivebase::hasStopped));

    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);
  }

  public Command getManipulatorScoringCommand(double height, double angle){
    return Commands.parallel(elevator.setElevatorPositionCommand(()->height), wrist.setPositionCommand(()->angle));
  }
  
  public Command getManipulatorScoringCommand(){
    return Commands.parallel(elevator.setElevatorPositionCommand(), wrist.setPositionCommand());
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
    boolean algae = driverController.getRawAxis(5) > 0;
    SmartDashboard.putBoolean("Algae Mode", algae);
    return algae;
  }

  public boolean triggerLidar(){
    return driverController.getRawAxis(7) > 0;
  }

  public boolean triggerAprilTags(){
    return driverController.getRawAxis(7) < 0;
  }

  public boolean notTriggerAlgae(){
    return driverController.getRawAxis(5) < .5;
  }

  // public void setMotorBrake(boolean brake)
  // {
  //   drivebase.setMotorBrake(brake);
  // }
}
