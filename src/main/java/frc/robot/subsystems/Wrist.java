// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
 private SparkMax wristMotor;
 public double position = Constants.WristConstants.WRIST_MAX_ANGLE;
  public Wrist(SparkMaxConfig config) {
    config.inverted(true);
    config.absoluteEncoder.positionConversionFactor(6);
    config.closedLoop
    .p(0.6)
    .i(0)
    .d(0)
    .outputRange(-1, 1).feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristMotor = new SparkMax(WristConstants.WRIST_MOTOR_ID, MotorType.kBrushless);
    wristMotor.configure(config, ResetMode.kResetSafeParameters, null);
  }

  public void periodic() {
    // This method will be called once per scheduler run
    wristMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }
  public void setPosition(double position) {
    this.position = position;
  }

  public Command setPositionCommand(double position) {
    return run(() -> setPosition(position)).withName("Wrist SetPosition");
  }
}
