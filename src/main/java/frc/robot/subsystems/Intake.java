// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  boolean intakeRan = true;
  private SparkMax intakeMotorLeft;
  private SparkMax intakeMotorRight;
  double speed = 0;
  public Intake(SparkMaxConfig config) {
    intakeMotorLeft = new SparkMax(IntakeConstants.INTAKE_MOTOR_LEFT_ID, MotorType.kBrushless);
    intakeMotorRight = new SparkMax(IntakeConstants.INTAKE_MOTOR_RIGHT_ID, MotorType.kBrushless);
    intakeMotorLeft.configure(config, ResetMode.kResetSafeParameters, null);
    intakeMotorRight.configure(config, ResetMode.kResetSafeParameters, null); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run    
    intakeMotorLeft.set(speed);
    intakeMotorRight.set(-speed);
  }
  public void setSpeed(double speed) {
    if(speed == Constants.IntakeConstants.INTAKE_IN_SPEED){
      intakeRan = true;
    }
    else{
      intakeRan = false;
    }
    this.speed = speed;
  }

  public void stopIntake(){
    speed = 0;
  }

  public Command setSpeedCommand(double speed){
    return run(()-> setSpeed(speed));
  }

  public boolean getIntakeRan(){
    return intakeRan;
  }
}
