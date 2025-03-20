// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.*;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax elevatorMotorLeft;
  private SparkMax elevatorMotorRight;
  public boolean servoOn = true;
  // Initialize the servo hub
  ServoHub m_servoHub = new ServoHub(2);
  double targetPosition = Constants.ElevatorConstants.ELEVATOR_L4_HEIGHT;

  // Obtain a servo channel controller
  ServoChannel servo = m_servoHub.getServoChannel(ChannelId.kChannelId5);
  public Elevator(SparkMaxConfig config) {    
    servo.setPowered(true);
    servo.setEnabled(true);
    servo.setPulseWidth(2500);
    
config.closedLoop
    .p(0.04)
    .i(0)
    .d(0)
    .outputRange(-1, 1);



    SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    elevatorFollowerConfig
        .smartCurrentLimit(40)
        .idleMode(IdleMode.kBrake).inverted(true);
    elevatorFollowerConfig.follow(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID);
    elevatorMotorLeft = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_LEFT_ID, MotorType.kBrushless);
    elevatorMotorRight = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
    elevatorMotorLeft.configure(elevatorFollowerConfig, ResetMode.kResetSafeParameters, null);
    elevatorMotorRight.configure(config, ResetMode.kResetSafeParameters, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(servoOn){
      setPosition(2500);
    }
    else{
      setPosition(500);
    }
    //System.out.println(targetPosition);
    SmartDashboard.putBoolean("Elevator Open", servoOn);
  }
  public void toggleServo() {
    servoOn = !servoOn;

  }

  public void setPosition(int position) {
    
    servo.setPulseWidth(position);

  }

public void setPower(double power) {
  elevatorMotorRight.set(power);
}  

  public void setElevatorPosition(double position) {
    elevatorMotorRight.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void setTargetElevatorPosition(double position){
    //System.out.println("changedPosition to: " + position);
    targetPosition = position;
  }

  public Command setElevatorPositionCommand(){
    return setElevatorPositionCommand(()->targetPosition);
  }

  public Command setElevatorPositionCommand(DoubleSupplier position) {
    return run(() -> 
     {//targetPosition = position;
      //System.out.println(targetPosition); 
      elevatorMotorRight.getClosedLoopController().setReference(position.getAsDouble(), ControlType.kPosition);}).withName("Elevator Set Positions");
  }

  public double getElevatorPosition(){
    return targetPosition;
  }
}
