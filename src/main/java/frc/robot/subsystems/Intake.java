// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SharpIR;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  boolean shouldStop = false;
  Timer timer = new Timer();
  boolean algaeMode = false;
  
  private SparkMax intakeMotorLeft;
  private SparkMax intakeMotorRight;
  public SharpIR sharp = SharpIR.GP2Y0A21YK0F(0);

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
    //System.out.println(sensor.getColor());

  }
  public void setSpeed(double speed) {
        if(algaeMode){
          if(speed == Constants.IntakeConstants.INTAKE_OUT_SPEED){
            this.speed = 1;
          }
          else{
            this.speed = speed;
          }
        }else{
          this.speed = speed;
        }
        
  }

  public void setAlgaeMode(boolean mode){
    algaeMode = mode;
  }

  public boolean shouldStop(){
    double value = sharp.getRangeInches();
    if(!shouldStop)
    if(this.speed == Constants.IntakeConstants.INTAKE_IN_SPEED && value < 4 && !shouldStop && !algaeMode){
      shouldStop = true;
      timer.start();
    }

    if(shouldStop && timer.hasElapsed(.5)){
      
      shouldStop = false;
      timer.stop();
      timer.reset();

      if(value > 4){
        return false;
      }
      return true;
    }
    else{
      return false;
    }
  }

  public void stopIntake(){
    speed = 0;
  }

  public Command setSpeedCommand(double speed){
    return run(()-> setSpeed(speed));
  }
}
