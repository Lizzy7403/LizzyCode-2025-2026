// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class Climb extends SubsystemBase {

  private static final int motorClimbID = 15;


  private SparkMax m_climbMotor = new SparkMax(motorClimbID, MotorType.kBrushless);

  private SparkClosedLoopController maxPid = m_climbMotor.getClosedLoopController();
  private final RelativeEncoder climbEncoder;
  private SparkMaxConfig config1 = new SparkMaxConfig();
  private double range=0.2;
 




  
  /** Creates a new Lift. */
  public Climb() {

    climbEncoder = m_climbMotor.getEncoder();
  

  

    //config for the leadMotor.

config1
    .idleMode(IdleMode.kBrake);
    config1.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1000);
config1.closedLoop
    .pid(0.2, 0, 0.08).outputRange(-range,range);




    
m_climbMotor.configure(config1, ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);






  }

  public void run(double speed){

    m_climbMotor.set(speed);

  }

  public void setPosition(double position,double speed) {
    config1.closedLoop
    .pid(0.2, 0, 0.08).outputRange(-speed,speed);
    maxPid.setReference(position, ControlType.kPosition);
  }



  public double getClimbEncoderPostion(){

    return climbEncoder.getPosition();
  }

  public void resetLiftEncoder() {
    climbEncoder.setPosition(0);
  }


  public void stop(){

    m_climbMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Climb Encoder", climbEncoder.getPosition());
    SmartDashboard.putNumber("Climb Motor speed", m_climbMotor.getAppliedOutput());
  }
}
