// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class Lift extends SubsystemBase {

  private static final int motorLift1ID = 19;
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  private SparkMax m_leadMotor = new SparkMax(motorLift1ID, MotorType.kBrushless);

  private SparkClosedLoopController maxPid = m_leadMotor.getClosedLoopController();
  private final RelativeEncoder liftEncoder;
  private SparkMaxConfig config1 = new SparkMaxConfig();
  private double range=1;
 




  
  /** Creates a new Lift. */
  public Lift() {

    liftEncoder = m_leadMotor.getEncoder();
  

  

    //config for the leadMotor.

config1
    .idleMode(IdleMode.kBrake);
    config1.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1000);
config1.closedLoop
    .pid(0.2, 0, 0.08).outputRange(-0.6,range);

config1.softLimit.forwardSoftLimitEnabled(true);
config1.softLimit.reverseSoftLimitEnabled(true);
config1.softLimit.forwardSoftLimit(165);
config1.softLimit.reverseSoftLimit(0);



m_leadMotor.configure(config1, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);




  }

  public void run(double speed){

    m_leadMotor.set(speed);

  }

  public void setPosition(double position,double speed) {
    config1.closedLoop
    .pid(2, 0, 0.0).outputRange(-speed,speed);
    maxPid.setReference(position, ControlType.kPosition);
  }



  public double getLiftEncoderPostion(){

    return liftEncoder.getPosition();
  }

  public void resetLiftEncoder() {
    liftEncoder.setPosition(0);
  }


  public void stop(){
    m_leadMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Encoder Lift", getLiftEncoderPostion());
    SmartDashboard.putNumber("Motor speed", m_leadMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Limit Switch", bottomlimitSwitch.get());
    if(bottomlimitSwitch.get()==false){
      resetLiftEncoder();
    }
    /* 
    if (m_leadMotor.getReverseLimitSwitch().isPressed()) {
      resetLiftEncoder();
      
      SmartDashboard.putString("Limit Switch", "Limit switch activated");
    } else {
      SmartDashboard.putString("Limit Switch", "limit switch not activated");
    */
  }
}
