// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

 package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
public class Dampner extends Command {

  
  private double speedMultiple;
  private double initialSpeed;

  public Dampner(double speedMultiple) {
    this.speedMultiple = speedMultiple; 

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialSpeed=RobotContainer.MaxSpeed;
    RobotContainer.MaxSpeed /= speedMultiple;
    SmartDashboard.putNumber("MaxSpeed", RobotContainer.MaxSpeed);
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    if(interrupted){
      RobotContainer.MaxSpeed=initialSpeed;
  }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }
 // This command completes immediately after setting the speed
  }
