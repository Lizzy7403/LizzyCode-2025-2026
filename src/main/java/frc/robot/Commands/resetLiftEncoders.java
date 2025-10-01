// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class resetLiftEncoders extends Command {

  private final Lift lift;



  /** Creates a new Up. */
  public resetLiftEncoders(Lift lift) {
    this.lift = lift;
      

  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

   lift.resetLiftEncoder();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
     
  }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }

 // This command completes immediately after setting the speed
 
}

