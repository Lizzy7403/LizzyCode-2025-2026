// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import commands classes
import frc.robot.Commands.AngletoPos;
import frc.robot.Commands.ClimbManualDown;
import frc.robot.Commands.ClimbManualUp;
import frc.robot.Commands.Dampner;
import frc.robot.Commands.LiftDownManual;
import frc.robot.Commands.LiftUpManual;
import frc.robot.Commands.LifttoPos;
import frc.robot.Commands.resetLiftEncoders;

//import frc.robot.Commands.Position;
import frc.robot.Commands.coralHandler;
import frc.robot.Commands.grabCoral;
import frc.robot.Commands.intakeManual;
import frc.robot.Commands.coralShooter;
import frc.robot.Commands.resetIntakeEncoder;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.Climb;
import frc.robot.Commands.maintainCoral;


import frc.robot.Constants.SwerveConstants;





public class RobotContainer {
    
    
    
    private final SendableChooser<Command> autoChooser;

    

    public static double MaxSpeed = Constants.SwerveConstants.speedP*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final PS5Controller joystick = new PS5Controller(0);
    //JoystickButton dampner = new JoystickButton(joystick, PS5Controller.Button.kR1.value);
    /* Setting up bindings for necessary control of the swerve drive platform */
    
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
             // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    


    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Lift m_Lift = new Lift();
    public final Arm m_arm = new Arm();
    public final intake m_intake = new intake();
    public final Climb m_climb = new Climb();


    public RobotContainer() {

    NamedCommands.registerCommand("Position3Button", Commands.parallel(new AngletoPos(m_intake, 18, 0.6).withTimeout(2),
    Commands.waitSeconds(0).andThen(new LifttoPos(m_Lift,170,0.5).withTimeout(4), Commands.waitSeconds(0).andThen(new maintainCoral(m_intake)).withTimeout(2))));

    NamedCommands.registerCommand("Position2Button", Commands.parallel(new AngletoPos(m_intake, 17, 0.6).withTimeout(2),
    Commands.waitSeconds(1).andThen(new LifttoPos(m_Lift,68,1).withTimeout(2), Commands.waitSeconds(0).andThen(new maintainCoral(m_intake)).withTimeout(2))));

    NamedCommands.registerCommand("Position1Button", new AngletoPos(m_intake, 19, 0.6));

    NamedCommands.registerCommand("Position0Button", new LifttoPos(m_Lift,0,0.3));
    NamedCommands.registerCommand("maintainCoral", new maintainCoral(m_intake));
    NamedCommands.registerCommand("readyCoral", new AngletoPos(m_intake,50,0.6));
    NamedCommands.registerCommand("GrabCoral", Commands.sequence(new LifttoPos(m_Lift, 0, .4).withTimeout(1).andThen(new AngletoPos(m_intake, 25, 0.6).withTimeout(1), 
    Commands.waitSeconds(0.5).andThen(new grabCoral(m_intake,.8)).withTimeout(4))));

    NamedCommands.registerCommand("GrabCoral2", Commands.parallel(new AngletoPos(m_intake, 50, 0.6).withTimeout(1),
    Commands.waitSeconds(0.2).andThen(new maintainCoral(m_intake).withTimeout(2))));
    NamedCommands.registerCommand("ShootCoral", Commands.sequence(new coralShooter(m_intake, 1).withTimeout(.5), Commands.parallel(Commands.waitSeconds(0).
    andThen(new AngletoPos(m_intake,50,0.6),Commands.waitSeconds(0).andThen(new LifttoPos(m_Lift, 0, 1)).withTimeout(4), Commands.waitSeconds(0).andThen(new AngletoPos(m_intake, 25, 0.6).withTimeout(1))))));
    

    autoChooser = AutoBuilder.buildAutoChooser();

    Telemetry.dampner(joystick);

    //REGISTER NAMED COMMANDS IN ORDER TO USE IN PATHPLANNER GUI
    NamedCommands.registerCommand("Angle to Position", new AngletoPos(m_intake, 19, 0.6) );
    NamedCommands.registerCommand("Hold Coral ", new maintainCoral(m_intake));
    NamedCommands.registerCommand("Lift to Position", new LifttoPos(m_Lift, 0, 0.3) );
    NamedCommands.registerCommand("Place Coral", new coralShooter(m_intake, 1) );
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    Waypoint x = new Waypoint(null, new Translation2d(7.134038461538461, 7.534), new Translation2d(6.393676512347697, 6.850411563748164));


    
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with positive Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with positive X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


   

    POVButton down = new POVButton(joystick, 180, 0);
    POVButton right = new POVButton(joystick, 90, 0);
    POVButton up = new POVButton(joystick, 0, 0);
    POVButton left = new POVButton(joystick, 270, 0);
    JoystickButton dampner = new JoystickButton(joystick, PS5Controller.Button.kR1.value);
    dampner.whileTrue(new Dampner(2.0));
    
    down.whileTrue(new intakeManual(m_intake, -0.2));
    up.whileTrue(new intakeManual(m_intake, 0.2));
    
    JoystickButton ManualUpLiftButton = new JoystickButton(joystick, PS5Controller.Button.kOptions.value);

    ManualUpLiftButton.whileTrue(new LiftUpManual(m_Lift,0.2));
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////

    JoystickButton ManualDownLiftButton = new JoystickButton(joystick, PS5Controller.Button.kCreate.value);

    ManualDownLiftButton.whileTrue(new LiftDownManual(m_Lift,0.2));
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////





    JoystickButton position0Button = new JoystickButton(joystick, PS5Controller.Button.kPS.value);
    position0Button.onTrue(new LifttoPos(m_Lift,0,0.3));

    JoystickButton position1Button = new JoystickButton(joystick, PS5Controller.Button.kCross.value);

    position1Button.onTrue(Commands.parallel(new AngletoPos(m_intake, 19, 0.6).withTimeout(2),
    Commands.waitSeconds(0).andThen(new LifttoPos(m_Lift,0,1).withTimeout(2), 
    Commands.waitSeconds(0).andThen(new maintainCoral(m_intake)).withTimeout(6)))); 


    
    JoystickButton position2Button = new JoystickButton(joystick, PS5Controller.Button.kCircle.value);
    //position2Button.onTrue(new AngletoPos(m_intake, 17, 0.6));
    //position2Button.onTrue(new LifttoPos(m_Lift,68,1));
    position2Button.onTrue(Commands.parallel(new AngletoPos(m_intake, 17, 0.6).withTimeout(2),
    Commands.waitSeconds(1).andThen(new LifttoPos(m_Lift,56,1).withTimeout(2), 
    Commands.waitSeconds(0).andThen(new maintainCoral(m_intake)).withTimeout(6)))); 
    
    
    
    JoystickButton grabCoral = new JoystickButton(joystick, PS5Controller.Button.kL1.value);
    grabCoral.onTrue(Commands.sequence(new LifttoPos(m_Lift, 0, .4).withTimeout(1).andThen(new AngletoPos(m_intake, 25, 0.6).withTimeout(1), 
    Commands.waitSeconds(0.5).andThen(new grabCoral(m_intake,0.75)).withTimeout(4),
    Commands.waitSeconds(0).andThen(new AngletoPos(m_intake, 50, 0.6).withTimeout(1),
    Commands.waitSeconds(0.2).andThen(new maintainCoral(m_intake).withTimeout(3))))));


    JoystickButton position3Button = new JoystickButton(joystick, PS5Controller.Button.kTriangle.value);
    //position3Button.onTrue(new AngletoPos(m_intake, 18, 0.6));
    //position3Button.onTrue(new LifttoPos(m_Lift,170,0.5));
    position3Button.onTrue(Commands.parallel(new AngletoPos(m_intake, 19, 0.6).withTimeout(2),
    Commands.waitSeconds(0).andThen(new LifttoPos(m_Lift,164,0.5).withTimeout(2), Commands.waitSeconds(0).andThen(new maintainCoral(m_intake)).withTimeout(6)))); 



   // left.onTrue(new resetIntakeEncoder(m_intake));


    right.onTrue(new resetLiftEncoders(m_Lift));

    


   /* JoystickButton verticalPositionCoral = new JoystickButton(joystick, PS5Controller.Button.kSquare.value);
    verticalPositionCoral.onTrue(new AngletoPos(m_intake,50,0.6));
    verticalPositionCoral.onTrue(new LifttoPos(m_Lift, 0, 0.3));*/


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // JoystickButton coralHandlerButton = new JoystickButton(joystick, PS5Controller.Button.kTriangle.value);
    //coralHandlerButton.onTrue(new AngletoPos(m_intake, 26, 0.6));
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    JoystickButton coralIntake = new JoystickButton(joystick, PS5Controller.Button.kL2.value);
    coralIntake.onTrue(Commands.sequence(new grabCoral(m_intake, 0.8).withTimeout(2).andThen(new maintainCoral(m_intake).withTimeout(6))));


    JoystickButton coralShooter = new JoystickButton(joystick, PS5Controller.Button.kR2.value);
    //processAlgaeButton.whileTrue(new processAlgae(m_intake, 1));
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    /*JoystickButton grabCoral = new JoystickButton(joystick, PS5Controller.Button.kL1.value);
    grabCoral.onTrue(Commands.sequence(new LifttoPos(m_Lift, 0, .4).withTimeout(1).andThen(new AngletoPos(m_intake, 25, 0.6).withTimeout(1), 
    Commands.waitSeconds(0.5).andThen(new grabCoral(m_intake,1)).withTimeout(4),
    Commands.waitSeconds(1).andThen(new AngletoPos(m_intake, 50, 0.6).withTimeout(1),
    Commands.waitSeconds(0.2).andThen(new maintainCoral(m_intake).withTimeout(7))))));*/
    

    
    
    coralShooter.onTrue(Commands.sequence(new coralShooter(m_intake, 1).withTimeout(1), Commands.parallel(Commands.waitSeconds(0).
    andThen(new AngletoPos(m_intake,50,0.6),
    Commands.waitSeconds(0).andThen(new LifttoPos(m_Lift, 0, 1)).withTimeout(3)))));
    
    /////////////////////////////////////////////CLIMB BUTTON BINDINGS ////////////////////////////////////////////////////////
    /// 


    JoystickButton climButton = new JoystickButton(joystick, PS5Controller.Button.kSquare.value);
    climButton.whileTrue(new ClimbManualUp(m_climb, MaxSpeed));

    //JoystickButton descendButton = new JoystickButton(joystick, PS5Controller.Button.kSquare.value);
    left.whileTrue(new ClimbManualDown(m_climb, MaxSpeed));
    SmartDashboard.putNumber("OG maxSpeed", MaxSpeed);


    
    //grabCoral.onTrue(new Graber (m_Lift, m_intake));
    ///////// grabCoral.onTrue(new grabAlgae(m_intake, 1));
    /////////grabCoral.onTrue(new AngletoPos(m_intake,31, 0.6));
    /////////grabCoral.onTrue(new LifttoPos(m_Lift,23,1));
    



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    JoystickButton fieldResetButton = new JoystickButton(joystick, PS5Controller.Button.kTouchpad.value);
    Command fieldResetCommand = new InstantCommand(() -> fieldReset());
    fieldResetButton.onTrue(fieldResetCommand);

    ////////////////////////////////////////////////////////////PathPlanerCommands/////////////////////////////////////////////////////////////////////
    


        /*joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        /* 
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.povUp().onTrue(drivetrain.runOnce(()-> drivetrain.seedFieldCentric()));
        
    //        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    */

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
      }

    /*public PathPlannerAuto test()
    {
        return new PathPlannerAuto("testAuto");
    }*/


    public void fieldReset(){ 
        
        drivetrain.seedFieldCentric();
    }
    
}
