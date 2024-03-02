// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.Constants.Swerve;
//import frc.robot.autos.*;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToTargetCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  /* Controllers */
  private final Joystick gamepad = new Joystick(0);
  private final Joystick driverStick = new Joystick(1);
  private final Joystick buttonBox = new Joystick(2);

  /* Drive Controls */
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kZ.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driverStick, 3);
  private final JoystickButton targetSpeakerTags = new JoystickButton(driverStick, 5);

  private final JoystickButton slowSpeed = new JoystickButton(driverStick, 2);
  private final JoystickButton highSpeed = new JoystickButton(driverStick,1);

  /* gamepad Buttons */
  private final JoystickButton intakeButton = new JoystickButton(gamepad, XboxController.Button.kY.value);
  private final JoystickButton outTakeButton = new JoystickButton(gamepad,XboxController.Button.kX.value);
  private final JoystickButton pivotDownButton = new JoystickButton(gamepad,XboxController.Button.kB.value);
  private final JoystickButton pivotUpButton = new JoystickButton(gamepad,XboxController.Button.kA.value);
  private final JoystickButton chargeShooterButton = new JoystickButton(gamepad,XboxController.Button.kLeftBumper.value);
  private final JoystickButton shooterButton = new JoystickButton(gamepad,XboxController.Button.kRightBumper.value);

  private final POVButton runFeeder = new POVButton(gamepad, 0);
  private final POVButton reverseFeeder = new POVButton(gamepad, 180);
  
  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final LimelightSubsystem s_limelight = new LimelightSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  
  public RobotContainer() {
    /* Named commands */
    NamedCommands.registerCommand("shootNote",
      new SequentialCommandGroup(
        new RunCommand(() -> shooterSubsystem.runShooter(1), shooterSubsystem)
          .until(shooterSubsystem::isShooterAtSpeed),
        new ParallelRaceGroup(
            new RunCommand(() -> shooterSubsystem.runFeederAndShooter(), shooterSubsystem),
            new WaitCommand(1)
        ),
        new InstantCommand(() -> shooterSubsystem.stopAllMotors(), shooterSubsystem)
      )
    );
    NamedCommands.registerCommand("runIntake",
      new ParallelRaceGroup(
        new RunCommand(() -> intakeSubsystem.intake(pivotSubsystem), intakeSubsystem),
        new RunCommand(() -> shooterSubsystem.runFeeder(0.2), shooterSubsystem)
      )
    );
    NamedCommands.registerCommand("stopIntake",
      new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem)
    );
    NamedCommands.registerCommand("brieflyReverseFeeder",
      new SequentialCommandGroup(
        new ParallelRaceGroup(
          new RunCommand(() -> shooterSubsystem.runFeeder(-0.1), shooterSubsystem),
          new WaitCommand(0.1)
        ),
        new InstantCommand(() -> shooterSubsystem.runFeeder(0), shooterSubsystem)
      )
    );
    NamedCommands.registerCommand("raisePivotToShoot",
      new InstantCommand(() -> pivotSubsystem.goToAngle(PivotSubsystem.Positions.SHOT_ANGLE), pivotSubsystem)
    );
    NamedCommands.registerCommand("lowerPivotToHorizontal",
      new InstantCommand(() -> pivotSubsystem.goToAngle(PivotSubsystem.Positions.HORIZONTAL), pivotSubsystem)
    );

    CameraServer.startAutomaticCapture();

    s_limelight.turnOnDriverCam();
    s_limelight.enableLimelight(false);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driverStick.getRawAxis(translationAxis),
            () -> -driverStick.getRawAxis(strafeAxis),
            () -> -driverStick.getRawAxis(rotationAxis),
            () -> false,
            () -> slowSpeed.getAsBoolean() /*|| s_elevator.isElevatorHigh()*/,
            () -> highSpeed.getAsBoolean()));

    climberSubsystem.setDefaultCommand(
      new RunCommand(
        () -> climberSubsystem.runArms(-gamepad.getRawAxis(XboxController.Axis.kRightY.value), gamepad.getRawAxis(XboxController.Axis.kLeftY.value) ),
            climberSubsystem));
      
    intakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> intakeSubsystem.stopIntake(),
            intakeSubsystem));

    shooterSubsystem.setDefaultCommand(
      new RunCommand(
        () -> shooterSubsystem.stopAllMotors(),
            shooterSubsystem));
    
            
    configureBindings();
  }

  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    new POVButton(driverStick, 0).whileTrue(new TurnToAngleCommand(s_Swerve, 180, 2));
    new POVButton(driverStick, 90).whileTrue(new TurnToAngleCommand(s_Swerve, 90, 2));
    new POVButton(driverStick, 180).whileTrue(new TurnToAngleCommand(s_Swerve, 0, 2));
    new POVButton(driverStick, 270).whileTrue(new TurnToAngleCommand(s_Swerve, -90, 2));

    new JoystickButton(driverStick, 4).whileTrue(new RunCommand(() -> s_Swerve.setX(), s_Swerve));
    targetSpeakerTags.whileTrue(new TurnToTargetCommand(s_Swerve, s_limelight, driverStick, 50));

    // Intake controls
    intakeButton.whileTrue(new RunCommand(() -> intakeSubsystem.intake(pivotSubsystem), intakeSubsystem));
    outTakeButton.whileTrue(new RunCommand(() -> intakeSubsystem.outtake(), intakeSubsystem));

    // Pivot controls
    pivotUpButton.whileTrue(
       new RunCommand(() -> pivotSubsystem.goToAngle(PivotSubsystem.Positions.VERTICAL), pivotSubsystem));

    pivotDownButton.whileTrue(
      new RunCommand(() -> pivotSubsystem.goToAngle(PivotSubsystem.Positions.HORIZONTAL), pivotSubsystem));

    new JoystickButton(gamepad, XboxController.Button.kBack.value).whileTrue(
      new RunCommand(() -> pivotSubsystem.goToAngle(PivotSubsystem.Positions.SHOT_ANGLE), pivotSubsystem));

    // Shooter controls
    shooterButton.whileTrue(
        new RunCommand(() -> shooterSubsystem.shoot(1,pivotSubsystem), shooterSubsystem));
    
    chargeShooterButton.whileTrue(
        new RunCommand(() -> shooterSubsystem.chargeShooter(1,pivotSubsystem), shooterSubsystem));

    runFeeder.whileTrue(new RunCommand(() -> shooterSubsystem.runFeeder(0.75), shooterSubsystem));
    reverseFeeder.whileTrue(new RunCommand(() -> shooterSubsystem.runFeeder(-0.2), shooterSubsystem));

    // Limelight controls
    new JoystickButton(gamepad, XboxController.Button.kStart.value)
        .whileTrue(
            new InstantCommand(
                () -> s_limelight.setPipeline(LimelightSubsystem.Pipeline.AprilTags)));

  }

  public Command getAutonomousCommand() {
    if (buttonBox.getRawButton(3)) {
      return new PathPlannerAuto("2 note center");
    } else if (buttonBox.getRawButton(4)) {
      return new PathPlannerAuto("2 note amp side");
    } else if (buttonBox.getRawButton(5)) {
      return new PathPlannerAuto("2 note source side");
    } else if (buttonBox.getRawButton(6)) {
      return new PathPlannerAuto("4 note center");
    }

    return Commands.print("No autonomous command configured");

    // if (buttonBox.getRawButton(3)) {
    //   return new AutoPreloadConeChargeStation(this);
    // } else if (buttonBox.getRawButton(4)) {
    //   return new AutoPreloadConeChargeStationPlusCone(this);
    // } else if (buttonBox.getRawButton(5)) {
    //   return new AutoPreloadCubeChargeStation(this);
    // } else if (buttonBox.getRawButton(6)) {
    //   return new AutoBlueLeftTwoHigh(this);
    // } else if (buttonBox.getRawButton(7)) {
    //   return new AutoRedRightTwoHigh(this);
    // } else {
    //   return new AutoPreloadConeChargeStationPlusCube(this);
    // }

    //return Commands.print("No autonomous command configured");
  }
}
