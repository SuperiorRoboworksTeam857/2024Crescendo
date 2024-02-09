// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.Constants.Swerve;

//import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
  private final JoystickButton robotCentric = new JoystickButton(driverStick, 5);

  private final JoystickButton slowSpeed = new JoystickButton(driverStick, 1);
  private final JoystickButton highSpeed = new JoystickButton(driverStick, 2);

  /* gamepad Buttons */
  private final JoystickButton intakeButton = new JoystickButton(gamepad, 4);


  /* Subsystems */
  public final Swerve s_Swerve = new Swerve();
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  public final LimelightSubsystem s_limelight = new LimelightSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  public RobotContainer() {
    //CameraServer.startAutomaticCapture();

    s_limelight.turnOnDriverCam();
    s_limelight.enableLimelight(false);

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driverStick.getRawAxis(translationAxis),
            () -> -driverStick.getRawAxis(strafeAxis),
            () -> -driverStick.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean(),
            () -> slowSpeed.getAsBoolean() /*|| s_elevator.isElevatorHigh()*/,
            () -> highSpeed.getAsBoolean()));

    climberSubsystem.setDefaultCommand(
      new RunCommand(
        () -> climberSubsystem.runArms(-gamepad.getRawAxis(XboxController.Axis.kRightY.value), gamepad.getRawAxis(XboxController.Axis.kLeftY.value) ),
            climberSubsystem));
      
    intakeSubsystem.setDefaultCommand(
      new RunCommand(
        () -> intakeSubsystem.runIntake(0),
            intakeSubsystem));
    
            
    configureBindings();
  }

  private void configureBindings() {
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    new POVButton(driverStick, 0).whileTrue(new TurnToAngleCommand(s_Swerve, 180, 2));
    new POVButton(driverStick, 90).whileTrue(new TurnToAngleCommand(s_Swerve, 90, 2));
    new POVButton(driverStick, 180).whileTrue(new TurnToAngleCommand(s_Swerve, 0, 2));
    new POVButton(driverStick, 270).whileTrue(new TurnToAngleCommand(s_Swerve, -90, 2));

    new JoystickButton(driverStick, 4).whileTrue(new RunCommand(() -> s_Swerve.setX(), s_Swerve));

    intakeButton.whileTrue(new RunCommand(() -> intakeSubsystem.runIntake(0.5), intakeSubsystem));

  }

  public Command getAutonomousCommand() {
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

    return Commands.print("No autonomous command configured");
  }
}
