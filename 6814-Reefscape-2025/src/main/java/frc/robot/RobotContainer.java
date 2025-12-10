// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.LimelightUpdate;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.OuttakeSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_Swerve = new SwerveSubsystem();
  public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  public final OuttakeSubsystem m_Out = new OuttakeSubsystem();
  public final ArmSubsystem m_Arm = new ArmSubsystem();
  private final SendableChooser<Command> autoChooser;

  public final Joystick m_ElevatorJoystick = new Joystick(OIConstants.kElevatorJoystickPort);

  public final Command m_limelightUpdater = new LimelightUpdate(m_Swerve);
  
  public RobotContainer() 
  {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureCameras();
    configureBindings();
  }




  private void configureCameras()
  {
    HttpCamera httpCamera = new HttpCamera("Limelight Camera", "http://limelight.local:5800");
    CameraServer.addCamera(httpCamera);
  }




  private void configureBindings() 
  {
    JoystickButton button_a = new JoystickButton(m_ElevatorJoystick, 1);
    JoystickButton button_b = new JoystickButton(m_ElevatorJoystick, 2);
    JoystickButton button_x = new JoystickButton(m_ElevatorJoystick, 3);
    JoystickButton button_y = new JoystickButton(m_ElevatorJoystick, 4);

    button_a.whileTrue(m_elevatorSubsystem  .quasiForward());
    button_b.whileTrue(m_elevatorSubsystem  .quasiReverse());
    button_x.whileTrue(m_elevatorSubsystem.dynamicForward());
    button_y.whileTrue(m_elevatorSubsystem.dynamicReverse());

    new POVButton(m_ElevatorJoystick, 0)  .onTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.kFourthLevel));
    new POVButton(m_ElevatorJoystick, 90) .onTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.kThirdLevel));
    new POVButton(m_ElevatorJoystick, 270).onTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.kSecondLevel));
    new POVButton(m_ElevatorJoystick, 180).onTrue(new ElevatorCommand(m_elevatorSubsystem, ElevatorConstants.kFirstLevel));
  }




  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
  

