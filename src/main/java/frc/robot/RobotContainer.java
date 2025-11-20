// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Mode;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTBSwerve;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NoteVisualizer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final NoteVisualizer visualizer = new NoteVisualizer();

  //divides the movement by the value of drive ratio.
  private double driveRatio = 1.0;
  private boolean slowMode = false;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0); //Change when done
  private final CommandXboxController operatorController = new CommandXboxController(1); //Change when done

  private final CommandXboxController outreachController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private static final Translation3d blueSpeaker = new Translation3d(0.225, 5.55, 2.1);
  private boolean autoShootToggle = true;

  public static boolean lobbing = false;

  public double rotMultiplier = 1;

  public enum shootPositions{
    STOW(-12, 0.0),
    SUBWOOFER(-4.7, 25.0),    
    PODIUM(-8, 25.0),
    AMP(7.5, 0.0),
    STAGE(-8.9, 30.0),
    WING(-9.825, 35.0),
    CENTER_AUTO_NOTE(-8, 25.0),
    LOB(-9, 5.0),
    SOURCE_SIDE_AUTO(-9.375, 30);

    private double shootSpeed;
    private double shootAngle;
    private shootPositions(double shootAngle, double shootSpeed){
        this.shootSpeed = shootSpeed;
        this.shootAngle = shootAngle;
    }

    public double getShootSpeed(){
        return shootSpeed;
    }

    public double getShootAngle(){
        return shootAngle;
    }
}

  public static shootPositions shootEnum = shootPositions.SUBWOOFER;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new VisionIOLimelight(),
                new ModuleIOTBSwerve(0),
                new ModuleIOTBSwerve(1),
                new ModuleIOTBSwerve(2),
                new ModuleIOTBSwerve(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new VisionIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new VisionIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    //Visualize command scheduler routine in SmartDashboard, turn on only for debugging
    //SmartDashboard.putData("Commands", CommandScheduler.getInstance());

    //Register NamedCommands for use in PathPlanner
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up FF characterization routines
    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

   
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Trigger armAmp = new Trigger(() -> shootEnum == shootPositions.AMP);
    armAmp
      .onTrue(new InstantCommand(() -> rotMultiplier = 0.5))
      .onFalse(new InstantCommand(() -> rotMultiplier = 1));

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY() / driveRatio,
            () -> driverController.getLeftX() / driveRatio,
            () -> driverController.getRightX() * rotMultiplier));
  }

  public void autonomousInit() {
    // arm.setArmSetpoint(arm.getArmAngleDegrees());
  }

  public void teleopInit() {
    // arm.setArmSetpoint(arm.getArmAngleDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoChooser.get() != null) {
      return autoChooser.get();
      // .beforeStarting(new InstantCommand(() -> intake.setBarAngle(Constants.INTAKE_LOW_POSITION)));
    }
    return null;
  }

  /**
   * Rumble driver controller for 1 second
   * @return driverRumbleCommand
   */
  private Command driverRumbleCommand() {
    return Commands.startEnd(
      () -> {driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);},
      () -> {driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);});
  }
}
