package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDownCommand;
import frc.robot.commands.IntakeUpCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.CleanerCommands.CleanerGetInCommand;
import frc.robot.commands.CleanerCommands.CleanerGetOutCommand;
import frc.robot.commands.ElevatorCommands.ElevatorDownCommand;
import frc.robot.commands.ElevatorCommands.ElevatorUpCommand;
import frc.robot.commands.ElevatorCommands.RaiseElevatorCommand;
import frc.robot.commands.GripperCommands.IntakeCoralCommand;
import frc.robot.commands.GripperCommands.OuttakeCoralCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ElevatorSubsytem.DesiredElevatorPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Target Options */
    public enum TargetAB{
        A, B
    }

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    private final ElevatorSubsytem elevatorSubsytem = new ElevatorSubsytem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final CleanerSubsystem cleaner = new CleanerSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(1), 
                () -> driver.getRawAxis(0), 
                () -> driver.getRawAxis(4), 
                () -> false
            )
        );

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
        // Unit Test commands
        // Go to human intake command
        /* 
        // Go to coprocessor commannd
        new JoystickButton(driver, Button.kA.value).whileTrue(new GoToCoProcessorCommand(s_Swerve).repeatedly());
        // Go to reef target commands
        new JoystickButton(driver, Button.kLeftBumper.value).whileTrue(new GoToReefTargetCommand(s_Swerve, s_Vision, TargetAB.A));
        new JoystickButton(driver, Button.kRightBumper.value).whileTrue(new GoToReefTargetCommand(s_Swerve, s_Vision, TargetAB.B));
        // Go to human intake commands
        new JoystickButton(driver, Button.kA.value).whileTrue(new GoToHumanIntakeCommand(s_Swerve, TargetAB.A));
        new JoystickButton(driver, Button.kB.value).whileTrue(new GoToHumanIntakeCommand(s_Swerve, TargetAB.B));
        */

        // Driver Buttons
      /* new JoystickButton(driver, Button.kY.value).whileTrue(new ResetOdometryCommand(s_Swerve));
        new JoystickButton(driver, Button.kB.value).whileTrue(new GoToHumanIntakeCommand(s_Swerve, TargetAB.A).repeatedly());
        new JoystickButton(driver, Button.kX.value).whileTrue(new GoToHumanIntakeCommand(s_Swerve, TargetAB.B).repeatedly());
        new JoystickButton(driver, Button.kA.value).whileTrue(new GoToCoProcessorCommand(s_Swerve).repeatedly());
        new JoystickButton(driver, Button.kLeftBumper.value).whileTrue(new GoToReefTargetCommand(s_Swerve, s_Vision, TargetAB.A).repeatedly());
        new JoystickButton(driver, Button.kRightBumper.value).whileTrue(new GoToReefTargetCommand(s_Swerve, s_Vision, TargetAB.B).repeatedly());*/

        // Operator Buttons
        new JoystickButton(driver, 1).whileTrue(new ElevatorUpCommand(elevatorSubsytem));
        new JoystickButton(driver, 2).whileTrue(new ElevatorDownCommand(elevatorSubsytem));
        new JoystickButton(driver, 3).whileTrue(new CleanerGetInCommand(cleaner));
        new JoystickButton(driver, 4).whileTrue(new CleanerGetOutCommand(cleaner));
        new JoystickButton(driver, 5).whileTrue(new IntakeCoralCommand(gripperSubsystem));
        new JoystickButton(driver, 6).whileTrue(new OuttakeCoralCommand(gripperSubsystem));

        // Elevator PID test
        new JoystickButton(operator, 1).whileTrue(new RaiseElevatorCommand(elevatorSubsytem, DesiredElevatorPosition.L1));
        new JoystickButton(operator, 2).whileTrue(new RaiseElevatorCommand(elevatorSubsytem, DesiredElevatorPosition.L2));
        new JoystickButton(operator, 3).whileTrue(new RaiseElevatorCommand(elevatorSubsytem, DesiredElevatorPosition.L3));
        //new JoystickButton(operator, 4).whileTrue(new RaiseElevatorCommand(elevatorSubsytem, DesiredElevatorPosition.L4));

        /*new JoystickButton(operator, 3).whileTrue(new IntakeCommand(intakeSubsystem));
        new JoystickButton(operator, 4).whileTrue(new OuttakeCommand(intakeSubsystem));
        */
        new JoystickButton(operator, 5).whileTrue(new IntakeUpCommand(intakeSubsystem));
        new JoystickButton(operator, 6).whileTrue(new IntakeDownCommand(intakeSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("New Auto");
    }
}
