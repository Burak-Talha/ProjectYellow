package frc.robot;


import java.lang.reflect.Field;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignToLeftReefTargetCommand;
import frc.robot.commands.AlignToRightReefTargetCommand;
import frc.robot.commands.GoToReefTargetCommand;
import frc.robot.commands.ResetOdometryCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.CleanerCommands.CleanerDownCommand;
import frc.robot.commands.CleanerCommands.CleanerUpCommand;
import frc.robot.commands.CleanerCommands.RotateCleanerCommand;
import frc.robot.commands.ElevatorCommands.ElevatorDownCommand;
import frc.robot.commands.ElevatorCommands.ElevatorUpCommand;
import frc.robot.commands.ElevatorCommands.RaiseElevatorCommand;
import frc.robot.commands.ElevatorCommands.RaiseElevatorWithLimitatorCommand;
import frc.robot.commands.GripperCommands.AutoIntakeCoralCommand;
import frc.robot.commands.GripperCommands.AutoOuttakeCoralCommand;
import frc.robot.commands.GripperCommands.IntakeCoralCommand;
import frc.robot.commands.GripperCommands.OuttakeCoralCommand;
import frc.robot.commands.IntakeCommands.IntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeDownCommand;
import frc.robot.commands.IntakeCommands.IntakeUpCommand;
import frc.robot.commands.IntakeCommands.OuttakeCommand;
import frc.robot.commands.IntakeCommands.RotateIntakeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CleanerSubsystem.DesiredCleanerPosition;
import frc.robot.subsystems.ElevatorSubsytem.DesiredElevatorPosition;
import frc.robot.subsystems.IntakeSubsystem.DesiredIntakePosition;

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
    private final XboxController driver = new XboxController(0);
    private final Joystick operator = new Joystick(1);

    /* Subsystems */
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ElevatorSubsytem elevatorSubsystem = new ElevatorSubsytem();
    private final GripperSubsystem gripperSubsystem = new GripperSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final CleanerSubsystem cleanerSubsystem = new CleanerSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        //cleanerSubsystem.setDefaultCommand(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.DEFAULT));
        NamedCommands.registerCommand("AlignToLeftTarget", new AlignToLeftReefTargetCommand(swerveSubsystem, visionSubsystem));
        NamedCommands.registerCommand("AlignToRightTarget", new AlignToRightReefTargetCommand(swerveSubsystem, visionSubsystem));
        NamedCommands.registerCommand("AutoIntake", new AutoIntakeCoralCommand(gripperSubsystem));
        NamedCommands.registerCommand("AutoOuttake", new AutoOuttakeCoralCommand(gripperSubsystem));
        NamedCommands.registerCommand("test", new WaitUntilCommand(gripperSubsystem::hasCoral).andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1)));
        NamedCommands.registerCommand("ScoreCoralToL1", new WaitUntilCommand(()->gripperSubsystem.hasCoral())
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1))
                                                    .andThen(new AutoOuttakeCoralCommand(gripperSubsystem)));
        NamedCommands.registerCommand("ScoreCoralToL2", new WaitUntilCommand(()->gripperSubsystem.hasCoral())
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L2))
                                                    .andThen(new AutoOuttakeCoralCommand(gripperSubsystem))
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1)));
        NamedCommands.registerCommand("ScoreCoralToL3", new WaitUntilCommand(()->gripperSubsystem.hasCoral())
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L3))
                                                    .andThen(new AutoOuttakeCoralCommand(gripperSubsystem))
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1)));
        NamedCommands.registerCommand("ScoreCoralToL4", new WaitUntilCommand(()->gripperSubsystem.hasCoral())
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1))
                                                    .andThen(new AutoOuttakeCoralCommand(gripperSubsystem))
                                                    .andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1)));
        if(DriverStation.getAlliance().get()==Alliance.Red){
        swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                swerveSubsystem,
                elevatorSubsystem,
                () -> driver.getRawAxis(1), 
                () -> driver.getRawAxis(0), 
                () -> -driver.getRawAxis(4), 
                () -> false
            )
        );
        } else{
        swerveSubsystem.setDefaultCommand(
            new TeleopSwerve(
                swerveSubsystem,
                elevatorSubsystem,
                () -> -driver.getRawAxis(1), 
                () -> -driver.getRawAxis(0), 
                () -> -driver.getRawAxis(4), 
                () -> false
                )
            );  
        }

        //intakeSubsystem.setDefaultCommand(new RotateIntakeCommand(intakeSubsystem, DesiredIntakePosition.DEFAULT));

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
        
        // limelight testing
        new JoystickButton(driver, 1).whileTrue(new ResetOdometryCommand(swerveSubsystem));
        new JoystickButton(driver, 3).whileTrue(new GoToReefTargetCommand(swerveSubsystem, visionSubsystem, FieldConstants.LEFT_TARGET_OFFSET));
        new JoystickButton(driver, 2).whileTrue(new GoToReefTargetCommand(swerveSubsystem, visionSubsystem, FieldConstants.RIGHT_TARGET_OFFSET));
        new JoystickButton(driver, 4).whileTrue(new GoToReefTargetCommand(swerveSubsystem, visionSubsystem, FieldConstants.CENTER_TARGET_OFFSET));

        new JoystickButton(driver, 5).whileTrue(new AutoIntakeCoralCommand(gripperSubsystem));
        new JoystickButton(driver, 6).whileTrue(new OuttakeCoralCommand(gripperSubsystem));


       /*new JoystickButton(operator, 3).whileTrue(new IntakeCommand(intakeSubsystem));
        new JoystickButton(operator, 4).whileTrue(new OuttakeCommand(intakeSubsystem));
        new JoystickButton(operator, 5).whileTrue(new IntakeUpCommand(intakeSubsystem));
        new JoystickButton(operator, 6).whileTrue(new IntakeDownCommand(intakeSubsystem));*/

        // Elevator PID test  
        new JoystickButton(operator, 1).whileTrue(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L1));
        new JoystickButton(operator, 2).whileTrue(new WaitUntilCommand(gripperSubsystem::hasCoral).andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.LOWER_ALGAE_CLEAN)));
        new JoystickButton(operator, 3).whileTrue(new WaitUntilCommand(gripperSubsystem::hasCoral).andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L3)));
        //new JoystickButton(operator, 4).whileTrue(new WaitUntilCommand(gripperSubsystem::hasCoral).andThen(new RaiseElevatorCommand(elevatorSubsystem, DesiredElevatorPosition.L4)));

        /*new JoystickButton(operator, 4).whileTrue(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.DEFAULT));
        new JoystickButton(operator, 5).whileTrue(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.LOWER_ALGAE));
        new JoystickButton(operator, 6).whileTrue(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.UPPER_ALGAE));*/

        /*new JoystickButton(operator, 1).whileTrue(new RotateIntakeCommand(intakeSubsystem, DesiredIntakePosition.DEFAULT));
        new JoystickButton(operator, 2).whileTrue(new RotateIntakeCommand(intakeSubsystem, DesiredIntakePosition.INTAKE));*/

        //new JoystickButton(operator, 1).whileTrue(new IntakeUpCommand(intakeSubsystem));
       // new JoystickButton(operator, 2).whileTrue(new IntakeDownCommand(intakeSubsystem));

        //new JoystickButton(operator, 5).whileTrue(new CleanerUpCommand(cleanerSubsystem));
        //new JoystickButton(operator, 6).whileTrue(new CleanerDownCommand(cleanerSubsystem));
        // Elevator test with limited chassis speed
        /*new JoystickButton(operator, 1).whileTrue(new RaiseElevatorWithLimitatorCommand(elevatorSubsystem, swerveSubsystem, DesiredElevatorPosition.L1));
        new JoystickButton(operator, 3).whileTrue(new RaiseElevatorWithLimitatorCommand(elevatorSubsystem, swerveSubsystem, DesiredElevatorPosition.L3));
        new JoystickButton(operator, 4).whileTrue(new RaiseElevatorWithLimitatorCommand(elevatorSubsystem, swerveSubsystem, DesiredElevatorPosition.L4));*/

        // Cleaner test
        //new JoystickButton(operator, 4).whileTrue(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.DEFAULT));
        //new JoystickButton(operator, 5).whileTrue(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.LOWER_ALGAE));
        //new JoystickButton(operator, 6).whileTrue(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.UPPER_ALGAE));
        new JoystickButton(operator, 5).whileTrue(new CleanerUpCommand(cleanerSubsystem));
        new JoystickButton(operator, 6).whileTrue(new CleanerDownCommand(cleanerSubsystem));

        // Intake PID test
       // new JoystickButton(operator, 1).whileTrue(Commands.run(()->intakeSubsystem.setDesiredIntakePosition(DesiredIntakePosition.DEFAULT)));
       // new JoystickButton(operator, 2).whileTrue(Commands.run(()->intakeSubsystem.setDesiredIntakePosition(DesiredIntakePosition.INTAKE)));

        // Main Intake logic
        /*new JoystickButton(operator, 1).whileTrue(new ParallelCommandGroup(
                                                                    new RotateIntakeCommand(intakeSubsystem, DesiredIntakePosition.INTAKE),
                                                                    new IntakeCommand(intakeSubsystem)));
        new JoystickButton(operator, 2).whileTrue(new OuttakeCommand(intakeSubsystem));*/

                // Clean Algae Commands
        /*new JoystickButton(driver, 1).whileTrue(
            new ParallelCommandGroup(
                                      new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.DEFAULT),
                                      new ParallelCommandGroup(
                                                               new GoToReefTargetCommand(swerveSubsystem, visionSubsystem, FieldConstants.LEFT_TARGET_OFFSET),
                                                               new WaitUntilCommand(() -> swerveSubsystem.robotAtSetpoint()).andThen(new RaiseElevatorCommand(elevatorSubsytem, DesiredElevatorPosition.LOWER_ALGAE_CLEAN))
                                      )).andThen(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.LOWER_ALGAE)));
        new JoystickButton(driver, 2).whileTrue(
            new ParallelCommandGroup(
                                      new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.DEFAULT),
                                      new ParallelCommandGroup(
                                                               new GoToReefTargetCommand(swerveSubsystem, visionSubsystem, FieldConstants.RIGHT_TARGET_OFFSET),
                                                               new WaitUntilCommand(() -> swerveSubsystem.robotAtSetpoint()).andThen(new RaiseElevatorCommand(elevatorSubsytem, DesiredElevatorPosition.UPPER_ALGAE_CLEAN))
                                      )).andThen(new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.UPPER_ALGAE).andThen(new CleanerDownCommand(cleanerSubsystem))));*/
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new RotateCleanerCommand(cleanerSubsystem, DesiredCleanerPosition.DEFAULT).andThen(AutoBuilder.buildAuto("New Auto"));
    }
}
