package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons - (will be removed after tested) */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve( );
    private final Vision s_Vision = new Vision();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(1), 
                () -> -driver.getRawAxis(0), 
                () -> driver.getRawAxis(4), 
                () -> robotCentric.getAsBoolean()
            )
        );
        // Configure the button bindings
        configureButtonBindings();
    }
     
    public enum TargetAB{
        A, B
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(driver, Button.kY.value).whileTrue(new ResetOdometryCommand(s_Swerve));

        // Autonomous approach commands
        new JoystickButton(driver, Button.kB.value).whileTrue(new GoToHumanIntakeCommand(s_Swerve, TargetAB.A).repeatedly());
        new JoystickButton(driver, Button.kX.value).whileTrue(new GoToHumanIntakeCommand(s_Swerve, TargetAB.B).repeatedly());

        new JoystickButton(driver, Button.kA.value).whileTrue(new GoToCoProcessorCommand(s_Swerve).repeatedly());

        new JoystickButton(driver, Button.kLeftBumper.value).whileTrue(new GoToReefTargetCommand(s_Swerve, s_Vision, TargetAB.A).repeatedly());
        new JoystickButton(driver, Button.kRightBumper.value).whileTrue(new GoToReefTargetCommand(s_Swerve, s_Vision, TargetAB.B).repeatedly());
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