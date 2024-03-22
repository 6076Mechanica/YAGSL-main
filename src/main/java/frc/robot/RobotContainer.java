// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil; 
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.ClimberSubsystem;
import frc.robot.subsystems.swervedrive.IntakeSubsys;
import frc.robot.subsystems.swervedrive.FeederSubsys;
import frc.robot.subsystems.swervedrive.ShooterSubsys;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  private final ClimberSubsystem m_climber = new ClimberSubsystem(true);
  private final FeederSubsys m_feeder = new FeederSubsys();
  private final IntakeSubsys m_intake = new IntakeSubsys();
  private final ShooterSubsys m_shooter = new ShooterSubsys();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick driverStick = new CommandJoystick(1);
  private final DeadBand m_deadband = new DeadBand();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()

    //Intake, feeder and shooter commands: 
    private Command GobbleCMD = Commands.runEnd(m_intake::intake, m_intake::stop, m_intake);
    private Command StopGobbler = Commands.runOnce(m_intake::stop, m_intake);
    private Command SpinUp = Commands.run(m_shooter::SpinUp, m_shooter);
    private Command SpinShooter = Commands.runOnce(m_shooter::spit, m_shooter);
    private Command Spit = Commands.runOnce(m_shooter::spitSlow, m_shooter);
    private Command RunFeeder = Commands.runOnce(m_feeder::feed, m_feeder);
    private Command StopFeeder = Commands.runOnce(m_feeder::stopFeed, m_feeder);
    private Command SpinDown = Commands.runOnce(m_shooter::Stop, m_shooter); 
    private Command SetShotSpeed = Commands.runOnce(m_shooter::setShootSpeed); 
    private Command SetSpitSpeed = Commands.runOnce(m_shooter::setSpitSpeed, m_shooter);
    private Command ConveyCMD = Commands.runOnce(m_intake::convey, m_intake);
    private Command feed = Commands.runEnd(m_feeder::feed, m_feeder::stopFeed, m_feeder);
    private Command EjectFeed = Commands.runEnd(m_feeder::eject, m_feeder::stopFeed, m_feeder);
    private Command EjectIntake = Commands.runEnd(m_intake::eject, m_intake::stop, m_intake);
    private Command ToggleClimber = Commands.runOnce(m_climber::toggle);
    private WaitCommand FeederWait = new WaitCommand(1.25);
    private WaitCommand shooterWait = new WaitCommand(1);
    private SequentialCommandGroup Shoot = new SequentialCommandGroup();
    private SequentialCommandGroup SlowShoot = new SequentialCommandGroup();
    private ParallelCommandGroup Eject = new ParallelCommandGroup();
    private ParallelCommandGroup Ejector = new ParallelCommandGroup();

  {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRightX(),
        () -> driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> m_deadband.deadbandresponse(driverStick.getY(), OperatorConstants.JOYSTICK_XY_DEADBAND, OperatorConstants.JOYSTICK_XY_EXP),
        () -> -m_deadband.deadbandresponse(driverStick.getX(), OperatorConstants.JOYSTICK_XY_DEADBAND, OperatorConstants.JOYSTICK_XY_EXP),
        () -> driverStick.getTwist());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driverXbox.getRawAxis(2));

    //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    
    Shoot.addCommands(SpinShooter, shooterWait, RunFeeder, ConveyCMD, FeederWait, StopGobbler, StopFeeder, SpinDown);
    Eject.addCommands(GobbleCMD, feed);
    Ejector.addCommands(EjectFeed, EjectIntake);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    Trigger spitTrigger = new Trigger(driverStick.button(4));
    Trigger SetShootSpeedTrigger = new Trigger(driverStick.button(6));
    Trigger ejectTrigger = new Trigger(driverStick.button(3));
    Trigger intakeTrigger = new Trigger(driverStick.button(2));
    Trigger shootTrigger = new Trigger(driverStick.button(1));
    Trigger feedTrigger = new Trigger(m_shooter::ready);
    Trigger stagedTrigger = new Trigger(m_feeder::isStaged);
    Trigger climbTrigger = new Trigger(driverStick.button(11));
    
    climbTrigger.onTrue(ToggleClimber);
    intakeTrigger.whileTrue(Eject);
    stagedTrigger.onTrue(StopGobbler);
    shootTrigger.onTrue(Shoot.withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    ejectTrigger.whileTrue(Ejector);
    spitTrigger.onTrue(SetSpitSpeed);
    SetShootSpeedTrigger.onTrue(SetShotSpeed);

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
