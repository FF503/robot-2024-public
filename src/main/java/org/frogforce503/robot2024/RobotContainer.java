package org.frogforce503.robot2024;

import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.frogforce503.lib.auto.AutoChooser;
import org.frogforce503.lib.util.Logic;

import org.frogforce503.robot2024.RobotStatus.AllianceColor;
import org.frogforce503.robot2024.fields.FieldConfig;
import org.frogforce503.robot2024.planners.LineupPlanner;
import org.frogforce503.robot2024.planners.ShotPlanner;
import org.frogforce503.robot2024.planners.ShotPlanner.Mode;
import org.frogforce503.robot2024.planners.ShotPlanner.Presets;
import org.frogforce503.robot2024.subsystems.Arm;
import org.frogforce503.robot2024.subsystems.Climber;
import org.frogforce503.robot2024.subsystems.DriverFeedback;
import org.frogforce503.robot2024.subsystems.Feeder;
import org.frogforce503.robot2024.subsystems.Intake;
import org.frogforce503.robot2024.subsystems.NoteDetector;
import org.frogforce503.robot2024.subsystems.Photon;
import org.frogforce503.robot2024.subsystems.Shooter;
import org.frogforce503.robot2024.subsystems.Wrist;
import org.frogforce503.robot2024.subsystems.Wrist.Goal;
import org.frogforce503.robot2024.subsystems.drive.Drive;
import org.frogforce503.robot2024.subsystems.sim.Visualizer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // Controllers
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);

  // Subsystems
  public static final Drive drive = Drive.getInstance();
  public static final Photon photon = Photon.getInstance();
  public static final Arm arm = Arm.getInstance();
  public static final Wrist wrist = Wrist.getInstance();
  public static final Intake intake = Intake.getInstance();
  public static final NoteDetector noteDetector = NoteDetector.getInstance();
  public static final Climber climber = Climber.getInstance();
  public static final Shooter shooter = Shooter.getInstance();
  public static final Feeder feeder = new Feeder();
 
  // Planners 
  public static final ShotPlanner shotplanner = ShotPlanner.getInstance(); 
  public static final LineupPlanner lineupplanner = LineupPlanner.getInstance(); 
 
  // Miscellaneous 
  public static final DriverFeedback driverfeedback = DriverFeedback.getInstance();
  public static final FieldConfig fieldconfig = FieldConfig.getInstance();

  // Needs
  public static final AutoChooser autoChooser = new AutoChooser();
  
  // Other Hardware
  // public static final PowerDistribution powerDistribution = new PowerDistribution();

  // Vision clients
  // public static final JetsonClient jetson = new JetsonClient();

  public static volatile int climbStep = 0;
  public static boolean areCamerasDisabled = false;

  public static boolean isHailMary = false;

  private static Trigger autoAim;
  public static BooleanSupplier isAutoAim;
  public static BooleanSupplier isDriverRBPressed;
  public static BooleanSupplier isDriverRightStickPressed;
  public static BooleanSupplier isOperatorUpPressed;
  public static BooleanSupplier isOperatorDownPressed;

  private static Map<Integer, Command> climbSteps = new HashMap<>();

  static {
      climbSteps.put(0, climbStep0());
      climbSteps.put(1, climbSteps1Through4());
      climbSteps.put(2, climbStep6());
  }

  public static void init() {
    configureButtonBindings();

    drive.setDefaultCommand(drive.driveWithJoysticks());

    shotplanner.initialize();

    drive.setPose(new Pose2d());
  }

  private static void configureButtonBindings() {
    driver.b().onTrue(Commands.runOnce(drive::seedFieldRelative));
    driver.back().onTrue(drive.toggleSlowMode());

    isOperatorUpPressed = operator.povUp();
    isOperatorDownPressed = operator.povDown();

    driver.rightBumper().whileTrue(aimTeleop());

    driver.rightTrigger()
      .onTrue(
        Commands.either(
          feeder.shoot().alongWith(
            Commands.either(
              Commands.parallel(
                shooter.hailMary(),
                wrist.hailMary(),
                arm.hailMary()
              ),
              shooter.shootIntoAmp(),
              isOperatorDownPressed
            )
          ),
          shootSequence(),
          Logic.or(isOperatorUpPressed, isOperatorDownPressed)
        ).alongWith(Visualizer.getInstance().visualizeShotNote())
      )
      .onFalse(
        cancelHint()
      );
    
    driver.leftTrigger().whileTrue(intake());
    driver.leftBumper().whileTrue(fetch());

    // FOR WHEN NO CAMERAS: Reset the pose to a known location (on the center spike mark intake facing away from driverstation)
    driver.povUp()
      .onTrue(Commands.runOnce(() -> {
        drive.setPose(red() ? new Pose2d(FieldConfig.getInstance().NOTE_B, new Rotation2d(Math.PI)) : new Pose2d(FieldConfig.getInstance().NOTE_J, new Rotation2d()));
      }));
;      
    driver.start().whileTrue(drive.driveRobotCentric());

    driver.a().whileTrue(intake.eject().alongWith(feeder.eject()));

    operator.rightTrigger().whileTrue(
      Commands.either(
        Commands.parallel(
          feeder.trapShoot().beforeStarting(waitSeconds(1.0)),
          shooter.shootIntoTrap()
        ),
        Commands.none(),
        () -> climbStep >= 1
      )
    );
    operator.leftTrigger().whileTrue(shooter.poop().alongWith(feeder.shoot().beforeStarting(Commands.waitSeconds(0.5))));

    Trigger intakeGotNote = new Trigger(feeder::noteInExitSensor);
    intakeGotNote.onTrue(driverfeedback.signalIntakeGot().withTimeout(1.0));

    Trigger insideAimingRegion = new Trigger(() -> 
      red() ? 
      drive.getPose().getX() >= FieldConfig.getInstance().RED_WING_LINE - 1.0 :
      drive.getPose().getX() <= FieldConfig.getInstance().BLUE_WING_LINE + 1.0
    );

    Trigger isHinting = new Trigger(() -> shotplanner.isHintingWithAng);

    Trigger isTeleopEnabled = new Trigger(() -> DriverStation.isTeleopEnabled());

    autoAim = intakeGotNote.and(insideAimingRegion.or(isHinting)).and(isTeleopEnabled).and(operator.povUp().negate()).and(operator.povDown().negate());
    autoAim.onTrue(startAim().onlyIf(() -> climbStep < 1 && wrist.currentGoal != Goal.AMP && wrist.currentGoal != Goal.AUTON_HINT));
    autoAim.onFalse(Commands.parallel(
      arm.setArmDown(),
      wrist.setWristDown(),
      shooter.setToIdle()
    ).onlyIf(() -> climbStep < 1 && wrist.currentGoal != Goal.AMP && !isOperatorUpPressed.getAsBoolean()));

    isAutoAim = autoAim::getAsBoolean;
    
    operator.povUp().and(() -> climbStep < 1).whileTrue(setToAmp());
    operator.povLeft().whileTrue(setAimingModeWhileHeld(Mode.COBRA));
    operator.povRight().whileTrue(setAimingModeWhileHeld(Mode.FRONT_BATTER));
    operator.povDown().whileTrue(setToHailMary());
    
    // ----------------------------------- SHOT PRESETS ------------------------------------ //

    operator.a().onTrue(selectPreset(Presets.BATTER));
    operator.b().onTrue(selectPreset(Presets.PODIUM));
    operator.x().onTrue(selectPreset(Presets.CENTRAL));

    operator.y().onTrue(cancelHint());

    // ----------------------------------- NORMAL CLIMB ------------------------------------ //

    operator.start().onTrue(
      Commands.select(climbSteps, () -> climbStep).beforeStarting(() -> {
        climbStep++;
        System.out.println("THE CLIMB STEP IS " + climbStep);
        if (climbStep >= 2) {
          climbStep = 2;
        }
      })
      .deadlineWith(Commands.waitSeconds(0.5).andThen(Commands.print(climbStep + "")))
    );

    // ----------------------------------- FORWARD CLIMB ----------------------------------- //

    operator.rightBumper().onTrue(extendClimbers_BACKWARD());
    operator.back().onTrue(retractClimbers_BACKWARD());

    // ------------------------------------------------------------------------------------- //

    operator.leftBumper().onTrue(climbStep0());

    
    driver.povDown().whileTrue(
      Commands.parallel(
        intake.eject(),
        feeder.eject(),
        shooter.reverse()
      )
    ); // basically intake note from shooter


    isDriverRBPressed = driver.rightBumper();
    isDriverRightStickPressed = driver.rightStick();

    if (RobotBase.isSimulation()) {
      Visualizer.getInstance().addFloorNotes(new Translation2d(1.0, 3.0));
    }

    driver.povLeft().onFalse(
      idle()
    );

    driver.povRight().onTrue(
      drive.getChoreoSwerveCommandFromTrajectory("NewPath")
    );
  }

  // Put any buttons that map to actions requiring data updates
  public static void periodic() {
    // driver.povLeft().whileTrue(
    //   Commands.sequence(
    //     lineupplanner.lineupTo(LineupGoal.AMP),
    //     selectPreset(Presets.AMP),
    //     waitSeconds(1.0),
    //     shootSequence()
    //   )
    // );

    // driver.povRight().onTrue(
    //   lineupplanner.pathing_DOP()
    // );
  }

  public static Command idle() {
    return Commands.parallel(
      cancelHint(),
      arm.setArmDown(),
      wrist.setWristDown(),
      shooter.setToIdle()
    );
  }

  public static Command startAim() {
    return wrist.aim().alongWith(arm.aim(), shooter.rampUp());
  }

  public static boolean red() {
    return RobotStatus.getInstance().getAllianceColor() == AllianceColor.RED;
  }

  public static Command climbStep0() {
    return Commands.parallel(climber.leftUnbucklePct(), climber.rightUnbucklePct());
  }

  public static Command climbSteps1Through4() {
    return Commands.sequence(
      shooter.climbOff(),
      arm.climbStep3(), // to 107
      Commands.waitUntil(() -> arm.getPosition() >= 30).alongWith(print("WAITING FOR ARM")),
      Commands.parallel(
        wrist.climbStep2(), // to 150
        climbStep4()
      )
    );
  }

  public static Command climbStep4() {
    return Commands.sequence(
      Commands.parallel(climber.leftUnbucklePct(), climber.rightUnbucklePct()),
      Commands.waitSeconds(0.5),
      Commands.parallel(climber.leftExtendPct(), climber.rightExtendPct())
    );
  }

  public static Command climbStep6() {
    return Commands.parallel(
      print("climb step 6"),
      climber.leftRetractPct(), 
      climber.rightRetractPct(),
      climbStep7().beforeStarting(Commands.waitUntil(() -> climber.getLeftPos() < 47 && climber.getRightPos() < 47))
    );
  }

  public static Command climbStep7() {
    return Commands.sequence(
      arm.climbStep7(),
      wrist.climbStep7().beforeStarting(Commands.waitUntil(arm::atGoal))
    );
  }

  public static Command extendClimbers_BACKWARD() {
    return Commands.sequence(
      Commands.parallel(climber.leftUnbucklePct(), climber.rightUnbucklePct()),
      Commands.waitSeconds(0.5),
      Commands.parallel(climber.leftExtendPct_BACKWARD(), climber.rightExtendPct_BACKWARD())
    );
  }

  public static Command retractClimbers_BACKWARD() {
    return Commands.parallel(
        climber.leftRetractPct_BACKWARD(), 
        climber.rightRetractPct_BACKWARD(),
        Commands.runOnce(() -> shooter.off(), shooter) // Makes sure chain doesn't rip up the wheels
    );
  }

  public static boolean atLoad() {
    return arm.getPosition() < 10 && wrist.getPosition() < 30;
  }

  @Deprecated
  public static Command hint(double distanceForHint) {
    return hint(() -> distanceForHint).beforeStarting(shooter.rampUp());
  }
  

  public static Command hint(Supplier<Double> distanceForHint) {
    return Commands.runOnce(() -> {
      shotplanner.setHintWithDistance(distanceForHint.get());
    });
  }

  public static Command hintAng(double armAng, double wristAng) {
    return Commands.runOnce(() -> shotplanner.setHintWithAng(armAng, wristAng));
  }

  public static Command hintAng(Pair<Double, Double> aim) {
    return hintAng(aim.getFirst(), aim.getSecond());
  }

  public static Command hintAuton(Pair<Double, Double> hint) {
    return Commands.parallel(
      RobotContainer.shooter.rampUp(),
      RobotContainer.hintAng(hint)
    );
  }

  public static Command hintAuton(Pair<Double, Double> hint, double timeout) {
    return hintAuton(hint).withTimeout(timeout);
  }

  public static Command hintAuton(Double armAng, Double wristAng) {
    return hintAuton(Pair.of(armAng, wristAng));
  }

  public static Command hintAuton(Double armAng, Double wristAng, double timeout) {
    return hintAuton(armAng, wristAng).withTimeout(timeout);
  }

  @Deprecated
  public static Command autonHint(double distance) {
    Command seq = Commands.parallel(
      shooter.rampUp(),
      wrist.autonHint(distance),
      arm.autonHint(distance)
    );
    seq.addRequirements(shooter, wrist, arm);
    seq.setName("AutonHint");
    return seq;
  }
  
  public static Command cancelHint() {
    return Commands.parallel(
      Commands.runOnce(() -> shotplanner.cancelHint())
    )
    .onlyIf(Logic.and(
      () -> (climbStep < 1),
      () -> (arm.currentGoal != org.frogforce503.robot2024.subsystems.Arm.Goal.AMP),
      () -> !isHailMary
    ));
  }

  public static Command setAimingModeWhileHeld(ShotPlanner.Mode mode) {
    return Commands.runEnd(() -> shotplanner.setMode(mode), () -> shotplanner.setMode(Mode.NORMAL));
  }

  @Deprecated
  public static Command aim() {
    Command seq = Commands.parallel(
      shooter.rampUp(),
      wrist.aim().onlyIf(() -> wrist.currentGoal != Goal.AUTON_HINT),
      arm.aim().onlyIf(() -> wrist.currentGoal != Goal.AUTON_HINT)
    );
    seq.addRequirements(shooter, wrist, arm);
    seq.setName("Aim");

    return Commands.either(Commands.none(), seq, () -> RobotContainer.areCamerasDisabled);
  }

  public static Command intake() {
    Command seq = 
    Commands.sequence(
      Commands.waitUntil(RobotContainer::atLoad),
      Commands.parallel(intake.intake(), feeder.intake()).until(() -> feeder.noteInExitSensor())
    ).onlyIf(() -> !feeder.noteInExitSensor());

    seq.addRequirements(intake, feeder);
    seq.setName("intakefull");

    return seq;
  }

  public static Command fetch() {
    return intake().alongWith(
      drive.chaseNote(),
      driverfeedback.fetch()
    ).withName("fetch");
  }

  public static Command aimTeleop() {
    return drive.aimTeleop();
  }

  @Deprecated
  public static Command shootContinuous() {
    return Commands.parallel(
      feeder.shootContinous(),
      intake.intake()
    );
  }

  public static Command shootSequence() {
    Command seq = Commands.parallel(
      startAim(),

      Commands.parallel(feeder.shoot(), intake.shootAssist())
      .beforeStarting(
        Commands.waitUntil(
          Logic.or(Logic.and(wrist::atGoal, arm::atGoal, shooter::atTargetVelocity), () -> RobotContainer.isHailMary, () -> shotplanner.currentMode == Mode.FRONT_BATTER || shotplanner.currentMode == Mode.COBRA)
        )
        .withTimeout(1.5) // might have to decrease timeout for auton or create specific shootSequence() for auton (basically copy-paste this method, rename it shootSequence_AUTON, & lower timeout)
      )
    ).until(() -> !feeder.noteInExitSensor());

    seq.addRequirements(feeder);
    seq.setName("Shoot Sequence");

    return seq;
  }

  @Deprecated
  public static Command shootSequenceIgnoreAngle() {
    Command seq = Commands.parallel(
      Commands.parallel(feeder.shoot()).beforeStarting(Commands.waitUntil(shooter::atTargetVelocity))
    )
    .until(() -> !feeder.noteInExitSensor() && !feeder.noteInEntrySensor());

    seq.addRequirements(feeder);
    seq.setName("Shoot Sequence");

    return seq.alongWith(Visualizer.getInstance().visualizeShotNote());
  }

  public static Command setToAmp() {
    return wrist.amp().alongWith(arm.amp()).alongWith(shooter.shootIntoAmp()).andThen(print("STOOPPED AMMPING")).deadlineWith(print("AMP"));
  }

  public static Command setToHailMary() {
    return Commands.runOnce(() -> RobotContainer.isHailMary = true).alongWith(wrist.hailMary()).alongWith(shooter.hailMary()).alongWith(arm.hailMary());
  }

  public static Command selectPreset(Presets preset) {
    return Commands.parallel(
      Commands.runOnce(() -> shotplanner.setHintWithAng(preset))
    );
  }

  public static Command setArmAndWristDown() {
    return wrist.setWristDown().alongWith(arm.setArmDown());
  }

  @Deprecated
  public static Command poopOutNote() {
    return Commands.parallel(
      shooter.poop(),
      feeder.shoot().beforeStarting(Commands.waitSeconds(0.5)),
      intake.shootAssist()
    );
  }

  @Deprecated
  public static Command setToSpit() {
    return shooter.spit();
  }

  @Deprecated
  public static Command spitNote() {
    return feeder.shoot().beforeStarting(Commands.waitUntil(shooter::atSpitSpeed))
      .until(Logic.not(feeder::noteInExitSensor))
      .andThen(shooter.rampUp());
  }

  /**
   * Intakes note fully & uses that time to aim (if holonomic angle is off in path)
   */
  public static Command autonAdjust() {
    return RobotContainer.intake().alongWith(RobotContainer.aimTeleop().until(RobotContainer.feeder::noteInExitSensor));
  }
}