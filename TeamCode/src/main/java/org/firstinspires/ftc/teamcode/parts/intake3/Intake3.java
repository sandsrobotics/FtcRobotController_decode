package org.firstinspires.ftc.teamcode.parts.intake3;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Tools.DataTypes.Vector2D;
import org.firstinspires.ftc.teamcode.Tools.Functions;
import org.firstinspires.ftc.teamcode.parts.artifact.ArtifactDetectionPipeline;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake3.hardware.IntakeHardware3;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;
import java.util.ArrayList;
import java.util.List;

//@Config
public class Intake3 extends ControllablePart<Robot, IntakeSettings3, IntakeHardware3, IntakeControl3> {
    private final EdgeConsumer homingBucketZero = new EdgeConsumer();
    protected Drive drive;
    public Intake3Tasks tasks;
    protected PositionTracker pt;
    protected PositionSolver positionSolver;
    public boolean isTeleop;
    public int launchRPM;
    public int intakeRPM;
    public IntakeSettings3.LaunchData launchData;
    public Artifacts artifacts;
//    protected ArtifactDetectionPipeline artifactPipeline;
    protected LimeLight limeLight;
    public Vector2D targetVector;

    //***** Constructors *****
    public Intake3(Robot parent, String modeName) {
        super(parent, "Slider", () -> new IntakeControl3(false));
        this.isTeleop = modeName.equalsIgnoreCase("Teleop");
        setConfig(
                IntakeSettings3.makeDefault(),
                IntakeHardware3.makeDefault(parent.opMode.hardwareMap)
        );
    }

    /**
     * Main alignment method
     */
    private void alignToTarget(DriveControl control) {
        // Check if target is visible
        double turnPower = 0;
        if(IntakeSettings3.alignTarget) {
            if (limeLight.tv) {
                boolean aligned = isAligned();

                if (!aligned) {
                    turnPower = calculateTurnPower(limeLight.tx);
                    control.power = control.power.addZ(turnPower / 3);
                }
            }
        }
        displayTelemetry(turnPower);
    }

    public boolean isAligned() {
        return Math.abs(limeLight.tx) < IntakeSettings3.HEADING_TOLERANCE;
    }

    public void setLaunchRPM(int RPM) {
        this.launchRPM = RPM;
        double targetTPS = (RPM / 60.0) * IntakeSettings3.ticksPerRev;
        getHardware().launchMotor.setVelocity(targetTPS);
    }

    public double getTargetLaunchRPM() {
        return this.launchRPM;
    }

    public double getCurrentLaunchRPM() {
        return ((getHardware().launchMotor.getVelocity()/IntakeSettings3.ticksPerRev) * 60);
    }

    public boolean launchRPMInTolerance() {
        return ((getHardware().launchMotor.getVelocity() * 60) / IntakeSettings3.ticksPerRev) >= (this.launchRPM - IntakeSettings3.launchRPMTolerance);
    }

    public void setIntakeRPM(int RPM) {
        this.intakeRPM = RPM;
        double targetTPS = (RPM / 60.0) * IntakeSettings3.ticksPerRev1150;
        getHardware().intakeMotor.setVelocity(targetTPS);
    }

    public double getTargetIntakeRPM() {
        return this.intakeRPM;
    }
    /**
     * AUTONOMOUS - Non-blocking method that computes launch order
     * Returns an array of servo indices in the order they should fire.
     */
    public int[] computeLaunchOrderAndLaunch(ArtifactDetectionPipeline.ArtifactColor[] desiredOrder) {
        ArtifactDetectionPipeline.Artifact[] current = artifacts.getArtifactList();

        // Build list of artifact indices that have actual artifacts
        List<Integer> artifactIndices = new ArrayList<>();
        for (int i = 0; i < current.length; i++) {
            if (current[i].color != ArtifactDetectionPipeline.ArtifactColor.NONE) {
                artifactIndices.add(i);
            }
        }

        // No artifacts detected - return null for backward compatibility
        if (artifactIndices.isEmpty()) {
            parent.opMode.telemetry.addData("Launch", "No artifacts");
            parent.opMode.telemetry.update();
            return null;
        }

        // No desired order specified - use sequential
        if (desiredOrder == null) {
            parent.opMode.telemetry.addData("Launch", "No pattern, sequential order");
            parent.opMode.telemetry.update();
            return artifactIndices.stream().mapToInt(Integer::intValue).toArray();
        }

        // Artifact count doesn't match pattern - use sequential
        if (artifactIndices.size() != desiredOrder.length) {
            parent.opMode.telemetry.addData("Launch", "No pattern, sequential order");
            parent.opMode.telemetry.update();
            return artifactIndices.stream().mapToInt(Integer::intValue).toArray();
        }

        // Try to match colors to desired order
        boolean[] used = new boolean[current.length];
        int[] launchOrder = new int[desiredOrder.length];

        for (int i = 0; i < desiredOrder.length; i++) {
            ArtifactDetectionPipeline.ArtifactColor wanted = desiredOrder[i];
            boolean found = false;

            for (int j : artifactIndices) {
                if (!used[j] && current[j].color == wanted) {
                    launchOrder[i] = j;
                    used[j] = true;
                    found = true;
                    break;
                }
            }

            // If any color not found, fall back to sequential immediately
            if (!found) {
                parent.opMode.telemetry.addData("Launch", "Mismatch, sequential order");
                parent.opMode.telemetry.update();
                return artifactIndices.stream().mapToInt(Integer::intValue).toArray();
            }
        }

        // All colors matched successfully
        parent.opMode.telemetry.addData("Launch", "Pattern matched!");
        parent.opMode.telemetry.update();
        return launchOrder;
    }

    public void launchServoByIndex(int index) {
        switch (index) {
            case 0:
                getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Launch);
                break;
            case 1:
                getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Launch);
                break;
            case 2:
                getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Launch);
                break;
        }
    }

    public boolean servoIsDoneByIndex(int index) {
        switch (index) {
            case 0: return getHardware().launchServo0.isDone();
            case 1: return getHardware().launchServo1.isDone();
            case 2: return getHardware().launchServo2.isDone();
            default: return true;
        }
    }
    /**
     * TELEOP ONLY - Blocking launch method with smart fallback
     * Auto-starts launcher, attempts color matching, falls back to sequential if:
     * - No April tag detected
     * - All 3 artifacts are the same color
     * - Artifact colors don't match the desired pattern
     */
    public void computeLaunchOrderAndLaunchBlocking(ArtifactDetectionPipeline.ArtifactColor[] desiredOrder) {
        // Auto-start launcher if not running
        if (getTargetLaunchRPM() < 500) {
            IntakeSettings3.launchArmed = true;

            // Calculate RPM based on distance immediately
            double distance = getTargetVector(
                    IntakeSettings3.isRedSide ? IntakeSettings3.targetRed : IntakeSettings3.targetBlue
            ).distance;
            int calculatedRPM = (int) getSpinnerRPMfromDistance(distance);

            setLaunchRPM(calculatedRPM);
            parent.opMode.telemetry.addData("Launch", "Starting launcher...");
            parent.opMode.telemetry.addData("Distance", distance);
            parent.opMode.telemetry.addData("Calculated RPM", calculatedRPM);
            parent.opMode.telemetry.update();
        }

        // Wait for RPM once at start
        long startTime = System.currentTimeMillis();
        while (!launchRPMInTolerance()) {
            if (System.currentTimeMillis() - startTime >= IntakeSettings3.launchRPMToleranceTime) {
                parent.opMode.telemetry.addData("Launcher", "RPM timeout - launching anyway");
                parent.opMode.telemetry.update();
                break;
            }
        }

        // Get current artifacts
        ArtifactDetectionPipeline.Artifact[] current = artifacts.getArtifactList();
        List<Integer> artifactIndices = new ArrayList<>();
        for (int i = 0; i < current.length; i++) {
            if (current[i].color != ArtifactDetectionPipeline.ArtifactColor.NONE) {
                artifactIndices.add(i);
            }
        }

        // No artifacts detected - exit
        if (artifactIndices.isEmpty()) {
            parent.opMode.telemetry.addData("Launch", "No artifacts detected!");
            parent.opMode.telemetry.update();
            return;
        }

        // Unlock servos before firing
        getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Unlock);
        parent.opMode.sleep(IntakeSettings3.lockServoUnlockDelay); // Wait for lock servo to fully unlock

        // Determine if color matching should be attempted
        boolean shouldAttemptColorMatch = true;
        String fallbackReason = "";

        // Check 1: No April tag detected
        if (desiredOrder == null) {
            shouldAttemptColorMatch = false;
            fallbackReason = "No April tag detected";
        }
        // Check 2: All artifacts are the same color (only if we still want to try matching)
        else if (artifactIndices.size() >= 2) {
            ArtifactDetectionPipeline.ArtifactColor firstColor = current[artifactIndices.get(0)].color;
            boolean allSameColor = true;
            for (int idx : artifactIndices) {
                if (current[idx].color != firstColor) {
                    allSameColor = false;
                    break;
                }
            }
            if (allSameColor) {
                shouldAttemptColorMatch = false;
                fallbackReason = "All artifacts same color (" + firstColor + ")";
            }
        }

        // Check 3: Wrong number of artifacts vs pattern (only if still want to try)
        if (shouldAttemptColorMatch && artifactIndices.size() != desiredOrder.length) {
            shouldAttemptColorMatch = false;
            fallbackReason = "Artifact count mismatch";
        }

        int[] launchOrder = null;

        // Try color matching if conditions are good
        if (shouldAttemptColorMatch) {
            boolean[] used = new boolean[current.length];
            int[] tempOrder = new int[desiredOrder.length];
            boolean matched = true;

            for (int i = 0; i < desiredOrder.length; i++) {
                boolean found = false;
                for (int j : artifactIndices) {
                    if (!used[j] && current[j].color == desiredOrder[i]) {
                        tempOrder[i] = j;
                        used[j] = true;
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    matched = false;
                    fallbackReason = "Colors don't match pattern";
                    break;
                }
            }

            // Only use matched order if ALL colors matched
            if (matched) {
                launchOrder = tempOrder;
            }
        }

        // Use color order if matched, otherwise sequential
        if (launchOrder == null) {
            launchOrder = artifactIndices.stream().mapToInt(Integer::intValue).toArray();
            parent.opMode.telemetry.addData("Launch", "Sequential - " + fallbackReason);
        } else {
            parent.opMode.telemetry.addData("Launch", "Color matched!");
        }
        parent.opMode.telemetry.update();

        // Fire servos in order
        for (int idx : launchOrder) {
            launchServoByIndex(idx);
            parent.opMode.sleep(IntakeSettings3.launchServoSweepTime + IntakeSettings3.launchServoSettleTime); // Wait for full movement + settling
            resetServoByIndex(idx);
            parent.opMode.sleep(IntakeSettings3.launchServoDelay); // Wait before next launch
        }

        // Re-lock servos after all launches complete
        parent.opMode.sleep(IntakeSettings3.launchServoResetSettleTime); // Wait for last servo to fully reset
        getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Lock);
    }

    private void waitForLauncherToleranceOrTimeout() {
        long startTime = System.currentTimeMillis();

        while (!launchRPMInTolerance()) {
            if (System.currentTimeMillis() - startTime >= IntakeSettings3.launchRPMToleranceTime) {
                parent.opMode.telemetry.addData("Launcher", "RPM timeout");
                parent.opMode.telemetry.update();
                break;
            }
        }
    }
    // Old Blocking Code, Remove this later.
//    private void fireServoForIndexBlocking(int index) {
//        waitForLauncherToleranceOrTimeout();
//        getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Unlock);
//
//        switch (index) {
//            case 0:
//                getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Launch);
//                break;
//            case 1:
//                getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Launch);
//                break;
//            case 2:
//                getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Launch);
//                break;
//        }
//    }

    private void resetServoByIndex(int index) {
        switch (index) {
            case 0:
                getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
                break;
            case 1:
                getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
                break;
            case 2:
                getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
                break;
        }
    }

    public static double getSpinnerRPMfromDistance(double distance) {
        return Functions.interpolate(distance, IntakeSettings3.nearTest, IntakeSettings3.farTest, IntakeSettings3.spinNear, IntakeSettings3.spinFar);
    }

    public Vector2D getTargetVector(Vector3 target) {
        if (target==null || pt==null) return new Vector2D();
        return new Vector2D(pt.getCurrentPosition(), target);
    }

    private double calculateTurnPower(double tx) {
        double power = tx * IntakeSettings3.P_TURN_GAIN;

        // Apply min/max constraints
        if (Math.abs(power) < IntakeSettings3.MIN_TURN_SPEED && Math.abs(limeLight.tx) > IntakeSettings3.HEADING_TOLERANCE) {
            power = IntakeSettings3.MIN_TURN_SPEED * Math.signum(power);
        }

        return Range.clip(power, -IntakeSettings3.MAX_TURN_SPEED, IntakeSettings3.MAX_TURN_SPEED);
    }

    private void displayTelemetry(double turnPower) {
        parent.opMode.telemetry.addData("Target Valid", limeLight.tv ? "YES" : "NO");
        parent.opMode.telemetry.addData("Horizontal Offset (tx)", "%.2f degrees", limeLight.tx);
//        parent.opMode.telemetry.addData("Target Area (ta)", "%.2f%%", limeLight.ta);
        parent.opMode.telemetry.addData("Turn Power", "%.2f", turnPower);
        parent.opMode.telemetry.addData("Aligned", isAligned() ? "YES" : "NO");
    }

    public void eStop() {
        stopAllIntakeTasks();
        setLaunchRPM(0);
        setIntakeRPM(0);
        getHardware().pixel.stop();
        getHardware().launchServo0.stop();
        getHardware().launchServo1.stop();
        getHardware().launchServo2.stop();
        getHardware().lockServo0.stop();
    }

    public void initializeServos() {
        getHardware().launchServo0.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo1.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo2.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().lockServo0.setPosition(IntakeSettings3.lockServoSweepTime);
        getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
        getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
        getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
    }

    @Override
    public void onInit() {
        getHardware().pixel.setPosition(LEDColor.ORANGE.getLedPwm());
        initializeServos();
        parent.opMode.sleep(1200);
        initializeServos();
        tasks = new Intake3Tasks(this, parent);
        tasks.constructAllIntakeTasks();
        getHardware().pixel.setPosition(LEDColor.GREEN.getLedPwm());
    }

    public void stopAllIntakeTasks() {
        tasks.movementTask.runCommand(Group.Command.PAUSE);
        tasks.movementTask.getActiveRunnables().clear();    // this is the magic sauce... must be used after the PAUSE or it will stop working
    }
    @Override
    public void onBeanLoad() {
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        pt = getBeanManager().getBestMatch(PositionTracker.class, false);
        artifacts = getBeanManager().getBestMatch(Artifacts.class, false);
        limeLight = getBeanManager().getBestMatch(LimeLight.class, false);
        drive = getBeanManager().getBestMatch(Drive.class, false);
    }

    @Override
    public void onRun(IntakeControl3 control) {
        if (control.robotEStop) {
            eStop();
        }
        if (IntakeSettings3.isRedSide) {
            targetVector = getTargetVector(IntakeSettings3.targetRed);
        } else {
            targetVector = getTargetVector(IntakeSettings3.targetBlue);
        }
        parent.opMode.telemetry.addData("Target Distance", targetVector.distance);
        // update launch RPM
        if (IntakeSettings3.launchArmed) {
            int launchRPM = (int) getSpinnerRPMfromDistance(targetVector.distance);
            setLaunchRPM(launchRPM);
            parent.opMode.telemetry.addData("Calc launch RPM", launchRPM);
        }
    }

    @Override
    public void onStart() {
        getHardware().pixel.setPosition(LEDColor.OFF.getLedPwm());
        getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Unlock);  // Unlock at start
        drive.addController(Intake3.ControllerNames.alignController, this::alignToTarget);
    }

    @Override
    public void onStop() {
        drive.removeController(Intake3.ControllerNames.alignController);
    }

    public static final class ControllerNames {
        public static final String alignController = "align controller";
    }

    public enum LEDColor {
        OFF("off",0.0),
        RED("Red",0.279),
        ORANGE("Orange",0.333),
        YELLOW("Yellow",0.388),
        SAGE("Sage",0.444),
        GREEN("Green", 0.500),
        AZURE("Azure", 0.555),
        BLUE("Blue", 0.611),
        INDIGO("Indigo",0.666),
        VIOLET("Violet", 0.722),
        WHITE("White", 1.0);

        private final String name;
        private final Double ledPwm;

        public Double getLedPwm(){
            return ledPwm;
        }

        String getName(){
            return name;
        }

        LEDColor(String name, Double ledPwm){
            this.ledPwm = ledPwm;
            this.name = name;
        }
    }
}