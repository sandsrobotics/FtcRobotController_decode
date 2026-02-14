package org.firstinspires.ftc.teamcode.parts.intake3;

import static org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
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
                IntakeSettings3.isAligned = aligned;

                if (!aligned) {
                    turnPower = calculateTurnPower(adjustForDist(limeLight.tx));
                    control.power = control.power.addZ(turnPower / 3);
                } else {
                    control.power = control.power.withZ(0);
                }
            } else {
                IntakeSettings3.isAligned = false;  // no target visible
            }
        } else {
            IntakeSettings3.isAligned = false;  //  alignment not active
        }
        displayTelemetry(turnPower);
    }

    /**
     * // if Launching from tiny triangle half red or blue adjust tx to point to back corner of goal
     * @param tx
     * @return adjusted tx
     */
    private double adjustForDist(double tx) {
        if (pt.getCurrentPosition().X > 40) { // position on tiny triangle half
            if (IntakeSettings3.isRedSide) {
                return tx + limelightFarXOffset; // adjust a bit right of april tag
            } else {
                return tx - limelightFarXOffset; // adjust a bit left of april tag
            }
        } else {
            return tx;
        }
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
     * Colored artifacts matching the pattern fire first, then NONE artifacts.
     */
    public int[] computeLaunchOrder(ArtifactDetectionPipeline.ArtifactColor[] desiredOrder) {
        ArtifactDetectionPipeline.Artifact[] current = artifacts.getArtifactList();

        // Separate colored and NONE artifacts
        List<Integer> coloredIndices = new ArrayList<>();
        List<Integer> noneIndices = new ArrayList<>();

        for (int i = 0; i < current.length; i++) {
            if (current[i].color != ArtifactDetectionPipeline.ArtifactColor.NONE) {
                coloredIndices.add(i);
            } else {
                noneIndices.add(i);
            }
        }

        // If ALL artifacts are NONE, launch them sequentially
        if (coloredIndices.isEmpty()) {
            parent.opMode.telemetry.addData("Launch", "All NONE - sequential order");
            parent.opMode.telemetry.update();
            return noneIndices.stream().mapToInt(Integer::intValue).toArray();
        }

        // No desired order specified - launch colored first, then NONE
        if (desiredOrder == null) {
            parent.opMode.telemetry.addData("Launch", "No pattern - colored first, then NONE");
            parent.opMode.telemetry.update();
            List<Integer> combined = new ArrayList<>(coloredIndices);
            combined.addAll(noneIndices);
            return combined.stream().mapToInt(Integer::intValue).toArray();
        }

        // Try to match colored artifacts to desired order
        boolean[] used = new boolean[current.length];
        List<Integer> launchOrder = new ArrayList<>();

        for (int i = 0; i < desiredOrder.length; i++) {
            ArtifactDetectionPipeline.ArtifactColor wanted = desiredOrder[i];
            boolean found = false;

            for (int j : coloredIndices) {
                if (!used[j] && current[j].color == wanted) {
                    launchOrder.add(j);
                    used[j] = true;
                    found = true;
                    break;
                }
            }

            // If color not found, continue trying to match remaining colors
            if (!found) {
                parent.opMode.telemetry.addData("Launch", "Partial match - missing " + wanted);
            }
        }

        // If we matched at least one color, use that order + remaining colored + NONE
        if (!launchOrder.isEmpty()) {
            // Add any colored artifacts that weren't matched
            for (int j : coloredIndices) {
                if (!used[j]) {
                    launchOrder.add(j);
                }
            }
            // Add NONE artifacts at the end
            launchOrder.addAll(noneIndices);

            parent.opMode.telemetry.addData("Launch", "Pattern matched with NONE last");
            parent.opMode.telemetry.update();
            return launchOrder.stream().mapToInt(Integer::intValue).toArray();
        }

        // No matches - launch all colored first, then NONE
        parent.opMode.telemetry.addData("Launch", "No matches - colored first, then NONE");
        parent.opMode.telemetry.update();
        List<Integer> combined = new ArrayList<>(coloredIndices);
        combined.addAll(noneIndices);
        return combined.stream().mapToInt(Integer::intValue).toArray();
    }

    private double index0Rpm = 0;
    private double index1Rpm = 0;
    private double index2Rpm = 0;

    public void launchServoByIndex(int index) {
        switch (index) {
            case 0:
                index0Rpm = getCurrentLaunchRPM();
                getHardware().launchServo0.setPosition(launchServo0Launch);
                break;
            case 1:
                index1Rpm = getCurrentLaunchRPM();
                getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Launch);
                break;
            case 2:
                index2Rpm = getCurrentLaunchRPM();
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
     * Launches colored artifacts in matched order first, then NONE artifacts.
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

        // Get current artifacts - separate colored and NONE
        ArtifactDetectionPipeline.Artifact[] current = artifacts.getArtifactList();
        List<Integer> coloredIndices = new ArrayList<>();
        List<Integer> noneIndices = new ArrayList<>();

        for (int i = 0; i < current.length; i++) {
            if (current[i].color != ArtifactDetectionPipeline.ArtifactColor.NONE) {
                coloredIndices.add(i);
            } else {
                noneIndices.add(i);
            }
        }

        // If ALL are NONE, launch sequentially and exit
        if (coloredIndices.isEmpty() && noneIndices.isEmpty()) {
            parent.opMode.telemetry.addData("Launch", "No artifacts detected!");
            parent.opMode.telemetry.update();
            return;
        }

        // Unlock servos before firing
        getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Unlock);
        parent.opMode.sleep(IntakeSettings3.lockServoUnlockDelay);

        List<Integer> launchOrder = new ArrayList<>();
        String launchMessage = "";

        // If we have colored artifacts, try to match them
        if (!coloredIndices.isEmpty()) {
            boolean shouldAttemptColorMatch = true;
            String fallbackReason = "";

            // Check 1: No April tag detected
            if (desiredOrder == null) {
                shouldAttemptColorMatch = false;
                fallbackReason = "No April tag detected";
            }
            // Check 2: All colored artifacts are the same color
            else if (coloredIndices.size() >= 2) {
                ArtifactDetectionPipeline.ArtifactColor firstColor = current[coloredIndices.get(0)].color;
                boolean allSameColor = true;
                for (int idx : coloredIndices) {
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

            // Try color matching if conditions are good
            if (shouldAttemptColorMatch) {
                boolean[] used = new boolean[current.length];

                for (int i = 0; i < desiredOrder.length; i++) {
                    boolean found = false;
                    for (int j : coloredIndices) {
                        if (!used[j] && current[j].color == desiredOrder[i]) {
                            launchOrder.add(j);
                            used[j] = true;
                            found = true;
                            break;
                        }
                    }
                }

                // Add any unmatched colored artifacts
                for (int j : coloredIndices) {
                    if (!used[j]) {
                        launchOrder.add(j);
                    }
                }

                if (launchOrder.size() == coloredIndices.size()) {
                    launchMessage = "Color matched!";
                } else {
                    launchMessage = "Partial match";
                }
            } else {
                // Use sequential order for colored artifacts
                launchOrder.addAll(coloredIndices);
                launchMessage = "Sequential - " + fallbackReason;
            }
        }

        // Add NONE artifacts at the end
        launchOrder.addAll(noneIndices);

        if (!noneIndices.isEmpty()) {
            launchMessage += " (NONE last)";
        }

        parent.opMode.telemetry.addData("Launch", launchMessage);
        parent.opMode.telemetry.update();

        // Fire servos in order
        for (int idx : launchOrder) {
            launchServoByIndex(idx);
            parent.opMode.sleep(IntakeSettings3.launchServoSweepTime + IntakeSettings3.launchServoSettleTime);
            resetServoByIndex(idx);
            parent.opMode.sleep(IntakeSettings3.launchServoDelay);
        }

        // Re-lock servos after all launches complete
        parent.opMode.sleep(IntakeSettings3.launchServoResetSettleTime);
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

    private void resetServoByIndex(int index) {
        switch (index) {
            case 0:
                getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
                break;
            case 1:
                getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
                break;
            case 2:
                getHardware().launchServo2.setPosition(launchServo2Rest);
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
//        getHardware().pixel.stop();
        getHardware().launchServo0.stop();
        getHardware().launchServo1.stop();
        getHardware().launchServo2.stop();
        getHardware().lockServo0.stop();
        launchArmed = false;
    }

    public void initializeServos() {
        getHardware().launchServo0.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo1.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo2.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().lockServo0.setSweepTime(IntakeSettings3.lockServoSweepTime); //testing if the servos stall on init.

        Launchers[0] = new IntakeSettings3.Launcher(getHardware().launchServo0, launchServo0Launch, launchServo0Rest);
        Launchers[1] = new IntakeSettings3.Launcher(getHardware().launchServo1, launchServo1Launch, launchServo1Rest);
        Launchers[2] = new IntakeSettings3.Launcher(getHardware().launchServo2, launchServo2Launch, launchServo2Rest);
    }

    public void setInitialServoPositions() {
        getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
        getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
        getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
        getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Lock);
    }

    @Override
    public void onInit() {
//        getHardware().pixel.setPosition(LEDColor.ORANGE.getLedPwm());
        initializeServos();
        parent.opMode.sleep(1200);
        initializeServos();
        tasks = new Intake3Tasks(this, parent);
        tasks.constructAllIntakeTasks();
        setLaunchRPM(0);
        getHardware().launchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, IntakeSettings3.spinnerPID);
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

        parent.opMode.telemetry.addData("Launch Servos", (int)index0Rpm + "," + (int)index1Rpm + "," + (int)index2Rpm);
    }

    @Override
    public void onStart() {
        getHardware().pixel.setPosition(LEDColor.OFF.getLedPwm());
        setInitialServoPositions();
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