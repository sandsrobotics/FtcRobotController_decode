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
    boolean launchArmed = true;

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

    private boolean isAligned() {
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

    public void computeLaunchOrderAndLaunch(ArtifactDetectionPipeline.ArtifactColor[] desiredOrder) {
        ArtifactDetectionPipeline.Artifact[] current = artifacts.getArtifactList();

        // Snapshot: only consider detected artifacts (ignore NONE)
        List<Integer> artifactIndices = new ArrayList<>();
        for (int i = 0; i < current.length; i++) {
            if (current[i].color != ArtifactDetectionPipeline.ArtifactColor.NONE) {
                artifactIndices.add(i);
            }
        }

        // If no valid artifacts, just return
        if (artifactIndices.isEmpty()) return;

        // Attempt color-matched launch using Limelight pattern (closest AprilTag)
        boolean matched = attemptColorMatchedLaunch(artifactIndices, current, desiredOrder);

        if (!matched) {
            // Telemetry for fallback
            parent.opMode.telemetry.addData("Launch", "Pattern not detected or mismatch, launching all artifacts as fallback");
            parent.opMode.telemetry.update();

            // Fallback: launch all remaining artifacts in any order
            for (int idx : artifactIndices) {
                fireServoForIndex(idx);
                parent.opMode.sleep(IntakeSettings3.launchServoSweepTime);
                parent.opMode.sleep(IntakeSettings3.launchServoDelay);
                resetServoForIndex(idx);
            }
        } else {
            // Telemetry for matched pattern
            parent.opMode.telemetry.addData("Launch", "Pattern matched, launching in order");
            parent.opMode.telemetry.update();
        }
    }

    /**
     * Tries to launch artifacts in color-matched order.
     * Returns true if successful, false if not possible (fallback).
     */
    private boolean attemptColorMatchedLaunch(List<Integer> artifactIndices, ArtifactDetectionPipeline.Artifact[] current,
                                              ArtifactDetectionPipeline.ArtifactColor[] desiredOrder) {
        if (desiredOrder == null || artifactIndices.size() != desiredOrder.length) return false;

        // Map desiredOrder to actual artifact indices
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

            if (!found) {
                return false; // Could not match desired order → fallback
            }
        }

        // Fire servos in order
        for (int idx : launchOrder) {
            fireServoForIndex(idx);
            parent.opMode.sleep(IntakeSettings3.launchServoSweepTime);
            parent.opMode.sleep(IntakeSettings3.launchServoDelay);
            resetServoForIndex(idx);
        }

        return true;
    }


    private void fireServoForIndex(int index) {
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

    private void resetServoForIndex(int index) {
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
        getHardware().pixel.stop();
        getHardware().intakeMotor.setVelocity(0);
        getHardware().launchMotor.setVelocity(0);
        getHardware().launchServo0.stop();
        getHardware().launchServo1.stop();
        getHardware().launchServo2.stop();
    }

    public void initializeServos() {
        getHardware().launchServo0.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo1.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo2.setSweepTime(IntakeSettings3.launchServoSweepTime);
        getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
        getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
        getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
    }

    @Override
    public void onInit() {
        getHardware().pixel.setPosition(LEDColor.BLUE.getLedPwm());
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
//        artifactPipeline = getBeanManager().getBestMatch(ArtifactDetectionPipeline.class, false);
        limeLight = getBeanManager().getBestMatch(LimeLight.class, false);
        drive = getBeanManager().getBestMatch(Drive.class, false);
    }

    @Override
    public void onRun(IntakeControl3 control) {
        if (control.robotEStop) {
            eStop();
        }
        // update internal target vector
        targetVector = getTargetVector(IntakeSettings3.targetRed);
        parent.opMode.telemetry.addData("Target Distance", targetVector.distance);
        // update launch RPM
        if (IntakeSettings3.launchArmed) {
            int launchRPM = (int) getSpinnerRPMfromDistance(targetVector.distance);
            setLaunchRPM(launchRPM);
            parent.opMode.telemetry.addData("Launch RPM Interp", launchRPM);
        }
    }

    @Override
    public void onStart() {
        getHardware().pixel.setPosition(LEDColor.OFF.getLedPwm());
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

        Double getLedPwm(){
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