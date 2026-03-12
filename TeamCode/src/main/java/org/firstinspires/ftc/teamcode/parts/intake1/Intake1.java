package org.firstinspires.ftc.teamcode.parts.intake1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.parts.artifact.ArtifactDetectionPipeline;
import org.firstinspires.ftc.teamcode.parts.artifact.Artifacts;
import org.firstinspires.ftc.teamcode.parts.drive.headeraimer.HeaderAimer;
import org.firstinspires.ftc.teamcode.parts.limelight.LimeLight;
import org.firstinspires.ftc.teamcode.parts.intake1.hardware.Intake1Hardware;
import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1Settings;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.positionsolver.HeadingSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.pinpoint.Pinpoint;

import java.util.Arrays;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;

public class Intake1 extends ControllablePart<Robot, Intake1Settings, Intake1Hardware, Intake1Control> {

    public Intake1Tasks tasks;
    protected Drive drive;
    protected Pinpoint pinpoint;
    protected PositionSolver positionSolver;
    protected PositionTracker positionTracker;
    protected HeadingSolver headingSolver;  // LK New Test
    protected HeaderAimer headerAimer;      // LK New Test

    public int launchRPM;
    public boolean autoRPM = false;
    public boolean launchOff = false;

    public Artifacts artifacts;
    protected ArtifactDetectionPipeline artifactPipeline;
    protected LimeLight limeLight;

    public static char[] classificationOrder21 = {'G', 'P', 'P'};
    public static char[] classificationOrder22 = {'P', 'G', 'P'};
    public static char[] classificationOrder23 = {'P', 'P', 'G'};

    private double spinnerSliderPower = 0.0;// what is this?
    public int lastHue;

    // for testing PID
    public PIDFCoefficients pidf_rue = new PIDFCoefficients();
    public PIDFCoefficients pidf_rtp = new PIDFCoefficients();

    public static PIDFCoefficients launchSpinPID = new PIDFCoefficients(100,0,0,12.4);
//    public PIDFCoefficients sample_pidf_rue = new PIDFCoefficients();
//    public PIDFCoefficients sample_pidf_rtp = new PIDFCoefficients();
    public float pIncrement = 1;

    public boolean preventUserControl = false;

    //***** Constructors *****
    public Intake1(Robot parent) {
        super(parent, "Slider", () -> new Intake1Control(0));
        setConfig(
                Intake1Settings.makeDefault(),
                Intake1Hardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake1(Robot parent, Intake1Settings settings, Intake1Hardware hardware) {
        super(parent, "slider", () -> new Intake1Control(0));
        setConfig(settings, hardware);
    }

    private void setMotorsToRunConfig() {
        getHardware().intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void initializeServos() {
        getHardware().pinkServo.setPosition(Intake1Settings.servoPinkLow);
        getHardware().blueServo.setPosition(Intake1Settings.servoBlueLow);
        getHardware().greenServo.setPosition(Intake1Settings.servoGreenDock);
        getHardware().intakeServo.setPosition(Intake1Settings.intakeServoOff);
        getHardware().gateServo.setPosition(Intake1Settings.servoGateOpen);
    }

    public void initializeMotors() {
        getHardware().intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        getHardware().launchMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        getHardware().launchMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        getHardware().launchMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        getHardware().launchMotorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,launchSpinPID);

        getHardware().launchMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        getHardware().launchMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        getHardware().launchMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        getHardware().launchMotorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,launchSpinPID);
    }

    public int getCurrentLaunchMotorRPM() {
        int launchMotorVelocity = (int) getHardware().launchMotorLeft.getVelocity();
        return (int) ((launchMotorVelocity/Intake1Settings.ticksPerRevolution)*60);
    }

    public void setLaunchMotors(int launchRPM) {
        this.launchRPM = launchRPM;
        launchOff = launchRPM == 0;
        getHardware().launchMotorLeft.setVelocity(launchRPM / 60.0 * Intake1Settings.ticksPerRevolution);
        getHardware().launchMotorRight.setVelocity(launchRPM / 60.0 * Intake1Settings.ticksPerRevolution);
    }

    public void setLaunchRPM (int launchRPM) {
        this.launchRPM = launchRPM;
    }

    public int getLaunchRPM () {
        return this.launchRPM;
    }

    public boolean launchRPMInTolerance() {
        //TODO: Improve this to disallow overspeed and account for speed lower than setting
//        int under = 100;  // expected undershoot
//        int target = launchRPM - under;  // adjusted target speed accounting for undershoot
//        int diff = Math.abs(getCurrentLaunchMotorRPM() - target);  // difference between actual and target
//        return diff <= Intake1Settings.launchRPMTolerance;
        return ((getHardware().launchMotorLeft.getVelocity() * 60) / Intake1Settings.ticksPerRevolution) >= (this.launchRPM - Intake1Settings.launchRPMTolerance);
    }

    public boolean launchRPMInToleranceV2() {
        if (launchRPM == 0) return false;  // don't want to shoot if launcher speed is 0
        // todo: figure out if undershoot is consistent or should be a multiplier
        int target = launchRPM - Intake1Settings.launchRPMToleranceV2Undershoot;  // adjusted target speed accounting for undershoot
        int diff = Math.abs(getCurrentLaunchMotorRPM() - target);  // difference between actual and target
        return diff <= Intake1Settings.launchRPMToleranceV2;
        // below does the same thing, but isn't the above easier to read?
        // return Math.abs(getCurrentLaunchMotorRPM() - (launchRPM - Intake1Settings.launchRPMToleranceV2Undershoot)) <= Intake1Settings.launchRPMToleranceV2;
    }

    // Based on GameClassificationId, compute Launch Order.
    public int[] computeLaunchOrder(Integer classificationId) {

        // Setup "desiredOrder" based on classificationId.
        char [] desiredOrder = (classificationId == 21) ? classificationOrder21 :
                (classificationId == 22 ) ? classificationOrder22 :
                (classificationId == 23 ) ? classificationOrder23 : classificationOrder21; // default is classificationOrder21.

        int[] launchOrder = new int[3];

        // Cleanse "current" artifact data.
        ArtifactDetectionPipeline.Artifact[] current = artifacts.getArtifactList();

        char[] currentColors = new char[3];
        for (int i  = 0; i < 3; i++) {
           if (current[i].color.name().equalsIgnoreCase("purple")) {
               currentColors[i] = 'P';
           }
           else if (current[i].color.name().equalsIgnoreCase("green")) {
                currentColors[i] = 'G';
           }
           else if (current[i].color.name().equalsIgnoreCase("none")) {
               currentColors[i] = 'P';     // Defaulting to 'P'.
            }
        }

        // Need One-Green and Two-Purples to make a valid Pattern.
        int countG = 0; int countP = 0;
        for (int i = 0; i < 3; i++) {
            if (currentColors[i] == 'P') countP++;
            else if (currentColors[i] == 'G') countG++;
        }

        boolean validPattern =  (countG == 1 && countP == 2) ? true : false;
        if (!validPattern) {            // Default Order (0,1,2).
            for (int i = 0; i < 3; i++) {
                launchOrder[i] = i;
            }
        } else {  // Compute Launch Order.
            boolean[] used = new boolean[current.length];
            for (int i = 0; i < desiredOrder.length; i++) {
                char wanted = desiredOrder[i];
                for (int j = 0; j < currentColors.length; j++) {
                    if (!used[j] && currentColors[j] == wanted) {
                        launchOrder[i] = j;
                        used[j] = true;
                        break;
                    }
                }
            }
        }

        parent.opMode.telemetry.addData("Current Colors", Arrays.toString(currentColors));
        parent.opMode.telemetry.addData("Launch Order", Arrays.toString(launchOrder));

        return launchOrder;

        // launchServos in defaultOrder.
//        this.tasks.pinkBlueGreenServoLaunch.restart();

//        // launch Servos in DesiredOrder.
//        for (int idx : launchOrder) {
//            launchServoForIndex(idx);
//            parent.opMode.sleep(300);
//        }
    }

    private void launchServoForIndex(int index) {
        switch (index) {
            case 0:
                this.tasks.greenServoLaunch.restart();
                break;
            case 1:
                this.tasks.blueServoLaunch.restart();
                break;
            case 2:
                this.tasks.pinkServoLaunch.restart();
                break;
        }
    }

    public void eStop() {
        preventUserControl = false;
        // TODO: Stop LaunchMotors?
        this.getHardware().launchMotorLeft.setVelocity(0);
        this.getHardware().launchMotorRight.setVelocity(0);
        this.getHardware().intakeServo.setPosition(Intake1Settings.intakeServoOff);

        // stop all tasks in the intake group
        stopAllIntakeTasks();
        //stop the position solver?
        positionSolver.stopSolver();
    }

    public void stopAllIntakeTasks() {
        preventUserControl = false;
        tasks.intakeTasksGroup.runCommand(Group.Command.PAUSE);
        tasks.intakeTasksGroup.getActiveRunnables().clear(); // this is the magic sauce... must be used after the PAUSE or it will stop working
    }

    private static int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(val, max));
    }

    @Override
    public void onInit() {
        initializeMotors();
        // TODO: InitServos during "Start"?
        if (DecodeSettings.isAuto()) {
            initializeServos();
            if (DecodeSettings.firstRun) {
                // the first time the servo controller comes online the positions set may be lost, so wait and try again
                DecodeSettings.firstRun = false;
                parent.opMode.sleep(1500);
                initializeServos();
            }
        }
        pinpoint = getBeanManager().getBestMatch(Pinpoint.class, false);
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false);  // should this be here or onBeanLoad()???
        tasks = new Intake1Tasks(this, parent);
        tasks.constructAllIntakeTasks();

        //==== pre-positioned example code (not an Om part, so needs an init and a periodic run)
        LedStick.init(parent.opMode.hardwareMap);
    }


    @Override
    public void onBeanLoad() {
        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        artifacts = getBeanManager().getBestMatch(Artifacts.class, false);
//        artifactPipeline = getBeanManager().getBestMatch(ArtifactDetectionPipeline.class, false);
        limeLight = getBeanManager().getBestMatch(LimeLight.class, false);
        // LK New Test
        if (DecodeSettings.lkTestMode1) headingSolver = getBeanManager().getBestMatch(HeadingSolver.class, false);
        if (DecodeSettings.lkTestMode2) headerAimer = getBeanManager().getBestMatch(HeaderAimer.class, false);

    }

    @Override
    public void onRun(Intake1Control control) {
        spinnerSliderPower = 0.0; // control.strafePower;
        if (autoRPM && !launchOff) {
            int rpm = (int)(calcSpinnerRPM());
            if (rpm > 0) setLaunchMotors(rpm);
//            setLaunchMotors((int)calcSpinnerRPM());
        }
        //==== pre-positioned example code (not an Om part, so needs an init and a periodic run)
        LedStick.runLoop();
    }

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
        if (DecodeSettings.isTeleOp()) {
            initializeServos();
            if (DecodeSettings.firstRun) {
                // the first time the servo controller comes online the positions set may be lost, so wait and try again
                DecodeSettings.firstRun = false;
                parent.opMode.sleep(1500);
                initializeServos();
            }
        }
    }

    @Override
    public void onStop() {
        LedStick.stop();
        drive.removeController(ControllerNames.distanceController);
    }

    public static final class ControllerNames {
        public static final String distanceController = "distance controller";
    }

    // LK demo additions

    public static final double nearest = 64;  // 1 tile diagonally
    public static final double farthest = 152;
    public static final double spinNear = 2450; //2400; 2500;
    public static final double spinFar = 3000; // 3200; 3300;
//    public static Vector3 storedTarget = new Vector3();
    public Vector3 storedTarget = DecodeSettings.targetBlue;

    public double calcSpinnerVelocity() {
        return calcSpinnerRPM() / 60 * Intake1Settings.ticksPerRevolution;
    }

    public double calcSpinnerRPM() {
        return getSpinnerRPMfromDistance(calculateTargetDistance(positionTracker.getOverridePosition(), storedTarget));
    }

    public int getLaunchMotorRPM(int RPM) {
        return (RPM*28/60);
    }

    public static double getSpinnerRPMfromDistance(double distance) {
        return interpolate(distance, nearest, farthest, spinNear, spinFar);
    }

    public static double calculateTargetDistance(Vector3 currentPos, Vector3 target) {
        // Exit if either position is null
        if (currentPos==null) return -1;
        if (target==null) return -1;

        // Calculate the vector from the current position to the target
        double x = target.X - currentPos.X;
        double y = target.Y - currentPos.Y;
        double distance = Math.sqrt(x*x + y*y);
//        double angle = AngleMath.scaleAngle(Math.toDegrees(Math.atan2(y, x)));

        // Return the distance
        return distance;
    }

    public static double interpolate(double x, double x1, double x2, double y1, double y2) {
        return y1 + (x-x1)*(y2-y1)/(x2-x1);
    }

    public void toggleAutoRPM() {
        autoRPM = !autoRPM;
        if (autoRPM) launchOff = false;
    }

    // Helper to nearLaunch Servo in Order.
    public void nearLaunchInOrder (int index) {
        switch (index) {
            case 0:
                this.tasks.greenServoLaunch.restart();
                break;
            case 1:
                this.tasks.blueServoLaunch.restart();
                break;
            case 2:
                this.tasks.pinkServoLaunch.restart();
                break;
        }
    }

    // Helper to launch Servo in Order.
    public void launchInOrder (int index) {
        switch (index) {
            case 0:
                this.tasks.launchOrderZero.restart();
                break;
            case 1:
                this.tasks.launchOrderOne.restart();
                break;
            case 2:
                this.tasks.launchOrderTwo.restart();
                break;
        }
    }

}

