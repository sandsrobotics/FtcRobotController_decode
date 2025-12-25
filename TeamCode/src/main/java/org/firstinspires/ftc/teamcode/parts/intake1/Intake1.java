package org.firstinspires.ftc.teamcode.parts.intake1;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.parts.intake1.hardware.Intake1Hardware;
import org.firstinspires.ftc.teamcode.parts.intake1.settings.Intake1Settings;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.consumer.EdgeConsumer;
import om.self.task.core.Group;

public class Intake1 extends ControllablePart<Robot, Intake1Settings, Intake1Hardware, Intake1Control> {

    public Intake1Tasks tasks;
    protected Drive drive;
//    protected Pinpoint pinpoint;
//    protected PositionSolver positionSolver;

    public int slideTargetPosition;
    public int liftTargetPosition;
    private int currentSlidePos;
    private int currentLiftPos;
    public int lastSample = -1;
    public double lastSampleDistance = 10;  // in cm
    public double lastRearDistance = 323;  // in inches
    private double spinnerSliderPower = 0.0;// what is this?
    public double rangePower = 0.40; //todo: finalize
    public boolean rangeIsDone = false;
    public boolean rangeEnabled = false;
    public Vector3 adjustedDestination = null;
    public int lastHue;
    private boolean enableLiftLimit = true;

    // for testing PID
    public PIDFCoefficients pidf_rue = new PIDFCoefficients();
    public PIDFCoefficients pidf_rtp = new PIDFCoefficients();

    public static PIDFCoefficients launchSpinPID = new PIDFCoefficients(100,0,0,12.4);
//    public PIDFCoefficients sample_pidf_rue = new PIDFCoefficients();
//    public PIDFCoefficients sample_pidf_rtp = new PIDFCoefficients();
    public float pIncrement = 1;

    // for testing outtake speed
    public double testSpinnerOut = 0.0;
    public double sIncrement = 0.05;

    public boolean slideIsUnderControl = false;
    public boolean preventUserControl = false;

    // this is part of the resets lift to 0 each time it hits the limit switch
    private final EdgeConsumer homingLiftZero = new EdgeConsumer();
    private final EdgeConsumer homingSlideZero = new EdgeConsumer();

    public final Vector3 p_nearObsZone = new Vector3(33.5, -56.5, 90); // p_12: Position near ObsZone for Pickup-Specimen.
    public final Vector3 p_atObsZone = new Vector3(33.5, -61.5, 90); // p_13: Position at ObsZone for Pickup-Specimen.
    public final Vector3 p_beforeHighRung = new Vector3(2.75 - 6, -40.25 + 1 , -90); // Y: (-40.25 + 5); p_19: Position before High-Rung for Hang-Specimen.

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
        getHardware().pinkServo.setPosition(getSettings().servoPinkDock);
        getHardware().blueServo.setPosition(getSettings().servoBlueDock);
        getHardware().greenServo.setPosition(getSettings().servoGreenDock);
        // apply settings
        if (DecodeSettings.isAuto()) {
//            getHardware().pinkServo.setPosition(getSettings().servoPinkDock);
//            getHardware().blueServo.setPosition(getSettings().servoBlueDock);
//            getHardware().greenServo.setPosition(getSettings().servoGreenDock);
        }
        else {
        }
    }

    public void initilaizeMotors() {
        getHardware().intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        getHardware().launchMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        getHardware().launchMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        getHardware().launchMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        getHardware().launchMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        getHardware().launchMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        getHardware().launchMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        getHardware().launchMotorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,launchSpinPID);
        getHardware().launchMotorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,launchSpinPID);

    }

    public int getLaunchMotorRPM() {
        int launchMotorVelocity = (int) getHardware().launchMotorRight.getVelocity();
        return (launchMotorVelocity/28)*60;
    }








    // ranging - This will not maintain angle.
    // 2m distance sensor
    //




    //   method to get a modified target position based on the distance/range sensor.
//    public Vector3 adjustTargetPositionByRangeY(Vector3 targetPosition, double targetDistance) {
//        //Returns null if a suitable position is not found, otherwise a new target position based on current position
//        //Reads the distance sensor and then gets a new current pinpoint position (to do: work with positiontracker instead)
//        //The X and Z will be preserved, but Y will be changed to reflect best available position data
//        if (targetPosition.Z != 90 && targetPosition.Z != -90) return null;  //only currently works for pure Y
//        final double acceptableDiff = 5;  //how much farther away is OK for calculations?
//        final double acceptableAngle = 5;  //how far can odo angle be from target angle for calculations?
//        getRangeDistance();
//        parent.opMode.telemetry.addData ("ranging:", lastRearDistance);
//        if (lastRearDistance <= targetDistance + acceptableDiff) {   // if it's in acceptable range for calculations...
//            Vector3 currentPosition = pinpoint.getValidPosition();   // get the current odo position
//            if (currentPosition == null) return null;                // if not valid odo position, return null
//            if (Math.abs(currentPosition.Z - targetPosition.Z) > acceptableAngle) return null;   // return null if angle too large
//            double yAdjust = (lastRearDistance - targetDistance) * -1.0 * Math.signum(targetPosition.Z);  // distance + at 90°, - at -90°
//            // new position has original X and Z, but Y is based on currentPosition, targetDistance, and range
//            return new Vector3(targetPosition.X, currentPosition.Y + yAdjust, targetPosition.Z);
//        }
//        return null;
//    }

//    public boolean adjustTarget(Vector3 targetPosition, double targetDistance) {
//        adjustedDestination = adjustTargetPositionByRangeY(targetPosition, targetDistance);
//        return adjustedDestination != null;
//    }
//    public void stopSlide() { getHardware().slideMotor.setPower(0); }
//    public void stopLift() { getHardware().liftMotor.setPower(0); }
//    public void setSlidePower (double m0) { getHardware().slideMotor.setPower(m0); }
//    public void setLiftPower (double m1) { getHardware().liftMotor.setPower(m1); }







    public void eStop() {
        preventUserControl = false;
        // stop all tasks in the intake group
        stopAllIntakeTasks();
        //stop the position solver?
//        positionSolver.stopSolver();
    }

    public void stopAllIntakeTasks() {
        preventUserControl = false;
        tasks.intakeTasksGroup.runCommand(Group.Command.PAUSE);
        tasks.intakeTasksGroup.getActiveRunnables().clear(); // this is the magic sauce... must be used after the PAUSE or it will stop working
    }
    public void strafeRobot(DriveControl control) {
        if (abs(spinnerSliderPower) > .01) {
            control.power = control.power.addX(spinnerSliderPower / 3);
        }
    }

    private static int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(val, max));
    }

    @Override
    public void onInit() {
        initilaizeMotors();
        initializeServos();
        if (DecodeSettings.isAuto()) {
            initializeServos();
            if (DecodeSettings.firstRun) {
                // the first time the servo controller comes online the positions set may be lost, so wait and try again
                DecodeSettings.firstRun = false;
                parent.opMode.sleep(1500);
                initializeServos();
            }
        }
//        pinpoint = getBeanManager().getBestMatch(Pinpoint.class, false);
//        positionSolver = getBeanManager().getBestMatch(PositionSolver.class, false);
        tasks = new Intake1Tasks(this, parent);
        tasks.constructAllIntakeTasks();
    }


    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(Intake1Control control) {
        spinnerSliderPower = 0.0; // control.strafePower;
    }

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
//        drive.addController(Intake.ControllerNames.distanceController, this::strafeRobot);
        if (DecodeSettings.isTeleOp()) initializeServos();
    }

    @Override
    public void onStop() {
        drive.removeController(ControllerNames.distanceController);
    }

    public static final class ControllerNames {
        public static final String distanceController = "distance controller";
    }

}

