package org.firstinspires.ftc.teamcode.parts.limelight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.intake1.DecodeSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import java.util.List;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

public class LimeLight extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    private Limelight3A limelight;
    public LimeLight(Robot parent) {
        super(parent, "limelight");
    }

    // public variables for configuration
    int sizeOfBuffer = 50;                           // number of transforms for averaging & standard deviation
    public double acceptableTx = 17.5;               // acceptable Tx (left/right) in degrees; edges less accurate
    public Vector3 acceptableStdDev = new Vector3(1,1,1);  // Std Dev in x inches, y inches, z degrees
    public long manualIndicatorTimeout = 5000;       // ms to wait before turning off LED
    public long automaticIndicatorTimeout = 500;     // ms to wait before turning off LED
    public boolean automaticTransform = false;       // apply the transform automatically? Otherwise manual
    public String ledServoName = "servo5B";          // name of the servo for the LED indicator

    // internal variables
    protected PositionTracker positionTracker;
    final Vector3 zero = new Vector3(0,0,0);
    int bufferPointer = 0;
    boolean transformValid = false;                  // Flips to true once buffer has filled
    public boolean stdDevValid = false;              // True when the standard deviation is acceptable
    Vector3[] transformBuffer = new Vector3[sizeOfBuffer];     // Holds a buffer of transforms for smoothing
    Vector3 llPosition = new Vector3();              // Holds the robot position reported by the limelight
    Vector3 llLastPosition = new Vector3();          // Holds the last valid position (to verify changing)
    int llStuck = 0;                                 // Counter for determining if the position is changing
    Vector3 llSmoothTransform = new Vector3();       // Holds averaged transform
    Vector3 llStandardDeviation = new Vector3();     // Holds standard deviation of buffer
    public Vector3 llSavedTransform = new Vector3(); // Holds a transform once requested by driver
    Vector3 llLastValidTransform = new Vector3();    // Holds the last valid transform
    double llLastValidTransformTime;                 // Time for tracking LED behavior
    public Vector3 llFusedPosition = new Vector3();  // Holds a transformed position
    double llTimeStamp = 0;                          //
    double llLastTimeStamp = 0;                      //
    boolean isStuck = false;                         //
    Servo ledIndicator;

    // classificationId Defaults to 21.
    // Valid values are 21(GPP), 22(PGP), 23(PPG).
    private static Integer classificationId = 21;

    @Override
    public void onRun() {
        LLStatus status = limelight.getStatus();
//        parent.opMode.telemetry.addData("Name", "%s", status.getName());
//        parent.opMode.telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
//                status.getTemp(), status.getCpu(),(int)status.getFps());
//        parent.opMode.telemetry.addData("Pipeline", "Index: %d, Type: %s",
//                status.getPipelineIndex(), status.getPipelineType());
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
//            Pose3D botpose = result.getBotpose();
//            parent.opMode.telemetry.addData("tx", result.getTx());
//            parent.opMode.telemetry.addData("txnc", result.getTxNC());
//            parent.opMode.telemetry.addData("ty", result.getTy());
//            parent.opMode.telemetry.addData("tync", result.getTyNC());
//            parent.opMode.telemetry.addData("Botpose", botpose.toString());
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                parent.opMode.telemetry.addData("April Tag", "ID: %d", fr.getFiducialId());
                int id = fr.getFiducialId();
                if (id == 21 || id == 22 || id == 23) {
                    classificationId = id;
                }
            }
            // Access color results
//            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//            for (LLResultTypes.ColorResult cr : colorResults) {
//                parent.opMode.telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
//            }
        } else {
            parent.opMode.telemetry.addData("Limelight", "No data available");
        }

        positionTransformLoop(result);
    }

    public Integer getClassificationId() {
        return classificationId;
    }

    @Override
    public void onBeanLoad() {
        positionTracker = getBeanManager().getBestMatch(PositionTracker.class, false, false);
    }

    @Override
    public void onInit() {
        limelight = parent.opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
//        parent.opMode.telemetry.setMsTransmissionInterval(11);
        limelight.start();
        if (ledServoName != null && !ledServoName.isEmpty())
            ledIndicator = parent.opMode.hardwareMap.get(Servo.class, ledServoName);
    }

    @Override
    public void onStart() {}

    @Override
    public void onStop() {
        limelight.stop();
    }

    void positionTransformLoop (LLResult llResult) {

        stdDevValid = false;

        // Clear the LED indicator if timeout has passed
        if (llLastValidTransformTime != 0 && System.currentTimeMillis()-llLastValidTransformTime >
                (automaticTransform ? automaticIndicatorTimeout : manualIndicatorTimeout)) {
            llLastValidTransformTime = 0;
            setLedIndicator(rgbIndicatorColor.Off);
        }

        // Calculate the fused position using the odo position and saved transform
        Vector3 currentPos = positionTracker.getCurrentPosition();
        if (currentPos != null && llSavedTransform != null) {
            llFusedPosition = llSavedTransform.transformPosition(currentPos);
            DecodeSettings.storeFusedPosition(llFusedPosition);
            parent.opMode.telemetry.addData("Fused", llFusedPosition.toString());
        }

        // Verify that the results are updating (not stuck) and abort if not;
        // this deals with LL disconnects where the result stops updating
        llTimeStamp = llResult.getTimestamp();
        parent.opMode.telemetry.addData("LLTIME", llTimeStamp);
        if (llTimeStamp != 0 && llTimeStamp == llLastTimeStamp) {
            llStuck++;
            if (llStuck > 2) {
                isStuck = true;
                setLedIndicator(rgbIndicatorColor.Orange);
                parent.opMode.telemetry.addData("LLPOS", "Stuck / Disconnect");
                finalTelemetry();
                return;
            }
        }
        else if (isStuck) {
            llStuck = 0;
            isStuck = false;
            ////setLedIndicator(rgbIndicatorColor.Off);
            // The above was causing "disco lights" strobing because apparently the limelight on the
            // real bot isn't updating as often as on the test bot?
            // The following hack uses the led clear process above to delay turning off the LED.
            llLastValidTransformTime = System.currentTimeMillis() + 350 -
                    (automaticTransform ? automaticIndicatorTimeout : manualIndicatorTimeout);
        }
        llLastTimeStamp = llTimeStamp;

        if (llResult != null && llResult.isValid()) {

            // Get the robot position as calculated by MegaTag in the LL
            llPosition = new Vector3(
                    llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).x,
                    llResult.getBotpose().getPosition().toUnit(DistanceUnit.INCH).y,
                    llResult.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES));

//            // Verify that the position is updating (not stuck) and abort if not;
//            // this deals with LL disconnects where the result stops updating
//            if (!isPositionChanging()) {
//                setLedIndicator(rgbIndicatorColor.Orange);
//                parent.opMode.telemetry.addData("LLPOS", "Stuck / Disconnect");
//                finalTelemetry();
//                return;
//            }

            // Calculate an "offset" for the tag in the image for the purpose of ignoring
            // positions that are less accurate (e.g., off to the edges of the video frame)
            Vector3 llOffset = new Vector3(
                    llResult.getTx(),  // How far left or right the target is (degrees)
                    llResult.getTy(),  // How far up or down the target is (degrees)
                    llResult.getTa()); // How big the target looks (0%-100% of the image)

            parent.opMode.telemetry.addData("LLPOS", llPosition.toString());

            // Ignore zero position such as when viewing the obelisk (or other reasons?)
            if (llPosition.isEqualTo(zero)) {finalTelemetry(); return;}

            // Don't use the position if the offset is unacceptable.
            // For now, this is based on Tx (left/right), but could add other parameters
            if (Math.abs(llOffset.X) > acceptableTx) {finalTelemetry(); return;}

            // Calculate a transformation Vector3 for rotating the odometry position to the
            // Limelight position. This is a "transformation of coordinates" to rotate
            // xy-Cartesian positions. The odometry position will be rotated by this position
            // to match the Limelight position.
            if (currentPos == null) {finalTelemetry(); return;}
            Vector3 llTransform = currentPos.getOffset(llPosition);

            // Add that transform to the buffer array for averaging (to smooth out noisy data)
            // and to calculate a standard deviation to determine if the buffer is good/stable.
            addTransformToArray(llTransform);
            if (!transformValid) {finalTelemetry(); return;}  // if the buffer isn't filled yet, leave
            llSmoothTransform = getAverageTransform(transformBuffer);
            llStandardDeviation = getStandardDeviation(llSmoothTransform, transformBuffer);

            // If the standard deviation is acceptable, store the transform (and apply it if auto is set)
            stdDevValid = llStandardDeviation != null &&
                    Math.abs(llStandardDeviation.X) <= acceptableStdDev.X &&
                    Math.abs(llStandardDeviation.Y) <= acceptableStdDev.Y &&
                    Math.abs(llStandardDeviation.Z) <= acceptableStdDev.Z;
            if (stdDevValid) {
                llLastValidTransform = llSmoothTransform.copy();
                llLastValidTransformTime = System.currentTimeMillis();
                if (automaticTransform) {
                    setLedIndicator(rgbIndicatorColor.Violet);
                    applyTransform();
                }
                else {
                    setLedIndicator(rgbIndicatorColor.Green);
                }
            }

            finalTelemetry();

        }
        else {
            // if there isn't a valid result, keep the telemetry consistent
//            parent.opMode.telemetry.addData("LLTIME", "n/a");
            parent.opMode.telemetry.addData("LLPOS", "n/a");
            finalTelemetry();
        }
    }

    void finalTelemetry () {
        // Used to keep the telemetry consistent each loop (not bouncing)
        parent.opMode.telemetry.addData("Accpt", acceptableStdDev.toString());
        parent.opMode.telemetry.addData("StDev", llStandardDeviation.toString());
        parent.opMode.telemetry.addData("Trans", llSavedTransform.toString());
        parent.opMode.telemetry.addData("LastV", llLastValidTransform.toString());
    }

    void setLedIndicator(rgbIndicatorColor color) {
        // Update the color of the LED if it exists
        if (ledIndicator != null) ledIndicator.setPosition(color.color);
    }

    public void toggleAuto() {
        // With auto set, the last valid transform will always update the saved transform
        // With auto not set (i.e., manual), the user must manually update the transform by calling applyTransform()
        automaticTransform = !automaticTransform;
    }
    public void setAuto(boolean auto) {
        automaticTransform = auto;
    }

    public void setSizeOfBuffer(int size) {
        // Allows the user to change the size of the buffer array for averaging and standard deviation
        if (size < 2 || size > 100) return;
        sizeOfBuffer = size;
        transformBuffer = new Vector3[sizeOfBuffer];
        bufferPointer = 0;
        transformValid = false;
    }

    public void applyTransform() {
        // Updates the saved transform to the last valid transform
        llSavedTransform = llLastValidTransform.copy();
        positionTracker.setOverrideTransform(llSavedTransform);
        if (!automaticTransform) {
            setLedIndicator(rgbIndicatorColor.Off);
            llLastValidTransformTime = 0;
        }
    }

//    public void applyTransformIfCurrent() {
//        if (stdDevValid) llSavedTransform = llLastValidTransform.copy();
//    }

    public Vector3 getFusedPosition() {
        return llFusedPosition;
    }
    
    void addTransformToArray (Vector3 transform) {
        // Adds transform to the buffer array and increments the pointer
        if (transform == null) return;  //never add null values to the array
        transformBuffer[bufferPointer] = transform;
        bufferPointer++;
        if (bufferPointer >= transformBuffer.length) {
            bufferPointer = 0;
            transformValid = true;  // once the buffer is full, calculations can be performed on it
        }
    }

    Vector3 getAverageTransform(Vector3[] buffer) {
        // Calculate the smoothed transform (add, then divide)
        if (!transformValid) return null;
        double x = 0, y = 0, z = 0;
        for (Vector3 vector3 : buffer) {
            x += vector3.X;
            y += vector3.Y;
            z += vector3.Z;
        }
        return new Vector3(x / buffer.length, y / buffer.length, z / buffer.length);
    }

    Vector3 getStandardDeviation(Vector3 average, Vector3[] buffer) {
        // Calculate the standard deviation
        if (!transformValid || average == null) return null;
        double x = 0, y = 0, z = 0;
        for (Vector3 vector3 : buffer) {
            if (vector3 == null) continue;
            x += Math.pow(vector3.X - average.X, 2);
            y += Math.pow(vector3.Y - average.Y, 2);
            z += Math.pow(vector3.Z - average.Z, 2);
        }
        x = Math.sqrt(x / buffer.length);
        y = Math.sqrt(y / buffer.length);
        z = Math.sqrt(z / buffer.length);
        return new Vector3(x, y, z);
    }

//    public boolean isPositionChanging() {
//        // Compares position to previous position to see if it's changing;
//        // consider it stuck after a certain number of repeats (currently 5)
//        if (llPosition.isEqualTo(llLastPosition) && !llPosition.isEqualTo(zero)) {
//            llStuck++;  // rollover is not a danger
//            if (llStuck >= 5) return false;
//        }
//        else {
//            llStuck = 0;
//            llLastPosition = llPosition.copy();
//        }
//        return true;
//    }

    public enum rgbIndicatorColor {
        Off (0.0),
        Red (0.279),
        Orange (0.333),
        Yellow (0.388),
        Sage (0.444),
        Green (0.500),
        Azure (0.555),
        Blue (0.611),
        Indigo (0.666),
        Violet (0.715), //(0.722),
        White (1.0);

        private final double color;

        rgbIndicatorColor(double color) {
            this.color = color;
        }
    }

}
