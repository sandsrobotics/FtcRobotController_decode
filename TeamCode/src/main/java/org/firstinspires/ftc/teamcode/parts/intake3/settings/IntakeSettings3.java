package org.firstinspires.ftc.teamcode.parts.intake3.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.HashMap;
import java.util.Map;
import om.self.ezftc.utils.Vector3;

@Config
public class IntakeSettings3 {
    public static final double launchServo0Launch = 0.367;
    public static final double launchServo0Rest = 0.645; //0.687
    public static final double launchServo1Launch = 0.5;
    public static final double launchServo1Rest = 0.35; //.267
    public static final double launchServo2Launch = 0.5;
    public static final double launchServo2Rest = 0.267;
    public static int launchServoDelay = 200;
    public static int launchServoSweepTime = 1000;
    public static int launchRPMTolerance = 200;
    public static final Map<String, LaunchData> launchPosiMap = new HashMap<String, LaunchData>();
    public static PIDFCoefficients spinnerPID = new PIDFCoefficients(100,0,0,12.4);
    public static int launchRPM = 3150; // for teleop tests
    public static final int intakeRPM = 600;
    public static final double ticksPerRev = 28;
    public static final double ticksPerRev1150 = 145.1;

    public static double spinNear                 = 2900; // 3300
    public static final double spinMiddle               = 0;// 3900
    public static double spinFar                  = 3700; // 4500

    public static double nearTest      = 70;  // 1 tile diagonally 40
    public static final double midTest       = 98; //98
    public static double farTest       = 140; // 140

    public static final Vector3 targetRed              = new Vector3(-70.5, 70.5, 0.0);
    public static final Vector3 targetBlue             = new Vector3(-70.5, -70.5, 0.0);

    public static final int redTagId = 24;
    public static final int blueTagId = 20;
    public static boolean launchArmed = false;
    public static boolean alignTarget = false;

    // Alignment constants
    public static final double HEADING_TOLERANCE = 1.0;  // degrees
    public static final double DISTANCE_TOLERANCE = 2.0; // inches or your unit
    public static final double MAX_TURN_SPEED = 0.4;
    public static final double MIN_TURN_SPEED = 0.1;
    public static  double P_TURN_GAIN = 0.05;       // Proportional gain for turning
    public static final double P_DRIVE_GAIN = 0.01;      // Proportional gain for forward/back

    public IntakeSettings3() {}

    public static IntakeSettings3 makeDefault(){
        launchPosiMap.put("blueshoot1", new LaunchData(3200,new Vector3(-12, -12, -139)));
        launchPosiMap.put("blueshoot2", new LaunchData(10,new Vector3(2,0,0)));
        launchPosiMap.put("blueshoot3", new LaunchData(10,new Vector3(3,0,0)));
        launchPosiMap.put("bluefartriangle", new LaunchData(3400,new Vector3(49,16,157)));
        return new IntakeSettings3();
    }

    public static class LaunchData {
        private Integer RPM;
        private Vector3 position;

        public LaunchData(Integer RPM, Vector3 position) {
            this.RPM = RPM;
            this.position = position;
        }

        public Integer getRPM() {return RPM;}
        public Vector3 getPosition() {return position;}
    }
}
