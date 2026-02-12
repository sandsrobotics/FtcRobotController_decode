package org.firstinspires.ftc.teamcode.parts.intake3.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.lib.ServoSSR;

import java.util.HashMap;
import java.util.Map;
import om.self.ezftc.utils.Vector3;

@Config
public class IntakeSettings3 {
    public static  double launchServo0Launch = 0.441; //.367
    public static  double launchServo0Rest = 0.627; //0.654 - ground, Qualifier - 0.62
    public static  double launchServo1Launch = 0.54;
    public static  double launchServo1Rest = 0.35; //.324 - ground, Qualifier - 0.35
    public static  double launchServo2Launch = 0.45; //.5
    public static  double launchServo2Rest = 0.265; //.237 - ground, Qualifier - 0.265
    // lock servo values
    public static  double lockServo0Lock = 0.708; // 0
    public static  double lockServo0Unlock = 0.875; //0.48 //0.843 - last value

    // Add these with your other timing constants
    public static int lockServoUnlockDelay = 300;        // Time to wait for lock servo to fully unlock
    public static int launchServoSettleTime = 100;       // Extra time for servo to settle at launch position
    public static int launchServoResetSettleTime = 300;  // Time to wait for servo to fully reset before locking

    // launch servo values
    public static int launchServoDelay = 100; //use tolerance not delay
    public static int launchServoSweepTime = 400;
    public static int lockServoSweepTime = 400;
    public static int launchRPMTolerance = 75;
    public static int launchRPMToleranceTime = 4500;
    public static final Map<String, LaunchData> launchPosiMap = new HashMap<String, LaunchData>();
    public static PIDFCoefficients spinnerPID = new PIDFCoefficients(100,0,0,12.4);
    public static int launchRPM = 3150; // for teleop tests
    public static int launchAllRPM = 3500;
    public static int intakeRPM = 950;
    public static final double ticksPerRev = 28;
    public static final double ticksPerRev1150 = 145.1;

    public static double spinNear                 = 2450; // 2500 at 72
    public static final double spinMiddle               = 0;//
    public static double spinFar                  = 3050; // 3250 at 137

    public static double nearTest      = 71.9;  // 1 tile diagonally 40
    public static final double midTest       = 98; //98
    public static double farTest       = 148; // 140

    public static final Vector3 targetRed              = new Vector3(-70.5, 70.5, 0.0);
    public static final Vector3 targetBlue             = new Vector3(-70.5, -70.5, 0.0);
    // Add these with your other position definitions (near targetRed, targetBlue, etc.)
    // Add these with your other Vector3 positions

    // SHOOTING AUTOMATIONS ON CONTROLLER
    public static Vector3 endgameBlue = new Vector3(37.6, 33.8, 0); // End Game Blue ABHI: //X:36,Y:36
    public static Vector3 endgameRed = new Vector3(37.6, -33.8, 0); // End Game Red ABHI: //X:36,Y:-36

    public static Vector3 shootingBlue = new Vector3(-20.3, -8.67, -133.667); // Shooting Pos Blue // -26, -14, -128
    public static Vector3 shootingRed = new Vector3(-20.3, 8.67, 133.667); // Shooting Pos Red


    public static Vector3 shootingTriBlue = new Vector3(43.51, 3.603, -150.479); // Shooting Pos Blue
    public static Vector3 shootingTriRed = new Vector3(43.51, -3.603, 150.479); // Shooting Pos Red


    public static Vector3 shootingWallBlue = new Vector3(-57.064, -0.244, -89.097); // Shooting Pos Blue
    public static Vector3 shootingWallRed = new Vector3(-57.064, 0.244, 89.097); // Shooting Pos Red

    //
    public static boolean isRedSide = false;
    public static double limelightFarXOffset = 2; // degrees to shift left on blue and right on red at tiny triangle

    public static final int redTagId = 24;
    public static final int blueTagId = 20;
    public static boolean launchArmed = false;
    public static boolean alignTarget = false;
    public static boolean isAligned = false;

    // Alignment constants
    public static final double HEADING_TOLERANCE = 1.0;  // degrees
    public static final double DISTANCE_TOLERANCE = 2.0; // inches or your unit
    public static double MAX_TURN_SPEED = 0.5;
    public static final double MIN_TURN_SPEED = 0.1;
    public static  double P_TURN_GAIN = 0.05;       // Proportional gain for turning
    public static final double P_DRIVE_GAIN = 0.01;      // Proportional gain for forward/back
    public static final Launcher[] Launchers = new Launcher[3];

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

    public static class Launcher{
        private ServoSSR servo;
        private double launcherServoLaunch;
        private double launcherServoRest;

        public Launcher(ServoSSR servo, double launcherServoLaunch, double launcherServoRest) {
            this.servo = servo;
            this.launcherServoLaunch = launcherServoLaunch;
            this.launcherServoRest = launcherServoRest;
        }

        public ServoSSR getServo() {return servo;}
        public double getLauncherServoLaunch() {return launcherServoLaunch;}
        public double getLauncherServoRest() {return launcherServoRest;}

    }
}
