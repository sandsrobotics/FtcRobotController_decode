package org.firstinspires.ftc.teamcode.parts.intake3.settings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.HashMap;
import java.util.Map;
import om.self.ezftc.utils.Vector3;

@Config
public class IntakeSettings3 {
    public static double launchServo0Launch = 0.367;
    public static double launchServo0Rest = 0.645; //0.687
    public static double launchServo1Launch = 0.5;
    public static double launchServo1Rest = 0.35; //.267
    public static double launchServo2Launch = 0.5;
    public static double launchServo2Rest = 0.267;
    public static int launchServoDelay = 200;
    public static int launchServoSweepTime = 1000;
    public static int launchRPMTolerance = 200;
    public static Map<String, LaunchData> launchPosiMap = new HashMap<String, LaunchData>();
    public static PIDFCoefficients spinnerPID = new PIDFCoefficients(100,0,0,12.4);
    public static int launchRPM = 3150; // for teleop tests
    public static int intakeRPM = 600;
    public static double ticksPerRev = 28;
    public static double ticksPerRev1150 = 145.1;

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
