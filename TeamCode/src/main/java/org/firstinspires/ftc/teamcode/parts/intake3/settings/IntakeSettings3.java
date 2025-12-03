package org.firstinspires.ftc.teamcode.parts.intake3.settings;

import com.acmerobotics.dashboard.config.Config;
import java.util.HashMap;
import java.util.Map;
import om.self.ezftc.utils.Vector3;

@Config
public class IntakeSettings3 {
    public static double launchServoLaunch = 0.9;
    public static double launchServoRest = 0;
    public static int launchServoSweepTime = 1000;
    public static double ticksPerRev = 38;
    public static int launchRPMTolerance = 200;
    public static Map<String, LaunchData> launchPosiMap = new HashMap<String, LaunchData>();

    public IntakeSettings3() {
    }

    public static IntakeSettings3 makeDefault(){
        launchPosiMap.put("blueshoot1", new LaunchData(3000,new Vector3(-12, 12, 135)));
        launchPosiMap.put("blueshoot2", new LaunchData(10,new Vector3(2,0,0)));
        launchPosiMap.put("blueshoot3", new LaunchData(10,new Vector3(3,0,0)));
        launchPosiMap.put("blueshoot4", new LaunchData(10,new Vector3(4,0,0)));
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
