package org.firstinspires.ftc.teamcode.parts.Team1.Intake1.settings;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;

import java.util.HashMap;
import java.util.Map;

import om.self.ezftc.utils.Vector3;

@Config
public class IntakeSettings1 {
    public static double launchServoLaunch = 0.9;

    public static double LaunchServoRest = 0;

    public static int launchServoSweepTime = 1000;

    public static double ticksPerRev = 38;

    public static int launchRPMTolerance = 200;

    public static Map<String, IntakeSettings3.LaunchData> launchPosiMap = new HashMap<String, IntakeSettings3.LaunchData>();

    public IntakeSettings1() {
    }

    public static IntakeSettings1 makeDefault(){
        launchPosiMap.put("blueshoot1", new IntakeSettings3.LaunchData(3000,new Vector3(-12, 12, 135)));
        launchPosiMap.put("blueshoot2", new IntakeSettings3.LaunchData(10,new Vector3(2,0,0)));
        launchPosiMap.put("blueshoot3", new IntakeSettings3.LaunchData(10,new Vector3(3,0,0)));
        launchPosiMap.put("blueshoot4", new IntakeSettings3.LaunchData(10,new Vector3(4,0,0)));
        return new IntakeSettings1();
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

