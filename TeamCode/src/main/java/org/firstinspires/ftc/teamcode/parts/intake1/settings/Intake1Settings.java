package org.firstinspires.ftc.teamcode.parts.intake1.settings;

import com.acmerobotics.dashboard.config.Config;

import om.self.ezftc.utils.Vector3;

@Config
public class Intake1Settings {

    public static double intakeIn =                                    -.85;
    public static double intakeStop =                                   0;
    public static double intakeOut =                                    .85;
    public static final double intakeServoIn                                = 1;
    public static final double intakeServoOff                               = 0.5;
    public static final double intakeServoOut                               = 0;
    public static double augerMotorStop                                     = 0;
    public static double augerMotorRun                                      = 0.30;

    public static int launchServoDelay                          = 200;
    public static int launchServoSweepTime                      = 1000;
    public static int launchRPMTolerance                        = 100;
    public static int launchRPMToleranceV2                      = 75;
    public static int launchRPMToleranceV2Undershoot            = 100;

    public static double servoGreenDock =                           0.463; // 0.500; 0.463;
    public static double servoGreenLaunch =                         0.183; //.243

    public static double servoPinkDock =                            0.518; // 0.530; 0.490; 0.518;
    public static double servoPinkLaunch =                          0.680; // .732
    public static double servoPinkLow =                             0.434;

    public static double servoBlueDock =                            0.500; // 0.494;
    public static double servoBlueLaunch =                          0.671;
    public static double servoBlueLow =                             0.390;

    public static double ticksPerRevolution =                          28;
    public double secondsPerMinute =                                 60;
    public static double autoFarLaunchMotorRPM =                     3100; // 3000; 3100; 3200
    public static double farLaunchMotorRPM =                         3300; // 3100; 3200
    public static double threeLaunchMotorRPM =                       2700; // 2700; 2800; 2900
    public static double goalLaunchMotorRPM =                        2800; // 2700; 2600; 2500;

    public static double aLaunchMotorRPM =                        500;
    public static double bLaunchMotorRPM =                        1000;
    public static double yLaunchMotorRPM =                        1500;
    public static double xLaunchMotorRPM =                        2000;
    public double autoFarLaunchMotorVelocityStart =              autoFarLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double farLaunchMotorVelocityStart =                  farLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double threeLaunchMotorVelocityStart =                threeLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double goalLaunchMotorVelocityStart =                 goalLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double aLaunchMotorVelocityStart =                    aLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double bLaunchMotorVelocityStart =                    bLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double yLaunchMotorVelocityStart =                    yLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double xLaunchMotorVelocityStart =                    xLaunchMotorRPM/secondsPerMinute*ticksPerRevolution;

    public double launchMotorVelocityStop =                      0;


    public static Vector3 p_teleopFarRedLaunch_1 = new Vector3(45.65, -5.5, 150);  // 58,-16,150;
    public static Vector3 p_teleopFarRedLaunch_2 = new Vector3(58, -16, 153);
    public static Vector3 p_teleopFarRedLaunch_3 = new Vector3(58, -16, 153);

    public static Vector3 p_teleopNearRedLaunch = new Vector3(-32, -32, 111);
    public static Vector3 p_teleopThreeRedLaunch = new Vector3(-28.26, 16, 141.5);  // (-24, 24, 135);
    public static Vector3 p_teleopGoalRedLaunch = new Vector3(-24, 24, 135);

    public static Vector3 p_teleopFarBlueLaunch_1 = new Vector3(58, 16, -150);
    public static Vector3 p_teleopFarBlueLaunch_2 = new Vector3(58, 16, -153);
    public static Vector3 p_teleopFarBlueLaunch_3 = new Vector3(58, 16, -153);

    public static Vector3 p_teleopNearBlueLaunch = new Vector3(-32, 32, -111);
    public static Vector3 p_teleopThreeBlueLaunch = new Vector3(-24, -24, -135);
    public static Vector3 p_teleopGoalBlueLaunch = new Vector3(-24, -24, -135);

    public static Vector3 p_teleopMoveToRedLoadingZone_1 = new Vector3(52, -38, 90);
    public static Vector3 p_teleopMoveToRedLoadingZone_2 = new Vector3(52, -44, 90);
    public static Vector3 p_teleopMoveToBlueLoadingZone_1 = new Vector3(52, 38, -90);
    public static Vector3 p_teleopMoveToBlueLoadingZone_2 = new Vector3(-24, 44, -90);


    public Intake1Settings() {
    }

    public static Intake1Settings makeDefault(){
        return new Intake1Settings();
    }
}

