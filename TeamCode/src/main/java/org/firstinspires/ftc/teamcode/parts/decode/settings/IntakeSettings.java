package org.firstinspires.ftc.teamcode.parts.decode.settings;

public class  IntakeSettings {

    public double intakeIn =                                    -1;
    public double intakeStop =                                   0;
    public double intakeOut =                                    1;
    public double sorterStop =                                 0.5;
    public double sorterStart =                                  0;
    public double servoGreenDock =                           0.463;
    public double servoGreenLaunch =                         0.235;
    public double servoPinkDock =                            0.518;
    public double servoPinkLaunch =                          0.719;
    public double servoBlueDock =                            0.494;
    public double servoBlueLaunch =                          0.671;
    public double ticksPerRevolution =                          28;
    public double secondsPerMinute =                            60;
    public double launchMotorRPM =                             2500;
    public double launchMotorVelocityStart =                 launchMotorRPM/secondsPerMinute*ticksPerRevolution;
    public double launchMotorVelocityStop =                      0;



    public IntakeSettings() {
    }

    public static IntakeSettings makeDefault(){
        return new IntakeSettings();
    }
}

