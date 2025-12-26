package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import om.self.ezftc.utils.Vector3;
@Autonomous(name="32859 Red Launch", group="32859")
public class T3_AutoRedLaunch extends T3_AutoBlueLaunch {
    //Vector3 redLaunchStart = new Vector3(-51, -51, -126); //Start Correct

    @Override
    public void initAuto() {
        transformFunc = (v) -> new Vector3(v.X, -v.Y, -v.Z);
        fieldStartPos = blueLaunchStart;
    }
}
