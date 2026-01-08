package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import om.self.ezftc.utils.Vector3;

@Autonomous(name="Red Goal", group="32859")
public class T3_AutoRedLaunch extends T3_AutoBlueLaunch {
    @Override
    public void initAuto() {
        transformFunc = (v) -> new Vector3(v.X, -v.Y, -v.Z);
        fieldStartPos = transformFunc.apply(blueLaunchStart);
    }
}
