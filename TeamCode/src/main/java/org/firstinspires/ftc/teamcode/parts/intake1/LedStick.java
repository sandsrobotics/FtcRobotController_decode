package org.firstinspires.ftc.teamcode.parts.intake1;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.i2c.QwiicLEDStickLK;
import org.firstinspires.ftc.teamcode.parts.artifact.ArtifactDetectionPipeline;

public class LedStick {

    static QwiicLEDStickLK qLED = null;
    static int[] bufferDesired = {0,0,0};
    static int[] bufferActual = {0,0,0};
    static int pointer = 0;
    public static boolean updateLEDs = false;

    public static void init(HardwareMap hwMap) {
        qLED = hwMap.get(QwiicLEDStickLK.class, "ledstick");
        qLED.setBrightness(1);
        qLED.turnAllOff();
        updateLEDs = false;
    }

    public static void runLoop() {
        if (updateLEDs) updateStick();
    }

    public static void stop() {
        qLED.turnAllOff();
        updateLEDs = false;
    }

    public void setLedBuffer(int a, int b, int c) {
        // 0 = empty, 1 = green, 2 = purple, 3 = ???
        bufferDesired = new int[] {a,b,c};
    }

    public void setLedBuffer(int[] a) {
        // 0 = empty, 1 = green, 2 = purple, 3 = ???
        bufferDesired = new int[] {a[0], a[1], a[2]};
    }

    public void setLedBuffer(ArtifactDetectionPipeline.Artifact[] artifacts) {
        for (int i = 0; i < 3; i++) {
            switch (artifacts[i].color) {
                case GREEN:
                    bufferDesired[i] = 1;
                    break;
                case PURPLE:
                    bufferDesired[i] = 2;
                    break;
                case NONE:
                default:
                    bufferDesired[i] = 0;
                    break;
            }
        }
    }

    static void updateStick() {
        // This is the "round robin" pointer...  loops around for updating two led groups
        if (++pointer > 2) pointer = 0;

        // If the current candidate is already set properly, just exit
        if (bufferDesired[pointer] == bufferActual[pointer]) return;

        // Update different groups depending on the pointer
        switch (pointer) {
            case 0:
                bufferActual[0] = bufferDesired[0];
                bufferActual[1] = bufferDesired[1];
                qLED.setColorGroupX2(0,2,getColorValue(bufferActual[0]),
                        4,2,getColorValue(bufferActual[1]));
                break;
            case 1:
                bufferActual[1] = bufferDesired[1];
                bufferActual[2] = bufferDesired[2];
                qLED.setColorGroupX2(4,2,getColorValue(bufferActual[1]),
                        8,2,getColorValue(bufferActual[2]));
                break;
            case 2:
                bufferActual[0] = bufferDesired[0];
                bufferActual[2] = bufferDesired[2];
                qLED.setColorGroupX2(0,2,getColorValue(bufferActual[0]),
                        8,2,getColorValue(bufferActual[2]));
                break;
            default:
                break;
        }
    }

    static int getColorValue (int a) {
        switch (a) {
            case 1:
                return Color.rgb(0, 255, 0);    // green
            case 2:
                return Color.rgb(255, 0, 255);  // purple
            case 3:
                return Color.rgb(255, 255, 0);     // red (unknown)
            case 0:
            default:
                return Color.rgb(0, 0, 0);      // off (empty)
        }
    }
}