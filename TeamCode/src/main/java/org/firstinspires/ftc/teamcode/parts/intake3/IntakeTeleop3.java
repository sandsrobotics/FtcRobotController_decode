package org.firstinspires.ftc.teamcode.parts.intake3;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.Buttons;
import org.firstinspires.ftc.teamcode.lib.ButtonMgr.State;
import org.firstinspires.ftc.teamcode.parts.intake3.settings.IntakeSettings3;
import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;


//@Config
public class IntakeTeleop3 extends LoopedPartImpl<Intake3, IntakeSettings3, ObjectUtils.Null> {
    private IntakeSettings3 settings;
    ButtonMgr buttonMgr;

    public IntakeTeleop3(Intake3 parent) {
        super(parent, "Intake teleop");
        setSettings(IntakeSettings3.makeDefault());
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeTeleop3(Intake3 parent, IntakeSettings3 settings) {
        super(parent, "Intake teleop");
        setSettings(settings);
        buttonMgr = parent.parent.buttonMgr;
    }

    public IntakeSettings3 getSettings() {
        return settings;
    }

    public void setSettings(IntakeSettings3 settings) {
        this.settings = settings;
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {}

    @Override
    public void onStart() {
        parent.setBaseController(() -> new IntakeControl3(false), true);
        parent.getHardware().launchMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, IntakeSettings3.spinnerPID);
    }

    @Override
    public void onRun() {
        driverControls();
    }

    @Override
    public void onStop() {
        parent.setBaseControllerToDefault(parent.isControlActive());
    }

    /* **********************************************************
     *
     * IMPORTANT NOTE FROM MR. KN:
     *
     * State.isPressed should not be used when controlling tasks (or triggering anything).
     * It returns the current state of the button, not a rising or falling edge event.
     * That means it triggers the task repeatedly for the entire duration it is pressed.
     * Not good!
     *
     * Better choices would be:
     *    wasPressed;       // Rise
     *    wasReleased;      // Fall
     *    wasHeld;          // Rise + Long Hold + Fall
     *    wasTapped;        // Rise + Short Hold + Fall
     *    wasSingleTapped;  // Rise + Short Hold + Fall + Gap  [note the gap/delay at end!]
     *    wasDoubleTapped;  // Rise + Short Hold + Fall + Short Gap x2
     *
     * ***********************************************************/

    public void driverControls() {

        // ============================================
        // CONTROLLER 1 (DRIVER) CONTROLS
        // ============================================

        // BACK BUTTON - Emergency stop
        if (buttonMgr.getState(1, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // D-PAD DOWN - Start intake (collect artifacts)
        if (buttonMgr.getState(1, Buttons.dpad_down, State.wasTapped)) {
            parent.setIntakeRPM(IntakeSettings3.intakeRPM);
        }

        // D-PAD UP - Reverse intake (eject artifacts)
        if (buttonMgr.getState(1, Buttons.dpad_up, State.wasTapped)) {
            parent.setIntakeRPM(-IntakeSettings3.intakeRPM);
        }

        // D-PAD DOWN/UP DOUBLE TAP - Stop intake
        if (buttonMgr.getState(1, Buttons.dpad_down, State.wasDoubleTapped) ||
                buttonMgr.getState(1, Buttons.dpad_up, State.wasDoubleTapped)) {
            parent.setIntakeRPM(0);
        }

        // B BUTTON - test and find out.
        if (buttonMgr.getState(1, Buttons.b, State.wasTapped)) {
            IntakeSettings3.launchArmed = false;
            parent.setLaunchRPM(0);
        }
        // LEFT TRIGGER- Move to endgame position (hold to move, release to stop)
        if (buttonMgr.getState(1, Buttons.left_trigger, State.isHeld)) {
            // Select position based on alliance color
            Vector3 endgamePosition = IntakeSettings3.isRedSide ?
                    IntakeSettings3.endgameRed :
                    IntakeSettings3.endgameBlue;

            // Keep setting the target while held (position solver handles the rest)
            parent.positionSolver.setNewTarget(endgamePosition, false);
        } else if (buttonMgr.getState(1, Buttons.left_trigger, State.wasReleased)) {
            // Stop when released
            parent.positionSolver.stopSolver();
        }

        // RIGHT TRIGGER- Move to SHOOTING position (hold to move, release to stop)
        if (buttonMgr.getState(1, Buttons.right_trigger, State.isHeld)) {
            // Select position based on alliance color
            Vector3 endgamePosition = IntakeSettings3.isRedSide ?
                    IntakeSettings3.shootingRed :
                    IntakeSettings3.shootingBlue;

            // Keep setting the target while held (position solver handles the rest)
            parent.positionSolver.setNewTarget(endgamePosition, false);
        } else if (buttonMgr.getState(1, Buttons.right_trigger, State.wasReleased)) {
            // Stop when released
            parent.positionSolver.stopSolver();
        }

        // LEFT BUMPER - Move to triangle SHOOTING position (hold to move, release to stop)
        if (buttonMgr.getState(1, Buttons.left_bumper, State.isHeld)) {
            // Select position based on alliance color
            Vector3 endgamePosition = IntakeSettings3.isRedSide ?
                    IntakeSettings3.shootingTriRed :
                    IntakeSettings3.shootingTriBlue;

            // Keep setting the target while held (position solver handles the rest)
            parent.positionSolver.setNewTarget(endgamePosition, false);
        } else if (buttonMgr.getState(1, Buttons.left_bumper, State.wasReleased)) {
            // Stop when released
            parent.positionSolver.stopSolver();
        }

        // RIGHT BUMPER - Move to wall SHOOTING position (hold to move, release to stop)
        if (buttonMgr.getState(1, Buttons.right_bumper, State.isHeld)) {
            // Select position based on alliance color
            Vector3 endgamePosition = IntakeSettings3.isRedSide ?
                    IntakeSettings3.shootingWallRed :
                    IntakeSettings3.shootingWallBlue;

            // Keep setting the target while held (position solver handles the rest)
            parent.positionSolver.setNewTarget(endgamePosition, false);
        } else if (buttonMgr.getState(1, Buttons.right_bumper, State.wasReleased)) {
            // Stop when released
            parent.positionSolver.stopSolver();
        }

        // INTAKE BY TRIGGERS (NOT GOOD)
        // LEFT TRIGGER - TURN ON INTAKE
        // RIGHT TRIGGER - REVERSE INTAKE
        // Priority: Most recently pressed trigger wins
//        if (buttonMgr.getState(1, Buttons.left_trigger, State.wasPressed)) {
//            parent.setIntakeRPM(IntakeSettings3.intakeRPM);  // Left just pressed - forward
//        } else if (buttonMgr.getState(1, Buttons.right_trigger, State.wasPressed)) {
//            parent.setIntakeRPM(-IntakeSettings3.intakeRPM); // Right just pressed - reverse
//        } else if (buttonMgr.getState(1, Buttons.left_trigger, State.isPressed)) {
//            parent.setIntakeRPM(IntakeSettings3.intakeRPM);  // Left held - forward
//        } else if (buttonMgr.getState(1, Buttons.right_trigger, State.isPressed)) {
//            parent.setIntakeRPM(-IntakeSettings3.intakeRPM); // Right held - reverse
//        } else {
//            parent.setIntakeRPM(0);  // Neither pressed - stop
//        }


        // AKHIL BACKUP
//        if (buttonMgr.getState(1, Buttons.dpad_right, State.isHeld)) {
//            // Select position based on alliance color
//            Vector3 endgamePosition = IntakeSettings3.isRedSide ?
//                    IntakeSettings3.endgameRed :
//                    IntakeSettings3.endgameBlue;
//
//            // Keep setting the target while held (position solver handles the rest)
//            parent.positionSolver.setNewTarget(endgamePosition, false);
//        } else if (buttonMgr.getState(1, Buttons.dpad_right, State.wasReleased)) {
//            // Stop when released
//            parent.positionSolver.stopSolver();
//        }

        // ============================================
        // CONTROLLER 2 (OPERATOR) CONTROLS
        // ============================================

        // BACK BUTTON - Emergency stop
        if (buttonMgr.getState(2, Buttons.back, State.wasPressed)) {
            parent.eStop();
        }

        // LEFT BUMPER - Auto-align to April tag
        if (buttonMgr.getState(2, Buttons.left_bumper, State.isHeld)) {
            IntakeSettings3.alignTarget = true;
        } else {
            IntakeSettings3.alignTarget = false;
        }

        // LEFT TRIGGER - Intake control (hold to turn off, release to turn on)
        if (buttonMgr.getState(2, Buttons.left_trigger, State.isHeld)) {
            parent.setIntakeRPM(0);  // Turn off while holding
        } else if (buttonMgr.getState(2, Buttons.left_trigger, State.wasReleased)) {
            parent.setIntakeRPM(IntakeSettings3.intakeRPM);  // Turn on when released
        }



        // RIGHT TRIGGER - Color-ordered launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.right_trigger, State.wasPressed)) {
//            parent.setIntakeRPM(0);
            parent.computeLaunchOrderAndLaunchBlocking(
                    parent.limeLight.getClassificationPattern()
            );
//            parent.setIntakeRPM(IntakeSettings3.intakeRPM);
            parent.getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Lock);
        }

        // Y BUTTON - Start/Stop launcher
        if (buttonMgr.getState(2, Buttons.y, State.wasTapped)) {
            parent.setLaunchRPM(IntakeSettings3.launchRPM);
            IntakeSettings3.launchArmed = true;  // use true for interpolated
        }

        if (buttonMgr.getState(2, Buttons.y, State.wasDoubleTapped)) {
            IntakeSettings3.launchArmed = false;
            parent.setLaunchRPM(0);
        }

        // X BUTTON - Move to blueshoot1 and launch (auto-starts launcher)
//        if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
//            if (parent.getTargetLaunchRPM() < 500) {
//                parent.setLaunchRPM(IntakeSettings3.launchRPM);
//            }
//            parent.launchData = IntakeSettings3.launchPosiMap.get("blueshoot1");
//            parent.tasks.moveAndLaunch.restart();
//        }



        // B BUTTON - Simultaneous launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.b, State.wasTapped)) {
            if (parent.getTargetLaunchRPM() < 500) {
                parent.setLaunchRPM(IntakeSettings3.launchAllRPM);
                IntakeSettings3.launchArmed = true;
            }
            parent.stopAllIntakeTasks();
            parent.tasks.sameTimeBallLaunchTask.restart();
        }

        // A BUTTON - Sequential launch (auto-starts launcher)
        if (buttonMgr.getState(2, Buttons.a, State.wasTapped)) {
            if (parent.getTargetLaunchRPM() < 500) {
                parent.setLaunchRPM(IntakeSettings3.launchRPM);
                IntakeSettings3.launchArmed = true;

            }
            parent.stopAllIntakeTasks();
            parent.tasks.ballLaunchTask.restart();
        }

        // X BUTTON - Reverse shooter
//        if (buttonMgr.getState(2, Buttons.x, State.wasTapped)) {
//            parent.setLaunchRPM(-IntakeSettings3.launchRPM);
//        }

        // B BUTTON - Lock servo (tap to lock, double tap to unlock)
        if (buttonMgr.getState(2, Buttons.right_stick_button, State.wasTapped)) {
            parent.getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Lock);
        }

        if (buttonMgr.getState(2, Buttons.right_stick_button, State.wasDoubleTapped)) {
            parent.getHardware().lockServo0.setPosition(IntakeSettings3.lockServo0Unlock);
        }
        // LEFT STICK BUTTON - Move to bluefartriangle and launch (auto-starts launcher)
//        if (buttonMgr.getState(2, Buttons.left_stick_button, State.wasTapped)) {
//            if (parent.getTargetLaunchRPM() < 500) {
//                parent.setLaunchRPM(IntakeSettings3.launchRPM);
//                IntakeSettings3.launchArmed = true;
//            }
//            parent.launchData = IntakeSettings3.launchPosiMap.get("bluefartriangle");
//            parent.tasks.moveAndLaunch.restart();
//        }

        //TEST CODE BEGIN

//        if (buttonMgr.getState(1, Buttons.start, State.isPressed) &&
//                buttonMgr.getState(1, Buttons.left_trigger, State.wasTapped)) {
//            parent.getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Rest);
//            parent.getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Rest);
//            parent.getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Rest);
//        }
//
//        if (buttonMgr.getState(1, Buttons.start, State.isPressed) &&
//                buttonMgr.getState(1, Buttons.left_bumper, State.wasTapped)) {
//            parent.getHardware().launchServo0.setPosition(IntakeSettings3.launchServo0Launch);
//            parent.getHardware().launchServo1.setPosition(IntakeSettings3.launchServo1Launch);
//            parent.getHardware().launchServo2.setPosition(IntakeSettings3.launchServo2Launch);
//        }

        //TEST CODE END
    }
}