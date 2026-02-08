package org.firstinspires.ftc.teamcode.parts.intake1;

import om.self.ezftc.utils.Vector3;

public class DecodeSettings {

   /* The purpose of this class is to store values accessible to all classes in FlipBot,
      and which can remain from run to run
    */

   static boolean modeTeleOp = true;

   static boolean allianceBlue = true;

   static Vector3 robotPosition                  = new Vector3();
   static Vector3 pos_fieldStart                 = new Vector3();
   static Vector3 pos_obeliskView                = new Vector3();

   static Vector3 pos_launchPosZero             = new Vector3();
   static Vector3 pos_launchPosOne              = new Vector3();
   static Vector3 pos_launchPosTwo              = new Vector3();

   static Vector3 pos_pre_intakeArtifactRow1    = new Vector3();
   static Vector3 pos_intakeArtifactRow1        = new Vector3();
   static Vector3 pos_pre_intakeArtifactRow2    = new Vector3();
   static Vector3 pos_intakeArtifactRow2        = new Vector3();
   static Vector3 pos_pre_intakeArtifactRow3    = new Vector3();
   static Vector3 pos_intakeArtifactRow3        = new Vector3();

   static Vector3 pos_parkAfterAuto             = new Vector3();
   static Vector3 pos_pre_leverOpen             = new Vector3();
   static Vector3 pos_leverOpen                 = new Vector3();

   static int   launchRPM                       = 3200; // "RequiredRPM" for the currentLaunch.

   static String currentOpMode                  = "NOT SET!";

   static Vector3 pos_targetGoal                 = new Vector3();  // targetRed or targetBlue.

   // LK new test stuff
   static Vector3 fusedRobotPosition = new Vector3();
   public static Vector3 targetRed  = new Vector3(-70.5, 70.5, 0.0);
   public static Vector3 targetBlue = new Vector3(-70.5, -70.5, 0.0);
   public static boolean lkPinpoint = false;
   public static boolean lkTestMode1 = false;
   public static boolean lkTestMode2 = false;
   // --

   public static boolean autonomousDebugMode = false;
   public static boolean odoFirstRun = true;
   public static boolean firstRun = true;
   static Vector3 controlGovernor = new Vector3(1,1,1);
   public static boolean isDemoMode = false;
   public static double demoDriverDefaultMultiplier = 0.5;
   public static double demoDriverMultiplier = 0;

   public static int pinpointSettingsXoffset = +105;
   public static int pinpointSettingsYoffset = +150;
   public static float pinpointSettingsResolution = 13.26291192f;

   static int classificationId = 21; // Defaults to 21.

   public static void setClassificationId (int Id) { classificationId = Id; };
   public static int getClassificationId () { return classificationId; };

   public static void setTeleOp () { modeTeleOp = true; }
   public static void setAuto () { modeTeleOp = false; }
   public static boolean isTeleOp() { return modeTeleOp; }
   public static boolean isAuto() { return !modeTeleOp; }

   public static void setAllianceBlue () { allianceBlue = true; }
   public static void setAllianceRed() { allianceBlue = false; }
   public static boolean isAllianceBlue() { return allianceBlue; }
   public static boolean isAllianceRed() { return !allianceBlue; }

   public static void setControlGovernor(Vector3 multiplier) {
      controlGovernor = new Vector3(clamp(multiplier.X), clamp(multiplier.Y), clamp(multiplier.Z));
   }
   public static void setControlGovernor(double multiplier) {
      controlGovernor = new Vector3(clamp(multiplier));
      //controlGovernor = new Vector3(clamp(multiplier, 0.25, 1));
   }
   public static Vector3 getControlGovernor() {
      return controlGovernor;
   }

   public static void setRobotPosition(Vector3 currentPosition) {
      if (currentPosition.X == 0 && currentPosition.Y == 0 && currentPosition.Z == 0) return;
      robotPosition = currentPosition;
   }

   public static Vector3 getRobotPosition() {
      return robotPosition;
   }

   public static void storeFusedPosition(Vector3 fusedPosition) {
      fusedRobotPosition = fusedPosition;
   }

   public static Vector3 getFusedRobotPosition() {
      if (fusedRobotPosition != null && (fusedRobotPosition.X != 0 || fusedRobotPosition.Y != 0 || fusedRobotPosition.Z != 0 )) {
         return fusedRobotPosition;
      }
      else return robotPosition;
   }

   public static void setFieldStartPos(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_fieldStart = position;
   }
   public static Vector3 getFieldStartPos() {
      return pos_fieldStart;
   }

   public static void setObeliskViewPos(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_obeliskView = position;
   }
   public static Vector3 getObeliskViewPos() {
      return pos_obeliskView;
   }

   public static void setLaunchPositionZero(Vector3 launchPosition) {
      if (launchPosition.X == 0 && launchPosition.Y == 0 && launchPosition.Z == 0) return;
      pos_launchPosZero = launchPosition;
   }
   public static Vector3 getLaunchPositionZero() {
      return pos_launchPosZero;
   }

   public static void setLaunchPositionOne(Vector3 launchPosition) {
      if (launchPosition.X == 0 && launchPosition.Y == 0 && launchPosition.Z == 0) return;
      pos_launchPosOne = launchPosition;
   }
   public static Vector3 getLaunchPositionOne() {
      return pos_launchPosOne;
   }

   public static void setLaunchPositionTwo(Vector3 launchPosition) {
      if (launchPosition.X == 0 && launchPosition.Y == 0 && launchPosition.Z == 0) return;
      pos_launchPosTwo = launchPosition;
   }
   public static Vector3 getLaunchPositionTwo() {
      return pos_launchPosTwo;
   }

   public static void setPreIntakeArtifactRow1(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_pre_intakeArtifactRow1 = position;
   }
   public static Vector3 getPreIntakeArtifactRow1() {
      return pos_pre_intakeArtifactRow1;
   }

   public static void setIntakeArtifactRow1(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_intakeArtifactRow1 = position;
   }
   public static Vector3 getIntakeArtifactRow1() {
      return pos_intakeArtifactRow1;
   }

   public static void setPreIntakeArtifactRow2(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_pre_intakeArtifactRow2 = position;
   }
   public static Vector3 getPreIntakeArtifactRow2() {
      return pos_pre_intakeArtifactRow2;
   }

   public static void setIntakeArtifactRow2(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_intakeArtifactRow2 = position;
   }
   public static Vector3 getIntakeArtifactRow2() {
      return pos_intakeArtifactRow2;
   }

   public static void setPreIntakeArtifactRow3(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_pre_intakeArtifactRow3 = position;
   }
   public static Vector3 getPreIntakeArtifactRow3() {
      return pos_pre_intakeArtifactRow3;
   }

   public static void setIntakeArtifactRow3(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_intakeArtifactRow3 = position;
   }
   public static Vector3 getIntakeArtifactRow3() {
      return pos_intakeArtifactRow3;
   }

   public static void setParkAfterAutoPos(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_parkAfterAuto = position;
   }
   public static Vector3 getParkAfterAutoPos() {
      return pos_parkAfterAuto;
   }

   public static void setLeverOpenPos(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_leverOpen = position;
   }
   public static Vector3 getLeverOpenPos() {
      return pos_leverOpen;
   }

   public static void setPreLeverOpenPos(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_pre_leverOpen = position;
   }
   public static Vector3 getPreLeverOpenPos() {
      return pos_pre_leverOpen;
   }

   public static void setTargetGoalPos(Vector3 position) {
      if (position.X == 0 && position.Y == 0 && position.Z == 0) return;
      pos_targetGoal = position;
   }
   public static Vector3 getTargetGoalPos() {
      return pos_targetGoal;
   }


   public static void setLaunchRPM(Integer requiredRPM) {
      if (requiredRPM == 0) return;
      launchRPM = requiredRPM;
   }
   public static Integer getLaunchRPM() {
      return launchRPM;
   }

   public static void setCurrentOpMode (String currOpMode) {
      if (currOpMode != null && !currOpMode.isEmpty()) {
         currentOpMode = currOpMode;
      }
   }
   public static  String  getCurrentOpMode() { return  currentOpMode; }

   private static double clamp(double pos) {
      return Math.max(0, Math.min(pos, 1));
   }
   private static double clamp(double val, double min, double max) {
      return Math.max(min, Math.min(val, max));
   }

}