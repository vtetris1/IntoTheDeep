package org.firstinspires.ftc.teamcode.autonomous;
//test

/*
//ignore this for now
@Autonomous(name="Red_Far_v1_F2_E6")
public class Red_Far_v1_F2_E6 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //reset encoder
        robot.setAutoDriveMotorMode();

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
  //          robot.tiltServo.setPosition(0.65);

            sleep(100);

            robot.grabServoRight.setPosition(1.0);
            robot.grabServoLeft.setPosition(0.0);

            int forwardTicks = 1215;
            driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                    true, robot.yaw0);

            telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\n",
                    robot.distanceR.getDistance(DistanceUnit.INCH),
                    robot.distanceL.getDistance(DistanceUnit.INCH)));
            telemetry.update();

            robot.grabServoRight.setPosition(1.0);
            robot.grabServoLeft.setPosition(0.0);

            sleep(1000);




            if (robot.distanceL.getDistance(DistanceUnit.INCH) < 10) {

                forwardTicks = -500;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                turnToTargetYaw(35 + robot.yaw0, 0.7, 2000);

                forwardTicks = 150;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                robot.tiltServoLeft.setPosition(1.0);

                sleep(1250);

                robot.grabServoRight.setPosition(0.0);
                robot.grabServoLeft.setPosition(0.0); //so pixel doesn't fall out

                sleep(1000);

                robot.tiltServoLeft.setPosition(0.0);
                sleep(100);
                robot.grabServoRight.setPosition(1.0);

                sleep(1500);

                forwardTicks = -200;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                turnToTargetYaw(0 + robot.yaw0, 0.7, 2000);

                forwardTicks = 1700;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                turnToTargetYaw(90 + robot.yaw0, 0.6, 3750);

                forwardTicks = -2300;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.55,
                        true,(90 + robot.yaw0));

                turnToTargetYaw(45 + robot.yaw0, 0.7, 2000);

                forwardTicks = -1000;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.55,
                        true,(45 + robot.yaw0));

                turnToTargetYaw(90 + robot.yaw0, 0.7, 2000);

                robot.liftHex.setPower(0.5);
                sleep(1500);
                robot.liftHex.setPower(0);

               // robot.tiltServo.setPosition(0.2);
                forwardTicks = -1300;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.5,
                        true,(90 + robot.yaw0));

                robot.grabServoLeft.setPosition(1.0);


            }
            else if (robot.distanceR.getDistance(DistanceUnit.INCH) < 10) {

                forwardTicks = -500;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                turnToTargetYaw(-24 + robot.yaw0, 0.7, 1800);

                forwardTicks = 180;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                robot.tiltServoLeft.setPosition(1.0);

                forwardTicks = 120;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                sleep(1250);

                robot.grabServoRight.setPosition(0.0);
                robot.grabServoLeft.setPosition(0.0); //so pixel doesn't fall out

                sleep(1000);

                robot.tiltServoLeft.setPosition(0.0);
                sleep(100);
                robot.grabServoRight.setPosition(1.0);

                sleep(1500);

                forwardTicks = -200;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                turnToTargetYaw(0 + robot.yaw0, 0.7, 2000);

                forwardTicks = 1500;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, robot.yaw0);

                turnToTargetYaw(90 + robot.yaw0, 0.6, 3750);

                forwardTicks = -2300;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.55,
                        true,(90 + robot.yaw0));

                turnToTargetYaw(45 + robot.yaw0, 0.7, 2000);

                forwardTicks = -1000;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.55,
                        true,(45 + robot.yaw0));

                turnToTargetYaw(90 + robot.yaw0, 0.7, 2000);

                robot.liftHex.setPower(0.5);
                sleep(1500);
                robot.liftHex.setPower(0);

                // robot.tiltServo.setPosition(0.2);
                forwardTicks = -1400;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.5,
                        true,(90 + robot.yaw0));

                robot.grabServoLeft.setPosition(1.0);


            } else {
                forwardTicks = -200;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                        false, robot.yaw0);

                sleep(100);

                robot.tiltServoLeft.setPosition(1.0);

                sleep(500);

                forwardTicks = 270;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                        false, robot.yaw0);

                sleep(500);

                robot.grabServoRight.setPosition(0.0);
                robot.grabServoLeft.setPosition(0.0); //so pixel doesn't fall out
    //            robot.autoPixel.setPosition(0.0);

                sleep(1000);

                robot.tiltServoLeft.setPosition(0.0);
                sleep(100);
                robot.grabServoRight.setPosition(1.0); //reset right grabber

                forwardTicks = -150;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                        false, robot.yaw0);

                turnToTargetYaw(90+robot.yaw0, 0.6, 5000);

                forwardTicks = -80;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                        false, robot.yaw0);


                int sideTicks = -150;
                driveMotors((int)(sideTicks*1.2), -sideTicks, -sideTicks, (int)(sideTicks*1.2), 0.5,
                        true, robot.yaw0);

                driveMotors((int)(sideTicks*1.2), -sideTicks, -sideTicks, (int)(sideTicks*1.2), 0.5,
                        true, robot.yaw0);

                turnToTargetYaw(90+robot.yaw0, 0.42, 3000);



                forwardTicks = -3100;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                        true, (90 + robot.yaw0));

                sideTicks = -150;
                driveMotors((int)(sideTicks*1.2), -sideTicks, -sideTicks, (int)(sideTicks*1.2), 0.5,
                        true, robot.yaw0);

                driveMotors((int)(sideTicks*1.2), -sideTicks, -sideTicks, (int)(sideTicks*1.2), 0.5,
                        true, robot.yaw0);

                turnToTargetYaw(90+robot.yaw0, 0.42, 3000);


                robot.liftHex.setPower(0.5);
                sleep(1500);
                robot.liftHex.setPower(0);



                forwardTicks = -1100;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                        true, (90 + robot.yaw0));

                robot.grabServoLeft.setPosition(1.0);


                sleep(100);
/*
                robot.boardPixel.setPosition(1.0);
                sleep(1000);
                robot.boardPixel.setPosition(0.7);
                sleep(1000);
                robot.boardPixel.setPosition(0.0);


            }

/*
            while (opModeIsActive()) {
                telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\nCurrent Yaw: %.1f",
                        robot.distanceR.getDistance(DistanceUnit.INCH),
                        robot.distanceL.getDistance(DistanceUnit.INCH),
                        robot.getCurrentYaw()));
                telemetry.update();
            }
        }
    }
*/
/*
    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        double leftRatioToCounterCOG = 0.95;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        robot.motorfl.setPower(power * leftRatioToCounterCOG);
        robot.motorbl.setPower(power * leftRatioToCounterCOG);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() &&
                        robot.motorbl.isBusy() &&
                        robot.motorfr.isBusy() &&
                        robot.motorbr.isBusy())){
            if (bKeepYaw) {

                currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }
            idle();
        }

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }

    private void nonEncoderDrive(){

    }

    private void driveStrafe(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        robot.motorfl.setPower(power);
        robot.motorbl.setPower(power);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() &&
                        robot.motorbl.isBusy() &&
                        robot.motorfr.isBusy() &&
                        robot.motorbr.isBusy())){
            if (bKeepYaw) {

                currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }
            idle();
        }

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }


    private void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 200)
                ticks = 200;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

    private void deployPreloadedPixel1(int timeIntervalMs) {
        // Deploy preloaded pixel 1
     //   robot.autoPixel.setPosition(1.0);
        sleep(timeIntervalMs);
     //   robot.autoPixel.setPosition(0.5);
        sleep(timeIntervalMs);

    }

}
*/
