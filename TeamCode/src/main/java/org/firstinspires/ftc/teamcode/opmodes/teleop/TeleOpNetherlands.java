package pedroPathing.examples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Netherlands TeleOp", group="TeleOp")
public class TeleOpNetherlands extends LinearOpMode {

    // Linear slide motors
    /*
    private DcMotor slideMotorLeft;
    private DcMotor slideMotorRight;
*/
    private DcMotor bl, br, fl, fr;
    // Arm servos
    /*
    private CRServo armServoLeft;
    private CRServo armServoRight;

    // Claw servo
    private Servo clawServo;

    // Linkage servos
    private CRServo linkageServoLeft;
    private CRServo linkageServoRight;

    private Servo wristServo;
    */


    // CONSTANTS
    private static final double TICKS_PER_INCH = (767.2*25.4)/(Math.PI * 48);
    private static final int SLIDE_START_TICKS = 0;
    private static final int DISTANCE = 15;
    private static final int SLIDE_ELEVATED_TICKS = (int)(DISTANCE * TICKS_PER_INCH * 1.2); // ≈ 1292

    // Arm positions
    private static final double ARM_POS_PICKUP = 80.0 / 355.0;
    private static final double ARM_POS_DEPOSIT = 190.0 / 355.0;

    // Claw
    private static final double CLAW_CLAMP = 0.737;
    private static final double CLAW_UNCLAMP = 0.4;

    // Linkage
    private static final double LINKAGE_RETRACTED = 0.0;
    private static final double LINKAGE_EXTENDED = 0.7;
    private static final long DELAY = 250;

    // Wrist rotation about X-axis (same direction)
    // Adjust these as needed for your servo's physical range and mounting
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;

    // These scale joystick input to servo movement range
    private static final double PITCH_RANGE = 0.5; // How much pitch the stick will generate
    private static final double ROLL_RANGE = 0.5;  // How much roll the stick will generate

    // Gear ratio: for every 1 degree the wrist moves, the servo must move 2.88 degrees
    private static final double GEAR_RATIO = 2.88;

    private ElapsedTime movementTimer = new ElapsedTime();
    private boolean servoMoving = false;

    // Servo config: 255° rotation range
    private static final double SERVO_DEGREES = 255.0;

    @Override
    public void runOpMode() {
        int incrementalCounter = 0;
        int incrementalCounter2 = 0;
        bl = hardwareMap.get(DcMotor.class, "backLeft");
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        fr = hardwareMap.get(DcMotor.class, "frontRight");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        /*
        slideMotorLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        slideMotorRight = hardwareMap.get(DcMotor.class, "slideRight");

        slideMotorRight.setDirection(DcMotor.Direction.REVERSE);

        armServoLeft = hardwareMap.get(CRServo.class, "armLeft");
        armServoRight = hardwareMap.get(CRServo.class, "armRight");
        armServoLeft.setDirection(CRServo.Direction.REVERSE);

        clawServo = hardwareMap.get(Servo.class, "claw");

        linkageServoLeft = hardwareMap.get(CRServo.class, "linkageLeft");
        linkageServoRight = hardwareMap.get(CRServo.class, "linkageRight");

        wristServo = hardwareMap.get(Servo.class, "wristLeft");
        // === MOTOR SETUP ===
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
*/
        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double v = gamepad1.right_stick_y;
            double z = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(z), 0.3);
            double flPower = (y+z-x+v)/denominator;
            double blPower = (y+z+x-v)/denominator;
            double frPower = (y-z+x-v)/denominator;
            double brPower = (y-z-x+v)/denominator;
            //Power toned down to accomodate driver
            fl.setPower(flPower * 0.5);
            bl.setPower(blPower * 0.5);
            fr.setPower(frPower * 0.5);
            br.setPower(brPower * 0.5);
            //Same Button Intake
            /*
            if (gamepad2.square) {
                if (incrementalCounter % 6 == 0) { //Bring arm down to a horizontal position and extend linkage
                    armServoLeft.setPower(0.6);
                    armServoRight.setPower(0.6);
                    sleep(1500);
                    armServoLeft.setPower(0);
                    armServoRight.setPower(0);
                    linkageServoLeft.setPower(-0.4);
                    linkageServoRight.setPower(0.4);
                    sleep(900);
                    linkageServoLeft.setPower(0);
                    linkageServoRight.setPower(0);
                }
                else if (incrementalCounter % 6 == 1) {
                    wristServo.setPosition(0.65);
                }
                else if (incrementalCounter % 6 == 2) { //Open claw
                    clawServo.setPosition(CLAW_UNCLAMP);
                }
                else if (incrementalCounter % 6 == 3) { //Close claw
                    clawServo.setPosition(CLAW_CLAMP);
                }
                else if (incrementalCounter % 6 == 4) { //Retract linkage
                    linkageServoLeft.setPower(0.4);
                    linkageServoRight.setPower(-0.4);
                    sleep(1000);
                    linkageServoLeft.setPower(0);
                    linkageServoRight.setPower(0);
                }
                else if (incrementalCounter % 6 == 5) { //Bring arm up to default position
                    armServoLeft.setPower(-1);
                    armServoLeft.setPower(-1);
                    sleep(900);
                    armServoLeft.setPower(0);
                    armServoRight.setPower(0);
                }
                incrementalCounter++;
                sleep(150);
            }
            //Same button outtake:
            if (gamepad2.circle) {
                if (incrementalCounter2 % 5 == 0) { //Bring arm and linkage up to the high goal in order to outtake
                    armServoLeft.setPower(-0.5);
                    armServoRight.setPower(-0.5);
                    sleep(300);
                    armServoLeft.setPower(0);
                    armServoRight.setPower(0);
                    linkageServoLeft.setPower(-1);
                    linkageServoRight.setPower(1);
                    sleep(1000);
                    linkageServoLeft.setPower(0);
                    linkageServoRight.setPower(0);
                }
                else if (incrementalCounter2 % 5 == 1) { //Turn wrist to deposit sample
                    wristServo.setPosition(0);
                }
                else if (incrementalCounter2 % 5 == 2) { //Open claw
                    clawServo.setPosition(CLAW_UNCLAMP);
                }
                else if (incrementalCounter2 % 5 == 3) { //Close claw
                    clawServo.setPosition(CLAW_CLAMP);
                }
                else if (incrementalCounter2 % 5 == 4) { //Move the arm and linkage back to their original positions
                    linkageServoLeft.setPower(0.5);
                    linkageServoRight.setPower(-0.5);
                    sleep(1000);
                    linkageServoLeft.setPower(0);
                    linkageServoRight.setPower(0);
                    armServoLeft.setPower(0.5);
                    armServoRight.setPower(0.5);
                    sleep(150);
                    armServoLeft.setPower(0);
                    armServoRight.setPower(0);
                }
                incrementalCounter2++;
                sleep(150);
            }
            //wristServoRight.setPosition(0.1);
            /*
            // Slide Control
            if (gamepad2.dpad_up) {
                moveSlidesTo(SLIDE_ELEVATED_TICKS);
            } else if (gamepad2.dpad_down) {
                moveSlidesTo(SLIDE_START_TICKS);
            }

            // Intake Mode (Triangle)
            if (gamepad2.triangle) {
                setArmPosition(ARM_POS_PICKUP);
                sleep(DELAY);
                setLinkagePosition(LINKAGE_EXTENDED);
                sleep(DELAY);
                setWristXRotation(WRIST_X_INTAKE);
            }

            // Outtake Mode (Circle)
            if (gamepad2.circle) {
                setLinkagePosition(LINKAGE_RETRACTED);
                sleep(DELAY);
                setArmPosition(ARM_POS_DEPOSIT);
                sleep(DELAY);
                setWristXRotation(WRIST_X_OUTTAKE);
            }
            */
            //
            /*
            if (gamepad2.right_bumper) {
                clawServo.setPosition(CLAW_CLAMP);
            } else if (gamepad2.left_bumper) {
                clawServo.setPosition(CLAW_UNCLAMP);
            }

             */
            /*
            // Wrist Y-axis Reset (Square)
            if (gamepad2.triangle) {
                armServoLeft.setPower(-0.6);
                armServoRight.setPower(-0.6);
                sleep(300);
            }
            if (gamepad2.cross) {
                armServoLeft.setPower(0.6);
                armServoRight.setPower(0.6);
                sleep(300);
            }

            if (gamepad2.right_stick_button) {
                slideMotorLeft.setPower(0.2);   // small upward force
                slideMotorRight.setPower(-0.2);
            } else if (gamepad2.left_stick_button) {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }
            if (gamepad2.right_stick_button) {
                moveSlidesTo(SLIDE_ELEVATED_TICKS);
            }
            else if (gamepad2.left_stick_button) {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }
            // Arm
            /*
            if (gamepad2.circle) {
                armServoLeft.setPower(0.4);
                armServoRight.setPower(0.4);
                sleep(600);
            }
            else if (gamepad2.triangle) {
                armServoLeft.setPower(-0.6);
                armServoRight.setPower(-0.6);
                sleep(600);
            }
            else {
                armServoLeft.setPower(0);
                armServoRight.setPower(0);
            }
             */
            /*
            //Linkage
            if (gamepad2.right_bumper) {
                linkageServoLeft.setPower(-0.8);
                linkageServoRight.setPower(0.8);
                sleep(2000);
                linkageServoLeft.setPower(0);
                linkageServoRight.setPower(0);
            }
            else if (gamepad2.left_bumper) {
                linkageServoLeft.setPower(0.6);
                linkageServoRight.setPower(-0.6);
                sleep(800);
                linkageServoLeft.setPower(0);
                linkageServoRight.setPower(0);
            }
            telemetry.addData("incrementalCounter", incrementalCounter);
            telemetry.addData("incrementalCounter2", incrementalCounter2);
            telemetry.addData("wristLeft", wristServo.getPosition());
            telemetry.addData("Slide L", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Slide R", slideMotorRight.getCurrentPosition());
            telemetry.addData("Claw", clawServo.getPosition());
            telemetry.addData("Linkage L", linkageServoLeft.getPower());
            telemetry.addData("Linkage R", linkageServoRight.getPower());
            telemetry.update();

             */
        }
    }

    private void moveSlidesTo(int targetTicks) {
        /*
        slideMotorLeft.setTargetPosition(targetTicks);
        slideMotorRight.setTargetPosition(targetTicks);

        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotorLeft.setPower(1);
        slideMotorRight.setPower(1);

        while (opModeIsActive() && (slideMotorLeft.isBusy() || slideMotorRight.isBusy())) {
            telemetry.addLine("Moving slides...");
            telemetry.update();
        }

        slideMotorLeft.setPower(0);
        slideMotorRight.setPower(0);

        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void extendLinkage(double EXTEND_POWER) {
        linkageServoLeft.setPower(-EXTEND_POWER);
        linkageServoRight.setPower(EXTEND_POWER);
        movementTimer.reset();
        servoMoving = true;
    }
    private void updateLinkage(double TIME) {
        if (!servoMoving) {
            return;
        }

        // Check if time has elapsed
        if (movementTimer.seconds() >= TIME) {
            linkageServoLeft.setPower(0);
            linkageServoRight.setPower(0);
            servoMoving = false;
        }
    }
    private boolean isExtending() {
        return servoMoving;

         */
    }
}