package org.firstinspires.ftc.teamcode.botconfigs;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.AutoMotor;
import org.firstinspires.ftc.teamcode.hardware.MecDriveFlip;

public class Roboticsclassrepository {

    // debugging device
    public Telemetry tele;

    // mecanum wheel drive train
    public MecanumDrive drive;
    public Motor motorFL;
    public Motor motorFR;
    public Motor motorBL;
    public Motor motorBR;

    public DcMotor slide;
    public Servo claw;
    public Servo wrist;
    public CRServo eeeRRR;

    // speeds
    public double linearSpeed = 1;
    public double turnSpeed = 1;
    public double speedFactor = 1;

    public double tickPerInch = 28 * 20 / (3 * Math.PI);
    public double inchPerRad = 13; //10;

    public double referenceHeading;

    public HolonomicBot(Telemetry tele, HardwareMap map) {

        // store debugging device
        this.tele = tele;

        // initialize drive train
        motorFL = new Motor(map, "motorFL");
        motorFR = new Motor(map, "motorFR");
        motorBL = new Motor(map, "motorBL");
        motorBR = new Motor(map, "motorBR");

        motorFL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        wrist = map.servo.get("wrist");
        claw = map.servo.get("claw");
        eeeRRR = map.crservo.get("eeeRRR");
        //drone = new SimpleServo(map, "drone_servo", 0, 90);
        slide = map.dcMotor.get("motorLS");

        drive = new MecDriveFlip(motorFL, motorFR, motorBL, motorBR);

        // run mode
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunMode(DcMotor.RunMode mode) {

        motorFL.motor.setMode(mode);
        motorFR.motor.setMode(mode);
        motorBL.motor.setMode(mode);
        motorBR.motor.setMode(mode);
    }

    public double[] toMotor(double x, double y, double rot) {

        return new double[]{
                -x - y + rot * inchPerRad,
                -x + y + rot * inchPerRad,
                +x - y + rot * inchPerRad,
                +x + y + rot * inchPerRad
        };
    }

    public void setTargetPosition(double x, double y, double rot) {

        tele.addData("SET TARGET (x, y, rot)", x + ", " + y + ", " + rot);
        double[] asMotor = toMotor(x, y, rot);
        setTargetPosition(new int[]{
                motorFL.motor.getCurrentPosition() + (int) (asMotor[0] * tickPerInch),
                motorFR.motor.getCurrentPosition() + (int) (asMotor[1] * tickPerInch),
                motorBL.motor.getCurrentPosition() + (int) (asMotor[2] * tickPerInch),
                motorBR.motor.getCurrentPosition() + (int) (asMotor[3] * tickPerInch)
        });
    }

    public void toTargetPosition(double speed) {

        tele.addData("TO TARGET (speed)", speed);
        motorFL.motor.setPower(speed);
        motorFR.motor.setPower(speed);
        motorBL.motor.setPower(speed);
        motorBR.motor.setPower(speed);
    }

    public void waitUntilNotBusy(double speed, LinearOpMode opMode) {

        tele.update();
        while (isBusy() && opMode.opModeIsActive()) {
            tele.update();
            toTargetPosition(speed);
        }
    }

    public boolean isBusy() {

        tele.addData("is busy", "" + motorFL.motor.isBusy() + motorFR.motor.isBusy() + motorBL.motor.isBusy() + motorBR.motor.isBusy());

        tele.addData("target", "" + motorFL.motor.getTargetPosition() + " " + motorFR.motor.getTargetPosition() + " " + motorBR.motor.getTargetPosition() + " " + motorBL.motor.getTargetPosition());
        tele.addData("current", "" + motorFL.motor.getCurrentPosition() + " " + motorFR.motor.getCurrentPosition() + " " + motorBR.motor.getCurrentPosition() + " " + motorBL.motor.getCurrentPosition());

        return motorFL.motor.isBusy() || motorFR.motor.isBusy() || motorBL.motor.isBusy() || motorBR.motor.isBusy();
    }

    public void setTargetPosition(int[] asMotor) {

        motorFL.motor.setTargetPosition(asMotor[0]);
        motorFR.motor.setTargetPosition(asMotor[1]);
        motorBL.motor.setTargetPosition(asMotor[2]);
        motorBR.motor.setTargetPosition(asMotor[3]);
    }

    public void autonomousMove(double x, double y, double rot, double speed, LinearOpMode opMode) {

        setTargetPosition(x, y, rot);
        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitUntilNotBusy(speed, opMode);
        driveRobotCentric(0, 0, 0);
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveRobotCentric(double x, double y, double rot) {

        x *= Math.abs(x);
        y *= Math.abs(y);
        rot *= Math.abs(rot);

        drive.driveRobotCentric(-y * linearSpeed * speedFactor, -x * linearSpeed * speedFactor, -rot * speedFactor);
    }

    public void driveFieldCentric(double x, double y, double rot, double heading) {

        drive.driveFieldCentric(-y * linearSpeed * speedFactor, -x * linearSpeed * speedFactor, -rot * speedFactor, -(heading - referenceHeading) / Math.PI * 180);
    }


}