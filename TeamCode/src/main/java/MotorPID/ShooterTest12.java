package MotorPID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
@TeleOp(name = "Shooter Dashboard TeleOp", group = "Test")
public class ShooterTest12 extends OpMode {

    private DcMotorEx shooterMotor; // config name: "shooter"
    private DcMotorEx shooterMotor2; // config name: "shooter2"
    private VoltageSensor battery;
    private MultipleTelemetry dashTelemetry;

    // Spiking issue fixed: feedforward voltage scaling and absolute velocity matching
    public static double kF = 0.0001458; //0.000167
    // Proportional: power per RPM of error. Keep small for flywheels!
    public static double kP = 0.0002; //0.00015, 0.001
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double targetRPM = 4500;
    public static double tolerance = 50;
    public static double maxRPM = 5000;

    // 0..1, lower = smoother but slower to react
    public static double filterAlpha = 0.8;
    // Max power change per second (soft start)
    public static double rampPerSec = 99.0;

    public static final double ticksPerRev = 28.0;
    public static final double NOMINAL_VOLTAGE = 14.0;

    private boolean on = false;
    private boolean lastX = false;
    private double activeTarget = 0;
    private double filteredRPM = 0;
    private double integral = 0;
    private double lastError = 0;
    private double lastPower = 0;
    private long lastTimeNs = 0;

    @Override
    public void init() {
        dashTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        battery = hardwareMap.voltageSensor.iterator().next();

// Flip these if one fights the other
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{shooterMotor, shooterMotor2}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    @Override
    public void start() {
        lastTimeNs = System.nanoTime();
        filteredRPM = 0;
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = Math.max((now - lastTimeNs) / 1e9, 1e-3);
        lastTimeNs = now;

        double motor1RPM = (Math.abs(shooterMotor.getVelocity()) / ticksPerRev) * 60.0;
        double motor2RPM = (Math.abs(shooterMotor2.getVelocity()) / ticksPerRev) * 60.0;
        double rawRPM = (motor1RPM + motor2RPM) / 2.0;
        filteredRPM = filterAlpha * rawRPM + (1 - filterAlpha) * filteredRPM;

        if (gamepad1.x && !lastX) {
            on = !on;
            if (on) {
// Reset ramp memory to prevent an explosive start
                lastPower = 0;
                integral = 0;
                lastError = 0;
            }
        }
        lastX = gamepad1.x;

        if (on) {
            activeTarget = Math.min(targetRPM, maxRPM);
        } else {
            activeTarget = 0;
            integral = 0;
            lastPower = 0;
        }

        if (!on) {
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
            lastPower = 0;
        } else {
            double error = activeTarget - filteredRPM;
            integral += error * dt;
            integral = Math.max(-1000, Math.min(1000, integral));
            double derivative = (error - lastError) / dt;
            lastError = error;

// 1. Calculate base feedforward
            double ffPower = kF * activeTarget;

// 2. ONLY apply battery compensation to the Feedforward block
            double volts = Math.max(battery.getVoltage(), 6.0);
            ffPower *= (NOMINAL_VOLTAGE / volts);

// 3. Combine Feedforward and PID feedback terms
            double power = ffPower + (kP * error) + (kI * integral) + (kD * derivative);

            power = Math.max(0, Math.min(1.0, power));

// 4. Soft-start ramp calculation
            double maxStep = rampPerSec * dt;
            if (power > lastPower + maxStep) {
                power = lastPower + maxStep;
            }
            lastPower = power;

            shooterMotor.setPower(power);
            shooterMotor2.setPower(power);
        }

        boolean atVelocity = on && Math.abs(filteredRPM - targetRPM) <= tolerance;

        dashTelemetry.addData("Shooter Status", on ? "RUNNING" : "STOPPED");
        dashTelemetry.addData("Target RPM", activeTarget);
        dashTelemetry.addData("Filtered RPM", filteredRPM);
        dashTelemetry.addData("At Velocity", atVelocity);
        dashTelemetry.addData("Motor 1 RPM", motor1RPM);
        dashTelemetry.addData("Motor 2 RPM", motor2RPM);
        dashTelemetry.addData("Power Sent", lastPower);
        dashTelemetry.addData("Voltage Sensor", battery.getVoltage());
        dashTelemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null && shooterMotor2 != null) {
            shooterMotor.setPower(0);
            shooterMotor2.setPower(0);
        }
    }
}

