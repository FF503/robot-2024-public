package org.frogforce503.lib.drivers;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

public class CANMotor {

    private final CANMotorType motorType;
    private final int CAN_ID;
    
    private CANSparkMaxWrapper sparkMax;
    private TalonFXWrapper fx;
    private TalonSRXWrapper srx;

    private int pidSlot;

    public Sim sim;

    private double lastSetpoint = 503.6328;
    private double lastFF = 503.6328;
    private MotorControlMode lastControlMode = null;

    private static class Sim {
        private double timeForOne; // number of seconds to complete a full revolution
        private double timeConstantFactor; // MOTOR IS MODELED AS LOGARITHMIC GROWTH, LIKE A CHARGING CAPACITOR
        private double CPR;

        private Setpoint lastCommand;
        private double lastUpdateTime;
        private double lastPosition = 0;
        
        private double currentPosition = 0;
        private double currentVelocity = 0;

        // private static final double LN_EPSILON = Math.log(0.0001);

        public Sim(double timeConstantFactor, double CPR) {
            this.timeConstantFactor = timeConstantFactor;
            this.timeForOne = Math.sqrt(timeConstantFactor); // 5 * (1 / timeConstantFactor); // https://www.electronics-tutorials.ws/rc/rc_1.html
            this.CPR = CPR;

            this.lastCommand = new Setpoint(Timer.getFPGATimestamp(), MotorControlMode.PercentOutput, 0, 0);
            this.lastUpdateTime = Timer.getFPGATimestamp();
        }

        public void set(MotorControlMode controlMode, double value) {
            if (controlMode == MotorControlMode.Position || controlMode == MotorControlMode.Profiled)
                value /= CPR;
            
            else if (controlMode == MotorControlMode.Velocity)
                value *= 60;

            lastCommand = new Setpoint(Timer.getFPGATimestamp(), controlMode, value, this.currentPosition);
        }

        public void update() {
            // updates speed and position
            double time = Timer.getFPGATimestamp();

            switch(lastCommand.controlMode) {
                case PercentOutput: {
                    this.currentVelocity = (1 / this.timeForOne) * this.lastCommand.demand; // basically, 1.0 power is max velocity
                    this.currentPosition += (time - this.lastUpdateTime) * this.currentVelocity;
                    break;
                }
                case Velocity: {
                    this.currentVelocity = this.lastCommand.demand;
                    this.currentPosition += (time - this.lastUpdateTime) * this.currentVelocity;
                    break;
                }
                case Profiled:
                case Position: {
                    // https://www.desmos.com/calculator/euno0nwy5h
                    double t = time - this.lastCommand.startTimestamp;

                    if (Math.abs(lastCommand.demand - this.currentPosition) < 0.01) {
                        this.currentPosition = lastCommand.demand;
                        break;
                    }

                    double a = this.lastCommand.demand - this.lastCommand.initialPosition;
                    double b = Math.abs(a) * (1 / this.timeConstantFactor);

                    double e = Math.pow(Math.E, -t/b);

                    double r = (Math.abs(a) * (1 - e));

                    this.currentPosition = (a < 0 ? -r : r) + this.lastCommand.initialPosition;
                    this.currentVelocity = (this.currentPosition - this.lastPosition) - (time - this.lastUpdateTime);

                    break;
                }
            }

            this.lastPosition = this.currentPosition;
            this.lastUpdateTime = time;
        }

        public double getPosition() {
            return (this.currentPosition * this.CPR);
        }

        public double getVelocity() {
            return (this.currentVelocity * this.CPR) / 60;
        }

        public void setEncoderPosition(double position) {
            this.currentPosition = position / this.CPR;
        }

        private class Setpoint {
            public final double startTimestamp;
            public final MotorControlMode controlMode;
            public final double demand;
            public final double initialPosition;

            public final double journeyDuration;

            public Setpoint(double t, MotorControlMode m, double d, double i) {
                this.startTimestamp = t;
                this.controlMode = m;
                this.demand = d;
                this.initialPosition = i;

                if (this.controlMode == MotorControlMode.Position) {
                    this.journeyDuration = timeForOne * (d - i);
                } else {
                    this.journeyDuration = 0;
                }
            }
        }
    }

    private HashMap<MotorControlMode, com.ctre.phoenix.motorcontrol.ControlMode> ctreModes = new HashMap<MotorControlMode, com.ctre.phoenix.motorcontrol.ControlMode>() {{
        put(MotorControlMode.PercentOutput, com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput);
        put(MotorControlMode.Position, com.ctre.phoenix.motorcontrol.ControlMode.Position);
        put(MotorControlMode.Velocity, com.ctre.phoenix.motorcontrol.ControlMode.Velocity);
        put(MotorControlMode.Profiled, com.ctre.phoenix.motorcontrol.ControlMode.MotionMagic);
    }};

    private HashMap<MotorControlMode, CANSparkMaxWrapper.ControlMode> revModes = new HashMap<MotorControlMode, CANSparkMaxWrapper.ControlMode>() {{
        put(MotorControlMode.PercentOutput, CANSparkMaxWrapper.ControlMode.PercentOutput);
        put(MotorControlMode.Position, CANSparkMaxWrapper.ControlMode.Position);
        put(MotorControlMode.Velocity, CANSparkMaxWrapper.ControlMode.Velocity);
        put(MotorControlMode.Profiled, CANSparkMaxWrapper.ControlMode.SmartMotion);
    }};

    private CANMotor(CANMotorType type, int CAN_ID, String canBus, MotorType revMotorType, boolean hasExternalEncoder) {
        this.motorType = type;
        this.CAN_ID = CAN_ID;

        if (this.motorType == CANMotorType.SPARK_MAX) {
            sparkMax = new CANSparkMaxWrapper(CAN_ID, revMotorType, hasExternalEncoder);
        } else if (this.motorType == CANMotorType.TALON_FX) {
            fx = canBus.equals("") ? new TalonFXWrapper(this.CAN_ID) : new TalonFXWrapper(this.CAN_ID, canBus);
        } else {
            srx = new TalonSRXWrapper(this.CAN_ID);
        }
    }

    /**
     * configure parameters for motor simulation
     * @param timeConstantFactor technically wrong in terms of physics, but just know that bigger = faster
     * @param CPR counts per rotation of the motor, this is the scale at which the thing will move
     */
    public void configureSim(double timeConstantFactor, double CPR) {
        this.sim = new Sim(timeConstantFactor, CPR);
    }

    // factory methods
    public static CANMotor createSparkMax(int id, MotorType revMotorType, boolean hasExternalEncoder) {
        return new CANMotor(CANMotorType.SPARK_MAX, id, null, revMotorType, hasExternalEncoder);
    }
    public static CANMotor createSparkMax(int id, MotorType revMotorType) {
        return CANMotor.createSparkMax(id, revMotorType, false);
    }

    public static CANMotor createTalonFX(int id, String canBus) {
        return new CANMotor(CANMotorType.TALON_FX, id, canBus, null, false);
    }
    public static CANMotor createTalonFX(int id) {
        return CANMotor.createTalonFX(id, "");
    }

    public static CANMotor createTalonSRX(int id) {
        return new CANMotor(CANMotorType.TALON_SRX, id, null, null, false);
    }

    public CANSparkMaxWrapper getSparkMax() {
        return sparkMax;
    }
    public TalonFXWrapper getTalonFX() {
        return fx;
    }
    public TalonSRXWrapper getTalonSRX() {
        return srx;
    }

    public void setPID(int slot, double p, double i, double d) {
        setPIDF(slot, p, i, d, 0.0);
    }

    public void setPIDF(int slot, double p, double i, double d, double f) {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            this.sparkMax.setP(slot, p);
            this.sparkMax.setI(slot, i);
            this.sparkMax.setD(slot, d);
            this.sparkMax.setFF(slot, f);
        } else if (this.motorType == CANMotorType.TALON_FX) {
            this.fx.config_kP(slot, p);
            this.fx.config_kI(slot, i);
            this.fx.config_kD(slot, d);
            this.fx.config_kF(slot, f);
        } else {
            this.srx.config_kP(slot, p);
            this.srx.config_kI(slot, i);
            this.srx.config_kD(slot, d);
            this.srx.config_kF(slot, f);
        }
    }

    public double getP() {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return this.sparkMax.getPIDController().getP();
        } else if (this.motorType == CANMotorType.TALON_FX) {
            
        } else {
            // srx
        }
        return 0.0;
    }

    public double getI() {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return this.sparkMax.getPIDController().getI();
        } else if (this.motorType == CANMotorType.TALON_FX) {
            
        } else {
            // srx
        }
        return 0.0;
    }

    public double getD() {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return this.sparkMax.getPIDController().getD();
        } else if (this.motorType == CANMotorType.TALON_FX) {
            
        } else {
            // srx
        }
        return 0.0;
    }

    public double getF() {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return this.sparkMax.getPIDController().getFF();
        } else if (this.motorType == CANMotorType.TALON_FX) {

        } else {
            // srx
        }
        return 0.0;
    }

    public void set(MotorControlMode mode, double value, double arbFF) {

        if (mode == this.lastControlMode && value == this.lastSetpoint && arbFF == this.lastFF)
            return;

        if (RobotBase.isSimulation()) {
            this.sim.set(mode, value);
        }

        if (this.motorType == CANMotorType.SPARK_MAX) {
            this.sparkMax.set(revModes.get(mode), value, arbFF);
        } else if (this.motorType == CANMotorType.TALON_FX) {
            if (arbFF != 0) {
                this.fx.set(ctreModes.get(mode), value, DemandType.ArbitraryFeedForward, arbFF);
            } else {
                this.fx.set(ctreModes.get(mode), value);
            }
        } else {
            if (arbFF != 0) {
                this.srx.set(ctreModes.get(mode), value, DemandType.ArbitraryFeedForward, arbFF);
            } else {
                this.srx.set(ctreModes.get(mode), value);
            }
        }

        this.lastSetpoint = value;
        this.lastControlMode = mode;
        this.lastFF = arbFF;
    }

    public void set(MotorControlMode mode, double value) {
        this.set(mode, value, 0);
    }

    public boolean withinVelocityTolerance(double tol) {
        if (this.lastSetpoint == 503.6328)
            return false;

        return Math.abs(this.getVelocity() - this.lastSetpoint) < tol;
    }

    public boolean withinTolerance(double tol) {
        if (this.lastSetpoint == 503.6328)
            return false;

        return Math.abs(this.getPosition() - this.lastSetpoint) < tol;
    }

    public void update() {
        if (RobotBase.isSimulation())
            this.sim.update();
    }

    public void setInverted(boolean set) {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            this.sparkMax.setInverted(set);
        } else if (this.motorType == CANMotorType.TALON_FX) {
            this.fx.setInverted(set);
        } else {
            this.srx.setInverted(set);
        }
    }

    public void setEncoderPosition(double position) {
        if (RobotBase.isSimulation())
            sim.setEncoderPosition(position);

        if (this.motorType == CANMotorType.SPARK_MAX) {
            sparkMax.setEncoderPosition(position);
        } else if (this.motorType == CANMotorType.TALON_FX) {
            fx.setSelectedSensorPosition(position);
        } else {
            srx.setSelectedSensorPosition(position);
        }
    }
    
    public double getPercent() {
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return sparkMax.getAppliedOutput();
        } else if (this.motorType == CANMotorType.TALON_FX) {
            return fx.getMotorOutputPercent();
        } else {
            return srx.getMotorOutputPercent();
        }
    }

    public double getPosition() {
        if (RobotBase.isSimulation())
            return sim.getPosition();
        
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return sparkMax.getEncoderPosition();
        } else if (this.motorType == CANMotorType.TALON_FX) {
            return fx.getSelectedSensorPosition();
        } else {
            return srx.getSelectedSensorPosition();
        }
    }

    public double getVelocity() {
        if (RobotBase.isSimulation())
            return sim.getVelocity();
        
        if (this.motorType == CANMotorType.SPARK_MAX) {
            return sparkMax.getEncoderVelocity();
        } else if (this.motorType == CANMotorType.TALON_FX) {
            return fx.getSelectedSensorVelocity();
        } else {
            return srx.getSelectedSensorVelocity();
        }
    }

    public double getOutputFromMode(MotorControlMode m) {
        switch (m) {
            case PercentOutput:
                return getPercent();
            case Position:
                return getPosition();
            case Velocity:
                return getVelocity();
            default:
                return getPercent();
        }
    }
    
    public enum CANMotorType {
        SPARK_MAX,
        TALON_FX,
        TALON_SRX
    }

    public enum MotorControlMode {
        PercentOutput,
        Position,
        Velocity,
        Profiled
    }

    public enum PIDSlot {
        POSITION(0), VELOCITY(1), PROFILED(2);

        public int id;
        PIDSlot(int id){
            this.id = id;
        }
    }

    public static class MotorSetpoint {
        public MotorControlMode controlMode;
        public double output;

        public MotorSetpoint(MotorControlMode controlMode, double output) {
            this.controlMode = controlMode;
            this.output = output;
        }

        public MotorSetpoint() {
            this(MotorControlMode.PercentOutput, 0);
        }
    }
}