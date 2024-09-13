from constants import *
from phoenix6.hardware import CANcoder,TalonFX
from math import fabs
import wpilib
from phoenix6.signals import NeutralModeValue

from phoenix6 import configs

class SwerveWheel:
    speed_motor: TalonFX
    direction_motor: TalonFX
    cancoder: CANcoder

    def setup(self) -> None:
        """
        This function is automatically called after the motors and encoders have been injected.
        """
        
        
        direction_talonfx_configs = configs.TalonFXConfiguration()
        speed_talonfx_configs = configs.TalonFXConfiguration()

        speed_talonfx_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE
        speed_talonfx_configs.motor_output.neutral_mode = NeutralModeValue.BRAKE

        # set slot 0 gains
        slot0_configs = direction_talonfx_configs.slot0
        slot0_configs.k_s = 0.25 # Add 0.25 V output to overcome static friction
        slot0_configs.k_v = 0.12 # A velocity target of 1 rps results in 0.12 V output
        slot0_configs.k_a = 0.01 # An acceleration of 1 rps/s requires 0.01 V output
        slot0_configs.k_p = 4.8 # A position error of 2.5 rotations results in 12 V output
        slot0_configs.k_i = 0 # no output for integrated error
        slot0_configs.k_d = 0.1 # A velocity error of 1 rps results in 0.1 V output

        # set Motion Magic settings
        direction_motion_magic_configs = direction_talonfx_configs.motion_magic
        direction_motion_magic_configs.motion_magic_cruise_velocity = 80 # Target cruise velocity of 80 rps
        direction_motion_magic_configs.motion_magic_acceleration = 160 # Target acceleration of 160 rps/s (0.5 seconds)
        direction_motion_magic_configs.motion_magic_jerk = 1600 # Target jerk of 1600 rps/s/s (0.1 seconds)

        self.speed_motor.configurator.apply(direction_talonfx_configs)

        # set slot 1 gains
        slot1_configs = speed_talonfx_configs.slot0
        slot1_configs.k_s = 0.25 # Add 0.25 V output to overcome static friction
        slot1_configs.k_v = 0.12 # A velocity target of 1 rps results in 0.12 V output
        slot1_configs.k_a = 0.01 # An acceleration of 1 rps/s requires 0.01 V output
        slot1_configs.k_p = 4.8 # A position error of 2.5 rotations results in 12 V output
        slot1_configs.k_i = 0 # no output for integrated error
        slot1_configs.k_d = 0.1 # A velocity error of 1 rps results in 0.1 V output

        # set Motion Magic settings
        speed_motion_magic_configs = speed_talonfx_configs.motion_magic
        speed_motion_magic_configs.motion_magic_cruise_velocity = 80 # Target cruise velocity of 80 rps
        speed_motion_magic_configs.motion_magic_acceleration = 160 # Target acceleration of 160 rps/s (0.5 seconds)
        speed_motion_magic_configs.motion_magic_jerk = 1600 # Target jerk of 1600 rps/s/s (0.1 seconds)


        self.direction_motor.configurator.apply(speed_talonfx_configs)
                


        
        
        self.direction_motor.configurator.
        self.speed_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, ktimeoutMs)


        self.direction_motor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)
        self.speed_motor.setSelectedSensorPosition(0.0, kPIDLoopIdx, ktimeoutMs)

        self.speed_motor.setInverted(True)
        
        self.cancoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, ktimeoutMs)
        self.cancoder.configSensorDirection(True, ktimeoutMs)

        # Sim CANCoder
        self.cancoder_sim = CANCoderSimCollection(self.cancoder)

        self.directionTargetPos = 0.0
        self.directionTargetAngle = 0.0
        self.isInverted = False
        self.desiredAngle = 0
        self.desiredSpeed = 0
        self.stopped = False

    """
    CONTROL METHODS
    """
    def setDesiredAngle(self, angle: int) -> None:
        """
        Sets the desired angle we want the direction motor to turn to
        when the execute command is ran.
        """
        self.desiredAngle = angle

    def setDesiredSpeed(self, speed: int) -> None:
        self.desiredSpeed = max(-1, min(1, speed))

    def stopWheel(self) -> None:
        self.speed_motor.set(TalonFXControlMode.PercentOutput, 0)
        self.direction_motor.set(TalonFXControlMode.PercentOutput, 0)
        self.direction_motor.setNeutralMode(NeutralMode.Coast)

        # Prevents SmartDashboard desync
        if kDebug:
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.device_id()) + " Mag", 0)

        self.stopped = True
    
    def getDirectionMotorPos(self) -> None:
        return self.speed_motor.get_position() / ksteeringGearRatio

    """
    EXECUTE
    """
    def execute(self) -> None:
        if self.stopped: # Stops angle from updating when stopped.
            self.stopped = False
            return

        """
        CHANGING DIRECTION
        """
        self.desiredAngle %= 360 # just making sure ;) (0-359)

        angleDist = fabs(self.desiredAngle - self.directionTargetAngle)

        # If the angleDist is more than 90 and less than 270, add 180 to the angle and %= 360 to get oppositeAngle.
        if (angleDist > 90 and angleDist < 270):
            targetAngle = (self.desiredAngle + 180) % 360
            self.isInverted = True
        
        # Else, then like, idk, just go to it??? smh
        else:
            targetAngle = self.desiredAngle
            self.isInverted = False

        # Before, to move the motor to the right spot, we would take the angle, convert that into talonFX units, then add (the amount of revolutions * 2048), then multiple everything by the motors gear ratio
        # However, to avoid having to deal with revolution compensation (which caused some issues), we now get the degree change, convert to motor units, then add or subtract depending on the direction we're rotating
        targetAngleDist = fabs(targetAngle - self.directionTargetAngle)

        # When going from x angle to 0, the robot will try and go "the long way around" to the angle. This just checks to make sure we're actually getting the right distance
        if targetAngleDist > 180:
            targetAngleDist = abs(targetAngleDist - 360)

        changeInTalonUnits = targetAngleDist / (360/2048)

        # Now that we have the correct angle, we figure out if we should rotate counterclockwise or clockwise
        angleDiff = targetAngle - self.directionTargetAngle

        # Accounting if the angleDiff is negative
        if angleDiff < 0:
            angleDiff += 360

        # If angleDiff is greater than 180, go counter-clockwise (ccw is positive for talonFX, and vice versa)
        if angleDiff > 180:
            self.directionTargetPos -= changeInTalonUnits

        # Else, go clockwise
        else:
            self.directionTargetPos += changeInTalonUnits

        self.directionTargetAngle = targetAngle

        if kDebug:
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.device_id()) + " dirTargetAngle", self.directionTargetAngle)
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.device_id()) + " dirTargetPos", self.directionTargetPos)
            wpilib.SmartDashboard.putBoolean(str(self.speed_motor.device_id()) + " Inverted?", self.isInverted)

        # Now we can actually turn the motor after like 60 lines lmao
        self.direction_motor.set(TalonFXControlMode.MotionMagic, self.directionTargetPos * ksteeringGearRatio)

        """
        ACTUALLY MOVING
        """
        if self.isInverted:
            self.desiredSpeed *= -1

        # Slows down motor at a curve depending on how far the motor angle is from the target angle.
        # Don't worry about the actual equation, I just eyeballed the numbers lmao
        angleDiff = self.directionTargetAngle - (self.direction_motor.getSelectedSensorPosition() / ksteeringGearRatio) * (360 / 2048)
        angleDiff = (angleDiff + 180) % 360 - 180

        slowdownMult = max(-1.0, min(1.0, (-(3.14514 / 112006) * (angleDiff ** 2)) + 1))
        if not wpilib.RobotBase.isReal():
            slowdownMult = 1

        self.speed_motor.set(TalonFXControlMode.PercentOutput, max(-1, min(1, self.desiredSpeed * slowdownMult)))

        if kDebug:
            wpilib.SmartDashboard.putNumber(str(self.speed_motor.getDeviceID()) + " Mag", max(-1, min(1, self.desiredSpeed * slowdownMult)))
        