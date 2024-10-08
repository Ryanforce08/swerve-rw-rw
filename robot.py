from components.swerve_drive import SwerveDrive
from components.swerve_wheel import SwerveWheel
from constants import *
from phoenix6.hardware import TalonFX
from phoenix6.hardware import CANcoder
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
import magicbot
import navx
import wpilib
from wpilib import SmartDashboard
from wpilib.simulation import FlywheelSim
from wpimath.system.plant import DCMotor

class Larry(magicbot.MagicRobot):
    swerve_drive: SwerveDrive
    front_left: SwerveWheel
    front_right: SwerveWheel
    rear_left: SwerveWheel
    rear_right: SwerveWheel
    
    def createObjects(self):
        # Swerve Wheels
        self.front_left_speed_motor = TalonFX(kleftFrontSpeedID)
        self.front_left_direction_motor = TalonFX(kleftFrontDirectionID)
        self.front_left_cancoder = CANcoder(kflCANcoderID)


        self.front_right_speed_motor = TalonFX(krightFrontSpeedID)
        self.front_right_direction_motor = TalonFX(krightFrontDirectionID)
        self.front_right_cancoder = CANcoder(kfrCANcoderID)


        self.rear_left_speed_motor = TalonFX(kleftRearSpeedID)
        self.rear_left_direction_motor = TalonFX(kleftRearDirectionID)
        self.rear_left_cancoder = CANcoder(krlCANcoderID)


        self.rear_right_speed_motor = TalonFX(krightRearSpeedID)
        self.rear_right_direction_motor = TalonFX(krightRearDirectionID)
        self.rear_right_cancoder = CANcoder(krrCANcoderID)


        # Swerve Drive
        self.navX = navx.AHRS.create_spi()

        # Field
        if not self.isReal():
            self.field2d = wpilib.Field2d()
            SmartDashboard.putData("Field", self.field2d)


    def teleopInit(self):
        self.driver_controller = wpilib.XboxController(0)

    def teleopPeriodic(self):
        left_joy_x = kdeadband(self.driver_controller.getLeftX(), kdeadband)
        left_joy_y = kdeadband(self.driver_controller.getLeftY(), kdeadband)
        right_joy_x = kdeadband(self.driver_controller.getRightX(), kdeadband)

        if left_joy_x == 0 and left_joy_y == 0 and right_joy_x == 0:
            self.swerve_drive.freeze()
        else:
            self.swerve_drive.unfreeze()

        if self.driver_controller.getLeftBumper() and self.driver_controller.getRightBumper():
            self.swerve_drive.setSpeedMultiplier(0.25)
        elif self.driver_controller.getLeftBumper() or self.driver_controller.getRightBumper():
            self.swerve_drive.setSpeedMultiplier(0.5)
        else:
            self.swerve_drive.setSpeedMultiplier(1)
        
        self.swerve_drive.setTranslationX(left_joy_x)
        self.swerve_drive.setTranslationY(left_joy_y)
        self.swerve_drive.setRotationX(right_joy_x)

    def robotPeriodic(self) -> None:
        super().robotPeriodic()

        
        if not self.isReal():
            # Simulation stuff
            # Update Swerve Drive Odometry
            self.swerve_drive.updateOdometry()

            # TODO: UPDATE FIELD ENCODERS
            self.field2d.setRobotPose(self.swerve_drive.odometry.getPose())
            SmartDashboard.putData("Field", self.field2d)

if __name__ == "__main__":
    wpilib.run(Larry)
