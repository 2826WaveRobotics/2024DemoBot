package frc.lib.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveModule;

public class ShuffleboardContent {
    public static final ShuffleboardTab competitionTab = Shuffleboard.getTab("Competition");

    public static void initSwerveModuleShuffleboard(SwerveModule module) {
        if(!Constants.enableNonEssentialShuffleboard) return;
        
        int moduleNumber = module.moduleIndex;
        String[] modulePosition = { "FL", "FR", "BL", "BR" };
        String modulePositionAbbreviation = modulePosition[moduleNumber];
        String swerveModuleHeader = module.getModuleName() + " Module (" + moduleNumber + ")";

        ShuffleboardLayout swerveModuleLayout = Shuffleboard.getTab("Swerve Module Data")
                .getLayout(swerveModuleHeader, BuiltInLayouts.kList).withPosition(moduleNumber * 2, 0)
                .withSize(2, 3).withProperties(Map.of("Label position", "LEFT"));

        swerveModuleLayout.addNumber("CANcoder reported position (" + modulePositionAbbreviation + ")", module::getReportedAbsoluteAngleDegrees);
        swerveModuleLayout.addNumber("Absolute position (" + modulePositionAbbreviation + ")", module::getAbsoluteAngleDegrees);
        swerveModuleLayout.addNumber("Drive current (" + modulePositionAbbreviation + ")", module::getAppliedDriveCurrent);
        swerveModuleLayout.addNumber("Turn current (" + modulePositionAbbreviation + ")", module::getAppliedAngleCurrent);
        swerveModuleLayout.addNumber("Drive motor RPM (" + modulePositionAbbreviation + ")", module::getDriveMotorVelocityRPM);
        swerveModuleLayout.addNumber("Turn velocity rad per sec (" + modulePositionAbbreviation + ")", module::getTurnVelocityRadPerSec);
    }
}
