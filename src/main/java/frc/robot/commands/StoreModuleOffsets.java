package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Swerve;

public class StoreModuleOffsets extends Command {
    private ArrayList<double[]> offsets = new ArrayList<>();

    @Override
    public void initialize() {
        offsets = new ArrayList<>();
        for(int i = 0; i < 4; i++) {
            offsets.add(new double[4]);
        }
    }

    @Override
    public void execute() {
        Swerve swerve = Swerve.getInstance();
        for(int i = 0; i < offsets.size(); i++) {
            offsets.add(swerve.getOffsets());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!DriverStation.isTest()) return;

        double[] sums = new double[4];
        for(var offset : offsets) {
            for(int i = 0; i < 4; i++) {
                sums[i] += offset[i];
            }
        }
        StringBuilder string = new StringBuilder();
        for(int i = 0; i < 4; i++) {
            double average = sums[i] / offsets.size();
            string.append(average);
            if(i < 3) string.append("\n");
        }

        try (
            BufferedWriter bw = new BufferedWriter(
                new FileWriter(new File(Filesystem.getDeployDirectory(), Constants.Swerve.swerveOffsetFileName))
            )
        ) {
            bw.write(string.toString(), 0, string.length());
        } catch(IOException ioException) {
            DriverStation.reportError("ERROR: IO exception while reading swerve offsets: " + ioException.getLocalizedMessage(), false);
        }

        DriverStation.reportWarning("Stored current module offsets!", null);
    }
}
