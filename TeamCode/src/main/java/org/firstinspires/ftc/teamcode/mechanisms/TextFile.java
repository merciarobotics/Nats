package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class TextFile {

    private static final File file =
            AppUtil.getInstance().getSettingsFile("file.txt");

    // Save all values by passing them in
    public static void saveAll(double posX, double posY, double bearing) {

        String data =
                posX + "," +
                posY + "," +
                bearing + ",";


        ReadWriteFile.writeFile(file, data);
    }

    // Load all values and return them
    public static double[] loadAll() {
        if (!file.exists()) return null;

        String[] parts = ReadWriteFile.readFile(file).trim().split(",");

        if(parts.length < 3){
            return new double[]{0,0,0};

        }

        double[] values = new double[3];
        for (int i = 0; i < 3; i++) {
            values[i] = Double.parseDouble(parts[i]);
        }
        return values;
    }
}
