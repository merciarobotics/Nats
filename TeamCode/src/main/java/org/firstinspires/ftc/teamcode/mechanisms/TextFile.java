package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class TextFile {

    private static final File file =
            AppUtil.getInstance().getSettingsFile("file.txt");

    // Save all values by passing them in
    public static void saveAll(int pos1, int pos3, int pos5,
                               double posX, double posY, double bearing,
                               int currentGreenPos,int motifGreenPos) {

        String data = pos1 + "," +
                pos3 + "," +
                pos5 + "," +
                posX + "," +
                posY + "," +
                bearing + "," +
                currentGreenPos +"," +
                motifGreenPos;

        ReadWriteFile.writeFile(file, data);
    }

    // Load all values and return them
    public static double[] loadAll() {
        if (!file.exists()) return null;

        String[] parts = ReadWriteFile.readFile(file).trim().split(",");

        if(parts.length < 8){
            return new double[]{0,0,0,0,0,0,0,0};

        }

        double[] values = new double[8];
        for (int i = 0; i < 8; i++) {
            values[i] = Double.parseDouble(parts[i]);
        }
        return values;
    }
}
