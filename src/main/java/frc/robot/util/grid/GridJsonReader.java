package frc.robot.util.grid;

import com.google.gson.Gson;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.FileNotFoundException;
import java.io.FileReader;

public class GridJsonReader {

    static Grid getRedGrid() {
        return getGrid("grid-red");
    }

    static Grid getBlueGrid() {
        return getGrid("grid-blue");
    }

    private static Grid getGrid(String gridKey) {
        Gson gson = new Gson();

        return gson.fromJson(getFile(gridKey), Grid.class);
    }

    private static FileReader getFile(String key) {
        FileReader json = null;
        try {
            json = new FileReader(Filesystem.getDeployDirectory().toPath().resolve(key + ".json").toFile());

            return json;
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
    }
}
