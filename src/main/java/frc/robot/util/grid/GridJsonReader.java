package frc.robot.util.grid;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;

public class GridJsonReader {

  static Grid getRedGrid() {
    return getGrid("grid-red");
  }

  static Grid getBlueGrid() {
    return getGrid("grid-blue");
  }

  private static Grid getGrid(String gridKey) {
    Gson gson = new Gson();
    List<GridElement> elements = new ArrayList<>();

    JsonElement gridOutput = gson.fromJson(getFile(gridKey), JsonElement.class);

    JsonElement actualGridObject = gridOutput.getAsJsonObject().get(gridKey);
    actualGridObject
        .getAsJsonArray()
        .forEach(
            (e) -> {
              JsonObject obj = e.getAsJsonObject();

              JsonObject translation = obj.get("translation").getAsJsonObject();

              elements.add(
                  new GridElement(
                      obj.get("row").getAsInt(),
                      obj.get("col").getAsInt(),
                      new Translation3d(
                          translation.get("x").getAsDouble(),
                          translation.get("y").getAsDouble(),
                          translation.get("z").getAsDouble())));
            });

    Grid grid = new Grid(elements);

    return grid;
  }

  private static FileReader getFile(String key) {
    FileReader json = null;
    try {
      json =
          new FileReader(Filesystem.getDeployDirectory().toPath().resolve(key + ".json").toFile());

      return json;
    } catch (FileNotFoundException e) {
      throw new RuntimeException(e);
    }
  }
}
