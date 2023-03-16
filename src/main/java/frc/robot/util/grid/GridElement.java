package frc.robot.util.grid;

import edu.wpi.first.math.geometry.Translation3d;

import static frc.robot.util.grid.GridElement.Type.*;

public class GridElement {

    private final int row;
    private final int col;
    private final Type type;

    private Translation3d location;

    GridElement(int row, int col, Translation3d location) {
        this.row = row;
        this.col = col;

        this.type = determineType();

        this.location = location;
    }

    /**
     * Get the location (in field space) of this element of the grid
     * @return the pose of the element
     */
    public Translation3d getLocation() {
        return location;
    }

    /**
     * Get the row of the element, starting from the top [0-2]
     * @return the row
     */
    public int getRow() {
        return row;
    }

    /**
     * Get the column of the element, starting from the left when looking at the driver stations [0-8]
     * @return
     */
    public int getCol() {
        return col;
    }

    private Type determineType() {
        if(row == 2) return Both; //The element can be either if it's on the bottom row
        if(col == 1 || col==4 || col==7) return Cube; //The element can only be a cube in the 2nd, 5th, and 8th columns
        return Cone; //The element can only be a cone if it's anywhere else
    }

    public enum Type {
        Cone,
        Cube,
        Both 
    }
}
