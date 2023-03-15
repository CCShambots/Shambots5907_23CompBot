package frc.robot.util.grid;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.util.grid.GridJsonReader.*;

public class GridInterface {

    private Grid grid;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("grid-ui");
    private List<GridElement> elementsToSend = new ArrayList<>(); //list of elements that have been placed to send to the ds

    public GridInterface(Alliance alliance) {
        grid = alliance == Alliance.Red ? getRedGrid() : getBlueGrid();
    }

    /**
     * Updates the grid interface by checking to see if the robot has placed elements and sending them to the dashboard
     * Must be called periodically
     */
    public void update() {
        if(dashboardAcceptingPlacedElements() && elementsToSend.size() > 0) {
            notifyElementPlaced(elementsToSend.get(0));

            elementsToSend.remove(0);
        }
    }

    /**
     * Indicate that the robot no long is forcing an element type
     */
    public void removeForceElement() {
        table.getEntry("force-type").setString(" none");
    }

    /**
     * Indicate that the robot is forcing a cone
     */
    public void forceCone() {
        table.getEntry("force-type").setString("cone");
    }

    /**
     * Indicate that the robot is forcing a cube
     */
    public void forceCube() {
        table.getEntry("force-type").setString("cube");
    }

    /**
     * Get the next element that should be placed by the robot
     * @return the next element to place
     */
    public GridElement getNextElement() {
        int row = (int) table.getEntry("next/row").getInteger(0);
        int col = (int) table.getEntry("next/col").getInteger(0);

        return grid.accessElement(row, col);
    }


    /**
     * Evaluate if the dashboard is ready to accept a newly placed element
     * @return if the dashboard can accept a new element
     */
    private boolean dashboardAcceptingPlacedElements() {
        return !table.getEntry("placed/just-placed").getBoolean(false);
    }

    /**
     * Indicate that an element has been placed
     * @param ele the element that has been placed
     */
    public void indicateElementPlaced(GridElement ele) {
        elementsToSend.add(ele);
    }

    /**
     * Indicate that an element has been placed
     * @param row the row of the element
     * @param col the column of the element
     */
    public void indicateElementPlaced(int row, int col) {
        indicateElementPlaced(grid.accessElement(row, col));
    }

    /**
     * Notifies the dashboard that an element has been placed
     * @param element
     */
    private void notifyElementPlaced(GridElement element) {

        table.getEntry("placed/row").setInteger(element.getRow());
        table.getEntry("placed/col").setInteger(element.getCol());

        table.getEntry("placed/just-placed").setBoolean(true);
    }
}