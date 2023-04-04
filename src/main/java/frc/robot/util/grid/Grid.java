package frc.robot.util.grid;

import java.util.List;

public class Grid {
    private final GridElement[][] elements;

    Grid(List<GridElement> elementsArray) {
        this.elements = parseElementsToArray(elementsArray);
    }

    private GridElement[][] parseElementsToArray(List<GridElement> elementList) {
        GridElement[][] elements = new GridElement[3][9];

        for(GridElement entry : elementList) {
            elements[entry.getRow()][entry.getCol()] = entry;
        }

        return elements;
    }

    GridElement accessElement(int row, int col) {
        return elements[row][col];
    }
}
