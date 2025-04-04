#ifndef __BMS_CONFIG_H
#define __BMS_CONFIG_H

#define TOTALBOARDS 2 // Including base
#define CELLS_IN_SERIES 16 // On each board

#define TOTAL_CELLS (TOTALBOARDS * CELLS_IN_SERIES) // Counts cells in parallel as one

// All of these parameters can be set through serial, and will be stored on the flash
typedef struct {
    uint16_t BroadcastID; // The ID of the board
    uint16_t CellCount; // Total number of cells
    uint16_t CellCountInSeries; // Number of cells in series
    uint16_t CellCountInParallel; // Number of cells in parallel
    uint16_t CellVoltageLimitLow; // The minimum voltage of a cell
    uint16_t CellVoltageLimitHigh; // The maximum voltage of a cell
} BMS_ConfigTypeDef;

#endif // __BMS_CONFIG_H