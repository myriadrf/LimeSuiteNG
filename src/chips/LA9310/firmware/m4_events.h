#ifndef LIME_M4_EVENTS_H
#define LIME_M4_EVENTS_H

// Indexes of the events that the M4 firmware signals to the host (LA9310_EVENT_COUNT in la9310_host_if.h).
// The driver only transports them, their meaning is agreed between the M4 firmware and the host application.
enum M4_EVENT {
    HOST_COMMAND_DONE = 1,
    VSPA_DDR_WRITE_DONE = 2,
    VSPA_DDR_READ_DONE = 3,
};

// Signals that the host sends to the M4 (message unit MSIIR bit index)
enum LA9310_SIGNALS {
    IRQ_FLAGS_CHANGED = 0,
    HOST_COMMAND_POSTED = 1,
};

#endif // LIME_M4_EVENTS_H
