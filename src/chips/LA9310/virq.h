#ifndef LIME_LA9310_VIRQ_H
#define LIME_LA9310_VIRQ_H

namespace lime {

enum LA9310_SIGNALS {
    IRQ_FLAGS_CHANGED = 0,
    HOST_COMMAND_POSTED = 1,
};

enum LA9310_VIRQ {
    HOST_COMMAND_DONE = 1,
    VSPA_DDR_WRITE_DONE = 2,
    VSPA_DDR_READ_DONE = 3,
};

} // namespace lime

#endif // LIME_LA9310_VIRQ_H