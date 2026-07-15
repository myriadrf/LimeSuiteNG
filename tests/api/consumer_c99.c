/* Plain C99 program using only the public C API headers. Proves the
   installed headers compile as C and the library links from C, the
   completion proof asked for in issue #187. Runs without hardware:
   finding zero devices is a pass. */

#include "limesuiteng/limesuiteng.h"

#include <stdio.h>
#include <string.h>

int main(void)
{
    printf("limesuiteng %s, api %s, abi %s\n", lime_get_library_version(), lime_get_api_version(), lime_get_abi_version());

    lime_DeviceHandle handles[8];
    const int found = lime_enumerate(handles, 8);
    if (found < 0)
        return 1;
    printf("devices found: %d\n", found);

    /* the stream entry points must reject invalid input and accept NULL handles */
    lime_StreamConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.struct_size = (uint32_t)sizeof(cfg);
    if (lime_stream_create(NULL, &cfg) != NULL)
        return 1;
    lime_stream_stop(NULL);
    lime_stream_destroy(NULL);
    return 0;
}
