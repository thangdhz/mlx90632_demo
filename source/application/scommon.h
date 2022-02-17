#ifndef __SCOMMON_H
#define __SCOMMON_H

#include <stdint.h>
#include <string.h>

#ifndef MAX
#define MAX(a,b)                                   ((a > b)? a : b)
#define MIN(a,b)                                   ((a > b)? b : a)
#endif /* MAX */

#ifndef HI_UINT16
#define HI_UINT16(a)                               (((a) >> 8) & 0xFF)
#define LO_UINT16(a)                               ((a) & 0xFF)
#define HI_UINT8(a)                                (((a) >> 4) & 0x0F)
#define LO_UINT8(a)                                ((a) & 0x0F)
#endif /* HI_UINT16 */

#define BYTE0_UINT32(x)                            (((x) >> 24) & 0xFF)
#define BYTE1_UINT32(x)                            (((x) >> 16) & 0xFF)
#define BYTE2_UINT32(x)                            (((x) >> 8) & 0xFF)
#define BYTE3_UINT32(x)                            ((x) & 0xFF)

#undef unlikely
#define unlikely(x)                                (x)

#if (!defined(require))
#define require(X, LABEL)                                                     \
    do {                                                                      \
        if (unlikely(!(X))) {                                                 \
            goto LABEL;                                                       \
        }                                                                     \
                                                                              \
    } while (0)
#endif

#if (!defined(require_action))
#define require_action(X, LABEL, ACTION)                                      \
    do {                                                                      \
        if (unlikely(!(X))) {                                                 \
            {                                                                 \
                ACTION;                                                       \
            }                                                                 \
            goto LABEL;                                                       \
        }                                                                     \
                                                                              \
    } while (0)
#endif

#endif // __SCOMMON_H