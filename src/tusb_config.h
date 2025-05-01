#if !defined(_TUSB_CONFIG_H_)
#define _TUSB_CONFIG_H_

#include <tusb_option.h>

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)

#define CFG_TUD_CDC 2
#define CFG_TUD_CDC_RX_BUFSIZE 1024
#define CFG_TUD_CDC_TX_BUFSIZE 1024

#endif /* _TUSB_CONFIG_H_ */
