











#ifndef __ICS2_RA6T2_H__
#define __ICS2_RA6T2_H__

#define ICS_SCI9_P605_P606     (0x90U)

void     ics2_init(uint8_t port, uint8_t speed, uint8_t mode);
void     ics2_watchpoint(void);
uint32_t ics2_version(void);


#endif /* __ICS2_RA6T2_H__ */
