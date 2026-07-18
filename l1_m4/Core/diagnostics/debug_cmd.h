/**
 * @file    debug_cmd.h
 * @brief   Debug command handler API.
 */
#ifndef DEBUG_CMD_H
#define DEBUG_CMD_H

#include <stdint.h>

void debug_cmd_init(void);
void debug_cmd_process_char(char c);
void debug_cmd_process_buffer(const uint8_t *data, uint32_t len);

#endif /* DEBUG_CMD_H */
