/**
 * @file    crash_dump.h
 * @brief   Crash dump analysis API.
 */
#ifndef CRASH_DUMP_H
#define CRASH_DUMP_H

int  crash_dump_analyse(const char *map_path);
int  crash_dump_available(void);
void crash_dump_clear(void);

#endif /* CRASH_DUMP_H */
