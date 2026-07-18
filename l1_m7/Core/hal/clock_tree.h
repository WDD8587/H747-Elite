/**
 * @file    clock_tree.h
 * @brief   Clock tree configuration API.
 */
#ifndef CLOCK_TREE_H
#define CLOCK_TREE_H

#include <stdint.h>

void clock_tree_init(void);
void clock_tree_get_frequencies(uint32_t *hclk_freq,
                                 uint32_t *apb1_freq,
                                 uint32_t *apb2_freq);

#endif /* CLOCK_TREE_H */
