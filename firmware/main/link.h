#pragma once
#include <stdint.h>
#include <stddef.h>

typedef void (*link_rx_cb_t)(uint8_t type, const uint8_t *payload, size_t len);

void link_init(link_rx_cb_t cb);
void link_send(uint8_t type, const void *payload, size_t len);
