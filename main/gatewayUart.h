
#ifndef GATEWAYUART_H_
#define GATEWAYUART_H_

#define TAG_GW                       "GW"

int GatewayInit(void);

// Snapshot de mediciones para enviar por LoRa
#include <stdbool.h>
typedef struct {
    float voltage[3];
    float current[3];
    float frequency;
    float power_factor[3];
    float active_power_total_import;
    float active_power_total_export;
    float reactive_power_total_import;
    float reactive_power_total_export;
    float apparent_power_total;
    bool  valid;
} GW_MeterSnapshot;

// Devuelve true si hay mediciones válidas disponibles
bool GW_GetLatestSnapshot(GW_MeterSnapshot* out);

#endif /* GATEWAYUART_H_ */
