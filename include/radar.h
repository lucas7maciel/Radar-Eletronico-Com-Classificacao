#pragma once

#include <zephyr/kernel.h>

enum radar_vehicle_type {
	RADAR_VEHICLE_LIGHT = 0,
	RADAR_VEHICLE_HEAVY,
};

enum radar_status {
	RADAR_STATUS_NORMAL = 0,
	RADAR_STATUS_WARNING,
	RADAR_STATUS_INFRACTION,
};

struct radar_classification {
	enum radar_vehicle_type type;
	uint32_t speed_kph;
	uint32_t limit_kph;
	uint32_t warning_kph;
	enum radar_status status;
};

uint32_t radar_calc_speed_kph(uint32_t distance_mm, uint32_t delta_ms);
enum radar_vehicle_type radar_classify_vehicle(uint32_t axle_pulses);
void radar_eval_status(uint32_t speed_kph,
		       enum radar_vehicle_type type,
		       uint32_t warning_percent,
		       uint32_t light_limit_kph,
		       uint32_t heavy_limit_kph,
		       struct radar_classification *out);
bool radar_plate_is_valid(const char *plate);

static inline const char *radar_vehicle_name(enum radar_vehicle_type type)
{
	return (type == RADAR_VEHICLE_HEAVY) ? "Pesado" : "Leve";
}

static inline const char *radar_status_name(enum radar_status st)
{
	switch (st) {
	case RADAR_STATUS_NORMAL:
		return "Normal";
	case RADAR_STATUS_WARNING:
		return "Alerta";
	case RADAR_STATUS_INFRACTION:
		return "Infração";
	default:
		return "Desconhecido";
	}
}
