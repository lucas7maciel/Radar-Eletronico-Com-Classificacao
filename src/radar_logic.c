#include "radar.h"

#include <ctype.h>
#include <stdbool.h>
#include <string.h>

#define SPEED_SCALE_NUM 36U
#define SPEED_SCALE_DEN 10U

uint32_t radar_calc_speed_kph(uint32_t distance_mm, uint32_t delta_ms)
{
	if (delta_ms == 0U) {
		return 0U;
	}

	uint64_t scaled = (uint64_t)distance_mm * SPEED_SCALE_NUM;
	uint64_t denom = (uint64_t)delta_ms * SPEED_SCALE_DEN;

	return (uint32_t)(scaled / denom);
}

enum radar_vehicle_type radar_classify_vehicle(uint32_t axle_pulses)
{
	return (axle_pulses >= 3U) ? RADAR_VEHICLE_HEAVY : RADAR_VEHICLE_LIGHT;
}

void radar_eval_status(uint32_t speed_kph,
		       enum radar_vehicle_type type,
		       uint32_t warning_percent,
		       uint32_t light_limit_kph,
		       uint32_t heavy_limit_kph,
		       struct radar_classification *out)
{
	if (out == NULL) {
		return;
	}

	out->type = type;
	out->limit_kph = (type == RADAR_VEHICLE_HEAVY) ? heavy_limit_kph : light_limit_kph;
	out->warning_kph = (out->limit_kph * warning_percent) / 100U;
	out->speed_kph = speed_kph;

	if (speed_kph > out->limit_kph) {
		out->status = RADAR_STATUS_INFRACTION;
	} else if (speed_kph >= out->warning_kph) {
		out->status = RADAR_STATUS_WARNING;
	} else {
		out->status = RADAR_STATUS_NORMAL;
	}
}

static bool is_letter(char c)
{
	return (c >= 'A' && c <= 'Z');
}

static bool is_digit(char c)
{
	return (c >= '0' && c <= '9');
}

bool radar_plate_is_valid(const char *plate)
{
	if (plate == NULL) {
		return false;
	}

	/* Formato Mercosul: LLLDLDN (ex: ABC1D23) */
	if (strlen(plate) != 7U) {
		return false;
	}

	if (!is_letter(plate[0]) || !is_letter(plate[1]) || !is_letter(plate[2])) {
		return false;
	}

	if (!is_digit(plate[3])) {
		return false;
	}

	if (!is_letter(plate[4])) {
		return false;
	}

	if (!is_digit(plate[5]) || !is_digit(plate[6])) {
		return false;
	}

	return true;
}
