#include <zephyr/ztest.h>

#include "radar.h"

ZTEST(radar_unit, test_speed_calc)
{
	/* 4 m in 400 ms => 36 km/h */
	uint32_t speed = radar_calc_speed_kph(4000U, 400U);
	zassert_equal(speed, 36U, "speed=%u", speed);
}

ZTEST(radar_unit, test_classify_vehicle)
{
	zassert_equal(radar_classify_vehicle(2U), RADAR_VEHICLE_LIGHT);
	zassert_equal(radar_classify_vehicle(3U), RADAR_VEHICLE_HEAVY);
	zassert_equal(radar_classify_vehicle(5U), RADAR_VEHICLE_HEAVY);
}

ZTEST(radar_unit, test_plate_validation)
{
	zassert_true(radar_plate_is_valid("ABC1D23"));
	zassert_true(radar_plate_is_valid("XYZ9A00"));

	zassert_false(radar_plate_is_valid("ABC1234"));   /* formato antigo */
	zassert_false(radar_plate_is_valid("AB1CD23"));   /* letras/digitos trocados */
	zassert_false(radar_plate_is_valid("ABC1D2"));    /* tamanho incorreto */
}

ZTEST_SUITE(radar_unit, NULL, NULL, NULL, NULL, NULL);
