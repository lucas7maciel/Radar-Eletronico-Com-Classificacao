/*
 * Radar eletronico (Zephyr RTOS) - pipeline completa com sensores simulados,
 * classificacao, display colorido e camera simulada via ZBUS.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/zbus/zbus.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "radar.h"

LOG_MODULE_REGISTER(radar, LOG_LEVEL_INF);

#define SENSOR_QUEUE_LEN 8
#define DISPLAY_QUEUE_LEN 8

#define SENSOR_AXLE_GAP_MS 50
#define SENSOR_CAR_GAP_MS 700
#define CAMERA_PROCESS_MIN_MS 120
#define CAMERA_PROCESS_MAX_MS 320

#define ANSI_RESET "\x1B[0m"
#define ANSI_GREEN "\x1B[32m"
#define ANSI_BLUE "\x1B[34m"
#define ANSI_MAGENTA "\x1B[35m"
#define ANSI_CYAN "\x1B[36m"
#define ANSI_YELLOW "\x1B[33m"
#define ANSI_RED "\x1B[31m"

#define PLATE_MAX_LEN 8

struct sensor_event {
	uint32_t id;
	uint32_t timestamp_ms;
	uint32_t delta_ms;
	uint8_t axles;
};

struct radar_display_msg {
	uint32_t id;
	struct radar_classification cls;
	bool plate_valid;
	char plate[PLATE_MAX_LEN];
};

struct camera_request {
	uint32_t sample_id;
	enum radar_vehicle_type type;
	uint32_t speed_kph;
	uint32_t limit_kph;
};

struct camera_response {
	uint32_t sample_id;
	bool success;
	bool plate_valid;
	char plate[PLATE_MAX_LEN];
};

K_MSGQ_DEFINE(sensor_queue, sizeof(struct sensor_event), SENSOR_QUEUE_LEN, 4);
K_MSGQ_DEFINE(display_queue, sizeof(struct radar_display_msg), DISPLAY_QUEUE_LEN, 4);

struct camera_ctx_entry {
	bool in_use;
	uint32_t id;
	struct radar_display_msg msg;
};

#define CAMERA_CTX_MAX 8
static struct camera_ctx_entry camera_ctx[CAMERA_CTX_MAX];
static K_MUTEX_DEFINE(camera_ctx_lock);

ZBUS_SUBSCRIBER_DEFINE(camera_sub, 4);
ZBUS_SUBSCRIBER_DEFINE(camera_resp_sub, 4);

ZBUS_CHAN_DEFINE(camera_request_chan,
		 struct camera_request,
		 NULL, NULL,
		 ZBUS_OBSERVERS(camera_sub),
		 ZBUS_MSG_INIT(.sample_id = 0));

ZBUS_CHAN_DEFINE(camera_response_chan,
		 struct camera_response,
		 NULL, NULL,
		 ZBUS_OBSERVERS(camera_resp_sub),
		 ZBUS_MSG_INIT(.sample_id = 0));

static struct {
	bool active;
	uint8_t axles;
	uint32_t start_ms;
} sensor_state;

static struct k_spinlock sensor_lock;
static atomic_t sample_counter = ATOMIC_INIT(0);

static uint32_t rand_range(uint32_t min, uint32_t max)
{
	uint32_t span = max - min + 1U;
	return min + (sys_rand32_get() % span);
}

static uint32_t calc_delta_for_speed(uint32_t distance_mm, uint32_t speed_kph)
{
	if (speed_kph == 0U) {
		return 1000U;
	}

	/* delta_ms = (dist_mm * 3.6) / speed_kph */
	uint64_t num = (uint64_t)distance_mm * 36ULL;
	uint64_t den = (uint64_t)speed_kph * 10ULL;

	return (uint32_t)MAX(1ULL, (num + den - 1ULL) / den);
}

static void sensor_reset(void)
{
	sensor_state.active = false;
	sensor_state.axles = 0U;
	sensor_state.start_ms = 0U;
}

static void sensor_pulse_a(void)
{
	k_spinlock_key_t key = k_spin_lock(&sensor_lock);
	uint32_t now = k_uptime_get_32();

	if (!sensor_state.active) {
		sensor_state.active = true;
		sensor_state.start_ms = now;
		sensor_state.axles = 0U;
	}

	sensor_state.axles++;
	k_spin_unlock(&sensor_lock, key);
}

static void sensor_pulse_b(void)
{
	k_spinlock_key_t key = k_spin_lock(&sensor_lock);

	if (!sensor_state.active) {
		k_spin_unlock(&sensor_lock, key);
		return;
	}

	uint32_t now = k_uptime_get_32();
	uint32_t delta_ms = now - sensor_state.start_ms;
	uint8_t axles = sensor_state.axles;
	sensor_reset();
	k_spin_unlock(&sensor_lock, key);

	struct sensor_event evt = {
		.id = (uint32_t)atomic_inc(&sample_counter),
		.timestamp_ms = now,
		.delta_ms = delta_ms,
		.axles = axles,
	};

	if (k_msgq_put(&sensor_queue, &evt, K_NO_WAIT) != 0) {
		LOG_WRN("Fila de sensores cheia; amostra %u descartada", evt.id);
	}
}

static void render_display(const struct radar_display_msg *msg)
{
	const char *color = ANSI_GREEN;
	const char *status = "OK";

	switch (msg->cls.status) {
	case RADAR_STATUS_INFRACTION:
		color = ANSI_RED;
		status = "Infra";
		break;
	case RADAR_STATUS_WARNING:
		color = ANSI_YELLOW;
		status = "Alerta";
		break;
	case RADAR_STATUS_NORMAL:
	default:
		break;
	}

	const char *plate = (msg->plate[0] != '\0') ? msg->plate : "--";
	const char *plate_state = "Captura pendente";

	if (msg->plate[0] != '\0') {
		plate_state = msg->plate_valid ? "Placa ok" : "Placa inválida";
	}

	printk("%s[Display]%s Id: %s%u%s | Tipo: %s%s%s | "
	       "Vel: %s%u%s | Km/h (Lim: %s%u%s Alerta: %s%u%s) "
	       "Estado:%s%s%s | Placa:%s%s%s (%s)%s\n",
	       ANSI_YELLOW, ANSI_RESET,
	       ANSI_CYAN, msg->id, ANSI_RESET,
	       ANSI_MAGENTA, radar_vehicle_name(msg->cls.type), ANSI_RESET,
	       ANSI_BLUE, msg->cls.speed_kph, ANSI_RESET,
	       ANSI_YELLOW, msg->cls.limit_kph, ANSI_RESET,
	       ANSI_YELLOW, msg->cls.warning_kph, ANSI_RESET,
	       color, status, ANSI_RESET,
	       ANSI_GREEN, plate, ANSI_RESET,
	       plate_state, ANSI_RESET);
}

static void camera_ctx_store(const struct radar_display_msg *msg)
{
	if (msg == NULL) {
		return;
	}

	k_mutex_lock(&camera_ctx_lock, K_FOREVER);
	int slot = -1;

	for (int i = 0; i < CAMERA_CTX_MAX; i++) {
		if (!camera_ctx[i].in_use) {
			slot = i;
			break;
		}
	}

	if (slot < 0) {
		slot = 0;
	}

	camera_ctx[slot].in_use = true;
	camera_ctx[slot].id = msg->id;
	camera_ctx[slot].msg = *msg;
	k_mutex_unlock(&camera_ctx_lock);
}

static bool camera_ctx_take(uint32_t id, struct radar_display_msg *out)
{
	bool found = false;

	k_mutex_lock(&camera_ctx_lock, K_FOREVER);
	for (int i = 0; i < CAMERA_CTX_MAX; i++) {
		if (camera_ctx[i].in_use && camera_ctx[i].id == id) {
			if (out != NULL) {
				*out = camera_ctx[i].msg;
			}
			camera_ctx[i].in_use = false;
			found = true;
			break;
		}
	}
	k_mutex_unlock(&camera_ctx_lock);

	return found;
}

static void control_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct sensor_event evt;

		k_msgq_get(&sensor_queue, &evt, K_FOREVER);

		enum radar_vehicle_type type = radar_classify_vehicle(evt.axles);
		uint32_t speed_kph = radar_calc_speed_kph(CONFIG_RADAR_SENSOR_DISTANCE_MM,
							  evt.delta_ms);

		struct radar_classification cls = { 0 };
		radar_eval_status(speed_kph, type,
				  CONFIG_RADAR_WARNING_THRESHOLD_PERCENT,
				  CONFIG_RADAR_SPEED_LIMIT_LIGHT_KMH,
				  CONFIG_RADAR_SPEED_LIMIT_HEAVY_KMH,
				  &cls);

		struct radar_display_msg display_msg = {
			.id = evt.id,
			.cls = cls,
			.plate_valid = false,
		};
		display_msg.plate[0] = '\0';

		if (k_msgq_put(&display_queue, &display_msg, K_NO_WAIT) != 0) {
			LOG_WRN("Fila de display cheia; amostra %u descartada", evt.id);
		}

		if (cls.status == RADAR_STATUS_INFRACTION) {
			struct camera_request req = {
				.sample_id = evt.id,
				.type = cls.type,
				.speed_kph = cls.speed_kph,
				.limit_kph = cls.limit_kph,
			};

			camera_ctx_store(&display_msg);

			int err = zbus_chan_pub(&camera_request_chan, &req, K_MSEC(50));
			if (err != 0) {
				LOG_WRN("Nao foi possivel publicar trigger da camera (%d)", err);
			}
		}
	}
}

static char random_letter(void)
{
	return (char)('A' + (sys_rand32_get() % 26U));
}

static char random_digit(void)
{
	return (char)('0' + (sys_rand32_get() % 10U));
}

static void make_plate(char *out, bool force_invalid)
{
	if (out == NULL) {
		return;
	}

	if (force_invalid) {
		snprintf(out, PLATE_MAX_LEN, "XX%04u", sys_rand32_get() % 10000U);
		return;
	}

	out[0] = random_letter();
	out[1] = random_letter();
	out[2] = random_letter();
	out[3] = random_digit();
	out[4] = random_letter();
	out[5] = random_digit();
	out[6] = random_digit();
	out[7] = '\0';
}

static void camera_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct zbus_channel *chan = NULL;
	struct camera_request req;

	while (true) {
		zbus_sub_wait(&camera_sub, &chan, K_FOREVER);
		zbus_chan_read(chan, &req, K_FOREVER);

		uint32_t process_ms = rand_range(CAMERA_PROCESS_MIN_MS, CAMERA_PROCESS_MAX_MS);
		k_msleep(process_ms);

		bool fail = (rand_range(0, 99) < CONFIG_RADAR_CAMERA_FAILURE_RATE_PERCENT);
		struct camera_response resp = {
			.sample_id = req.sample_id,
			.success = !fail,
			.plate_valid = false,
		};

		make_plate(resp.plate, fail);
		resp.plate_valid = radar_plate_is_valid(resp.plate);

		int err = zbus_chan_pub(&camera_response_chan, &resp, K_MSEC(50));
		if (err != 0) {
			LOG_WRN("Falha ao publicar resposta da camera (%d)", err);
		}
	}
}

static void camera_listener_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	const struct zbus_channel *chan = NULL;
	struct camera_response resp;

	while (true) {
		zbus_sub_wait(&camera_resp_sub, &chan, K_FOREVER);
		zbus_chan_read(chan, &resp, K_FOREVER);

		if (!resp.success) {
			LOG_WRN("[Camera] id:%u falha de captura (simulada)", resp.sample_id);

			struct radar_display_msg failed_msg;
			if (camera_ctx_take(resp.sample_id, &failed_msg)) {
				snprintf(failed_msg.plate, sizeof(failed_msg.plate), "FALHA");
				failed_msg.plate_valid = false;
				k_msgq_put(&display_queue, &failed_msg, K_NO_WAIT);
			}
			continue;
		}

		const char *valid = resp.plate_valid ? "valida" : "invalida";
		LOG_INF("[Camera] id:%u placa:%s (%s)", resp.sample_id, resp.plate, valid);

		struct radar_display_msg msg;
		if (camera_ctx_take(resp.sample_id, &msg)) {
			snprintf(msg.plate, sizeof(msg.plate), "%s", resp.plate);
			msg.plate_valid = resp.plate_valid;

			if (k_msgq_put(&display_queue, &msg, K_NO_WAIT) != 0) {
				LOG_WRN("Fila de display cheia; placa %u perdida", resp.sample_id);
			}
		}
	}
}

static void display_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		struct radar_display_msg msg;

		k_msgq_get(&display_queue, &msg, K_FOREVER);
		render_display(&msg);
	}
}

static void simulate_vehicle(uint32_t distance_mm)
{
	bool heavy = (rand_range(0, 99) < 35);
	uint8_t axles = heavy ? (uint8_t)rand_range(3, 4) : 2U;
	uint32_t limit = heavy ? CONFIG_RADAR_SPEED_LIMIT_HEAVY_KMH : CONFIG_RADAR_SPEED_LIMIT_LIGHT_KMH;
	uint32_t speed = rand_range((limit * 70U) / 100U, (limit * 130U) / 100U);
	uint32_t delta_ms = calc_delta_for_speed(distance_mm, speed);

	for (uint8_t i = 0; i < axles; i++) {
		sensor_pulse_a();
		k_msleep(SENSOR_AXLE_GAP_MS + rand_range(0, 10));
	}

	k_msleep(delta_ms);
	sensor_pulse_b();
}

static void sensor_sim_thread(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (true) {
		simulate_vehicle(CONFIG_RADAR_SENSOR_DISTANCE_MM);
		k_msleep(SENSOR_CAR_GAP_MS + rand_range(0, 400));
	}
}

K_THREAD_DEFINE(sensor_sim_tid, 1024, sensor_sim_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(control_tid, 1024, control_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(display_tid, 1024, display_thread, NULL, NULL, NULL, 7, 0, 0);
K_THREAD_DEFINE(camera_tid, 1024, camera_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(camera_listener_tid, 1024, camera_listener_thread, NULL, NULL, NULL, 5, 0, 0);
