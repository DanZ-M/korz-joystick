/*
 * korz-joystick: analog joystick -> Zephyr input events (mouse or scroll) + button.
 * ZMK's `zmk,input-listener` turns the events into HID mouse reports.
 */
#define DT_DRV_COMPAT korz_joystick

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(korz_joy, LOG_LEVEL_INF);

#if DT_HAS_COMPAT_STATUS_OKAY(korz_joystick)

#define CAL_SAMPLES 25 /* readings averaged at start to find the centre (stick at rest) */

struct joy_cfg {
	struct adc_dt_spec axis[2];
	struct gpio_dt_spec btn;
	bool has_btn;
	bool invert[2];
	bool swap;
};

struct joy_data {
	const struct device *dev;
	struct k_work_delayable work;
	int32_t sum[2];
	int32_t center[2];
	uint16_t cal_n;
	int32_t acc[2]; /* fractional movement, thousandths of a step */
	bool btn_state;
	bool btn_last_raw;
	uint16_t dbg_n;
};

static int read_axis(const struct adc_dt_spec *spec, int32_t *out)
{
	int16_t raw = 0;
	struct adc_sequence seq = {.buffer = &raw, .buffer_size = sizeof(raw)};
	int err = adc_sequence_init_dt(spec, &seq);

	if (err) {
		return err;
	}
	err = adc_read_dt(spec, &seq);
	if (err) {
		return err;
	}
	*out = raw;
	return 0;
}

/* deflection d (counts from the centre) -> 0..1000 after the dead zone */
static int32_t deflection(int32_t d)
{
	const int32_t range = CONFIG_KORZ_JOY_RANGE;
	const int32_t dz = range * CONFIG_KORZ_JOY_DEADZONE / 100;
	int32_t a = abs(d);

	if (a <= dz) {
		return 0;
	}
	a = (a - dz) * 1000 / (range - dz);
	return a > 1000 ? 1000 : a;
}

/* signed whole steps to send now; keeps the fraction in *acc */
static int32_t steps(int32_t n, bool neg, int32_t *acc)
{
	if (n == 0) {
		*acc = 0;
		return 0;
	}
	int32_t n2 = n * n / 1000; /* gentle at small deflection, full speed at the end */
	int32_t milli = n2 * CONFIG_KORZ_JOY_SPEED * CONFIG_KORZ_JOY_POLL_MS / 1000;

	*acc += neg ? -milli : milli;
	int32_t out = *acc / 1000;

	*acc -= out * 1000;
	return out;
}

static void joy_tick(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct joy_data *data = CONTAINER_OF(dwork, struct joy_data, work);
	const struct device *dev = data->dev;
	const struct joy_cfg *cfg = dev->config;
	int32_t raw[2];

	k_work_reschedule(&data->work, K_MSEC(CONFIG_KORZ_JOY_POLL_MS));

	for (int i = 0; i < 2; i++) {
		if (read_axis(&cfg->axis[i], &raw[i]) != 0) {
			return;
		}
	}

	if (data->cal_n < CAL_SAMPLES) {
		data->sum[0] += raw[0];
		data->sum[1] += raw[1];
		if (++data->cal_n == CAL_SAMPLES) {
			data->center[0] = data->sum[0] / CAL_SAMPLES;
			data->center[1] = data->sum[1] / CAL_SAMPLES;
			LOG_INF("joystick centre: x=%d y=%d", data->center[0], data->center[1]);
		}
		return;
	}

#ifdef CONFIG_KORZ_JOY_DEBUG
	if (++data->dbg_n >= 500 / CONFIG_KORZ_JOY_POLL_MS) {
		data->dbg_n = 0;
		LOG_INF("joystick raw x=%d y=%d (centre %d,%d)", raw[0], raw[1], data->center[0],
			data->center[1]);
	}
#endif

	int32_t d[2] = {raw[0] - data->center[0], raw[1] - data->center[1]};
	int32_t n[2], out[2];
	bool neg[2];

	if (cfg->swap) {
		int32_t t = d[0];

		d[0] = d[1];
		d[1] = t;
	}
	for (int i = 0; i < 2; i++) {
		n[i] = deflection(d[i]);
		neg[i] = (d[i] < 0) != cfg->invert[i];
		out[i] = steps(n[i], neg[i], &data->acc[i]);
	}

#ifdef CONFIG_KORZ_JOY_MODE_MOUSE
	const uint16_t code_x = INPUT_REL_X, code_y = INPUT_REL_Y;
	const int32_t sx = out[0], sy = out[1];
#else
	/* up on the stick = wheel up (positive), a stick pushed to the right scrolls right */
	const uint16_t code_x = INPUT_REL_HWHEEL, code_y = INPUT_REL_WHEEL;
	const int32_t sx = out[0], sy = -out[1];
#endif
	if (sx != 0) {
		input_report_rel(dev, code_x, sx, sy == 0, K_FOREVER);
	}
	if (sy != 0) {
		input_report_rel(dev, code_y, sy, true, K_FOREVER);
	}

	if (cfg->has_btn) {
		int v = gpio_pin_get_dt(&cfg->btn);
		bool now = v > 0;

		/* two equal readings in a row = debounced */
		if (now == data->btn_last_raw && now != data->btn_state) {
			data->btn_state = now;
			input_report_key(dev, INPUT_BTN_0, now ? 1 : 0, true, K_FOREVER);
		}
		data->btn_last_raw = now;
	}
}

static int joy_init(const struct device *dev)
{
	const struct joy_cfg *cfg = dev->config;
	struct joy_data *data = dev->data;

	data->dev = dev;
	for (int i = 0; i < 2; i++) {
		if (!adc_is_ready_dt(&cfg->axis[i])) {
			LOG_ERR("ADC not ready");
			return -ENODEV;
		}
		int err = adc_channel_setup_dt(&cfg->axis[i]);

		if (err) {
			LOG_ERR("ADC channel %d setup failed (%d)", i, err);
			return err;
		}
	}
	if (cfg->has_btn) {
		if (!gpio_is_ready_dt(&cfg->btn) ||
		    gpio_pin_configure_dt(&cfg->btn, GPIO_INPUT) != 0) {
			LOG_ERR("button GPIO failed");
			return -ENODEV;
		}
	}
	k_work_init_delayable(&data->work, joy_tick);
	/* wait a second so the stick is at rest when the centre is measured */
	k_work_schedule(&data->work, K_MSEC(1000));
	return 0;
}

#define JOY_INIT(n)                                                                                \
	static struct joy_data joy_data_##n;                                                       \
	static const struct joy_cfg joy_cfg_##n = {                                                \
		.axis = {ADC_DT_SPEC_INST_GET_BY_IDX(n, 0), ADC_DT_SPEC_INST_GET_BY_IDX(n, 1)},    \
		.btn = GPIO_DT_SPEC_INST_GET_OR(n, button_gpios, {0}),                             \
		.has_btn = DT_INST_NODE_HAS_PROP(n, button_gpios),                                 \
		.invert = {DT_INST_PROP(n, invert_x), DT_INST_PROP(n, invert_y)},                  \
		.swap = DT_INST_PROP(n, swap_xy),                                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, joy_init, NULL, &joy_data_##n, &joy_cfg_##n, POST_KERNEL,         \
			      CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(JOY_INIT)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(korz_joystick) */
