/*
 * Copyright (C) 2022 Jan Hamal Dvořák <mordae@anilinux.org>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include "renc.h"

#include <stdio.h>
#include <stdint.h>


/* How many microseconds to wait in order to resolve possible bounce. */
#if !defined(RENC_DEBOUNCE_US)
# define RENC_DEBOUNCE_US 1000
#endif

/* How many events can the queue hold. */
#if !defined(RENC_QUEUE_SIZE)
# define RENC_QUEUE_SIZE 16
#endif

/* Below how many milliseconds fixate the direction. */
#if !defined(RENC_FIX_DIR_MS)
# define RENC_FIX_DIR_MS 50
#endif

/* Under how many milliseconds discard completely. */
#if !defined(RENC_DISCARD_MS)
# define RENC_DISCARD_MS 5
#endif

/* Enable pull up on rotary encoder pins. */
#if !defined(RENC_PULL_UP)
# define RENC_PULL_UP 1
#endif


struct state {
	uint8_t cw_pin, ccw_pin, sw_pin, sens;
	uint32_t sw_mtime;
	uint32_t re_mtime;
	alarm_id_t sw_alarm;
	uint8_t state;
	bool sw : 1;
	int prev_direction : 3;
};


/* Encoder state machines. */
static struct state state[NUM_RENC];

/* Interrupt handler event queue. */
static queue_t queue;


/*
 * Alarm handler for `irq_handler_sw`.
 * See below for more information.
 */
static int64_t __no_inline_not_in_flash_func(debounce)(alarm_id_t id, void *arg)
{
	struct state *st = arg;

	/* Alarm has timed out and we got called. Clear the handle. */
	st->sw_alarm = -1;

	/* Read the current state. */
	bool sw = !gpio_get(st->sw_pin);

	/* Filter out repeated events as usual. */
	if (st->sw == sw)
		return 0;

	/*
	 * Update the state. We more-or-less trust this value, since human
	 * would not be able to repeat a button cycle this fast.
	 *
	 * If they are carefully holding the switch mid-press, they are going
	 * to be able to produce valid strings of random toggles though.
	 */
	st->sw = sw;

	/* Emit the event. */
	struct renc_event event = {
		.num = st - &state[0],
		.sw = sw,
	};
	(void)queue_try_add(&queue, &event);
	return 0;
}


__isr static void __no_inline_not_in_flash_func(irq_handler_sw)(void)
{
	const unsigned event_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

	for (int i = 0; i < NUM_RENC; i++) {
		struct state *st = &state[i];

		/* Skip unconfigured encoders. */
		if (st->cw_pin == st->ccw_pin)
			continue;

		unsigned events = gpio_get_irq_event_mask(st->sw_pin);
		if (events & event_mask) {
			/* Acknowledge interrupt. */
			gpio_acknowledge_irq(st->sw_pin, event_mask);

			/* Read current switch state. */
			bool sw = !gpio_get(st->sw_pin);

			/* Filter out repeated events. */
			if (st->sw == sw)
				continue;

			/*
			 * We might have set up an alarm to combat bounce.
			 * If we did, we need to cancel it since another
			 * interrupt (this one) occured sooner.
			 */
			if (st->sw_alarm >= 0) {
				cancel_alarm(st->sw_alarm);
				st->sw_alarm = -1;
			}

			/*
			 * If the switch state changes too fast, it might be
			 * a bounce. Humans are not that fast.
			 */
			uint32_t prev = st->sw_mtime;
			uint32_t now = time_us_32();

			if (now - prev < RENC_DEBOUNCE_US) {
				/* We are going to follow up on this change. */
				st->sw_alarm = add_alarm_in_us(RENC_DEBOUNCE_US, debounce, st, true);

				/* But we ignore it for now. */
				continue;
			}

			/* There was long enough delay, so we trust this value. */
			st->sw_mtime = now;
			st->sw = sw;

			/* Emit the event. */
			struct renc_event event = {
				.num = i,
				.sw  = st->sw,
			};
			(void)queue_try_add(&queue, &event);
		}
	}
}


__isr static void __no_inline_not_in_flash_func(irq_handler_re)(void)
{
	const unsigned event_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

	for (int i = 0; i < NUM_RENC; i++) {
		struct state *st = &state[i];

		/* Skip unconfigured encoders. */
		if (st->cw_pin == st->ccw_pin)
			continue;

		/* Acknowledge interrupts. */
		gpio_acknowledge_irq(st->cw_pin, event_mask);
		gpio_acknowledge_irq(st->ccw_pin, event_mask);

		/* Read the values anew. This is somewhat more
		 * reliable than counting the edges. */
		unsigned cw = !gpio_get(st->cw_pin);
		unsigned ccw = !gpio_get(st->ccw_pin);

		/* Shift in new encoder state. */
		uint8_t state0 = (st->state << 2) & 0b111100 | (ccw << 1) | cw;

		/*
		 * Only some transitions are valid. Namely those where new
		 * state is different from the old one and a not all bits
		 * are flipped at once.
		 *
		 * I have no idea how to calculate parity quickly on M0 CPU,
		 * so we just encode the truth table into 16 bits and look
		 * the transition up using bit shift.
		 */
		uint16_t valid = 0b0110100110010110;
		if (!((valid >> (state0 & 0b1111)) & 1))
			continue;

		/* Remember this new state. */
		st->state = state0;

		/* Finally, did we travel a whole step yet? */
		int direction = 0;

		/*
		 * 01-11-10 indicates clockwise step.
		 * 10-11-01 indicates a counterclockwise step.
		 */
		if (0b011110 == state0)
			direction = 1;
		else if (0b101101 == state0)
			direction = -1;

		/* If not, we are done here for now. */
		if (!direction)
			continue;

		/*
		 * Calculate how long it took since the last step.
		 */
		uint32_t prev = st->re_mtime;
		uint32_t now = time_us_32();
		st->re_mtime = now;

		/*
		 * If the steps follow shortly one after another, it means
		 * the human is turning the encoder quickly. Meaning we can
		 * generate some extra ticks for them to make it easier.
		 *
		 * We divide by 1024 instead of 1000 to get milliseconds
		 * because we do not need much precision here and proper
		 * division is costly.
		 */
		int delta = (now - prev) >> 10;

		/* Steps too close to each other are impossible for humans. */
		if (delta < RENC_DISCARD_MS)
			continue;

		int speed = 1;

		if (delta < st->sens) {
			speed = st->sens - delta;
			speed *= speed;
		}

		/*
		 * The trouble is that when the speed is too high, it is no
		 * longer possible to tell the direction correctly because
		 * some of the states get skipped. So let's just assume it
		 * stayed the same. It would be too hard for a human to
		 * reverse the direction of rotation that fast.
		 */
		if (delta < RENC_FIX_DIR_MS)
			direction = st->prev_direction;

		st->prev_direction = direction;

		struct renc_event event = {
			.num = i,
			.steps = direction * speed,
		};
		(void)queue_try_add(&queue, &event);
	}
}


void renc_init(void)
{
	queue_init(&queue, sizeof(struct renc_event), RENC_QUEUE_SIZE);
	alarm_pool_init_default();
	irq_set_enabled(IO_IRQ_BANK0, true);
}


void renc_config(unsigned num, uint8_t cw_pin, uint8_t ccw_pin, uint8_t sw_pin, uint8_t sens)
{
	if (state[num].cw_pin != state[num].ccw_pin)
		panic("renc_config: encoder num=%u already configured", num);

	if (state[num].cw_pin != 0)
		panic("renc_config: encoder num=%u already configured", num);

	state[num].cw_pin = cw_pin;
	state[num].ccw_pin = ccw_pin;
	state[num].sw_pin = sw_pin;
	state[num].sens = sens;

	gpio_init(cw_pin);
	gpio_set_dir(cw_pin, false);
#if RENC_PULL_UP
	gpio_pull_up(cw_pin);
#else
	gpio_disable_pulls(cw_pin);
#endif

	gpio_init(ccw_pin);
	gpio_set_dir(ccw_pin, false);
#if RENC_PULL_UP
	gpio_pull_up(ccw_pin);
#else
	gpio_disable_pulls(ccw_pin);
#endif

	gpio_init(sw_pin);
	gpio_set_dir(sw_pin, false);
#if RENC_PULL_UP
	gpio_pull_up(sw_pin);
#else
	gpio_disable_pulls(sw_pin);
#endif

	gpio_add_raw_irq_handler(sw_pin, irq_handler_sw);
	gpio_add_raw_irq_handler_masked((1 << cw_pin) | (1 << ccw_pin), irq_handler_re);

	unsigned event_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
	gpio_set_irq_enabled(sw_pin, event_mask, true);
	gpio_set_irq_enabled(cw_pin, event_mask, true);
	gpio_set_irq_enabled(ccw_pin, event_mask, true);

	printf("renc: Configured: num=%i, sw=%i, cw=%i, ccw=%i, sens=%i\n",
		num, sw_pin, cw_pin, ccw_pin, sens);
}


void renc_read_blocking(struct renc_event *event)
{
	(void)queue_remove_blocking(&queue, event);
}
