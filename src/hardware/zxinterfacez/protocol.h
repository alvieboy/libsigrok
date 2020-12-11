/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2013 Bert Vermeulen <bert@biot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_ZXINTERFACEZ_PROTOCOL_H
#define LIBSIGROK_HARDWARE_ZXINTERFACEZ_PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "hdlc_decoder.h"

#define LOG_PREFIX "zxinterfacez"

#define NUM_TRIGGER_STAGES         1
#define CLOCK_RATE                 SR_MHZ(98)
#define MIN_NUM_SAMPLES            4
#define DEFAULT_SAMPLERATE         SR_KHZ(24)

#define UNUSED(x) ((void)x)

#define CMD_GET_VERSION (0x01)
#define CMD_SET_TRIGGER (0x02)
#define CMD_START_CAPTURE (0x03)
#define CMD_GET_STATUS (0x04)
#define CMD_GET_SAMPLES (0x05)
#define REPLY(cmd) ((cmd)|0x80)


enum zxinterfacez_state {
        ZXI_IDLE,
        ZXI_WAIT_SETTINGS,
        ZXI_WAIT_START,
        ZXI_POLL,
        ZXI_WAIT_DATA

};


struct dev_context {
        uint32_t dummy;
        uint32_t cur_samplerate;
        struct sr_channel_group *nontrig_group;
        struct sr_channel_group *trig_group;
        uint32_t trigger_mask;
        uint32_t trigger_value;
        uint32_t trigger_edge;
        enum zxinterfacez_state state;
        //guint poll_timer_id;
       // guint cmd_timer_id;
        uint8_t cmd;
        int expected_len;
        uint8_t  fetch_ram;
        uint8_t  fetch_seq;
        uint16_t fetch_offset;

        hdlc_decoder_t async_hdlc_decoder;
        uint8_t hdlc_decoder_buf[256+4];
        uint8_t samples[2][4096];
        uint8_t converted_samples[ 1024 * 8 ];
        uint32_t trig_address; // Sample where we triggered.
};

struct dev_context *zxinterfacez_dev_new(const struct sr_dev_inst *sdi);

void zxinterfacez_setup_comms(struct sr_serial_dev_inst *serial, uint8_t *buf, unsigned size);

int zxinterfacez_convert_trigger(const struct sr_dev_inst *sdi);

const uint8_t *zxinterfacez_get_last_reply(void);

unsigned zxinterfacez_get_last_reply_size(void);

int zxinterfacez_receive_data(int fd, int revents, void *cb_data);

int zxinterfacez_setup_and_start(const struct sr_dev_inst *sdi);

int send_receive_cmd(struct sr_serial_dev_inst *serial,
                     const uint8_t cmd,
                     const uint8_t *tx,
                     unsigned len,
                     int expected_len,
                     uint8_t *rx);

int send_receive_cmd_async(const struct sr_dev_inst *sdi,
                           const uint8_t cmd,
                           const uint8_t *tx,
                           unsigned len,
                           int expected_len);

int zxinterfacez_stop(const struct sr_dev_inst *sdi);
int zxinterfacez_nontrig_size(void);
int zxinterfacez_trig_size(void);



#endif
