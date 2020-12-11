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

#include <config.h>
#include "protocol.h"

#include "hdlc_encoder.h"
#include "hdlc_decoder.h"


#define SOURCE_KEY_POLL (-2)

#ifdef ENABLE_RETRANSMIT
#define SOURCE_KEY_RETRANSMIT (-3)
#endif

static hdlc_encoder_t hdlc_encoder;
static hdlc_decoder_t hdlc_decoder;
static const uint8_t *last_reply;
static unsigned last_reply_size;

SR_PRIV void zx_hdlc_writer(void*pvt,const uint8_t v);
SR_PRIV void zx_hdlc_flusher(void*pvt);
SR_PRIV void zx_hdlc_data_ready(void*,const uint8_t *buf, unsigned size);
SR_PRIV int zxinterfacez_receive_data(int fd, int revents, void *cb_data);

static int zxinterfacez_trigger_set(const struct sr_dev_inst *sdi);
static int zxinterfacez_capture_started(const struct sr_dev_inst *sdi);
static int zxinterfacez_check_status(const struct sr_dev_inst *sdi, const uint8_t *reply);
static int zxinterfacez_got_sample_data(const struct sr_dev_inst *sdi, const uint8_t *data);
static int zxinterfacez_samples_ready(const struct sr_dev_inst *sdi);
static int zxinterfacez_fetch_samples(const struct sr_dev_inst *sdi);
static int zxinterfacez_timer_elapsed(int fd, int revents, void *user_data);



static void zxinterfacez_enter_state(struct dev_context *devc, enum zxinterfacez_state newstate)
{
        sr_dbg("Entering state %d", newstate);
        devc->state = newstate;
}

static void zx_async_hdlc_data_ready(void*user,const uint8_t *buf, unsigned size)
{
        const struct sr_dev_inst *sdi = user;
        struct dev_context *devc = sdi->priv;
        /* */
#ifdef ENABLE_RETRANSMIT
	sr_session_source_remove(sdi->session, SOURCE_KEY_RETRANSMIT);
#endif
        switch (devc->state) {
        case ZXI_IDLE:

                sr_warn("Unexpected HDLC data len %d in IDLE state", size);
                break;
        case ZXI_WAIT_SETTINGS:
                if (size!=1) {
                        sr_warn("Unexpected HDLC data len %d in WAIT_SETTINGS state", size);
                        break;
                }
                if (buf[0]!=REPLY(CMD_SET_TRIGGER)) {
                        sr_warn("Unexpected HDLC response %02x in WAIT_SETTINGS state", buf[0]);
                        break;
                }
                zxinterfacez_enter_state(devc,ZXI_IDLE);
                zxinterfacez_trigger_set(sdi);
                break;
        case ZXI_WAIT_START:
                if (size!=1) {
                        sr_warn("Unexpected HDLC data len %d in WAIT_START state", size);
                        break;
                }
                if (buf[0]!=REPLY(CMD_START_CAPTURE)) {
                        sr_warn("Unexpected HDLC response %02x in WAIT_START state", buf[0]);
                        break;
                }
                zxinterfacez_enter_state(devc,ZXI_IDLE);
                zxinterfacez_capture_started(sdi);
                break;
        case ZXI_POLL:
                if (size!=13) {
                        sr_warn("Unexpected HDLC data len %d in POLL state", size);
                        break;
                }
                if (buf[0]!=REPLY(CMD_GET_STATUS)) {
                        sr_warn("Unexpected HDLC response %02x in POLL state", buf[0]);
                        break;
                }
                zxinterfacez_enter_state(devc,ZXI_IDLE);
                zxinterfacez_check_status(sdi, &buf[1]);
                break;
        case ZXI_WAIT_DATA:
                if (size!=258) {
                        sr_warn("Unexpected HDLC data len %d in DATA state", size);
                        break;
                }
                if (buf[0]!=REPLY(CMD_GET_SAMPLES)) {
                        sr_warn("Unexpected HDLC response %02x in DATA state", buf[0]);
                        break;
                }
                zxinterfacez_enter_state(devc,ZXI_IDLE);
                zxinterfacez_got_sample_data(sdi, &buf[1]);

                break;
        }
}


SR_PRIV struct dev_context *zxinterfacez_dev_new(const struct sr_dev_inst *sdi)
{
        struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));

        devc->cur_samplerate = DEFAULT_SAMPLERATE;
        zxinterfacez_enter_state(devc,ZXI_IDLE);

        hdlc_decoder__init(&devc->async_hdlc_decoder,
                           devc->hdlc_decoder_buf,
                           sizeof(devc->hdlc_decoder_buf),
                           &zx_async_hdlc_data_ready,
                           (void*)sdi);
	return devc;
}


SR_PRIV int zxinterfacez_convert_trigger(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc;
	struct sr_trigger *trigger;
	struct sr_trigger_stage *stage;
	struct sr_trigger_match *match;
	const GSList *l, *m;
	devc = sdi->priv;

        devc->trigger_mask = 0;
        devc->trigger_value = 0;
        devc->trigger_edge = 0;

	if (!(trigger = sr_session_trigger_get(sdi->session)))
		return SR_OK;

        if (g_slist_length(trigger->stages) > 1) {
                sr_err("This device only supports one trigger stages.");
		return SR_ERR;
	}

        l = trigger->stages;

        stage = l->data;

        for (m = stage->matches; m; m = m->next) {
                match = m->data;
                if (!match->channel->enabled)
                        /* Ignore disabled channels with a trigger. */
                        continue;

                int index = match->channel->index;


                if (index<zxinterfacez_nontrig_size())
                        continue; // Not triggerable
                index -= zxinterfacez_nontrig_size();

                uint32_t mask = (1<<index);

                devc->trigger_mask |= mask;

                switch (match->match) {
                case SR_TRIGGER_ONE:
                        devc->trigger_value |= mask;
                        break;
                case SR_TRIGGER_FALLING:
                        devc->trigger_edge |= mask;
                        break;
                case SR_TRIGGER_RISING:
                        devc->trigger_value |= mask;
                        devc->trigger_edge |= mask;
                        break;
                case SR_TRIGGER_ZERO:
                        break;
                }
        }

        sr_info("Trigger settings: ");
        sr_info(" Mask: %08x", devc->trigger_mask);
        sr_info(" Val : %08x", devc->trigger_value);
        sr_info(" Edge: %08x", devc->trigger_edge);


	return SR_OK;
}

// TBD: some buffering

SR_PRIV void zx_hdlc_writer(void*pvt,const uint8_t v)
{
    struct sr_serial_dev_inst *serial =
        (struct sr_serial_dev_inst *)pvt;

    serial_write_blocking(serial, &v, 1, 1000);
}

SR_PRIV void zx_hdlc_flusher(void*pvt)
{
    struct sr_serial_dev_inst *serial =
        (struct sr_serial_dev_inst *)pvt;

    serial_drain(serial);
}

SR_PRIV void zx_hdlc_data_ready(void *user, const uint8_t *buf, unsigned size)
{
        UNUSED(user);
/*        const struct sr_dev_inst *sdi = user;
        struct dev_context *devc = sdi->priv;

        if (size<1) {
                // Error
                return;
        }

        if (devc->cmd_timer_id>0)
                g_source_remove(devc->cmd_timer_id);


                //UNUSED(user);
                */
        last_reply = buf;
        last_reply_size = size;
}


SR_PRIV const uint8_t *zxinterfacez_get_last_reply(void)
{
        return last_reply;
}

SR_PRIV unsigned zxinterfacez_get_last_reply_size(void)
{
        return last_reply_size;
}

SR_PRIV void zxinterfacez_setup_comms(struct sr_serial_dev_inst *serial, uint8_t *buf, unsigned size)
{
    hdlc_decoder__init(&hdlc_decoder,
                       buf,
                       size,
                       &zx_hdlc_data_ready,
                       serial);

    hdlc_encoder__init(&hdlc_encoder,
                       &zx_hdlc_writer,
                       &zx_hdlc_flusher,
                       serial);
}

static uint8_t zxinterfacez_get_divider(struct dev_context *devc)
{
        // 96000000
        return (96000000/(devc->cur_samplerate))-1;
}

SR_PRIV int zxinterfacez_setup_and_start(const struct sr_dev_inst *sdi)
{
 //       uint8_t rxbuf[16];
        uint8_t txbuf[16];

        struct sr_serial_dev_inst *serial = sdi->conn;
        struct dev_context *devc = sdi->priv;

        zxinterfacez_setup_comms(serial,
                                 devc->hdlc_decoder_buf,
                                 sizeof(devc->hdlc_decoder_buf)
                                );
        int index=0;
        txbuf[index++] = (devc->trigger_mask>>0) & 0xff;
        txbuf[index++] = (devc->trigger_mask>>8) & 0xff;
        txbuf[index++] = (devc->trigger_mask>>16) & 0xff;
        txbuf[index++] = (devc->trigger_mask>>24) & 0xff;
        txbuf[index++] = (devc->trigger_value>>0) & 0xff;
        txbuf[index++] = (devc->trigger_value>>8) & 0xff;
        txbuf[index++] = (devc->trigger_value>>16) & 0xff;
        txbuf[index++] = (devc->trigger_value>>24) & 0xff;
        txbuf[index++] = (devc->trigger_edge>>0) & 0xff;
        txbuf[index++] = (devc->trigger_edge>>8) & 0xff;
        txbuf[index++] = (devc->trigger_edge>>16) & 0xff;
        txbuf[index++] = (devc->trigger_edge>>24) & 0xff;

        zxinterfacez_enter_state(devc,ZXI_WAIT_SETTINGS);
        if (send_receive_cmd_async(sdi,
                                   CMD_SET_TRIGGER,
                                   txbuf,
                                   index, // Tx size
                                   1 // Expected reply size
                                   )<0) {
                sr_err("Cannot send/receive");
                return -1;
        }
        return 0;
}


int zxinterfacez_trigger_set(const struct sr_dev_inst *sdi)
{
        struct dev_context *devc = sdi->priv;
        uint8_t clockdiv = zxinterfacez_get_divider(devc);

        // Start capture
        zxinterfacez_enter_state(devc,ZXI_WAIT_START);


        if (send_receive_cmd_async(sdi,
                                   CMD_START_CAPTURE,
                                   &clockdiv,
                                   1, // TX size
                                   1 // Expected RX
                                   <0)) {
                sr_err("Cannot send/receive");
                return -1;
        }
        return 0;
}


int zxinterfacez_capture_started(const struct sr_dev_inst *sdi)
{
        struct dev_context *devc = sdi->priv;

        zxinterfacez_enter_state(devc, ZXI_POLL);


        return sr_session_source_add(sdi->session,
                                     SOURCE_KEY_POLL,
                                     0, // events
                                     1000, // Timeout
                                     zxinterfacez_timer_elapsed,
                                     (void*)sdi);
        return 0;
}

static inline uint32_t extractle32(const uint8_t *source)
{
    uint32_t ret = (((uint32_t)source[3]) << 24) +
        (((uint32_t)source[2]) << 16) +
        (((uint32_t)source[1]) << 8) +
        ((uint32_t)source[0]);
    return ret;
}

SR_PRIV int send_receive_cmd(struct sr_serial_dev_inst *serial,
                             const uint8_t cmd,
                             const uint8_t *tx,
                             unsigned len,
                             int expected_len,
                             uint8_t *rx)
{
        unsigned timeout  = 200;
        last_reply = NULL;

        sr_dbg("Sending cmd %02x",cmd);
        hdlc_encoder__begin(&hdlc_encoder);
        hdlc_encoder__write(&hdlc_encoder, &cmd, 1);
        if (tx && len>0)
                hdlc_encoder__write(&hdlc_encoder, tx, len);
        hdlc_encoder__end(&hdlc_encoder);

        while (timeout--) {
                uint8_t lbuf[8];
                int r;
                do {
                        r = serial_read_nonblocking(serial, lbuf, sizeof(lbuf));
                        if (r>0) {
                                hdlc_decoder__append_buffer(&hdlc_decoder, lbuf, r);
                        }
                } while (r>0);

                if (last_reply!=NULL) {
                        if (last_reply_size==0) {
                                sr_err("Error in HDLC reply for command %02x: reply size zero", cmd);
                                return -1;
                        }
                        if (expected_len > 0 && (int)last_reply_size!=expected_len) {
                                sr_err("Error in HDLC reply for command %02x: reply size %d expected %d",
                                       cmd,
                                       last_reply_size, expected_len);
                                return -1;
                        }
                        if (last_reply[0] != (cmd | 0x80)) {
                                sr_err("Error in HDLC reply for command %02x: reply %02x",
                                       cmd,
                                       last_reply[0]);
                        }
                        memcpy(rx, last_reply, last_reply_size);

                        return last_reply_size;
                }

                g_usleep(10000);
        }
        return -1;
}

static int send_receive_async_timeout(int fd, int revents, void *user_data)
{
	UNUSED(fd);
	UNUSED(revents);

        const struct sr_dev_inst *sdi = (const struct sr_dev_inst*) user_data;
        struct dev_context *devc = sdi->priv;
        //devc->cmd_timer_id = g_timeout_add_seconds(1, &send_receive_async_timeout, (gpointer)sdi);
        sr_err("TIMEOUT waiting for reply, state %d", devc->state);
        return false;
}

SR_PRIV int send_receive_cmd_async(const struct sr_dev_inst *sdi,
                                   const uint8_t cmd,
                                   const uint8_t *tx,
                                   unsigned len,
                                   int expected_len)
{
        //struct sr_serial_dev_inst *serial = sdi->conn;
        struct dev_context *devc = sdi->priv;

        sr_dbg("Sending cmd %02x",cmd);

        last_reply = NULL;
        hdlc_encoder__begin(&hdlc_encoder);
        hdlc_encoder__write(&hdlc_encoder, &cmd, 1);
        if (tx && len>0)
                hdlc_encoder__write(&hdlc_encoder, tx, len);
        hdlc_encoder__end(&hdlc_encoder);

        devc->cmd = cmd;
        devc->expected_len = expected_len;
#ifdef ENABLE_RETRANSMIT
	return sr_session_source_add(sdi->session,
                                     SOURCE_KEY_RETRANSMIT,
                                     0, // events
                                     1000, // Timeout
                                     send_receive_async_timeout,
                                     (void*)sdi);
#endif
        return 0;
}





static int zxinterfacez_request_sample(const struct sr_dev_inst *sdi)
{
        //struct sr_serial_dev_inst *serial = sdi->conn;
        struct dev_context *devc = sdi->priv;
        uint8_t txbuf[2];

        txbuf[0] = devc->fetch_offset;

        if (devc->fetch_ram !=0) {
                txbuf[0] |= 0x80;
        }

        txbuf[1] = devc->fetch_seq;

        zxinterfacez_enter_state(devc,ZXI_WAIT_DATA);

        if (send_receive_cmd_async(sdi,
                                   CMD_GET_SAMPLES,
                                   txbuf,
                                   2,
                                   258)<0) {
                sr_err("Cannot send/receive");
                return -1;
        }
        return 0;
}

static int zxinterfacez_got_sample_data(const struct sr_dev_inst *sdi, const uint8_t *data)
{
        struct dev_context *devc = sdi->priv;

        if (data[0]!=devc->fetch_seq) {
                sr_err("Cannot fetch sample data");
                zxinterfacez_stop(sdi);
        }

        sr_dbg("Got data, offset %d ram %d", devc->fetch_offset, devc->fetch_ram);

        devc->fetch_seq++;

        memcpy(&devc->samples[devc->fetch_ram][devc->fetch_offset*256], &data[1], 256);

        devc->fetch_offset++;

        if (devc->fetch_offset>=16) {
                devc->fetch_ram++;
                devc->fetch_offset = 0;
                if (devc->fetch_ram>=2) {
                        // Finished
                        return zxinterfacez_samples_ready(sdi);
                }
        }

        sr_dbg("Requesting more data, offset %d ram %d", devc->fetch_offset, devc->fetch_ram);
        return zxinterfacez_request_sample(sdi);
}




static int zxinterfacez_fetch_samples(const struct sr_dev_inst *sdi)
{
        struct dev_context *devc = sdi->priv;

        // Do it in chunks. Each chunk is 256 bytes (64 samples) for each capture area.
        sr_info("Fetching data samples");

        devc->fetch_ram = 0;
        devc->fetch_offset = 0;
        devc->fetch_seq = 0;

        zxinterfacez_request_sample(sdi);

        return 0;
}

static unsigned zxinterfacez_get_num_samples(struct dev_context *devc)
{
        UNUSED(devc);
        return 1024;
}

static unsigned zxinterfacez_get_post_trigger_samples(struct dev_context *devc)
{
        unsigned ns = zxinterfacez_get_num_samples(devc);
        ns--;
        ns&=~0xF;
        return ns+1;
}

static unsigned zxinterfacez_get_pre_trigger_samples(struct dev_context *devc)
{
        return zxinterfacez_get_num_samples(devc) -  zxinterfacez_get_post_trigger_samples(devc);
}

#undef TEST_DATA


static uint8_t *zxinterfacez_convert_single_sample(struct dev_context *devc, unsigned start, uint8_t *dest)
{
        // First place non-triggerable data.
        uint32_t nontrig = extractle32(&devc->samples[1][start*4]);
        unsigned mask = 0x01;
        int i;

        for (i=0; i<zxinterfacez_nontrig_size();i++) {
            if (nontrig&1) {
                *dest |= mask;
            } else {
                *dest &= ~mask;
            }
            nontrig>>=1;
            mask<<=1;
            if (mask&0x100) {
                mask=0x01;
                dest++;
            }
        }

        uint32_t trig = extractle32(&devc->samples[0][start*4]);

        for (i=0; i<zxinterfacez_trig_size();i++) {
            if (trig&1) {
                *dest |= mask;
            } else {
                *dest &= ~mask;
            }
            trig>>=1;
            mask<<=1;
            if (mask&0x100) {
                mask=0x01;
                dest++;
            }
        }

        // Move forward if we placed any bit
        if (mask!=0x01)
            dest++;

        return dest;
}

static int zxinterfacez_convert_samples(struct dev_context *devc, uint8_t **triggerpoint)
{
        // Start with trigger point, move back
        uint8_t *sample = &devc->converted_samples[0];
        unsigned mask = zxinterfacez_get_num_samples(devc)-1;

        unsigned startp = devc->trig_address - zxinterfacez_get_pre_trigger_samples(devc);
        startp &= mask;
	sr_info("Starting conversion at offset 0x%04x trigger address 0x%04x",
		startp,
		devc->trig_address);

        for (unsigned i = 0; i<zxinterfacez_get_pre_trigger_samples(devc); i++) {
                sample = zxinterfacez_convert_single_sample( devc, startp, sample );

                startp++;
                startp &= mask;
        }

        *triggerpoint = sample;

        for (unsigned i = 0; i<zxinterfacez_get_post_trigger_samples(devc); i++) {
                sample = zxinterfacez_convert_single_sample( devc, startp, sample );

                startp++;
                startp &= mask;
        }


        return 0;
}

static int zxinterfacez_unitsize()
{
    unsigned numbits = zxinterfacez_nontrig_size() + zxinterfacez_trig_size();
    numbits+=7;
    return numbits>>3;
}

static int zxinterfacez_samples_ready(const struct sr_dev_inst *sdi)
{
        uint8_t *triggerpoint;
        struct sr_serial_dev_inst *serial = sdi->conn;
        struct dev_context *devc = sdi->priv;

        struct sr_datafeed_packet packet;
        struct sr_datafeed_logic logic;

        serial_source_remove(sdi->session, serial);

        zxinterfacez_convert_samples(devc, &triggerpoint);

        /* There are pre-trigger samples, send those first. */
        packet.type = SR_DF_LOGIC;
        packet.payload = &logic;
        logic.length = zxinterfacez_get_pre_trigger_samples(devc) * zxinterfacez_unitsize();
        logic.unitsize = zxinterfacez_unitsize();
        logic.data = &devc->converted_samples[0];

        sr_session_send(sdi, &packet);


        /* Send the trigger. */
        std_session_send_df_trigger(sdi);


        /* Send post-trigger. */
        packet.type = SR_DF_LOGIC;
        packet.payload = &logic;
        logic.length = zxinterfacez_get_post_trigger_samples(devc) * zxinterfacez_unitsize();
        logic.unitsize = zxinterfacez_unitsize();
        logic.data = triggerpoint;

        sr_session_send(sdi, &packet);

        std_session_send_df_end(sdi);

        return 0;
}

static int zxinterfacez_check_status(const struct sr_dev_inst *sdi, const uint8_t *data)
{
 //       uint8_t *triggerpoint = 0;
        //struct sr_serial_dev_inst *serial = sdi->conn;
        struct dev_context *devc = sdi->priv;

        uint32_t status = extractle32(&data[0]);
        uint32_t counter = extractle32(&data[4]);
        uint32_t trig_address = extractle32(&data[8]);

        sr_info("Capture status %08x counter %08x address %08x", status, counter, trig_address);

        if (status & 1) {
                // Done.
		// Stop polling timer
		sr_session_source_remove(sdi->session, SOURCE_KEY_POLL);

                devc->trig_address = trig_address;

                zxinterfacez_fetch_samples(sdi);
        }

        return SR_OK;
}


static int zxinterfacez_timer_elapsed(int fd, int revents, void *user_data)
{
	UNUSED(fd);
	UNUSED(revents);
        const struct sr_dev_inst *sdi = user_data;
//        struct sr_serial_dev_inst *serial = sdi->conn;
        struct dev_context *devc = sdi->priv;

        // Send polling

        zxinterfacez_enter_state(devc,ZXI_POLL);

        if (send_receive_cmd_async(sdi,
                             CMD_GET_STATUS,
                             NULL,
                             0, // TX size
                             13 // Expected rX
                             )<0) {
                sr_err("Cannot hsend/receive");
                // TBD:
                zxinterfacez_stop(sdi);
                return false;
        }
        return true;
}

SR_PRIV int zxinterfacez_stop(const struct sr_dev_inst *sdi)
{
        //struct dev_context *devc = sdi->priv;
#ifdef ENABLE_RETRANSMIT
	sr_session_source_remove(sdi->session, SOURCE_KEY_RETRANSMIT);
#endif
	sr_session_source_remove(sdi->session, SOURCE_KEY_POLL);

	serial_source_remove(sdi->session, sdi->conn);

        return 0;
}

SR_PRIV int zxinterfacez_receive_data(int fd, int revents, void *cb_data)
{
        const struct sr_dev_inst *sdi = cb_data;
        struct dev_context *devc = sdi->priv;
        struct sr_serial_dev_inst *serial = sdi->conn;
        UNUSED(revents);
        UNUSED(fd);

        uint8_t lbuf[8];
        int r;
        do {
                r = serial_read_nonblocking(serial, lbuf, sizeof(lbuf));
                if (r>0) {
                        hdlc_decoder__append_buffer(&devc->async_hdlc_decoder, lbuf, r);
                }
        } while (r>0);
        if (r<0)
                return FALSE;

        return TRUE;
}

