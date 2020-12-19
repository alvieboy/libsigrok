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

#define SERIALCOMM "115200/8n1"

static const uint32_t scanopts[] = {
	SR_CONF_CONN,
	SR_CONF_SERIALCOMM,
};

static const uint32_t drvopts[] = {
	SR_CONF_LOGIC_ANALYZER,
};

static const uint32_t devopts[] = {
	SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST
};

static const uint32_t devopts_trig[] = {
	SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
};

static const uint32_t devopts_nontrig[] = {
};

static const int32_t trigger_matches[] = {
	SR_TRIGGER_ZERO,
	SR_TRIGGER_ONE,
	SR_TRIGGER_RISING,
        SR_TRIGGER_FALLING
};

/* Default supported samplerates, can be overridden by device metadata. */
static const uint64_t samplerates[] = {
	SR_MHZ(96),
        SR_MHZ(48),
        SR_MHZ(32),
        SR_MHZ(24),
        SR_KHZ(19200),
        SR_MHZ(16),
        SR_HZ(13714286),
        SR_MHZ(12)
};

static char *check_version_reply()
{
	unsigned last_reply_size = zxinterfacez_get_last_reply_size();
	if (last_reply_size>1) {
		const uint8_t *last_reply = zxinterfacez_get_last_reply();
		if (last_reply[0]==0x81) {
			return g_strndup((char*)&last_reply[1], last_reply_size-1);
		}
	}
	return NULL;
}

static void free_signal_group_names(struct dev_context *devc, scope_group_t group)
{
    unsigned i;
    for (i=0;i<MAX_CHANNELS_PER_GROUP;i++) {
        if (devc->channel_names[(int)group][i]!=NULL) {
            free( devc->channel_names[(int)group][i] );
            devc->channel_names[(int)group][i] = NULL;
        }
    }

}
static void free_signal_names(struct dev_context *devc)
{
    free_signal_group_names(devc, SCOPE_GROUP_NONTRIG);
    free_signal_group_names(devc, SCOPE_GROUP_TRIG);
}

static int scan_null(const uint8_t *ptr, int len)
{
    int pos = -1;

    if (len==0)
        return -1;

    do {
        pos++;
        if (*ptr=='\0')
            return pos;
        ptr++, len--;

    } while (len>0);
    if (len<0)
        pos=-1;
    return pos;
}

static const uint8_t *scan_post_null(const uint8_t *ptr, int *len)
{
    int delta = scan_null(ptr, *len);
    if (delta<0)
        return NULL;
    if ((*len)==delta)
        return NULL; // No more data
    *len = *len - delta;
    return ptr+delta+1;
}

static int build_group_names(struct dev_context *devc, scope_group_t group, const uint8_t **ptr, int *len)
{
    uint8_t grouplen = **ptr;
    int r = -1;

    if (*len<1) {
        return r;
    }

    sr_info("Channels: %d", (int)grouplen);

    (*ptr)++, (*len)--;
    do {
        // Load group names.
        for (unsigned i=0;i<grouplen;i++) {
            int chanlen = scan_null(*ptr,*len);
            if (chanlen<0) {
                sr_err("Cannot load name for index %d", i);
                break;
            }
            devc->channel_names[group][i] = g_strdup((const char*)*ptr);

            (*ptr) += chanlen;
            (*len) -= chanlen;
            // Move past null;
            if ((*len)<1)
                break;
            (*ptr)++, (*len)--;
            sr_info("Channel %d: %s\n", i, devc->channel_names[group][i]);
        }
        r = grouplen;
    } while (0);
    return r;
}

static int build_signal_names(struct dev_context *devc)
{
    const uint8_t *ptr = zxinterfacez_get_last_reply();
    int len = zxinterfacez_get_last_reply_size();

    int delta;
    int r = -1;

    free_signal_names(devc);
    devc->group_num_channels[SCOPE_GROUP_NONTRIG] = 0;
    devc->group_num_channels[SCOPE_GROUP_TRIG] = 0;

    // 1st, move past version NULL indicator
    do {
        ptr = scan_post_null(ptr, &len);
        if (ptr==NULL) {
            break;
        }
        int grouplen = build_group_names(devc, SCOPE_GROUP_NONTRIG, &ptr, &len);
        if (grouplen<0)
            break;
        devc->group_num_channels[SCOPE_GROUP_NONTRIG] = grouplen;

        grouplen = build_group_names(devc, SCOPE_GROUP_TRIG, &ptr, &len);
        if (grouplen<0)
            break;
        devc->group_num_channels[SCOPE_GROUP_TRIG] = grouplen;
        r = 0;

    } while (0);

    
    return r;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
    struct sr_config *src;
    struct sr_dev_inst *sdi;
    struct sr_serial_dev_inst *serial;
    struct dev_context *devc;
    int chan_index = 0;

    GSList *l;
    unsigned int i;
    const char *conn, *serialcomm;

    conn = serialcomm = NULL;
    for (l = options; l; l = l->next) {
        src = l->data;
        switch (src->key) {
        case SR_CONF_CONN:
            conn = g_variant_get_string(src->data, NULL);
            break;
        case SR_CONF_SERIALCOMM:
            serialcomm = g_variant_get_string(src->data, NULL);
            break;
        }
    }
    if (!conn)
        return NULL;

    if (!serialcomm)
        serialcomm = SERIALCOMM;

    serial = sr_serial_dev_inst_new(conn, serialcomm);

    sr_info("Probing %s.", conn);



    if (serial_open(serial, SERIAL_RDWR) != SR_OK)
        return NULL;

    uint8_t hdlc_decoder_buf[512];

    zxinterfacez_setup_comms(serial,
			     hdlc_decoder_buf,
			     sizeof(hdlc_decoder_buf)
			    );

    uint8_t vreply[512];

    if (send_receive_cmd(serial, 0x01, NULL, 0, -1, vreply)<0) {
        sr_dbg("No reply");
        serial_close(serial);
        return NULL;
    }
    char *version = check_version_reply();
    if (!version) {
        serial_close(serial);
        return NULL;
    }

    sdi = g_malloc0(sizeof(struct sr_dev_inst));

    devc = zxinterfacez_dev_new(sdi);

    if (build_signal_names(devc)<0) {
        sr_err("Cannot build signal names");
        free(devc);
        serial_close(serial);
        return NULL;
    }

    sdi->status = SR_ST_INACTIVE;
    sdi->vendor = g_strdup("Alvie");
    sdi->model =  g_strdup("ZX Interface Z");
    sdi->version = version;


    sdi->priv = devc;


    devc->nontrig_group = g_malloc0(sizeof(struct sr_channel_group));
    devc->trig_group = g_malloc0(sizeof(struct sr_channel_group));

    for (i = 0; i < devc->group_num_channels[SCOPE_GROUP_NONTRIG]; i++) {
        struct sr_channel *ch = sr_channel_new(sdi, chan_index, SR_CHANNEL_LOGIC, TRUE, devc->channel_names[SCOPE_GROUP_NONTRIG][i]);
        devc->nontrig_group->channels = g_slist_append(devc->nontrig_group->channels, ch);
        chan_index++;
    }

    devc->nontrig_group->name = g_strdup("NT");

    sdi->channel_groups = g_slist_append(sdi->channel_groups,
					 devc->nontrig_group);


    // Triggering group
    for (i = 0; i < devc->group_num_channels[SCOPE_GROUP_TRIG]; i++) {
	    struct sr_channel *ch = sr_channel_new(sdi, chan_index, SR_CHANNEL_LOGIC, TRUE,  devc->channel_names[SCOPE_GROUP_TRIG][i]);
            devc->trig_group->channels = g_slist_append(devc->trig_group->channels, ch);
	    chan_index++;
    }

    devc->trig_group->name = g_strdup("TT");

    sdi->channel_groups = g_slist_append(sdi->channel_groups,
					 devc->trig_group);



    sdi->inst_type = SR_INST_SERIAL;
    sdi->conn = serial;

    serial_close(serial);

    return std_scan_complete(di, g_slist_append(NULL, sdi));
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	(void)cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	unsigned int i;
	struct dev_context *devc = sdi ? sdi->priv : NULL;
	uint64_t tmp_u64;
	UNUSED(sdi);
	UNUSED(cg);

	switch (key) {
	case SR_CONF_SAMPLERATE:
		tmp_u64 = g_variant_get_uint64(data);
		if (devc==NULL)
			return SR_ERR_ARG;

		for (i=0; i<(sizeof(samplerates)/sizeof(samplerates[0]));i++)
		{
			if (tmp_u64 == samplerates[i]) {
				devc->cur_samplerate = samplerates[i];
				return SR_OK;
			}
		}
		return SR_ERR_SAMPLERATE;

	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;

	devc = sdi ? sdi->priv : NULL;
	const struct sr_key_info *srci;
	srci = sr_key_info_get(SR_KEY_CONFIG, key);

	switch (key) {
	case SR_CONF_SCAN_OPTIONS:
	case SR_CONF_DEVICE_OPTIONS:
		if (cg==NULL)
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
		if (!devc)
			return SR_ERR_ARG;
		if (cg==devc->trig_group) {
			sr_info("Return opts trig");
			*data = std_gvar_array_u32(ARRAY_AND_SIZE(devopts_trig));
			break;
		} else if (cg==devc->nontrig_group) {
			sr_info("Return opts nontrig");
			*data =std_gvar_array_u32(ARRAY_AND_SIZE(devopts_nontrig));
			break;
		}
		return SR_ERR_ARG;

	case SR_CONF_SAMPLERATE:
		*data = std_gvar_samplerates(ARRAY_AND_SIZE(samplerates));
                break;
	case SR_CONF_TRIGGER_MATCH:
		*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
		break;

	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
    struct sr_serial_dev_inst *serial;

    serial = sdi->conn;

    if (zxinterfacez_convert_trigger(sdi)!=SR_OK) {
	    sr_err("Cannot convert trigger");
    }

    // Set up parameters, and arm
    if (zxinterfacez_setup_and_start(sdi)<0) {
	    return SR_ERR;
    }

    serial_source_add(sdi->session, serial, G_IO_IN, 100,
		      zxinterfacez_receive_data,
		      (struct sr_dev_inst *)sdi);

    return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{

	struct sr_serial_dev_inst *serial;

	serial = sdi->conn;

	zxinterfacez_stop(sdi);

	serial_source_remove(sdi->session, serial);

	std_session_send_df_end(sdi);

//        sr_dev_close(sdi);

	return SR_OK;
}

static int dev_open(struct sr_dev_inst *sdi)
{
        sr_info("Opening serial port");
        return std_serial_dev_open(sdi);
}

static int dev_close(struct sr_dev_inst *sdi)
{
        sr_info("Closing serial port");
        return std_serial_dev_close(sdi);
}



static struct sr_dev_driver zxinterfacez_driver_info = {
	.name = "zxinterfacez",
        .longname = "ZX Spectrum Interface Z",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
	.context = NULL,
};
SR_REGISTER_DEV_DRIVER(zxinterfacez_driver_info);
