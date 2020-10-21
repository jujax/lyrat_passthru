#include "es8388.h"

static bool device_found = false;

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18)
    {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir)
    {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname)
    {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname)
    {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN)
        {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname)
        {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len)
        {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void filter_inquiry_scan_result(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *eir = NULL;
    esp_peer_bdname_t peer_bdname;
    esp_bt_gap_dev_prop_t *p;

    ESP_LOGI(TAG, "Scanned device: %s", bda2str(param->disc_res.bda, bda_str, 18));
    for (int i = 0; i < param->disc_res.num_prop; i++)
    {
        p = param->disc_res.prop + i;
        switch (p->type)
        {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(TAG, "--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(TAG, "--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR:
            eir = (uint8_t *)(p->val);
            get_name_from_eir(eir, (uint8_t *)&peer_bdname, NULL);
            ESP_LOGI(TAG, "--Name: %s", peer_bdname);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
        default:
            break;
        }
    }

    ESP_LOGI(TAG, "need device name %s", (uint8_t *)remote_bt_device_name);
    /* search for device named "peer_bdname" in its extended inquiry response */
    if (eir)
    {
        get_name_from_eir(eir, (uint8_t *)&peer_bdname, NULL);
        if (strcmp((char *)peer_bdname, (char *)remote_bt_device_name) != 0)
        {
            return;
        }

        ESP_LOGI(TAG, "Found a target device, address %s, name %s", bda_str, (uint8_t *)peer_bdname);
        device_found = true;
        memcpy(&remote_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
        ESP_LOGI(TAG, "Cancel device discovery ...");
        esp_bt_gap_cancel_discovery();
    }
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_DISC_RES_EVT:
    {
        filter_inquiry_scan_result(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
    {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED)
        {
            if (device_found)
            {
                ESP_LOGI(TAG, "Device discovery stopped.");
                ESP_LOGI(TAG, "a2dp connecting to peer: %s", remote_bt_device_name);
                device_found = false;
                esp_a2d_source_connect(remote_bd_addr);
            }
            else
            {
                // not discovered, continue to discover
                ESP_LOGI(TAG, "Device discovery failed, continue to discover...");
                esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            }
        }
        else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED)
        {
            ESP_LOGI(TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit)
        {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        }
        else
        {
            esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
    default:
        break;
    }
    return;
}

void bluetooth_output(void)
{
    // ESP_LOGI(TAG, "[ 1 ] Mount sdcard");
    // // Initialize peripherals management
    // esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    // esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    // audio_board_sdcard_init(set, SD_MODE_1_LINE);

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

    es8388_write_reg(ES8388_ADCCONTROL1, 0x00); // MIC Left and Right channel PGA gain
    es8388_write_reg(ES8388_ADCCONTROL2, ADC_INPUT_LINPUT2_RINPUT2);

    ESP_LOGI(TAG, "[3.0] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);
    mem_assert(pipeline);

    ESP_LOGI(TAG, "[3.2] Create i2s stream to read data from codec chip");
    i2s_stream_cfg_t i2s_cfg_read = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg_read.type = AUDIO_STREAM_READER;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg_read);

    ESP_LOGI(TAG, "[ 1 ] Create Bluetooth service");
    bluetooth_service_cfg_t bt_cfg = {
        .device_name = "ESP-ADF-SOURCE",
        .mode = BLUETOOTH_A2DP_SOURCE,
        .remote_name = CONFIG_BT_REMOTE_NAME,
    };
    bluetooth_service_start(&bt_cfg);

    ESP_LOGI(TAG, "[1.1] Get Bluetooth stream");
    bt_stream_writer = bluetooth_service_create_stream();

    ESP_LOGI(TAG, "[4.2] Create Bluetooth peripheral");
    esp_periph_handle_t bt_periph = bluetooth_service_create_periph();

    ESP_LOGI(TAG, "[3.4] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, i2s_stream_reader, "file");
    audio_pipeline_register(pipeline, bt_stream_writer, "bt");

    ESP_LOGI(TAG, "[3.5] Link it together [sdcard]-->fatfs_stream-->mp3_decoder-->bt_stream-->[bt sink]");
    const char *link_tag[2] = {"file", "bt"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[3.8] Start bt peripheral");
    esp_periph_start(set, bt_periph);

    ESP_LOGI(TAG, "[4.1] Initialize Touch peripheral");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 4 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[4.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[4.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG, "[ 5 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    while (1)
    {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        audio_element_info_t music_info = {0};
        audio_element_getinfo(i2s_stream_reader, &music_info);

        ESP_LOGW(TAG, "[ * ] Receive music info from Line, sample_rates=%d, bits=%d, ch=%d",
                 music_info.sample_rates, music_info.bits, music_info.codec_fmt);

        audio_element_setinfo(bt_stream_writer, &music_info);
        //i2s_stream_set_clk(bt_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);

        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        /* Stop when the Bluetooth is disconnected or suspended */
        if (msg.source_type == PERIPH_ID_BLUETOOTH && msg.source == (void *)bt_periph)
        {
            if ((msg.cmd == PERIPH_BLUETOOTH_DISCONNECTED) || (msg.cmd == PERIPH_BLUETOOTH_AUDIO_SUSPENDED))
            {
                ESP_LOGW(TAG, "[ * ] Bluetooth disconnected or suspended");
                periph_bluetooth_stop(bt_periph);
                break;
            }
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO)
        {
            continue;
        }

        if ((msg.source_type == PERIPH_ID_TOUCH || msg.source_type == PERIPH_ID_BUTTON || msg.source_type == PERIPH_ID_ADC_BTN) && (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED))
        {
            if ((int)msg.data == get_input_rec_id())
            {
                ESP_LOGI(TAG, "[ * ] [Rec] touch tap event");
                output_is_bt = !output_is_bt;
                err = nvs_set_i32(my_handle, "mode", output_is_bt);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err = nvs_commit(my_handle);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            }
        }
    }

    ESP_LOGI(TAG, "[ 9 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Stop all peripherals before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_unregister(pipeline, bt_stream_writer);
    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);

    audio_pipeline_deinit(pipeline);

    audio_element_deinit(bt_stream_writer);
    audio_element_deinit(i2s_stream_reader);
    audio_element_deinit(i2s_stream_writer);

    esp_periph_set_destroy(set);
    bluetooth_service_destroy();

    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
}