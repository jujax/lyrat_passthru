static bool source_is_bt = false;

void change_source(
    audio_pipeline_handle_t pipeline,
    audio_board_handle_t board_handle,
    esp_periph_handle_t bt_periph,
    audio_element_handle_t bt_stream_reader,
    audio_element_handle_t i2s_stream_reader,
    audio_event_iface_handle_t evt)
{
    audio_pipeline_pause(pipeline);
    audio_board_deinit(board_handle);
    board_handle = audio_board_init();
    ESP_LOGE(TAG, "Changing source to %s", source_is_bt ? "Bluetooth" : "Line");
    if (!source_is_bt)
    {
        //periph_bluetooth_pause(bt_periph);
        audio_pipeline_breakup_elements(pipeline, bt_stream_reader);
        audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
        audio_hal_set_volume(board_handle->audio_hal, 100);
        audio_pipeline_relink(pipeline, (const char *[]){"i2s_read", "i2s_write"}, 2);
        audio_pipeline_set_listener(pipeline, evt);
    }
    else
    {
        audio_pipeline_breakup_elements(pipeline, i2s_stream_reader);
        audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);
        audio_hal_set_volume(board_handle->audio_hal, 100);
        audio_pipeline_relink(pipeline, (const char *[]){"bt_read", "i2s_write"}, 2);
        audio_pipeline_set_listener(pipeline, evt);
        //periph_bluetooth_play(bt_periph);
    }
    audio_pipeline_run(pipeline);
    audio_pipeline_resume(pipeline);
    ESP_LOGE(TAG, "[ 4.1 ] Start playback new pipeline");
}

void line_out(void)
{

    ESP_LOGI(TAG, "[ 1 ] Init Bluetooth");
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = {'0', '8', '1', '0'};
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_dev_set_device_name("ESP_SINK_STREAM_DEMO");
    esp_bt_gap_set_pin(pin_type, 4, pin_code);
    esp_bt_gap_register_callback(bt_app_gap_cb);

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();

    ESP_LOGI(TAG, "[ 2 ] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[3.1] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[3.2] Create i2s stream to read data from codec chip");
    i2s_stream_cfg_t i2s_cfg_read = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg_read.type = AUDIO_STREAM_READER;
    i2s_stream_reader = i2s_stream_init(&i2s_cfg_read);

    ESP_LOGI(TAG, "[4.1] Get Bluetooth stream");
    a2dp_stream_config_t a2dp_config = {
        .type = AUDIO_STREAM_READER,
        .user_callback = {0},
    };
    bt_stream_reader = a2dp_stream_init(&a2dp_config);

    ESP_LOGI(TAG, "[3.3] Register all elements to audio pipeline");

    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_LINE_IN, AUDIO_HAL_CTRL_START);
    audio_hal_set_volume(board_handle->audio_hal, 100);

    audio_pipeline_register(pipeline, i2s_stream_reader, "i2s_read");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s_write");
    audio_pipeline_register(pipeline, bt_stream_reader, "bt_read");
    ESP_LOGI(TAG, "[3.4] Link it together [codec_chip]-->i2s_stream_reader-->i2s_stream_writer-->[codec_chip]");
    const char *link_tag[2] = {"i2s_read", "i2s_write"};
    audio_pipeline_link(pipeline, &link_tag[0], 2);

    //i2s_stream_set_clk(i2s_stream_reader, 44100, 16, 2);

    esp_periph_handle_t bt_periph = bt_create_periph();

    ESP_LOGI(TAG, "[ 4 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[3.8] Start bt peripheral");
    esp_periph_start(set, bt_periph);

    ESP_LOGI(TAG, "[4.1] Initialize Touch peripheral");
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 5 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[5.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[5.2] Listening event from peripherals");
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);

    ESP_LOGI(TAG, "[ 6 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG, "[ 6 ] Listen for all pipeline events");
    while (1)
    {

        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.cmd == AEL_MSG_CMD_ERROR)
        {
            ESP_LOGE(TAG, "[ * ] Action command error: src_type:%d, source:%p cmd:%d, data:%p, data_len:%d",
                     msg.source_type, msg.source, msg.cmd, msg.data, msg.data_len);
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)bt_stream_reader && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO)
        {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(bt_stream_reader, &music_info);

            ESP_LOGI(TAG, "[ * ] Receive music info from Bluetooth, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_setinfo(i2s_stream_writer, &music_info);
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
            continue;
        }

        if ((msg.source_type == PERIPH_ID_TOUCH || msg.source_type == PERIPH_ID_BUTTON || msg.source_type == PERIPH_ID_ADC_BTN) && (msg.cmd == PERIPH_TOUCH_TAP || msg.cmd == PERIPH_BUTTON_PRESSED || msg.cmd == PERIPH_ADC_BUTTON_PRESSED))
        {

            if ((int)msg.data == get_input_play_id() && source_is_bt)
            {
                ESP_LOGI(TAG, "[ * ] [Play] touch tap event");
                periph_bluetooth_play(bt_periph);
            }
            else if ((int)msg.data == get_input_set_id() && source_is_bt)
            {
                ESP_LOGI(TAG, "[ * ] [Set] touch tap event");
                periph_bluetooth_pause(bt_periph);
            }
            else if ((int)msg.data == get_input_volup_id() && source_is_bt)
            {
                ESP_LOGI(TAG, "[ * ] [Vol+] touch tap event");
                periph_bluetooth_next(bt_periph);
            }
            else if ((int)msg.data == get_input_voldown_id() && source_is_bt)
            {
                ESP_LOGI(TAG, "[ * ] [Vol-] touch tap event");
                periph_bluetooth_prev(bt_periph);
            }
            else if ((int)msg.data == get_input_mode_id())
            {
                ESP_LOGI(TAG, "[ * ] [Mode] touch tap event");
                source_is_bt = !source_is_bt;
                change_source(pipeline, board_handle, bt_periph, bt_stream_reader, i2s_stream_reader, evt);
            }
            else if ((int)msg.data == get_input_rec_id())
            {
                ESP_LOGI(TAG, "[ * ] [Rec] touch tap event");
                output_is_bt = !output_is_bt;
                change_source(pipeline, board_handle, bt_periph, bt_stream_reader, i2s_stream_reader, evt);
                err = nvs_set_i32(my_handle, "mode", output_is_bt);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                printf("Committing updates in NVS ... ");
                err = nvs_commit(my_handle);
                printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
                break;
            }
        }

        /* Stop when the Bluetooth is disconnected or suspended */
        if (msg.source_type == PERIPH_ID_BLUETOOTH && msg.source == (void *)bt_periph)
        {
            if (msg.cmd == PERIPH_BLUETOOTH_DISCONNECTED)
            {
                ESP_LOGW(TAG, "[ * ] Bluetooth disconnected");
                break;
            }
        }
        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *)i2s_stream_writer && msg.cmd == AEL_MSG_CMD_REPORT_STATUS && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED)))
        {
            ESP_LOGW(TAG, "[ * ] Stop event received");
            break;
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
    //audio_pipeline_unregister(pipeline, bt_stream_writer);
    audio_pipeline_unregister(pipeline, i2s_stream_reader);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
    audio_pipeline_unregister(pipeline, bt_stream_reader);

    audio_pipeline_deinit(pipeline);

    //if(output_is_bt) audio_element_deinit(bt_stream_writer);
    audio_element_deinit(bt_stream_reader);
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