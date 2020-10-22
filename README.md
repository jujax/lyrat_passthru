# Audio Passthru / Bluetooth Sink-Source with ESP32-LyraT V4.3 Board 

This can be used in multiple case:

- Passes audio from Line in or Bluetooth to Line out.
- Passes audio from Line in to Bluetooth.

## Compatibility

This example works with ESP32-LyraT	(with ES8388 audio driver), [ESP-IDF](https://github.com/espressif/esp-idf/) and [ESP-ADF](https://github.com/espressif/esp-adf/) libraries.

ESP-ADF library redirect automatically Aux input to Headphone output.
If you want to use Aux input to another output, you have to use these lines :
```c
audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

es8388_write_reg(ES8388_ADCCONTROL2, ADC_INPUT_LINPUT2_RINPUT2); 
```

Also sound is very dirty when redirected to bluetooth speaker.

It's because ES8388 is initialized with 9db PGA gain, so you have to use this (0x00 is corresponding to 0db, initialy 0xBB) :
```c
es8388_write_reg(ES8388_ADCCONTROL1, 0x00);
```

## Settings

You can set up device name, pin code and remote device name in `main/variables.h`.

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output.

```bash
idf.py set-target esp32
make -j4 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

## Example Output

You can choose between Line out and Bluetooth out when you press ```MODE``` button.

When you are in Line out mode, you can switch between Line input and Bluetooth input when you press ```REC``` button.

### Bluetooth input to Line out

- Connect with Bluetooth on your smartphone to the audio board identified as "ESP_SINK_STREAM_DEMO"
- Play some audio from the smartphone and it will be transmitted over Bluetooth to the audio bard.
- The audio playback may be controlled from the smartphone, as well as from the audio board. The following controls may be used:

    |   Smartphone   | Audio Board |
    |:--------------:|:-----------:|
    |   Play Music   |    [Play]   |
    |   Stop Music   |    [Set]    |
    |   Next Song    | [long Vol+] |
    | Previous Song  | [long Vol-] |
    |   Volume Up    |    [n/a]    |
    |  Volume Down   |    [n/a]    |

### Line in to Bluetooth output

You have to choose bluetooth speaker name in ```main/passthru.c``` line 37, eg :

```c
#define CONFIG_BT_REMOTE_NAME "WH-1000XM4"
```

It connect to my WH-1000XM4 headset after I pair it.