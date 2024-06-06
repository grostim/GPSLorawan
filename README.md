# GPS TTNmapper for Seeeduino LORAWAN

This is a crude GPS tracker on Lorawan.
Meant to be used with [Seeeduino LoRAWAN board]([https://](https://wiki.seeedstudio.com/Seeeduino_LoRAWAN/))

This crude program is widely inspired by the samples code provided by Seeduino with their library and has been extended with the following features:
- Packet sent at a given time interval, or as soon as the module has moved a given distance from the latest point.
- JS decoder compatible with the TTNMapper webhook
- a downlink is setup to adjust the time and distance interval.
- JS downlink encoder is provided.

The default program is set up for TTN EU868 radio parameters, but it could be easily adjusted to other region.

