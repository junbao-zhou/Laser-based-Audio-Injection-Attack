# Laser-based Audio Injection Attack

Most voice-controllable devices (e.g. Google Home) feature with MEMS microphone, which is vulnerable to laser command.

It means that we can modulate a laser beam with voice signal using AM modulation, and inject commands into MEMS microphone.

The inspiration has been explained well in Sugawara et al.'s work [1]

To implement the modulation process, sampling from a voice signal is necessary. Any MCU features with ADC (e.g. stm32) or FPGA is capable for doing such thing.

Besides, we should output a certain value to control the intensity of laser beam.

Overall, to complete a laser-based Audio Injection Attack, we should sample data from a voice signal, then do some preprocessing, and output the preprocessed value to an AM-modulated laser beam.

## References
<a id="1">[1]</a> 
Sugawara, Takeshi, et al.
"Light Commands:Laser-Based Audio Injection Attacks on Voice-Controllable Systems."
29th USENIX Security Symposium (USENIX Security 20). 2020.