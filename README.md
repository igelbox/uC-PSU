# Microcontrolled) CC/CV Power Supply Unit

Via [ngscopeclient](https://www.ngscopeclient.org/manual/PowerSupplyDrivers.html) `rigol_dp8xx` driver's SCPI-based protocol

![](https://github.com/user-attachments/assets/768120eb-cf5a-4ea2-833c-9a4e1426ecf7)

## It appears
 one doesn't need no digipot nor DAC. PWM + RC-filter works pretty well

**Voltage:** is pretty linear until 1.25 which is XL4015 internal refence voltage value
![duty-mv](https://github.com/user-attachments/assets/fc2cdb0e-eb81-4f2e-8c93-992bec833e58)

**Current:** (output) per pwm is linear all the way
but ADC has some non-linearity in lower ranges
![duty-ma](https://github.com/user-attachments/assets/5d13ae0b-77eb-4469-96df-4caf687438f3)


## BUT: XL5015 itself is quite noisy and tends to oscillate

|Case|Noise / Current limit, A|
|--:|:-:|
|3V / 0R|![Noise 3V 0R](https://github.com/user-attachments/assets/301ed400-f222-4005-b929-7249a74e455a)|
|3V / 4R7|![Noise 3V 4R7](https://github.com/user-attachments/assets/c19bb074-d3ab-4428-89d6-7cba4e3f784a)|
|6V / 4R7|![Noise 6V 4R7](https://github.com/user-attachments/assets/8cc3b74c-d368-4396-9dd0-1b7b26e97ee8)|
|9V / 4R7|![Noise 9V 4R7](https://github.com/user-attachments/assets/b1ed1d02-aa78-4a2d-bc2a-daeac5d449b0)|
