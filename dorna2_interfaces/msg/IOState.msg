# Digital inputs (in0..in7), true = high
bool[] inputs
# Digital outputs (out0..out7), true = high
bool[] outputs
# PWM enable state per channel
bool[] pwm_enable
# PWM frequency per channel (Hz)
float64[] pwm_freq
# PWM duty cycle per channel (0.0-1.0)
float64[] pwm_duty
# ADC readings per channel
float64[] adc
