from machine import Pin, ADC

current = ADC(Pin(34))
current.atten(ADC.ATTN_11DB)           
current.width(ADC.WIDTH_10BIT)         

voltage = ADC(Pin(33))
voltage.atten(ADC.ATTN_11DB)           
voltage.width(ADC.WIDTH_10BIT)         

def calculate_current_adc_error(adc_value):
    return (21.7
            +  (0.267 * adc_value)
            +  (-0.00103 * adc_value * adc_value)
            +  (0.00000172 * adc_value * adc_value * adc_value)
            +  (-0.00000000102 * adc_value * adc_value * adc_value * adc_value))

def calculate_voltage_adc_error(adc_value):
    return (21.7
            +  (0.267 * adc_value)
            +  (-0.00103 * adc_value * adc_value)
            +  (0.00000172 * adc_value * adc_value * adc_value)
            +  (-0.00000000102 * adc_value * adc_value * adc_value * adc_value))

def current_calculation():
    global current
    
    actual_current = current.read()
    current_value = round(actual_current + calculate_current_adc_error(actual_current))
    current_output = 617 - current_value + 155
    
    return current_output * 0.032415 

def voltage_calculation():
    global voltage
    
    actual_voltage = voltage.read()
    voltage_value = round(actual_voltage + calculate_voltage_adc_error(actual_voltage))
    
    return voltage_value * 0.3203684
    
def power_calculation():
    current_value = current_calculation()
    voltage_value = voltage_calculation()
    power_value = current_value * voltage_value
    
    return power_value