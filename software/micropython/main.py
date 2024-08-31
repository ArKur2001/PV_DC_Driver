import onewire, ds18x20
from machine import ADC, Pin, PWM, SoftI2C, Timer
from lcd_api import LcdApi
from i2c_lcd import I2cLcd
from time import sleep

current_adc = ADC(Pin(34))
voltage_adc = ADC(Pin(35))

increment_button = Pin(13, Pin.IN, Pin.PULL_UP)
decrement_button = Pin(12, Pin.IN, Pin.PULL_UP)

pwm = PWM(Pin(0))
pwm.freq(5000)


#======================PROGRAM_SETTINGS======================================
#----------------------I2c_and_lcd_settings----------------------------------
I2C_ADDR = 0x27
totalRows = 2
totalColumns = 16
i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000) 
lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)
#----------------------------------------------------------------------------
#---------------------ds18b20_sensor_settings--------------------------------
ds_pin = Pin(4)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))
#----------------------------------------------------------------------------
#-------------------current_measurement_Kalman_settings
I_kalman_adc_old = 0
I_P1 = 0
I_Q = 0.1    #0.0003
I_R = 5
I_Kg = 0
I_P = 1
#----------------------------------------------------------------------------
#-------------------voltage_measurement_Kalman_settings----------------------
V_kalman_adc_old = 0
V_P1 = 0
V_Q = 0.1    #0.0003
V_R = 5
V_Kg = 0
V_P = 1
#----------------------------------------------------------------------------
#============================================================================

def change_of_fill_factor():
    global fill_value
   
    if increment_button.value() == 0:
        if fill_value < 100:
            fill_value = fill_value + 1
    if decrement_button.value() == 0:
        if fill_value > 1:
            fill_value = fill_value - 1
                        
    print('Współczynnik wypełnienia:', fill_value, '%')

   
def display(Timer):
    lcd.move_to(0,0)
    lcd.putstr("Moc: " + str(round(power_value())) + " W     ")
    lcd.move_to(0,1)
    lcd.putstr("Temp: " + str(temp_value) + " " + chr(223) + "C")
    
def display_clear(Timer):
    lcd.clear()

def read_ds_sensor(Timer):
    global temp_value
    roms = ds_sensor.scan()
    ds_sensor.convert_temp()
    for rom in roms:
        temp_value = ds_sensor.read_temp(rom)
        if isinstance(temp_value, float):
            temp_value = round(temp_value, 1)

def current_value():
    global n_factor
    A_per_bit = 0.008056640625
    reference_adc_value = 3252 #trzeba skalibrować
    reference_adc_diff = reference_adc_value - current_kalman_filter(current_adc.read())
    
    if n_factor > 0:
        average_current_value = A_per_bit * reference_adc_diff
        max_current_value = average_current_value / n_factor
    
        return max_current_value
    
    else:
        return 0
    
def voltage_value():
    global n_factor
    V_per_bit = 0.0008056640625
    voltage_divider_factor = 10 #dopasowany do zasilacza, nie paneli rezystory, 100k i 10k
    
    if n_factor > 0:
        average_voltage_value = V_per_bit *  voltage_divider_factor * voltage_kalman_filter(voltage_adc.read())
        max_voltage_value = average_voltage_value / n_factor
    
        return max_voltage_value
    
    else:
        return 0
    
def power_value():
    global fill_value
    global n_factor
    
    n_factor = fill_value / 1023
    
    current_current = current_value()
    current_voltage = voltage_value()
    
    power_value = current_current * current_voltage * n_factor
    
    return power_value

def MPPT_algorithm():
    global fill_value
    
    fill_value_array = []
    power_value_array = []
    old_power_value = 0
    cell_number = 0
    
    for fill_value_counter in range(0, 5):
        fill_value_array.append(fill_value - 2 + fill_value_counter)
        
        if fill_value_array[fill_value_counter] < 2:
            fill_value_array[fill_value_counter] = 1
        elif fill_value_array[fill_value_counter] > 1022:
            fill_value_array[fill_value_counter] = 1023
        
    for power_value_counter in range(0, 5):
        fill_value = fill_value_array[power_value_counter]
        pwm.duty(fill_value)
        for kalman_wait in range(0, 100):
            power_value()
            if kalman_wait == 99:
                power_value_array.append(power_value())
                
    for highest_value_counter in range(0, 5):
        current_power_value = power_value_array[highest_value_counter]
        
        if current_power_value > old_power_value:
            cell_number = highest_value_counter
            old_power_value = current_power_value
        else:
            old_power_value = current_power_value
            
    fill_value = fill_value_array[cell_number]
    pwm.duty(fill_value)
    fill_value_array.clear()
    power_value_array.clear()
           
def current_kalman_filter(adc_value):
    global I_kalman_adc_old
    global I_P1
    global I_Q
    global I_R
    global I_Kg
    global I_P 
    
    NowData =  adc_value
    LastData = I_kalman_adc_old
    I_P = I_P1 + I_Q
    I_Kg = I_P / (I_P + I_R)
    I_kalman_adc = LastData + I_Kg * (NowData - I_kalman_adc_old)
    I_P1 = (1 - I_Kg) * I_P
    I_P = I_P1
    I_kalman_adc_old = I_kalman_adc
    
    return I_kalman_adc
    
def voltage_kalman_filter(adc_value):
    global V_kalman_adc_old
    global V_P1
    global V_Q
    global V_R
    global V_Kg
    global V_P 
    
    NowData =  adc_value
    LastData = V_kalman_adc_old
    V_P = V_P1 + V_Q
    V_Kg = V_P / (V_P + V_R)
    V_kalman_adc = LastData + V_Kg * (NowData - V_kalman_adc_old)
    V_P1 = (1 - V_Kg) * V_P
    V_P = V_P1
    V_kalman_adc_old = V_kalman_adc
    
    return V_kalman_adc

#=============================TIMERS=========================================
tim1 = Timer(0)
tim1.init(period = 5000, mode = Timer.PERIODIC, callback = display)

tim2 = Timer(1)
tim2.init(period = 4500, mode = Timer.ONE_SHOT, callback = display_clear)

tim3 = Timer(2)
tim3.init(period = 10000, mode = Timer.PERIODIC, callback = read_ds_sensor)
#============================================================================

#================STARTUP_PARAMETERS======================
fill_value = 512
temp_value = 0
n_factor = 0

lcd.putstr("MPPT driver v0.1")
lcd.move_to(0,1)
lcd.putstr("INICJALIZACJA...")
#========================================================

while True:
    MPPT_algorithm()
    sleep(0.1)