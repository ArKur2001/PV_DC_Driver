import onewire, ds18x20
from machine import Pin
from time import sleep_ms 

increment_button = Pin(14, Pin.IN, Pin.PULL_UP)
decrement_button = Pin(27, Pin.IN, Pin.PULL_UP)
ds_pin = Pin(4)
ds_sensor = ds18x20.DS18X20(onewire.OneWire(ds_pin))

water_temperature = 0
but_flag = 0 #if 0 enable, if 1 locked

def change_temp_factor(set_temperature):
    global but_flag
    
    but1_val = increment_button.value()
    but2_val = decrement_button.value()
    
    if but1_val == 0 and but_flag == 0:
        but_flag = 1
        if set_temperature < 90:
            set_temperature += 1
    elif but2_val == 0 and but_flag == 0:
        but_flag = 1
        if set_temperature > 10:
            set_temperature -= 1
    elif but1_val == 1 and but2_val == 1:
        but_flag = 0
        
    return set_temperature
        
def read_ds_sensor():
    roms = ds_sensor.scan()
    try:
        ds_sensor.convert_temp()
        sleep_ms(20)
        
        for rom in roms:
            temp = ds_sensor.read_temp(rom)
            if isinstance(temp, float):
                temp = round(temp, 2)
                water_temperature  = temp
            
    except onewire.OneWireError:
        sleep_ms(20)
        water_temperature = 0
        
    return water_temperature

 
    