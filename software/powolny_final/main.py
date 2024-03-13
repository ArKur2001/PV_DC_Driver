import _thread
from display_handling import display
from temp_api import read_ds_sensor, change_temp_factor
from output_api import output_state
from power_api import power_calculation, current_calculation, voltage_calculation
from time import sleep
from machine import Timer, Pin, freq, WDT

#========================================================================================================

freq(240000000)
wdt = WDT(timeout=2000)

buzzer = Pin(2, Pin.OUT)
output = Pin(23, Pin.OUT)

global_time = 0
set_temperature = 50
water_temperature = 0.0
program_status = 0
current_value = 0.0
wait_time = 0
total_power_value = 0
power_value = 0
buzz_state = 0

#=========================================================================================================

def buzzer_handling():
    global program_status
    global buzz_state
    
    if program_status == 2 and buzz_state == 0:
        buzz_state = 1
        buzzer.value(1)
    elif program_status == 2 and buzz_state == 1:
        buzz_state = 0
        buzzer.value(0)

def output_handling(program_status):
    if program_status == 0:
        output.value(1)
    else:
        output.value(0)

def global_time_inc(Timer):
    global global_time
    global_time += 1
    
def power_data(Timer):
    global power_value
    global total_power_value
    global wait_time
    global program_status
    global buzz_state
    
    wait_time += 1
    
    total_power_value = total_power_value + (power_value/3600000)
    
    if wait_time == 3600:
        wait_time = 0
        f = open('power_data.txt', 'w')
        f.write(str(total_power_value))
        f.close()
    
    buzzer_handling()
         
         
data_timer = Timer(0)    
global_timer = Timer(1)

data_timer.init(period=1000, mode=Timer.PERIODIC, callback=power_data)
global_timer.init(period=10, mode=Timer.PERIODIC, callback=global_time_inc)

#==========================================================================================================

f = open('power_data.txt')
total_power_value = f.read()
total_power_value = float(total_power_value)
f.close()

def thread0():
    global water_temperature
    global set_temperature
    global power_value
    global program_status
    
    while True:
        wdt.feed()
        
        set_temperature = change_temp_factor(set_temperature)
        water_temperature = read_ds_sensor()
        power_value = round(power_calculation())
        output_handling(program_status)
        
_thread.start_new_thread(thread0,())     
   
while True:
    wdt.feed()
    
    if global_time % 100 == 0:
        display(water_temperature, set_temperature, program_status, power_value, total_power_value)
    if global_time % 43 == 0 and program_status != 2:
        program_status = output_state(water_temperature, set_temperature)