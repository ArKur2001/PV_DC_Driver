from machine import Pin

red_output = Pin(5,Pin.OUT)
green_output = Pin(18,Pin.OUT)

program_status = 0

def status_handling(water_temperature, set_temperature):
    global program_status
    
    if water_temperature <= set_temperature - 3 and water_temperature > 9:
        program_status = 0
    elif water_temperature >= set_temperature and water_temperature < 96:
        program_status = 1
    elif water_temperature > 96:
        program_status = 2
    elif water_temperature == 0:
        program_status = 1

def output_state(water_temperature, set_temperature):
    global program_status
    
    status_handling(water_temperature, set_temperature)
    
    if program_status == 1:
        red_output.value(1)
        green_output.value(1)
    elif program_status == 2:
        red_output.value(1)
        green_output.value(0)
    else:
        red_output.value(0)
        green_output.value(1)
        
    return program_status