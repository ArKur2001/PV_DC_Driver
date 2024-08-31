from machine import Pin, SoftI2C
from lcd_api import LcdApi
from i2c_lcd import I2cLcd

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=10000)     #initializing the I2C method for ESP32

I2C_ADDR = 0x27
totalRows = 4
totalColumns = 20
prstr0 = ""
prtemp = 0

lcd = I2cLcd(i2c, I2C_ADDR, totalRows, totalColumns)

def LCD_handling(row0, row1, row2, row3):
    
    global prstr0 
    
    if len(str(row0)) != len(str(prstr0)):
        lcd.clear()
        lcd.move_to(0,0)
        lcd.putstr(str(row0))
        lcd.move_to(0,1)
        lcd.putstr(str(row1))
        lcd.move_to(0,2)
        lcd.putstr(str(row2))
        lcd.move_to(0,3)
        lcd.putstr(str(row3))
        
    else:
        lcd.move_to(0,0)
        lcd.putstr(str(row0))
        lcd.move_to(0,1)
        lcd.putstr(str(row1))
        lcd.move_to(0,2)
        lcd.putstr(str(row2))
        lcd.move_to(0,3)
        lcd.putstr(str(row3))
    
    prstr0 = row0

def display(water_temperature, set_temperature, program_status, power_value, total_power_value):
    global prtemp
    
    if program_status == 0:
        program_status = "PRACA"
    elif program_status == 1:
        program_status = "STOP"
    elif program_status == 2:
        program_status = "ERROR"
    
    if prtemp != set_temperature:
        LCD_handling("Maksymalna dozwolona", "temperatura wody: "," ",str(set_temperature)+" "+chr(223)+"C    ")
    else:
        LCD_handling("Status:"+" "+str(program_status), "Temp: "+str(water_temperature)+" "+ chr(223)+"C   ", "Moc: "+str(power_value)+" W   ", "Ptot: "+str(round(total_power_value, 2))+" kWh   " )
        
    prtemp = set_temperature
