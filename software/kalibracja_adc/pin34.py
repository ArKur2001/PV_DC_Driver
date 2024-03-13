# Program do kalibracji przetwornika ADC w ESP32.
# W celu skalibrowania ADC należy podłączyć odpowiednio dobrany filtr RC low-pass do ESP32
# Zebrane dane znajdują się w pliku 'esp32_cal_data.csv', który należy zaimportować do arkusza google 'ESP32_ADC_callibration'
#    https://docs.google.com/spreadsheets/d/1wJlqRDmtxAtFvUfqmPdQIQIgYwRd36lFMD4gYKm5iSI/edit?usp=sharing
# Funkcję get_calibrate_data() należy zmodyfikować zgodnie z obliczonym wzorem z arkusza google
# W celu sprawdzenia wyników kalibracji użyć funkcji check_calculations() i programu 'serial_port_plotter' (funkcje muszą się zgadzać)

# *Działa dla zakresu 0-3.3V (tłumienie 11DB) oraz rozdzielczości 0-1023



from machine import Pin, PWM, ADC
from time import sleep


##----------------------------- CONFIG -----------------------------##

adc_pin = 33         #Pin przetwornika ADC -> wyjście filtru RC
pwm_pin = 23         #Pin PWM -> wejście filtru RC
pwm_freq = 10000     #Częstotliwość sygnału PWM

##------------------------------------------------------------------##


adc = ADC(Pin(adc_pin))
adc.atten(ADC.ATTN_11DB)           # Wybór tłumienia (zakresu ADC)
adc.width(ADC.WIDTH_10BIT)         # Wybór rozdzielczości ADC

pwm = PWM(Pin(pwm_pin))
pwm.freq(pwm_freq)
pwm.duty(0)


# Funkcja obliczająca różnicę na podstawie dopasowanej funkcji błędu,
# należy zmodyfikować wg. arkusza excel i przekopiować do docelowego programu :)
## -------------------------------------------------------------------------------- ##
def calculate_adc_error(adc_value):
    return (21.7
            +  (0.267 * adc_value)
            +  (-0.00103 * adc_value * adc_value)
            +  (0.00000172 * adc_value * adc_value * adc_value)
            +  (-0.00000000102 * adc_value * adc_value * adc_value * adc_value))
## -------------------------------------------------------------------------------- ##


def get_calibrate_data():
    print("Creating new data file..")
    data_file = open("esp32_cal_data.csv", "w")
    data_file.write("adc_value desired_adc_value voltage desired_voltage\n")
    data_file.close()

    print("Start saving data in 5 seconds..")
    sleep(5)

    data_file = open("esp32_cal_data.csv", "a")

    for pwm_duty in range(1024):
        pwm.duty(pwm_duty)
        
        sleep(0.05)
        
        adc_value = 0
        for i in range(20):
            adc_value = adc_value + adc.read()
        adc_value = adc_value/20
        
        voltage = adc_value*3300/1023
        
        desired_adc_value = pwm_duty
        desired_voltage = 33*pwm_duty*100/1023
        
        print("$" + str(adc_value) + " " + str(desired_adc_value) + " " + str(voltage) + " " + str(desired_voltage) + ";")       
        data_file.write(str(round(adc_value)).replace(".", ",") + " " + str(desired_adc_value).replace(".", ",") + " " + str(voltage).replace(".", ",") + " " + str(desired_voltage).replace(".", ",") + "\n")

    data_file.close()
    print("Data saved in esp32_cal_data.csv")
    
    
def check_calculations():
    sleep(10)
    for i in range(1024):
        print("$" + str(calculate_adc_error(i)) + ";")
    
    
## ---------------------------------------------- PROGRAM ---------------------------------------------- ##

#get_calibrate_data()    #Zebranie danych do pliku .csv
check_calculations()     #Sprawdzenie czy znaleziona funkcja matematyczna działa poprawnie (wymaga programu serial_port_plotter)

## ----------------------------------------------------------------------------------------------------- ##