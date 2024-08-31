# PV_DC_driver  

Sterownik do instalacji PV ogrzewającej zasobnik CWU - zarys teoretyczny bez wyprowadzeń wzorów

### Funkcjonalności

- Sterowanie grzałką (włącz / wyłacz) --> MPPT PWM
- Pomiar napięcia
  - na panelu PV
  - na grzałce
- Pomiar prądu
  - na grzałce
- Pomiar mocy
  - obliczony z napięcia i prądu grzałki
- Pomiar temperatury
  - wody w zasobniku CWU
  - w obudowie sterownika / radiatora
- Sterowanie
  - z poziomu sterownika -> wyświetlacz i przyciski
  - przez sieć LAN - strona www lub aplikacja
- Zapis i wizualizacja danych na serwerze

### MPPT Driver 1.0

Progam ma za zadanie poszukiwanie w czasie rzeczywistym maksymalnego punktu pracy paneli fotowoltaicznych
w celu uzyskania jak najwyższej mocy na grzałce w zasobniku wody, włączanie i wyłączanie grzałki i kilka 
mniejszych funkcjonalności. Napisany zostanie w języku Python.

- Wymagania względem programu:
  - pomiar temperatury wody w zasobniku
  - pomiar napięcia
  - pomiar prądu
  - obsługa PWM(MPPT)
  - obsługa przycisków(zmniejszanie i zwiękasznie dozwolonej temperatury wody w zasobniku)
  - odsługa wyświetlacza(1[Status], 2[Temperatura], 3[Moc], 4[Całkowita moc])
  - obsługa status_LEDa
  - obsługa błędów
  - obsługa buzzera(ALARM)
  - zapis do pamięci sumy dostarczonej energii(kWh)

- Szczegółowe wymagania:
  - gdy temperatura w zbiorniku osiągnie zadaną, program przechodzi w stan "STOP"
  - temperatura bezpieczna <95 *C, powyżej 95 *C uruchamia się alarm
  - możliwość ustawiania temperatury do 95 *C
  - zapis do pamięci całkowitej mocy odbywa się w interwałąch czasu(raz na godzinę(w celu ochrony pamięci przed degradacją)) 
  - status programu[0=PRACA, 1=STOP, 2=ERROR]
  - kolor status_LEDa[zielony=PRACA, żółty=STOP, czerwony=ERROR]
  - PRACA program działa normalnie 
  - STOP została przekroczona temperatura, współczynnik wypełnienia wynosi 0
  - ERROR zatrzymuje program całkowicie, wymagany będzie reset
  - błędy powodujące wejście w stan ERROR:
    - napięcie wynosi 0, ale płynie prąd
    - napięcie > 0, współczynnik wypełnienia != 0, nie płynie prąd
    - temperatura wynosi więcej niż 99*C
    - i coś tam jeszcze pewnie po drodze wpadnie
	
Struktura programu opiera się na wykorzystaniu dwóch rdzeni dostępnych w mikrokontrolerze esp32, 
rdzeń0[thread0](MPPT, przyciski, pomiary, zmiana temperatury), rdzeń1[thread1](wyświetlacz, status_LED,
status programu, obsługa błędów, zapis do pamięci, buzzer). Rdzenie będą komunikować się między sobą za pomocą 
zmiennych globalnych. Nad wszystkim będzie czuwał timer odpowiadający za globalny czas programu. 
Wykorzystany zostanie Watchdog.

- Zmienne globalne:
  - fill_value
  - voltage_value
  - current_value
  - power_value
  - water_temperature
  - set_tempereture
  - global_time
  - program_status_flag

- Funkcje(zdefiniowane w modułach):  
  - temperature_handling() => obsługa przycisków od zmiany temperatury (w zakersie 20-99*C), odczyt temperatury
  - power_calculation() => pomiar napięcia, prądu, mocy
  - MPPT_algorithm() => obliczanie MPPT
  - PWM_output() => sterowanie współczynnikiem wypełnienia dla tranzystora
  - lcd.move_to() => zmiana linijki wyświetlacza
  - lcd.putstr() => dane do wyświetlenia
  - power_data() => obliczanie i zapis całkowitej mocy do pamięci
  - buzzer() => obsługa alarmu 
  - program_status() => sprawdzanie błędów i ustawianie statusu
  - status_LED() => obsługa status_LEDa

Struktura:
```python
===================================main.py====================================
===================================IMPORT=====================================
import...

===============================DEFINICJE_PINÓW================================
np. adc

========================INICJALIZACJA_ZMIENNYCH_GLOBALNYCH====================
global fill_value
global voltage_value
global current_value
global power_value
global water_temperature
global set_tempereture
global global_time
global program_status_flag

=========================INICJALIZACJA_THREADINGU,_TIMER======================
lock0 = _thread.allocate_lock()
lock1 = _thread.allocate_lock()

global_time_inc(Timer)

TIMER

WATCHDOG
==================================THREADING===================================
def thread0():			
    while True:
      global
      lock0.acquire()
      temperature_handling()
      power_calculation()
      MPPT_algorithm()
      lock0.release()

def thread1():
    while True:
      global
      lock1.acquire()
      lcd.move_to()
      lcd.putstr()
      power_data()
      buzzer()
      program_status()
      status_LED()
      lock1.release()

==============================URUCHOMINIE_RDZENI==============================
_thread.start_new_thread(thread0, ())
_thread.start_new_thread(thread1, ())

===================================main.py====================================
```