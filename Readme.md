# Malý dron ovládaný pomocou príkazov z IMU jednotky
Hlavným cieľom je vytvoriť "virtuálny" joystick z IMU senzora a mikročipu STM32, ktorým bude možné ovládať pohyby malého Tello drona. Dron má reagovať na pohyby, respektíve náklony ovládacej jednotky (napríklad pri náklone dopredu sa začne pohybovať dopredu). Riadiace príkazy budú z vývojovej dosky Nucleo-STM32 posielané USART zbernicou do PC. Do drona budú vysielané bezdrôtovo cez Python API.

Pre získavanie IMU dát použijeme senzorickú dosku X-NUCLEO-IKS01A1, ktorá je vybavená senzormi:
- LSM6DS0 (MEMS 3D akcelerometer + 3D gyroskop)
- LIS3MDL (MEMS 3D magnetometer)

*(Potrebu jednotlivých senzorov a snímaných dát si upresníme počas práce na zadaní.)*

## Zapojenie systému jedného "virtuálneho" joysticku
- (1ks) Nucleo-STM32-F303K8
- (1ks) Senzorická doska IKS01A1
- (2ks) Kolískové spínače (GPIO-DigitalInput)
- (1ks) PC z Python API aplikáciou
- (1ks) Tello dron

IMU dáta ako napríklad náklony v jednotlivých osiach budú snímané senzormi IMU jednotky. Nasnímané dáta budú periodicky vyčítavané cez I2C zbernicu do STM32. STM32 bude ďalej spracúvať doplnkové vstupy pre ovládanie ostatných funkcií drona ako napríklad ARM, DISARM a iné. Výstupom spracovania dát budú konkrétne príkazy a hodnoty pre ovládanie drona. Tieto dáta budú cez USART komunikáciu posielané do počítača, kde cez Python API budú preposielané cez bezdrôtové WiFi spojenie do drona.

<p align="center">
    <img src="https://github.com/hURKa-friends/VRS-TelloCopter-STM/blob/master/imgs/Zapojenie.drawio.png" width="850" title="Project layout scheme">
</p>

## Materiál potrebný pre vývoj (vypracovanie zadania)
- [x] (2ks) Nucleo-STM32-F303K8
- [x] (2ks) Senzorická doska IKS01A1
- [x] (2ks) Tello dron
- [ ] ( ks) Náhradné vrtuľky pre Tello dron

## Deľba práce - Hlavné zodpovednosti
Úlohy budeme mať rozdelené podľa potreby, ale hlavné kompetencie budú rozdelené nasledovne:
- **Denis Práznovský** - Knižnice pre komunikáciu so senzormi, Spracovanie dát, Vytvorenie USART príkazov
- **Filip Kuruc**      - Python API, USART rozhranie
- **Marek Sýkorka**    - Manažment projektu, LL I2C, USART, GPIO, NVIC, DMA, TIM, SYS, Teoretické úvahy
- **Tomáš Čvirik**     - LL I2C, USART, GPIO, atď