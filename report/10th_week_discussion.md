Architektúra projektu
Diskusia o štruktúre STM32 programu
Nazdar,
 
Pred tým než začneme čokoľvek programovať potrebujeme menšiu diskusiu o samotnej internej architektúre STM projektu a o tom ako to teda ideme dať dokopy. Prosím všetci sa nejak pokúste vyjadriť čo si myslíte o jednotlivých bodoch. Ďalej dúfam, že ste spokojný a súhlasíte so základným rozdelením úloh. 
 
Pre rekapituláciu:
Denis Práznovský - Knižnice pre komunikáciu so senzormi, Spracovanie dát, Vytvorenie príkazov pre USART riadenie Tello drona. 
Filip Kuruc - Python API, Vytvorenie príkazov pre USART riadenie Tello drona
Marek Sýkorka - Menežment, LL I2C implementácia, LL USART implementácia, LL GPIO, NVIC, DMA, Clock, TIMER, SYS implementácia
Tomáš Čvirik - LL I2C implementácia, LL USART implementácia, LL GPIO, NVIC, DMA, Clock, TIMER, SYS implementácia
 
# Začnime teda s diskusiou
## 1. I2C komunikácia so senzormi

Hlavnou časťou zadania je získavanie gyroskopických a akceleračných dát zo senzora/ov. "Dohodli" sme sa že pre komunikáciu so senzormi použijeme I2C, keďže jej maximálna prenosová rýchlosť bude pravdepodobne dostatočná pre získavanie všetkých dát. Teraz je otázkou ako ju implementujeme:


- Maximálny hodinový cyklus STM32F303x8 je 72 MHz čož znamená, že dokáže vykonať 72 miliónov cyklov prázdnej hlavnej slučky za sekundu alebo 72 cyklov za 1us. V závislosti od komplexnosti a dĺžky cyklov sa tieto čísla budú meniť.


- Frekvencia hodinového signálu I2C v štandardnom móde je 100kHz (0,00001s = 0,01ms = 10us). Každému hodinovému cyklu štandardne prislúcha prenos 1bitu. Preto maximálny teoretický tok dát je 100kbit/s.


- Maximálny výstupný tok dát senzora LSM6DS0 je 952Hz, teda senzor aktualizuje hodnoty X_G, Y_G, Z_G, X_L, Y_L, Z_L s frekvenciou 952Hz (0.00105s = 1,05ms = 1050us). Pre prečítanie jednej vzorky dát však potrebujeme preniesť minimálne 12Bajtov = 96bitov. Keď k tomu pripočítame adresu zariadenia, adresu registra a start/stop bity máme minimálny prenos okolo 126bitov. Ak nechceme vynechať ani jednu vzorku dát potrebujeme ich čítať s frekvenciou minimálne 952Hz (kým nebude nahradená inou vzorkou) a to zanedbávame Shannon Kotelnikov vzorkovací teorem. To znamená že pre plnohodnotné čítanie všetkých dát senzora  pri ODR = 952Hz potrebujeme prenosovú rýchlosť MINIMÁLNE 119 952bit/s !!! 


- Z tohto vyplýva že nebudeme schopný čítať dáta zo senzora v štandardnom móde I2C, ale asi len vo Fast-Mode, ktorého frekvencia hodinového signálu je 400kHz (0,0000025s = 0,0025ms = 2.5us). Maximálny teoretický tok dát by mal potom byť 400kbit/s. 


- Prosím Denis Práznovský a Tomáš Čvirik preskúmajte ešte možnosť "Burst read" senzora LSM6DS0, či nám neskráti komunikáciu tak aby sme to stíhali prenášať aj v 100kHz a taktiež preskúmajte možnosti FIFO, filtrovania apod.


- Maximálny výstupný tok dát senzora LIS3MDL , ktorý pravdepodobne nepotrebujeme je 1000Hz, teda senzor aktualizuje hodnoty X, Y, Z s frekvenciou 1000Hz (0.001s = 1ms = 1000us). Pre prečítanie jednej vzorky dát však potrebujeme preniesť minimálne 6Bajtov = 48bitov. Keď k tomu pripočítame adresu zariadenia, adresu registra a start/stop bity máme minimálny prenos okolo 67bitov. Ak nechceme vynechať ani jednu vzorku dát potrebujeme ich čítať s frekvenciou minimálne 1000Hz (kým nebude nahradená inou vzorkou) a to zanedbávame Shannon Kotelnikov vzorkovací teorem. To znamená že pre plnohodnotné čítanie všetkých dát senzora potrebujeme prenosovú rýchlosť MINIMÁLNE 67 000bit/s !!! Ak by sme to pridali k už tak do zahltenej I2C komunikácii komplet by sme nestíhali. Denis Práznovský zisti či potrebujeme magnetometrické dáta pre riadenie pitch, roll, yaw, up/down, left/right, front/back na drone.


- S týmto všetkým sa viaže to ... že ako by sme čítali tieto dáta zo strany STM. V podstate potrebujeme s frekvenciou maximálne 1000Hz teda každú 1ms prečítať dáta zo senzorov. Pri polling metóde, akú sme pravdepodobne všetci robili by komunikácia pre jeden senzor pre 126bitov trvala 0,00126s = 1,26ms pre standard mode I2C, alebo 0,000315s = 0.315ms pre fast-mode I2C. Keby sme toto čítanie dali do časoaného prerušenia tak by sme si stále nepomohli s tým, že by sme museli čakať tento čas pre prečítanie dát... Pravdepodobne použitie DMA a prerušení, kde čítané dáta sú na pozadí chrlené do nejakého buffera, by tento oblsužný čas znížilo ... avšak nemôžeme mať dĺžku 1ms vyvolávaného prerušenia väčšiu než 1ms, ideálne ani nie viac ako polovicu 1ms, v takom prípade by sa nám nič iné nevykonávalo len toto prerušenie main by bol mŕtvy ...
 
## 2. USART komunikácia s Python API

- Štandardná používaná Baudová rýchlosť je 115200baud môžeme sa rozhodnúť ju zmeniť, ale pri tejto rýchlosti vieme v konfigurácii 8bits No parity 1stop prenášať 115200bit/s to znamená že ak budeme mať na USART komunikácii 1 - 2 slovné príkazy napríklad (RO - ROLL, FW - Forward) s číselnými argumentami že o koľko sa má tá operácia vykonať o dĺžke napríklad 3desatinné miesta mali by sme asi 2+1+1+3 znakov za príkaz cca 6-10 znakov na jeden príkaz to je 48bitov na príkaz. Ak by sme príkazy posielali cyklicky s frekvenciou 1000Hz teda každú 0,001s = 1ms tak by sme prenášali pre jeden druh príkazu 48 000bit/s. Filip Kuruc a Tomáš Čvirik pokúste sa vymyslieť nejaké rozhranie USART komunikácie medzi Python API a STM. Nejaký "kvázi" znakový protokol, ktorým by sme posielali všetky potrebné príkazy do PC.
 
## 3. Spracovanie dát

- Denis Práznovský Nemám vôbec predstavu ako časovo náročné na výpočty bude spracovanie dát, ale prakticky sa musí zmestiť do voľného výpočtového času medzi komunikáciami. Pravdepodobne bude spracovanie prebiehať nad väčším množstvom dát nie len jednou vzorkou ... a môže byť trigerované od prečítania senzorických dát.

 
## Finálny oznam
Po tom ako sa dohodneme na týchto základných veciach pripravím 2 repozitáre (jeden pre Python API app a jeden pre STM) a v samostatnom príspevku tu v Teams pošlem na nich odkazy. Chcel by som aby initial commit pre STM32 už obsahoval v sebe základnú HW konfiguráciu v .ioc vygenerovanú do kódu, ale na to si potrebujeme ujasniť tieto veci v tomto príspevku.
 