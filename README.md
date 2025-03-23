# Lora APRS Repeater

Projekt **Lora APRS Repeater** bazuje na mikrokontrolerze **STM32L072xx** i wykorzystuje **dwa moduły RFM96W-433S2**.
Celem jest nasłuchiwanie sygnałów APRS na częstotliwości **434.855.000 kHz** i retransmisja odebranych ramek na częstotliwości **433.775.000 kHz**.

---

## Opis projektu

- **Mikrokontroler**: STM32L072xx  
- **Moduły radiowe**: 2 × RFM96W-433S2 (LoRa w paśmie 433 MHz)  
- **Częstotliwość odbioru**: 434.855.000 kHz (nasłuch ramek APRS)  
- **Częstotliwość nadawania**: 433.775.000 kHz (retransmisja odebranych ramek)  

Projekt jest w fazie projektowania płytki PCB. Pierwsza wersja płytki zostanie wykonana w najbliższym czasie.  
Oprogramowanie (firmware) będzie rozwijane po przygotowaniu i przetestowaniu prototypu sprzętu.

---

## Struktura repozytorium

W repozytorium znajdują (lub będą się znajdować) następujące katalogi:

- `hardware/` – pliki związane z projektem płytki PCB, schematami, dokumentacją techniczną.  
- `firmware/` – źródła oprogramowania dla STM32L072xx, konfiguracje LoRa i obsługa komunikacji APRS.  
- `docs/` – dodatkowa dokumentacja (instrukcje, opisy protokołów, linki do materiałów zewnętrznych).

---

## Wymagania sprzętowe

1. **Płytka bazowa**: z mikrokontrolerem STM32L072xx i dwoma modułami RFM96W-433S2.  
2. **Zasilanie**:  
   - Panel fotowoltaiczny **20 W**  
   - Akumulator Li-Ion 18650 (4 sztuki)  
   - Moduł zasilania z **MPPT** do ładowania akumulatorów  
3. **Antena**: odpowiednio dostrojona dla częstotliwości ~433 MHz.  
4. **Dodatkowe podzespoły**: ewentualne przetwornice, stabilizatory dla mikrokontrolera (np. 3.3 V).

---

## Wymagania programowe

- **Kompilator i środowisko**: Zalecane użycie narzędzi ARM-GCC w wersji obsługującej STM32L0, np. `arm-none-eabi-gcc`.  
- **Biblioteki i frameworki**:  
  - STM32CubeL0 (HAL/LL)  
  - ewentualnie platforma [PlatformIO](https://platformio.org/) do zarządzania projektem.  
- **Programator**: ST-Link (wersja 2 lub nowsza).

---

## Uruchomienie i kompilacja (w fazie planowania)

1. **Sklonuj repozytorium**:
   ```bash
   git clone https://github.com/twoja-nazwa-uzytkownika/Lora-APRS-Repeater.git
   ```
2. **Przejdź do katalogu projektu**:
   ```bash
   cd Lora-APRS-Repeater/firmware
3. **Zbuduj projekt (w zależności od używanego środowiska, np. Makefile):**
4. **Wgraj wsad na mikrokontroler przy użyciu ST-Link lub innego programatora.**

---

## Planowane funkcjonalności

Odbiór ramek APRS na 434.855.000 kHz z wykorzystaniem modułu RFM96W.
Parsowanie i buforowanie odbieranych ramek w pamięci mikrokontrolera.
Retransmisja ramek na 433.775.000 kHz za pomocą drugiego modułu RFM96W.
Możliwa konfiguracja i diagnostyka przez interfejs szeregowy (UART) bądź interfejs USB (w zależności od wersji sprzętu).
Tryb oszczędzania energii (low-power) w celu dłuższej pracy w terenie na zasilaniu akumulatorowym.
Zasilanie solarne – obsługa ładowania akumulatora przez MPPT, monitorowanie stanu baterii.

---

## Status projektu

Faza: Projektowanie płytki PCB.
Kolejne kroki:
    Zamówienie i montaż prototypu PCB.
    Weryfikacja działania hardware’u.
    Implementacja oprogramowania na STM32L072xx.
    Testy laboratoryjne i terenowe retransmisji APRS.

---

## Wsparcie i kontakt

Jeśli chcesz się zaangażować lub masz pytania:

    Zgłaszaj issues bezpośrednio w repozytorium (zgłaszanie błędów, pomysłów).
    Zapraszamy do pull requestów z poprawkami i usprawnieniami.
    Dodatkowy kontakt: waperr@interia.pl.

---

## Licencja

Projekt udostępniany jest na warunkach licencji MIT.
Zapoznaj się z treścią pliku LICENSE w repozytorium, aby poznać szczegóły.

---

Dziękuję za zainteresowanie projektem Lora APRS Repeater!
Zachęcam do śledzenia postępów i aktywnego udziału w rozwoju projektu.
