#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <USBAPI.h>
#include <avr/interrupt.h>
#include <LowPower.h>

// Pin-Definitionen
const int LDR_PIN = A1;
const int POTI_PIN = A6;
const int VIN_PIN = A3;          // Neue Spannungsüberwachung
const int POWER_MESS = 9;
const int END_OFFEN = 8;
const int END_GESCHLOSSEN = 7;
const int S1a_OEFFNEN = 3;
const int S1b_SCHLIESSEN = 2;
const int MOTOR_AUF = 6;
const int MOTOR_ZU = 5;
const int MOTOR_ZU_PIPER = 10;
const int MOTOR_LED = LED_BUILTIN;

// Konstanten
const unsigned long ENTSPRELL_ZEIT = 50;
const unsigned long NACHLAUF_ZEIT = 5000;
const unsigned long MOTOR_MAX_ON = 130000;
const unsigned long USB_AKTIV_ZEIT = 300000;
const unsigned long MESSINTERVALL = 60000;
const unsigned long SCHWELLWERT = 90;
const unsigned long PIPER_ON = 100;
const unsigned long PIPER_OFF = 300;
const unsigned long VIN_CHECK_INTERVAL = 60000; // 10 Sekunden
const int PIPER_CYCLES = 18; //pipst x mal beim Schließen der Türe
const int LDR_BUFFER_SIZE = 16;
const int MIN_VIN_WERT = 550;    // Mindestspannungsschwelle  AKKU Tiefentladeschutz

// Debug-Modus 
//#define DEBUG //unkomment if Datamonitoring über USB Serieller Monitor in der Arduino IDE gewünscht ist
//#define RUNN

// Globale Variablen
enum State {
  INIT,
  MANUELL_OEFFNEN,
  MANUELL_SCHLIESSEN,
  AUTO_MODE,
  AUTO_OEFFNEN,
  AUTO_SCHLIESSEN,
  NACHLAUF,
  ERROR
};

// Ringpuffer für LDR-Werte
int ldrWerte[LDR_BUFFER_SIZE] = {0};
int ldrIndex = 0;
bool ldrBufferFull = false;

// Debouncing Variablen
unsigned long lastDebounceTimeS1a = 0;
unsigned long lastDebounceTimeS1b = 0;
unsigned long lastDebounceTimeEndOffen = 0;
unsigned long lastDebounceTimeEndGeschlossen = 0;

bool stableS1aState = HIGH;
bool stableS1bState = HIGH;
bool stableEndOffenState = HIGH;
bool stableEndGeschlossenState = HIGH;

bool lastRawS1a = HIGH;
bool lastRawS1b = HIGH;
bool lastRawEndOffen = HIGH;
bool lastRawEndGeschlossen = HIGH;

volatile bool schalterAenderung = false;
State currentState = INIT;
unsigned long usbStartZeit = 0;
bool usbAktiv = false;
int ldrMittelwert = 0;
int potiMittelwert = 0;
unsigned long motorStartZeit = 0;
unsigned long nachlaufStartZeit = 0;
unsigned long lastManualDebug = 0;
unsigned long lastVinCheck = 0;

// Piper-Variablen
bool piperActive = false;
unsigned long piperLastChange = 0;
int piperCycleCount = 0;
bool piperState = LOW;

// Funktionen
void motorenStoppen() {
  digitalWrite(MOTOR_AUF, LOW);
  digitalWrite(MOTOR_ZU, LOW);
  digitalWrite(MOTOR_ZU_PIPER, LOW);
  piperActive = false;
}

void motorenOeffnen() {
  digitalWrite(MOTOR_ZU, LOW);
  digitalWrite(MOTOR_ZU_PIPER, LOW);
  digitalWrite(MOTOR_AUF, HIGH);
  piperActive = false;
}

void startePiper() {
  piperActive = true;
  piperCycleCount = 0;
  piperState = HIGH;
  digitalWrite(MOTOR_ZU_PIPER, HIGH);
  piperLastChange = millis();
}

void updatePiper() {
  if (!piperActive) return;
  
  unsigned long currentMillis = millis();
  
  if (piperState == HIGH) {
    if (currentMillis - piperLastChange >= PIPER_ON) {
      piperState = LOW;
      digitalWrite(MOTOR_ZU_PIPER, LOW);
      piperLastChange = currentMillis;
    }
  } else {
    if (currentMillis - piperLastChange >= PIPER_OFF) {
      piperCycleCount++;
      if (piperCycleCount >= PIPER_CYCLES) {
        piperActive = false;
        digitalWrite(MOTOR_ZU_PIPER, LOW);
      } else {
        piperState = HIGH;
        digitalWrite(MOTOR_ZU_PIPER, HIGH);
        piperLastChange = currentMillis;
      }
    }
  }
}

void aktiviereUSB() {
  if (!usbAktiv) {
    usbAktiv = true;
    usbStartZeit = millis();
    USBDevice.attach();
    #ifdef DEBUG
    Serial.begin(9600);
    while (!Serial) {}
    #endif
  }
}

void deaktiviereUSB() {
  if (usbAktiv) {
    #ifdef DEBUG
    if (Serial) Serial.end();
    #else
     
    #endif
    USBDevice.detach();
    usbAktiv = false;
    
  }
   // Fügen Sie dies am Ende des setup() hinzu
//      power_twi_disable();      // I2C deaktivieren
//      power_timer1_disable();   // Timer1 (wenn nicht für PWM benötigt)
//      power_timer2_disable();   // Timer1 (wenn nicht für PWM benötigt)
      power_usart0_disable();   // Serial0 (falls nicht verwendet)
      power_spi_disable();      // SPI deaktivieren
      power_usb_disable();
}

void handleUSBTimeout() {
  if (usbAktiv) {
    #ifdef DEBUG
    unsigned long timeout = USB_AKTIV_ZEIT * 5;
    #else
    unsigned long timeout = USB_AKTIV_ZEIT;
    #endif
    
    if (millis() - usbStartZeit > timeout) {
      deaktiviereUSB();
    }
  }
}

void checkSpannung() {
  if (millis() - lastVinCheck > VIN_CHECK_INTERVAL) {
    lastVinCheck = millis();
    
    // ADC für Messung aktivieren
    power_adc_enable();
    delay(50);
    int vinValue = analogRead(VIN_PIN);
    power_adc_disable();
    
    #ifdef DEBUG
    aktiviereUSB();
    if (Serial) {
      Serial.print("VIN: ");
      Serial.println(vinValue);
    }
    #endif
    
    if (vinValue < MIN_VIN_WERT) {
      #ifdef RUNN
        currentState = ERROR;
      #endif
      //motorenStoppen();
      #ifdef DEBUG
      aktiviereUSB();
      if (Serial) Serial.println("ERROR: Spannung zu niedrig!");
      #endif
    }
  }
}

void initialSensorMessung() {
  // ADC für Messung aktivieren
  power_adc_enable();
  digitalWrite(POWER_MESS, HIGH);
  delay(50);
  
  // Potentiometer messen (3 Messungen mitteln)
  int potiSum = 0;
  for (int i = 0; i < 3; i++) {
    potiSum += analogRead(POTI_PIN);
    delay(100);
  }
  potiMittelwert = potiSum / 3;
  
  // LDR messen und Ringpuffer füllen
  long ldrSum = 0;
  for (int i = 0; i < LDR_BUFFER_SIZE; i++) {
    ldrWerte[i] = analogRead(LDR_PIN);
    ldrSum += ldrWerte[i];
    delay(20);
  }
  ldrMittelwert = ldrSum / LDR_BUFFER_SIZE;
  ldrBufferFull = true;
  ldrIndex = 0;
  
  digitalWrite(POWER_MESS, LOW);
  
  #ifdef DEBUG
  aktiviereUSB();
  if (Serial) {
    Serial.print("INIT: LDR=");
    Serial.print(ldrMittelwert);
    Serial.print(" Poti=");
    Serial.println(potiMittelwert);
  }
  #endif
}

void automatikSensorMessung() {
  power_adc_enable();
  digitalWrite(POWER_MESS, HIGH);
  delay(100);
  
  // Potentiometer messen (3 Messungen mitteln)
  int potiSum = 0;
  for (int i = 0; i < 3; i++) {
    potiSum += analogRead(POTI_PIN);
    delayMicroseconds(250);
    delay(100);
  }
  potiMittelwert = potiSum / 3;
  
  // Neuen LDR-Wert lesen und in Ringpuffer speichern
  int neuerWert = analogRead(LDR_PIN);
  ldrWerte[ldrIndex] = neuerWert;
  ldrIndex = (ldrIndex + 1) % LDR_BUFFER_SIZE;
  
  // Mittelwert berechnen
  long ldrSum = 0;
  for (int i = 0; i < LDR_BUFFER_SIZE; i++) {
    ldrSum += ldrWerte[i];
  }
  ldrMittelwert = ldrSum / LDR_BUFFER_SIZE;
  
  digitalWrite(POWER_MESS, LOW);
  power_adc_disable();
  
  #ifdef DEBUG
  aktiviereUSB();
  if (Serial) {
    Serial.print("AUTO: LDR=");
    Serial.print(ldrMittelwert);
    Serial.print(" Poti=");
    Serial.println(potiMittelwert);
    Serial.print("EndOffen: ");
    Serial.print(stableEndOffenState);
    Serial.print(" EndGeschlossen: ");
    Serial.println(stableEndGeschlossenState);
  }
  #endif
}

// Debouncing Funktion für alle Schalter
void updateSchalter() {
  // S1a entprellen
  bool reading = digitalRead(S1a_OEFFNEN);
  if (reading != lastRawS1a) {
    lastDebounceTimeS1a = millis();
  }
  if ((millis() - lastDebounceTimeS1a) > ENTSPRELL_ZEIT) {
    if (reading != stableS1aState) {
      stableS1aState = reading;
      schalterAenderung = true;
    }
  }
  lastRawS1a = reading;

  // S1b entprellen
  reading = digitalRead(S1b_SCHLIESSEN);
  if (reading != lastRawS1b) {
    lastDebounceTimeS1b = millis();
  }
  if ((millis() - lastDebounceTimeS1b) > ENTSPRELL_ZEIT) {
    if (reading != stableS1bState) {
      stableS1bState = reading;
      schalterAenderung = true;
    }
  }
  lastRawS1b = reading;

  // Endschalter OFFEN entprellen
  reading = digitalRead(END_OFFEN);
  if (reading != lastRawEndOffen) {
    lastDebounceTimeEndOffen = millis();
  }
  if ((millis() - lastDebounceTimeEndOffen) > ENTSPRELL_ZEIT) {
    if (reading != stableEndOffenState) {
      stableEndOffenState = reading;
    }
  }
  lastRawEndOffen = reading;

  // Endschalter GESCHLOSSEN entprellen
  reading = digitalRead(END_GESCHLOSSEN);
  if (reading != lastRawEndGeschlossen) {
    lastDebounceTimeEndGeschlossen = millis();
  }
  if ((millis() - lastDebounceTimeEndGeschlossen) > ENTSPRELL_ZEIT) {
    if (reading != stableEndGeschlossenState) {
      stableEndGeschlossenState = reading;
    }
  }
  lastRawEndGeschlossen = reading;
}

// Interrupt Service Routinen
void schalterISR0() { schalterAenderung = true; }
void schalterISR1() { schalterAenderung = true; }

void setup() {
  // Pinmodes setzen
  pinMode(LDR_PIN, INPUT);
  pinMode(POTI_PIN, INPUT);
  pinMode(VIN_PIN, INPUT);  // Spannungsmessung
  pinMode(POWER_MESS, OUTPUT);
  digitalWrite(POWER_MESS, LOW);
  
  pinMode(END_OFFEN, INPUT_PULLUP);
  pinMode(END_GESCHLOSSEN, INPUT_PULLUP);
  pinMode(S1a_OEFFNEN, INPUT_PULLUP);
  pinMode(S1b_SCHLIESSEN, INPUT_PULLUP);
  
  pinMode(MOTOR_AUF, OUTPUT);
  pinMode(MOTOR_ZU, OUTPUT);
  pinMode(MOTOR_ZU_PIPER, OUTPUT);
  pinMode(MOTOR_LED, OUTPUT);
  
  motorenStoppen();
  
  // Interrupts konfigurieren
  attachInterrupt(digitalPinToInterrupt(S1a_OEFFNEN), schalterISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(S1b_SCHLIESSEN), schalterISR1, CHANGE);

  
  //analogReference(INTERNAL); // 1.1V Referenz verwenden
  analogReference(DEFAULT); // 5V Referenz verwenden
  // Initiale Schalterwerte lesen
  updateSchalter();
  
     power_twi_disable();      // I2C deaktivieren
    power_timer1_disable();   // Timer1 (wenn nicht für PWM benötigt)
    power_timer2_disable();   // Timer1 (wenn nicht für PWM benötigt)
  #ifdef DEBUG
  // USB initialisieren
  aktiviereUSB();
  if (Serial) {
    Serial.println("Systemstart");
    Serial.print("EndOffen: ");
    Serial.println(stableEndOffenState);
    Serial.print("EndGeschlossen: ");
    Serial.println(stableEndGeschlossenState);
    Serial.print("Startzustand: ");
    Serial.println(currentState);
  }
  #else
    delay(10000); // time at startup for programming only 
    USBDevice.detach();
    usbAktiv = false;
    
    // Fügen Sie dies am Ende des setup() hinzu
    power_twi_disable();      // I2C deaktivieren
    power_timer1_disable();   // Timer1 (wenn nicht für PWM benötigt)
    power_timer2_disable();   // Timer1 (wenn nicht für PWM benötigt)
    power_usart0_disable();   // Serial0 (falls nicht verwendet)
    power_spi_disable();      // SPI deaktivieren
    power_usb_disable();
  #endif
  delay(10000);
}

void loop() {
  // Spannungsüberwachung (in jedem Zustand)
  checkSpannung();
  
  // Schalterzustände aktualisieren
  updateSchalter();
  
  // Piper steuern
  updatePiper();
  
  // USB-Timeout verwalten
  handleUSBTimeout();
  
  #ifdef DEBUG
  static State lastState = INIT;
  if (currentState != lastState) {
    aktiviereUSB();
    if (Serial) {
      Serial.print("Zustandswechsel: ");
      Serial.print(lastState);
      Serial.print(" -> ");
      Serial.println(currentState);
    }
    lastState = currentState;
  }
  #endif

  // Zustandsmaschine
  switch (currentState) {
    case INIT:
      #ifdef DEBUG
      aktiviereUSB();
      if (Serial) Serial.println("INIT: Starte Initialisierung");
      #endif
      
      initialSensorMessung();
      
      // Endschalter prüfen
      if (stableEndOffenState == LOW) {
        #ifdef DEBUG
        if (Serial) Serial.println("INIT: Tür ist offen -> AUTO_MODE");
        #endif
        currentState = AUTO_MODE;
      }
      else if (stableEndGeschlossenState == LOW) {
        #ifdef DEBUG
        if (Serial) Serial.println("INIT: Tür ist geschlossen -> AUTO_MODE");
        #endif
        currentState = AUTO_MODE;
      }
      else {
        #ifdef DEBUG
        if (Serial) Serial.println("INIT: Tür in Mittelposition -> Öffnen");
        #endif
        currentState = AUTO_OEFFNEN;
        motorStartZeit = millis();
      }
      break;
      
    case AUTO_MODE:
      motorenStoppen();
      motorStartZeit = 0;
      // ATmega32U4
      //  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
      //        TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
//      #ifdef DEBUG
//        LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,  TIMER0_ON, SPI_OFF, USART1_ON, TWI_OFF, USB_ON);
//      #else
//        LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,  TIMER0_ON, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
//      #endif
      
      if (stableEndOffenState == HIGH && stableEndGeschlossenState == HIGH) {
        // Tür ist weder ganz offen noch ganz geschlossen
        if (stableS1aState == LOW) {
          currentState = MANUELL_OEFFNEN;
          #ifdef DEBUG
          aktiviereUSB();
          if (Serial) Serial.println("MANUELL_OEFFNEN: Start");
          #endif
        } 
        else if (stableS1bState == LOW) {
          currentState = MANUELL_SCHLIESSEN;
           #ifdef DEBUG
          aktiviereUSB();
          if (Serial) Serial.println("MANUELL_SCHLIESSEN: Start");
          #endif
        }
        
      }
      // Manueller Modus hat Priorität
      if (stableS1aState == LOW && stableEndOffenState == HIGH) {
        currentState = MANUELL_OEFFNEN;
        motorStartZeit = millis();
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("MANUELL_OEFFNEN: Start");
        #endif
      } 
      if (stableS1bState == LOW && stableEndGeschlossenState == HIGH) {
        currentState = MANUELL_SCHLIESSEN;
        motorStartZeit = millis();
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("MANUELL_SCHLIESSEN: Start");
        #endif
      }
      if (stableS1aState == HIGH && stableS1bState == HIGH) {
      
        static unsigned long lastMeasure = 0;
        if (millis() - lastMeasure > MESSINTERVALL) {
          automatikSensorMessung();
          lastMeasure = millis();
          
          if (ldrMittelwert > potiMittelwert + SCHWELLWERT) {
            if (stableEndOffenState == HIGH) { // Nur öffnen wenn nicht bereits offen
              currentState = AUTO_OEFFNEN;
              motorStartZeit = millis();
              #ifdef DEBUG
              aktiviereUSB();
              if (Serial) Serial.println("AUTO_OEFFNEN: Start (Helligkeit)");
              #endif
            }
          } 
          else if (ldrMittelwert < potiMittelwert) {
            if (stableEndGeschlossenState == HIGH) { // Nur schließen wenn nicht bereits geschlossen
              currentState = AUTO_SCHLIESSEN;
              motorStartZeit = millis();
              #ifdef DEBUG
              aktiviereUSB();
              if (Serial) Serial.println("AUTO_SCHLIESSEN: Start (Helligkeit)");
              #endif
            }
          }
        }
      }
      break;
      
    case MANUELL_OEFFNEN:
      // Debug-Ausgabe nur alle 60 Sekunden
      #ifdef DEBUG
        if (millis() - lastManualDebug > MESSINTERVALL) {
          aktiviereUSB();
          if (Serial) Serial.println("MANUELL_OEFFNEN: Aktiv");
          lastManualDebug = millis();
        }
      #endif
      
      // Tür ist bereits offen
      if (stableEndOffenState == LOW) {
        motorenStoppen();
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("MANUELL_OEFFNEN: Tür offen - Stop");
        #endif
        
        // Warte auf Freigabe beider Schalter
        //if (stableS1aState == HIGH && stableS1bState == HIGH) {
          currentState = AUTO_MODE;
          #ifdef DEBUG
          if (Serial) Serial.println("MANUELL_OEFFNEN: Wechsel zu AUTO_MODE");
          #endif
        //}
      } 
      // Tür öffnen
      else if (stableEndOffenState == HIGH){
        motorenOeffnen();
        
        // Zeitüberwachung
        if (millis() - motorStartZeit > MOTOR_MAX_ON) {
          currentState = ERROR;
          #ifdef DEBUG
          aktiviereUSB();
          if (Serial) Serial.println("ERROR: Motorlaufzeit überschritten (Öffnen)");
          #endif
        }
      }
//        #ifdef DEBUG
//        LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,  TIMER0_ON, SPI_OFF, USART1_ON, TWI_OFF, USB_ON);
//        #else
//        LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,  TIMER0_ON, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
//        //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); 
//        #endif

      break;
      
    case MANUELL_SCHLIESSEN:
      // Debug-Ausgabe nur alle 60 Sekunden
      #ifdef DEBUG
      if (millis() - lastManualDebug > MESSINTERVALL) {
        aktiviereUSB();
        if (Serial) Serial.println("MANUELL_SCHLIESSEN: Aktiv");
        lastManualDebug = millis();
      }
      #endif
      
      // Tür ist bereits geschlossen
      if (stableEndGeschlossenState == LOW) {
        motorenStoppen();
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("MANUELL_SCHLIESSEN: Tür geschlossen - Stop");
        #endif
        
        // Starte Nachlauf
        currentState = NACHLAUF;
        nachlaufStartZeit = millis();
      } 
      // Tür schließen
      else {
        digitalWrite(MOTOR_ZU, HIGH); // Hauptmotor läuft weiter
        digitalWrite(MOTOR_AUF, LOW);
        
        // Zeitüberwachung
        if (millis() - motorStartZeit > MOTOR_MAX_ON) {
          currentState = ERROR;
          #ifdef DEBUG
          aktiviereUSB();
          if (Serial) Serial.println("ERROR: Motorlaufzeit überschritten (Schließen)");
          #endif
        }
      }
      break;
      
    case AUTO_OEFFNEN:
      // Tür ist bereits offen
      if (stableEndOffenState == LOW) {
        motorenStoppen();
        currentState = AUTO_MODE;
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("AUTO_OEFFNEN: Tür offen - Wechsel zu AUTO_MODE");
        #endif
      } 
      // Tür öffnen
      else {
        motorenOeffnen();
        
        // Zeitüberwachung
        if (millis() - motorStartZeit > MOTOR_MAX_ON) {
          currentState = ERROR;
          #ifdef DEBUG
          aktiviereUSB();
          if (Serial) Serial.println("ERROR: Motorlaufzeit überschritten (Auto-Öffnen)");
          #endif
        }
      }
      break;
      
    case AUTO_SCHLIESSEN:
      // Piper nur beim Start des Schließvorgangs aktivieren
      static bool piperStarted = false;
      if (!piperStarted) {
        startePiper();
        piperStarted = true;
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("AUTO_SCHLIESSEN: Piper gestartet");
        #endif
      }
      
      // Tür ist bereits geschlossen
      if (stableEndGeschlossenState == LOW) {
        piperStarted = false;  // Zurücksetzen für nächsten Zyklus
        // Starte Nachlauf
        currentState = NACHLAUF;
        nachlaufStartZeit = millis();
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("AUTO_SCHLIESSEN: Tür geschlossen - Starte Nachlauf");
        #endif
      } 
      // Tür schließen
      else {
        digitalWrite(MOTOR_ZU, HIGH); // Hauptmotor läuft weiter
        digitalWrite(MOTOR_AUF, LOW);
        
        // Zeitüberwachung
        if (millis() - motorStartZeit > MOTOR_MAX_ON) {
          piperStarted = false;  // Zurücksetzen für nächsten Zyklus
          currentState = ERROR;
          #ifdef DEBUG
          aktiviereUSB();
          if (Serial) Serial.println("ERROR: Motorlaufzeit überschritten (Auto-Schließen)");
          #endif
        }
      }
      break;
      
    case NACHLAUF:
      digitalWrite(MOTOR_ZU, HIGH); // Hauptmotor läuft weiter
      digitalWrite(MOTOR_AUF, LOW);
      
      // Nachlaufzeit abgelaufen
      if (millis() - nachlaufStartZeit > NACHLAUF_ZEIT) {
        digitalWrite(MOTOR_ZU_PIPER, HIGH);
        delay(100);
        digitalWrite(MOTOR_ZU_PIPER, LOW);
        motorenStoppen();
        currentState = AUTO_MODE;
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("NACHLAUF: Beendet - Wechsel zu AUTO_MODE");
        #endif
      }
      // Maximale Laufzeit
      else if (millis() - motorStartZeit > MOTOR_MAX_ON) {
        currentState = ERROR;
        #ifdef DEBUG
        aktiviereUSB();
        if (Serial) Serial.println("ERROR: Motorlaufzeit überschritten (Nachlauf)");
        #endif
      }
      break;
      
    case ERROR:
      motorenStoppen();
      digitalWrite(MOTOR_LED, !digitalRead(MOTOR_LED));
      delay(100);
        #ifdef DEBUG
        LowPower.idle(SLEEP_500MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,  TIMER0_ON, SPI_OFF, USART1_ON, TWI_OFF, USB_ON);
        #else
        LowPower.idle(SLEEP_500MS, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF,  TIMER0_ON, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
        //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // funktioniert nicht weil der WDT einen Neustart auslößt und somit die LDR Array ... Daten fehlen
        #endif

      
      break;
  }
  
  // Kurze Verzögerung zur Entlastung des Prozessors
  delay(10);
}
