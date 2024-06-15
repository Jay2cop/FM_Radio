#include <Wire.h>
#include "DHT.h"
#include <EEPROM.h>
#include <RTClib.h>
#include <Rtc_Pcf8563.h>
#include <TimeLib.h>
#include <Encoder.h>
#include "SparkFunHTU21D.h"
#include <LiquidCrystal.h>
#include "SparkFunSi4703.h"
#include <IRremote.h>

HTU21D sensorHTU21D;

#define IR_RECEIVE_PIN A1
#define IR_BUTTON_1 12
#define IR_BUTTON_2 24
#define IR_BUTTON_3 94
#define IR_BUTTON_4 8
#define IR_BUTTON_5 28
#define IR_BUTTON_VolUp 70
#define IR_BUTTON_VolDown 21
#define IR_BUTTON_Next 67
#define IR_BUTTON_Prev 68

#define SWPIN 2
#define ENCPIN1 4
#define ENCPIN2 3
#define BTN1PIN A6
#define BTN2PIN A7
#define I2C_ADDR 0x51
#define PCF8563_I2C_ADDRESS 0x51
#define FAV_STATIONS_COUNT 5
#define FAV_STATIONS_EEPROM_ADDR 10
#define TRIG_PIN_L 11 //TRIGGER PIN (LEFT PROXIMITY SENSOR)
#define ECHO_PIN_L 12 //ECHO PIN (LEFT PROXIMITY SENSOR)
#define ECHO_PIN_R A3 //ECHO PIN (RIGHT PROXIMITY SENSOR)
#define TRIG_PIN_R A2 //TRIG PIN (RIGHT PROXIMITY SENSOR)
#define MAX_DISTANCE 300
Encoder myEncoder(ENCPIN1, ENCPIN2);

LiquidCrystal lcd(5, 6, 7, 8, 9, 10);

Rtc_Pcf8563 rtc;
RTC_PCF8563 RTC;

unsigned long lastDebounceTime = 0;
struct AlarmData {
  uint8_t enabled : 1;
  uint8_t hour;
  uint8_t minute;
  unsigned long time;
  uint8_t triggered : 1;
};

AlarmData alarmData;

unsigned long buttonState1 = 0;
unsigned long buttonState2 = 0;
uint8_t resetPin = A0;
uint8_t SDIO = A4;
uint8_t SCLK = A5;

Si4703_Breakout radio(resetPin, SDIO, SCLK);

float channel;
uint8_t volume;
float currentChannel = 0.0;
uint8_t currentVolume = 1;
char rdsBuffer[10];

struct Flags {
  uint8_t temperatureDisplayed : 1;
  uint8_t timeDisplayed : 1;
  uint8_t greetingDisplayed : 1;
  uint8_t alarmDisplayed : 1;
  uint8_t inDateTimeSetting : 1;
  uint8_t alarmSetting : 1;
  uint8_t menuShown : 1;
  uint8_t alarmDisplay : 1;
  uint8_t alarmState : 1;
  uint8_t enteringFmRadioMenu : 1;
  uint8_t radioSettingsApplied : 1;
  uint8_t newRDSDataAvailable : 1;
};

Flags flags;
enum MenuState { HIDDEN, SHOW_MENU, SHOW_ALARM_MENU, SET_ALARM, FM_RADIO_SEEK, FM_RADIO_MENU, RESET, BACK, FM_RADIO_SEEK_MANUAL, 
  SHOW_SAVED_STATIONS_MENU, SAVE_FAV_STATION_MENU};

const char* menuItems[] = {
  "FM Radio",
  "Set Time/Date",
  "Set Alarm",
  "Reset",
  "Back"
};

const char* fmRadioMenuItems[] = {
  "Auto-seek",
  "Seek up/down",
  "Saved stations",
  "back"
};

const char *savedStationsMenuItems[] = {
  "Fav1",
  "Fav2", 
  "Fav3", 
  "Fav4", 
  "Fav5", 
  "Back"
};

const int MENU_ITEM_COUNT = sizeof(menuItems) / sizeof(menuItems[0]);
int prevSelectedItem = -1;
MenuState prevMenuState = HIDDEN;
MenuState currentMenuState = HIDDEN;
static int prevAlarmMenuState = HIDDEN;
const int FM_RADIO_MENU_ITEM_COUNT = sizeof(fmRadioMenuItems) / sizeof(fmRadioMenuItems[0]);
int prevFmRadioSelectedItem = -1;
MenuState prevFmRadioMenuState = HIDDEN;
int fmRadioSelectedItem = 1;
int manualTuningFrequency = 875;
int selectedFavoriteIndex = 0;
int currentFavoriteIndex = 0;
float favoriteStations[FAV_STATIONS_COUNT] = {0, 0, 0, 0, 0};
byte saveFavStationMenuState = HIDDEN;
byte savedStationsMenuState = HIDDEN;
byte saveFavStationSelectedItem = 1;
byte prevSaveFavStationMenuState = HIDDEN;
byte savedStationsSelectedItem = 1;
byte prevSavedStationsMenuState = HIDDEN;
int prevSaveFavStationSelectedItem = -1;
int prevSavedStationsSelectedItem = -1;
const int SAVED_STATIONS_MENU_ITEM_COUNT = sizeof(savedStationsMenuItems) / sizeof(savedStationsMenuItems[0]);
unsigned long lastRDSReadTime = 0;
const unsigned long RDS_READ_INTERVAL = 100;
const unsigned long RDS_READ_TIMEOUT = 20;
unsigned long rdsUpdateInterval = 5000; // Time between RDS updates in milliseconds
unsigned long lastRdsUpdateTime = 0;
bool newRDSDataAvailable = false;
int RightEcho = analogRead(ECHO_PIN_R);
int RightTrig = analogRead(TRIG_PIN_R);
float time_L, distance_L, time_R, distance_R;
unsigned long timeInPosition_L = 0, totalTime_L = 0;
bool inPosition_L = false;
unsigned long timeInPosition_R = 0, totalTime_R = 0;
bool inPosition_R = false;
static bool volumeDisplayed = false;
float actDist = 10.0;
int currentFavIndex = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  sensorHTU21D.begin();
  lcd.begin(16, 2);
  displayGreetings();
  currentFavIndex = 0; // Set the current favorite station index to 0 (first station)
  float fav1 = favoriteStations[0]; // Access the first favorite station directly
  radio.powerOn();
  setChannelAndVolume(fav1, 1);
  delay(200);
  
  pinMode(TRIG_PIN_L, OUTPUT);
  pinMode(ECHO_PIN_L, INPUT);
  pinMode(TRIG_PIN_R, OUTPUT);
  pinMode(ECHO_PIN_R, INPUT);
  pinMode(IR_RECEIVE_PIN, INPUT);
  digitalWrite(IR_RECEIVE_PIN, HIGH);
  IrReceiver.begin(IR_RECEIVE_PIN);
  pinMode(SWPIN, INPUT_PULLUP);
  pinMode(ENCPIN1, INPUT_PULLUP);
  pinMode(ENCPIN2, INPUT_PULLUP);
  alarmData.time = millis();
}

void loop() {
  long newPosition = myEncoder.read();
  static int selectedItem = 1;
  static int alarmMenuSelectedItem = 0;
  static int prevAlarmMenuSelectedItem = 0;
  static int prevSavedStationsMenuState = 0;
  static int prevSaveFavStationMenuState = 0;
  static int savedStationsSelectedItem = 0;
  static int saveFavStationSelectedItem = 0;

  static unsigned long lastUpdateTime = 0;
  unsigned long currentTime = millis();

  unsigned long debounceDelay = 500;
  int debouncedState1 = debounce_analogue(BTN1PIN, buttonState1, debounceDelay);
  int debouncedState2 = debounce_analogue(BTN2PIN, buttonState2, debounceDelay);

  static bool button1Released = true;
  static bool button2Released = true;
  myEncoder.write(0);
  static long encAccumulator = 0;
  encAccumulator += newPosition;
  
  if (abs(encAccumulator) >= 2) {
    int updateValue = (encAccumulator > 0) ? 1 : -1;
    encAccumulator -= updateValue * 2;

    if (currentMenuState == SHOW_MENU || currentMenuState == HIDDEN) {
      selectedItem += updateValue;
      // Keep selectedItem within valid range
      selectedItem = max(1, min(MENU_ITEM_COUNT, selectedItem));
    } else if (currentMenuState == SHOW_ALARM_MENU) {
      alarmMenuSelectedItem += updateValue;
      alarmMenuSelectedItem = max(0, min(1, alarmMenuSelectedItem));
    } else if (currentMenuState == FM_RADIO_MENU) {
      fmRadioSelectedItem += updateValue;
      fmRadioSelectedItem = max(1, min(FM_RADIO_MENU_ITEM_COUNT, fmRadioSelectedItem));
    } else if (currentMenuState == FM_RADIO_SEEK_MANUAL) { // Add this condition to handle manual tuning
      manualTuningFrequency += updateValue;
      manualTuningFrequency = max(875, min(1080, manualTuningFrequency)); // Limit the frequency range between 87.5MHz and 108.0MHz
      channel = manualTuningFrequency;
    } else if (currentMenuState == SAVE_FAV_STATION_MENU) {
        saveFavStationSelectedItem += updateValue;
        saveFavStationSelectedItem = max(1, min(SAVED_STATIONS_MENU_ITEM_COUNT, saveFavStationSelectedItem));
    } else if (currentMenuState == SHOW_SAVED_STATIONS_MENU) {
        savedStationsSelectedItem += updateValue;
        savedStationsSelectedItem = max(1, min(SAVED_STATIONS_MENU_ITEM_COUNT, savedStationsSelectedItem));
    }
  }
  // Left proximity sensor
  /*digitalWrite(TRIG_PIN_L, LOW);
  delay(50);
  digitalWrite(TRIG_PIN_L, HIGH);
  delay(150);
  digitalWrite(TRIG_PIN_L, LOW);
  time_L = pulseIn(ECHO_PIN_L, HIGH);
  distance_L = (time_L * 0.0340) / 2;
  totalTime_L = millis() - timeInPosition_L;

  if (distance_L <= actDist) {
    if (!inPosition_L) {
      timeInPosition_L = millis();
    }
    inPosition_L = true;

    if (totalTime_L > 1500 && totalTime_L < 10000) {
      
      if (currentVolume < 10) {
        currentVolume++;
      }
      // Implement the code to change the volume of your audio device here
      lcd.clear();
      lcd.print("Volume +");
      lcd.setCursor(0,1);
      lcd.print("Volume: ");
      lcd.print(currentVolume);
      delay(500);
      lcd.clear();
    }
  } else if (distance_L > actDist && inPosition_L) {
    if (totalTime_L > 200 && totalTime_L < 1000) {
      // Next station
      currentFavIndex++;
      loadFavoriteStationsFromEEPROM(currentFavIndex);
    }
    inPosition_L = false;
    timeInPosition_L = 0;
    delay(500);
    displayGreetings();
  }

  // Right proximity sensor
  digitalWrite(TRIG_PIN_R, LOW);
  delay(50);
  digitalWrite(TRIG_PIN_R, HIGH);
  delay(150);
  digitalWrite(TRIG_PIN_R, LOW);
  time_R = pulseIn(ECHO_PIN_R, HIGH);
  distance_R = (time_R * 0.0340) / 2;
  totalTime_R = millis() - timeInPosition_R;

  if (distance_R <= actDist) {
    if (!inPosition_R) {
      timeInPosition_R = millis();
    }
    inPosition_R = true;

    if (totalTime_R > 1500 && totalTime_R < 10000) {
      if (currentVolume > 0) {
        currentVolume--;
      }
      // Implement the code to change the volume of your audio device here
      lcd.clear();
      lcd.print("Volume -");
      lcd.setCursor(0,1);
      lcd.print("Volume: ");
      lcd.print(currentVolume);
      delay(500);
      lcd.clear();
    }
  } else if (distance_R > actDist && inPosition_R) {
    if (totalTime_R > 200 && totalTime_R < 1000) {
      // Previous station
      currentFavIndex++;
      loadFavoriteStationsFromEEPROM(currentFavIndex);
    }
    inPosition_R = false;
    timeInPosition_R = 0;
    delay(500);
    displayGreetings();
  }
*/

  

  if (IrReceiver.decode()) {
    IrReceiver.resume();
    int command = IrReceiver.decodedIRData.command;
    switch (command) {
      case IR_BUTTON_1: {
        float fav1 = favoriteStations[0];
        lcd.clear();
        lcd.print("Tunning to fav1");
        setChannelAndVolume(fav1, currentVolume);
        delay(1000);
        displayGreetings();
        break;
      }
      case IR_BUTTON_2: {
        float fav2 = favoriteStations[1];
        lcd.clear();
        lcd.print("Tunning to fav2");
        setChannelAndVolume(fav2, currentVolume);
        delay(1000);
        lcd.clear();
        displayGreetings();
        break;
      }
      case IR_BUTTON_3: {
        float fav3 = favoriteStations[2];
        lcd.clear();
        lcd.print("Tunning to fav3");
        setChannelAndVolume(fav3, currentVolume);
        delay(1000);
        lcd.clear();
        displayGreetings();
        break;
      }
      case IR_BUTTON_4: {
        float fav4 = favoriteStations[3];
        lcd.clear();
        lcd.print("Tunning to fav4");
        setChannelAndVolume(fav4, currentVolume);
        delay(1000);
        lcd.clear();
        displayGreetings();
        break;
      }
      case IR_BUTTON_5: {
        float fav5 = favoriteStations[4];
        lcd.clear();
        lcd.print("Tunning to fav5");
        setChannelAndVolume(fav5, currentVolume);
        delay(1000);
        lcd.clear();
        displayGreetings();
        break;
      }
      case IR_BUTTON_VolUp: {
        // Increase the volume
        adjustVolume(1);
        delay(1000);
        displayGreetings();
        break;
      }
      case IR_BUTTON_VolDown: {
         // Decrease the volume
        adjustVolume(-1);
        delay(1000);
        displayGreetings();
        break;
      }
      case IR_BUTTON_Next: {
        currentFavoriteIndex = (currentFavoriteIndex + 1) % 5;
            channel = radio.seekUp(); // or radio.seekDown() for seeking down
            currentChannel = channel;
            displayInfo();
            radio.setVolume(currentVolume); // Make sure the volume is set correctly
            delay(1000);
            lcd.clear();
            displayGreetings();
            break;
      }
      case IR_BUTTON_Prev: {
        currentFavoriteIndex = (currentFavoriteIndex - 1 + 5) % 5;
            channel = radio.seekDown(); // or radio.seekDown() for seeking down
            currentChannel = channel;
            displayInfo();
            radio.setVolume(currentVolume); // Make sure the volume is set correctly
            delay(1000);
            lcd.clear();
            displayGreetings();
            break;
      }

      default: {
      }
    }
  }


  
 
  if (debouncedState1 == LOW) {
    button1Released = false;
  } else if (!button1Released) {
    button1Released = true;
    if (flags.timeDisplayed) {
      flags.timeDisplayed = false;
      flags.greetingDisplayed = true;
    } else{
      flags.timeDisplayed = true;
      flags.greetingDisplayed = false;
      flags.temperatureDisplayed = false;
      flags.alarmDisplayed = false;
    }
  }

  if (debouncedState2 == LOW) {
    button2Released = false;
  } else if (!button2Released) {
    button2Released = true;
    if (flags.temperatureDisplayed) {
      flags.temperatureDisplayed = false;
      flags.greetingDisplayed = true;
    } else {
      flags.temperatureDisplayed = true;
      flags.greetingDisplayed = false;
      flags.timeDisplayed = false;
      flags.alarmDisplayed = false;
    }
  }
  if (digitalRead(SWPIN) == LOW) {
    lcd.clear();
    delay(20); // Debounce
    while (digitalRead(SWPIN) == LOW); // Wait for button release

    if (currentMenuState == HIDDEN) {
      currentMenuState = SHOW_MENU;
      selectedItem = 1;
    } else {
      if (currentMenuState == SHOW_MENU) {
        switch (selectedItem) {
          case 1:
            flags.enteringFmRadioMenu = true;
            currentMenuState = FM_RADIO_SEEK; // Change this from FM_RADIO to FM_RADIO_MENU
            fmRadioSelectedItem = 1;
            updateDisplay(fmRadioSelectedItem, fmRadioMenuItems, FM_RADIO_MENU_ITEM_COUNT); // Add this line
            break;
          case 2:
            flags.inDateTimeSetting = true; // Set the flag
            setDateTime(); // Call the setDateTime() function when the button is pressed
            flags.inDateTimeSetting = false;
            currentMenuState = HIDDEN;
            break;
          case 3:
            currentMenuState = SHOW_ALARM_MENU;
            displayAlarmMenu(alarmMenuSelectedItem);
            alarmMenuSelectedItem = 0;       
            break;
          case 4:
            clearEEPROM();
            currentMenuState = RESET;
            currentMenuState = HIDDEN;
            break;
          case 5:
            currentMenuState = BACK;
            currentMenuState = HIDDEN;
            break;
        }  
      } else if (currentMenuState == SHOW_ALARM_MENU) {
        switch (alarmMenuSelectedItem) {
          case 0:
            flags.alarmSetting = true;
            setAlarm(); // Call the setAlarm() function when the button is pressed
            flags.alarmSetting = false;
            currentMenuState = HIDDEN;
            break;
          case 1:
            // Go back to the main menu
            currentMenuState = SHOW_MENU;
            selectedItem = 1;
            break;
        }
      } else if (currentMenuState == FM_RADIO_MENU) {
        switch (fmRadioSelectedItem) {
          // Add your desired actions for each FM radio menu item here
          case 1:
            channel = radio.seekUp(); // or radio.seekDown() for seeking down
            currentChannel = channel;
            displayInfo();
            radio.setVolume(currentVolume); // Make sure the volume is set correctly
            delay(1000);
            currentMenuState = FM_RADIO_MENU;
            prevFmRadioMenuState = HIDDEN;
            break;
          case 2:
            currentMenuState = FM_RADIO_SEEK_MANUAL;
            break;
          case 3:
            currentMenuState = SHOW_SAVED_STATIONS_MENU;
            savedStationsSelectedItem = 1;
            updateDisplay(savedStationsSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
            prevSavedStationsMenuState = HIDDEN;
            break;
          case 4:
            // Handle "back" selection
            currentMenuState = SHOW_MENU;
            selectedItem = 1;
            prevMenuState = HIDDEN;
            break;
        }
      } else if (currentMenuState == FM_RADIO_SEEK_MANUAL) {
        if (selectedFavoriteIndex >= 0 && selectedFavoriteIndex < FAV_STATIONS_COUNT) {
          favoriteStations[selectedFavoriteIndex] = channel;
          displayInfo();
          radio.setVolume(currentVolume); // Make sure the volume is set correctly
          currentMenuState = SAVE_FAV_STATION_MENU; // Go to the save favorite station menu
          saveFavStationSelectedItem = 1; // Select the first save slot by default
          updateDisplay(saveFavStationSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
        }
      } else if (currentMenuState == SAVE_FAV_STATION_MENU) {
          switch (saveFavStationSelectedItem) {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
              if (saveFavStationSelectedItem >= 1 && saveFavStationSelectedItem <= FAV_STATIONS_COUNT) {
                // Save the current frequency to the corresponding favorite slot
                channel = manualTuningFrequency;
                currentChannel = channel;
                favoriteStations[saveFavStationSelectedItem - 1] = channel;
                saveFavoriteStationsToEEPROM(channel, saveFavStationSelectedItem - 1);
                
                currentMenuState = FM_RADIO_MENU; // Go back to the SHOW_SAVED_STATIONS_MENU
                fmRadioSelectedItem = 1;
                updateDisplay(savedStationsSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
                prevFmRadioMenuState = HIDDEN;
              }
              break; // Add break statement here
            case 6:
              // Handle "back" selection
              currentMenuState = FM_RADIO_MENU;
              fmRadioSelectedItem = 1;
              updateDisplay(fmRadioSelectedItem, fmRadioMenuItems, FM_RADIO_MENU_ITEM_COUNT);
              prevFmRadioMenuState = HIDDEN;
              break;
        }
      } else if (currentMenuState == SHOW_SAVED_STATIONS_MENU) {
          switch (savedStationsSelectedItem) {
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
              if (savedStationsSelectedItem >= 1 && savedStationsSelectedItem <= FAV_STATIONS_COUNT) {
                channel = favoriteStations[savedStationsSelectedItem - 1];
                loadFavoriteStationsFromEEPROM(savedStationsSelectedItem - 1);
                radio.setChannel(channel);

                savedStationsSelectedItem = savedStationsSelectedItem;
                updateDisplay(savedStationsSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
                prevFmRadioMenuState = SHOW_SAVED_STATIONS_MENU;
              }
              break;
            case 6:
              currentMenuState = FM_RADIO_MENU;
              fmRadioSelectedItem = 1;
              updateDisplay(fmRadioSelectedItem, fmRadioMenuItems, FM_RADIO_MENU_ITEM_COUNT);
              prevFmRadioMenuState = HIDDEN;
              break;
           }
        } 
    }
  }
  if (currentMenuState == SHOW_ALARM_MENU && (alarmMenuSelectedItem != prevAlarmMenuSelectedItem || currentMenuState != prevAlarmMenuState)) {
    displayAlarmMenu(alarmMenuSelectedItem);
    prevAlarmMenuSelectedItem = alarmMenuSelectedItem;
    prevAlarmMenuState = currentMenuState;
    flags.greetingDisplayed = false; // Reset the flag when the menu is shown

  // Add this line to call printAlarm() when entering the alarm menu
  if (prevAlarmMenuState == HIDDEN) {
    printAlarm();
  }
  } else if (currentMenuState == HIDDEN && prevAlarmMenuState != HIDDEN) {
  if (!flags.greetingDisplayed) {
    displayGreetings();
    flags.greetingDisplayed = true;
  }
  prevAlarmMenuState = HIDDEN;
  }
      
  if (currentMenuState == FM_RADIO_MENU && (fmRadioSelectedItem != prevFmRadioSelectedItem || currentMenuState != prevFmRadioMenuState)) {
    if (flags.enteringFmRadioMenu && !flags.radioSettingsApplied) {
      radio.setChannel(currentChannel);
      radio.setVolume(currentVolume);
      flags.enteringFmRadioMenu = false;
      flags.radioSettingsApplied = true;
    }
    updateDisplay(fmRadioSelectedItem, fmRadioMenuItems, FM_RADIO_MENU_ITEM_COUNT);
    prevFmRadioSelectedItem = fmRadioSelectedItem;
    prevFmRadioMenuState = currentMenuState;
    flags.greetingDisplayed = false; // Reset the flag when the menu is shown
  } else if (currentMenuState == HIDDEN && prevFmRadioMenuState != HIDDEN) {
      if (!flags.greetingDisplayed) {
        displayGreetings();
        flags.greetingDisplayed = true;
      }
      flags.radioSettingsApplied = false;
      prevFmRadioMenuState = HIDDEN;
  } else if (currentMenuState == FM_RADIO_SEEK) {
      channel = radio.seekUp(); // or radio.seekDown() for seeking down
      radio.setVolume(currentVolume); // Make sure the volume is set correctly
      delay(200); // Display the frequency and volume for 2 seconds
      currentMenuState = FM_RADIO_MENU; // Set the state back to FM_RADIO_MENU
  } else if (currentMenuState == FM_RADIO_SEEK_MANUAL) {
    if (digitalRead(SWPIN) == LOW) { // Check if the button is pressed
      channel = manualTuningFrequency; // Set the channel to the manually tuned frequency
      radio.setChannel(channel); // Set the radio channel
      displayInfo(); // Display the info
      radio.setVolume(currentVolume); // Make sure the volume is set correctly
      delay(200);

      currentMenuState = SAVE_FAV_STATION_MENU; // Go to the save favorite station menu
      saveFavStationSelectedItem = 1; // Select the first save slot by default
      updateDisplay(savedStationsSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
    } else {
      channel = manualTuningFrequency;
      radio.setChannel(channel);
      displayManualTuning(manualTuningFrequency);
      radio.setVolume(currentVolume);
    }
  }

  if (currentMenuState == SHOW_MENU && (selectedItem != prevSelectedItem || currentMenuState != prevMenuState)) {
    updateDisplay(selectedItem, menuItems, MENU_ITEM_COUNT);
    prevSelectedItem = selectedItem;
    prevMenuState = currentMenuState;
    flags.greetingDisplayed = false; // Reset the flag when the menu is shown
  } else if (currentMenuState == HIDDEN && prevMenuState != HIDDEN) {
    if (!flags.greetingDisplayed) {
      displayGreetings();
      flags.greetingDisplayed = true;
    }
    prevMenuState = HIDDEN;
  }
  if (currentMenuState == SAVE_FAV_STATION_MENU && (saveFavStationSelectedItem != prevSaveFavStationSelectedItem || currentMenuState != prevSaveFavStationMenuState)) {
  updateDisplay(saveFavStationSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
  prevSaveFavStationSelectedItem = saveFavStationSelectedItem;
  prevSaveFavStationMenuState = currentMenuState;
  flags.greetingDisplayed = false;
  } else if (currentMenuState == HIDDEN && prevSaveFavStationMenuState != HIDDEN) {
  if (!flags.greetingDisplayed) {
    displayGreetings();
    flags.greetingDisplayed = true;
  }
  prevSaveFavStationMenuState = HIDDEN;
  }

  if (currentMenuState == SHOW_SAVED_STATIONS_MENU && (savedStationsSelectedItem != prevSavedStationsSelectedItem || currentMenuState != prevSavedStationsMenuState)) {
  updateDisplay(savedStationsSelectedItem, savedStationsMenuItems, SAVED_STATIONS_MENU_ITEM_COUNT);
  prevSavedStationsSelectedItem = savedStationsSelectedItem;
  prevSavedStationsMenuState = currentMenuState;
  flags.greetingDisplayed = false;
  } else if (currentMenuState == HIDDEN && prevSavedStationsMenuState != HIDDEN) {
  if (!flags.greetingDisplayed) {
    displayGreetings();
    flags.greetingDisplayed = true;
  }
  prevSavedStationsMenuState = HIDDEN;
  }
  if (currentTime - lastUpdateTime >= 1000) {
    if (flags.timeDisplayed) {
      printTime();
    }
    else if (flags.temperatureDisplayed) {
      displayTemperature();
    }
    else if (flags.greetingDisplayed && !volumeDisplayed) {
      displayGreetings();
    }
    lastUpdateTime = currentTime;
    }
    delay(20);
  }

void displayManualTuning(int frequency) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Manual Tuning");
  lcd.setCursor(0, 1);
  lcd.print("Freq: ");
  lcd.print((float)frequency /10, 1);
  lcd.print(" MHz");
}

void displayAlarmMenu(int selectedItem) {
    lcd.clear();
    printAlarm();


  if (selectedItem == 0) {
    lcd.setCursor(0, 1);
    lcd.print("> Set alarm ");
  }
  else if (selectedItem == 1) {
    lcd.setCursor(0, 1);
    lcd.print("> Back       ");
  }  
}


void saveFavoriteStationsToEEPROM(float stationToSave, int favIndex) {
  int address = FAV_STATIONS_EEPROM_ADDR + (favIndex * sizeof(float));
  EEPROM.put(address, stationToSave);
  lcd.print("Station saved");
  delay(1500);
}

void loadFavoriteStationsFromEEPROM(int favIndex) {
  int address = FAV_STATIONS_EEPROM_ADDR + (favIndex * sizeof(float));
  float station;

  EEPROM.get(address, station);

  lcd.clear();
  lcd.print("Frequency ");
  lcd.setCursor(0, 1);
  lcd.print(station / 10.0, 1);
  lcd.print(" MHz");
  delay(1500);
}

void loadFavoriteStationsFromEEPROM_ALL() {
  for (int i = 0; i < FAV_STATIONS_COUNT; i++) {
    int address = FAV_STATIONS_EEPROM_ADDR + (i * sizeof(float));
    float station;

    EEPROM.get(address, station);
    favoriteStations[i] = station; // Store the loaded station in the favoriteStations array
  }
}

void readRDSData() {
  if (millis() - lastRDSReadTime > RDS_READ_INTERVAL) {
    lastRDSReadTime = millis();
    radio.readRDS(rdsBuffer, RDS_READ_TIMEOUT);
    if (strlen(rdsBuffer) > 0) {
      newRDSDataAvailable = true;
    } else {
      newRDSDataAvailable = false;
    }
  }
}

void updateRDSDataDisplay() {
  if (millis() - lastRdsUpdateTime >= rdsUpdateInterval) {
    readRDSData();
    if (newRDSDataAvailable) {
      displayRDSData();
      newRDSDataAvailable = false;
    }
    lastRdsUpdateTime = millis();
  }
}

void displayRDSData() {
  lcd.setCursor(0, 1);
  lcd.print(rdsBuffer);
}

void setChannelAndVolume(float channel, int volume) {
  radio.setChannel(channel);
  radio.setVolume(volume);
  currentChannel = channel;
  currentVolume = volume;
}

void displayInfo() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Channel:");
  lcd.setCursor(0, 1);
  float displayChannel = currentChannel / 10.0; // Convert the channel value to a float
  lcd.print(displayChannel, 1); // Print the float value with 1 decimal place
  lcd.print(" MHz");
}

void displayGreetings() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FM Radio");
}

void adjustVolume(int change) {
  static unsigned long volumeDisplayTimer = 0;
  static int maxVolume = 9;
  currentVolume += change;
  currentVolume = constrain(currentVolume, 0, maxVolume);
  radio.setVolume(currentVolume);

  // Update the volume display
  lcd.setCursor(0, 1);
  lcd.print("Volume:");
  lcd.print(currentVolume);

  // Reset the timer for volume display
  volumeDisplayTimer = millis();
  volumeDisplayed = true;
}

bool handleRDS() {
  static unsigned long volumeDisplayTimer = 0;
  const unsigned long volumeDisplayDuration = 2000; // Time to display the volume in milliseconds
  static unsigned long rdsUpdateTimer = 0; // Add this line for RDS update timer
  static bool volumeDisplayed = false;

  static unsigned long lastEncoderRead = 0;
  const unsigned long encoderDebounceTime = 50;

  if (millis() - lastEncoderRead >= encoderDebounceTime) {
    lastEncoderRead = millis();

    long encVal = myEncoder.read(); // Read the current position of the encoder
    myEncoder.write(0);

    if (encVal != 0) {
      adjustVolume(encVal); // Adjust volume with the encoder value
    }
  }

  // RDS update snippet
  unsigned long currentMillis = millis();
  if (currentMillis - rdsUpdateTimer >= rdsUpdateInterval) {
    updateRDSDataDisplay();
    rdsUpdateTimer = currentMillis;
    if (newRDSDataAvailable && !volumeDisplayed) { // Add this condition to display RDS data when available and volume is not being displayed
      displayRDSData();
      newRDSDataAvailable = false;
    }
  }

  // Check if the volume display duration has elapsed and clear the volume display
  if (volumeDisplayed) {
    if (millis() - volumeDisplayTimer >= volumeDisplayDuration) {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      volumeDisplayed = false;
    }
  }
  return volumeDisplayed;
}


void displayTemperature(){
  lcd.clear();
  lcd.setCursor(0, 0);    
  lcd.print(("Humid: "));
  lcd.print(float(sensorHTU21D.readHumidity()),1);
  //lcd.print(float(sensorHTU21D.readHumidity()),1);
  lcd.print("%");
  lcd.setCursor(0, 1);  
  lcd.print(("Temp: "));
  lcd.print(float(sensorHTU21D.readTemperature()),1);
  //lcd.print(float(sensorHTU21D.readTemperature()),1);
  lcd.print(char(223));
  lcd.print("C");
}

void clearEEPROM() {
  lcd.clear();
  lcd.print("Data erased!");
  delay(2000);
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.update(i, 0);
  }
}

void printAlarm(){
  // Display the alarm time on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Alarm: ");
  lcd.setCursor(7, 0);
  lcd.print(alarmData.hour < 10 ? "0" : "");
  lcd.print(alarmData.hour);
  lcd.print(":");
  lcd.setCursor(10, 0);
  lcd.print(alarmData.minute < 10 ? "0" : "");
  lcd.print(alarmData.minute);
}

void setAlarm() {
  lcd.clear();
  lcd.print("Set Alarm");
  lcd.setCursor(0, 1);
  lcd.print("Confirm");
  delay(1000);

  DateTime now = RTC.now();
  int hour = now.hour();
  int min = now.minute();
  int day = 0;
  int weekday = 0;
  const int maxValues[2] = {23, 59};
  const int minValues[2] = {0, 0};
  bool isSetting = false; // Track if we are in setting mode or not
  int lastButtonState = HIGH;
  unsigned long lastDebounceTime = 0;
  bool changeDate = false;
  bool confirm = false;
  int settingIndex = -1;
  int settingValue = 0;
  long encAccumulator = 0;
  
  lcd.clear();
  printAlarm();

  while (!confirm) {
    long encVal = myEncoder.read(); // Read the current position of the encoder
    encAccumulator += encVal;
    myEncoder.write(0);
    
    int buttonState = debounce(SWPIN, lastButtonState, 100);

    if (buttonState == LOW && !isSetting) { // Only start setting if we are not currently setting
      isSetting = true;
    }

    if (isSetting) { // Only update values if we are currently setting
      if (encVal != 0) {
        int updateValue = (encAccumulator > 0) ? 1 : -1;
        encAccumulator -= updateValue*2;
        switch (settingIndex) {
          case 0:
            hour += updateValue;
            hour = constrain(hour, minValues[settingIndex], maxValues[settingIndex]);
            break;
          case 1:
            min += updateValue;
            min = constrain(min, minValues[settingIndex], maxValues[settingIndex]);
            break;
        }
      }

     if (settingIndex == 0) {
        lcd.setCursor(7, 0);
        lcd.print(hour < 10 ? "0" : "");
        lcd.print(hour);
      } else if (settingIndex == 1) {
        lcd.setCursor(10, 0);
        lcd.print(min < 10 ? "0" : "");
        lcd.print(min);
      }
    }
    
    if (buttonState == LOW) {
      if (settingIndex < 2) {
        settingIndex++;
        settingValue = minValues[settingIndex];
        delay(500); // Add a delay to prevent accidental triggering of the next case
      } else {
        rtc.setAlarm(min, hour, day, weekday); // Remove this line if you don't use the RTC alarm feature
        alarmData.hour = hour; // Store the set alarm hour
        alarmData.minute = min; // Store the set alarm minute
        alarmData.time = millis(); // Update the alarmTime with the current millis
        alarmData.enabled = true; // Enable the alarm state
        rtc.enableAlarm();
        printAlarm();
        confirm = true;
        lcd.clear();
        lcd.print("Alarm set!");
        delay(1000);
        break;
      }
    }
  }
}

int debounce_analogue(int buttonPin, unsigned long& lastButtonState, unsigned long debounceDelay) {
  int buttonState = analogRead(buttonPin);
  if (buttonState > 900 && lastButtonState <= 900) { // Button is pressed
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime > debounceDelay) {
      lastDebounceTime = currentTime;
      return LOW;
    }
  }
  lastButtonState = buttonState;
  return HIGH;
}

int debounce(int buttonPin, int& lastButtonState, unsigned long debounceDelay) {
  int buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime > debounceDelay) {
      lastDebounceTime = currentTime;
      if (buttonState == LOW) {
        return LOW;
      }
    }
  }
  lastButtonState = buttonState;
  return HIGH;
}

void printTime() {
  DateTime now = RTC.now();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Time:");
  if (now.hour() < 10) {
    lcd.print("0");
  }
  lcd.print(now.hour());
  lcd.print(":");
  if (now.minute() < 10) {
    lcd.print("0");
  }
  lcd.print(now.minute());
  lcd.setCursor(0, 1);
  lcd.print("Date:");
  if (now.day() < 10) {
    lcd.print("0");
  }
  lcd.print(now.day());
  lcd.print(" ");
  switch (now.month()) {
    case 1:
      lcd.print("JAN");
      break;
    case 2:
      lcd.print("FEB");
      break;
    case 3:
      lcd.print("MAR");
      break;
    case 4:
      lcd.print("APR");
      break;
    case 5:
      lcd.print("MAY");
      break;
    case 6:
      lcd.print("JUN");
      break;
    case 7:
      lcd.print("JUL");
      break;
    case 8:
      lcd.print("AUG");
      break;
    case 9:
      lcd.print("SEP");
      break;
    case 10:
      lcd.print("OCT");
      break;
    case 11:
      lcd.print("NOV");
      break;
    case 12:
      lcd.print("DEC");
      break;
  }
  lcd.print(" ");
  lcd.print(now.year());
}

void setDateTime() {
  lcd.clear();
  lcd.print("Set time/date");
  lcd.setCursor(0, 1);
  lcd.print("Confirm");
  delay(1000);

  DateTime now = RTC.now();
  int currentHour = now.hour();
  int currentMinute = now.minute();
  int currentDay = now.day();
  int currentMonth = now.month();
  int currentYear = now.year();
  const int maxValues[5] = {23, 59, 31, 12, 3000};
  const int minValues[5] = {0, 0, 1, 1, 2000};
  const char* monthNames[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
  bool isSetting = false; // Track if we are in setting mode or not
  int lastButtonState = HIGH;
  unsigned long lastDebounceTime = 0;
  bool confirm = false;
  int settingIndex = -1;
  int settingValue = 0;
  long encAccumulator = 0; // Add this line before the while loop
  int prevHour = -1;
  int prevMinute = -1;
  int prevDay = -1;
  int prevMonth = -1;
  int prevYear = -1;

  while (!confirm) {
    long encVal = myEncoder.read(); // Read the current position of the encoder and divide by 2
    myEncoder.write(0); // Reset the encoder's position
    encAccumulator += encVal;
      
  if (digitalRead(SWPIN) == LOW && !isSetting) { // Only start setting if we are not currently setting
    isSetting = true;
  }

  if (isSetting) { // Only update values if we are currently setting
  if (encVal != 0) {
  if (abs(encAccumulator) >= 2) {
    int updateValue = (encAccumulator > 0) ? 1 : -1;
    encAccumulator -= updateValue * 2;
    switch (settingIndex) {
      case 0:
        currentHour += updateValue;
        currentHour = constrain(currentHour, minValues[settingIndex], maxValues[settingIndex]);
        break;
      case 1:
        currentMinute += updateValue;
        currentMinute = constrain(currentMinute, minValues[settingIndex], maxValues[settingIndex]);
        break;
      case 2:
        currentDay += updateValue;
        currentDay = constrain(currentDay, minValues[settingIndex], maxValues[settingIndex]);
        break;
      case 3:
        currentMonth += updateValue;
        currentMonth = constrain(currentMonth, minValues[settingIndex], maxValues[settingIndex]);
        break;
      case 4:
        currentYear += updateValue;
        currentYear = constrain(currentYear, minValues[settingIndex], maxValues[settingIndex]);
        break;
      }
    }
  }

  if (prevHour != currentHour || prevMinute != currentMinute || prevDay != currentDay || prevMonth != currentMonth || prevYear != currentYear) {
    lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Time: ");
    lcd.setCursor(5, 0);
    lcd.print(currentHour < 10 ? "0" : "");
    lcd.print(currentHour);
    lcd.print(":");
    lcd.setCursor(8, 0);
    lcd.print(currentMinute < 10 ? "0" : "");
    lcd.print(currentMinute);
    lcd.setCursor(0, 1);
    lcd.print("Date:");
    lcd.setCursor(5, 1);
    lcd.print(currentDay < 10 ? "0" : "");
    lcd.print(currentDay);
    lcd.setCursor(8, 1);
    lcd.print(monthNames[currentMonth - 1]);
    lcd.print(" ");
    lcd.print(currentYear);

    // Update the previous values
    prevHour = currentHour;
    prevMinute = currentMinute;
    prevDay = currentDay;
    prevMonth = currentMonth;
    prevYear = currentYear;
    }
  }

    int buttonState = debounce(SWPIN, lastButtonState, 500);
    if (buttonState == LOW) {
      if (settingIndex < 4) {
        settingIndex++;
        settingValue = minValues[settingIndex];
      } else {
        DateTime newTime = DateTime(currentYear, currentMonth, currentDay, currentHour, currentMinute, 0);
        RTC.adjust(newTime);
        confirm = true;
        lcd.clear();
        lcd.print("Time set!");
        delay(1000);
        break;
      }
      delay(1000);
    }
  }
}

void updateDisplay(int selectedItem, const char** menuItems, int itemCount) {
  int firstVisibleItem;
  if (selectedItem == 1) {
    firstVisibleItem = 0;
  } else if (selectedItem == itemCount) {
    firstVisibleItem = itemCount - 2;
  } else {
    firstVisibleItem = selectedItem - 1;
  }

  for (int i = 0; i < 2; ++i) {
    lcd.setCursor(0, i);
    if (selectedItem == firstVisibleItem + i + 1) {
      lcd.print("> ");
    } else {
      lcd.print("  ");
    }
    lcd.setCursor(2, i);
    lcd.print("                ");
    lcd.setCursor(2, i);
    if (currentMenuState == SHOW_SAVED_STATIONS_MENU) {
      lcd.print(savedStationsMenuItems[firstVisibleItem + i]);
    } else {
      lcd.print(menuItems[firstVisibleItem + i]);
    }
  }
}


void handleAlarm() {
  DateTime now = RTC.now();
  if (!alarmData.triggered && alarmData.enabled && now.hour() == alarmData.hour && now.minute() == alarmData.minute) {
    alarmData.triggered = true;
    PCF8563osc1Hz(); // Generate the square wave for the buzzer

    // Stop the alarm after 30 seconds or if the button is pressed
    unsigned long startTime = millis();
    while (millis() < startTime + 30000) {
      int buttonState = digitalRead(SWPIN);
      if (buttonState == LOW) {
        alarmData.triggered = false;
        alarmData.enabled = false;
        rtc.clearAlarm(); // Clear the alarm flag on the RTC
        rtc.resetAlarm(); // Clear alarm flag but leave interrupt unchanged
        PCF8563oscOff(); // Stop generating the square wave for the buzzer
        lcd.clear();
        lcd.print("Alarm stopped");
        delay(2000);  
        break;
      }
    }
    if (alarmData.triggered) {
      alarmData.triggered = false;
      alarmData.enabled = false;
      rtc.clearAlarm(); // Clear the alarm flag on the RTC
      rtc.resetAlarm(); // Clear alarm flag but leave interrupt unchanged
      PCF8563oscOff(); // Stop generating the square wave for the buzzer
    }
  }
}

void printControl2Status() {
  byte control2Status = readRegister(RTCC_STAT2_ADDR);
}

byte readRegister(byte reg) {
  byte val;
  Wire.beginTransmission(PCF8563_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(PCF8563_I2C_ADDRESS, 1);
  if (Wire.available()) {
    val = Wire.read();
  }
  return val;
}

void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(PCF8563_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void PCF8563osc1Hz()
// sets oscillator to 1 Hz
{
  Wire.beginTransmission(PCF8563_I2C_ADDRESS);
  Wire.write(0x0D);
  Wire.write(B10000011);
  Wire.endTransmission();
}

void PCF8563oscOff() {
  Wire.beginTransmission(PCF8563_I2C_ADDRESS);
  Wire.write(0x0D); // Control register 2 address
  Wire.write(0x00); // Disable CLKOUT and set to high impedance
  Wire.endTransmission();
}