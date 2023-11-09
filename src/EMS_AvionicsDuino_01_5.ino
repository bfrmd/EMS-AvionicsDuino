/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//                                                 EMS Avionicsduino V 1.5
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  EMS Avionicsduino V 1.5 is free software    
  MIT License (MIT)
  
  Copyright (c) 2023 AvionicsDuino - benjamin.fremond@avionicsduino.com
  Contributors : Benjamin Frémond - Gabriel Consigny - Michel Riazuelo
  https://avionicsduino.com/index.php/en/ems-engine-monitoring-system-2/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
  of the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
    
 *****************************************************************************************************************************/ 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                Connexions physiques des différents composants avec la carte Teensy 4.1
// --------------------------------------------------------------------------------------------------------------------------------------------------------------
// L'écran est un module TFT LCD de 5", résolution 800x480, de marque RIVERDI (ref. RVT50HQTFWN00), piloté par un contrôleur RA8875 Adafruit(ref. Product ID: 1590), connecté en SPI1
// Library utilisée pour l'affichage : https://github.com/mjs513/RA8875/tree/RA8875_t4.
// -----------------RA8875----------------------Teensy 4.1------------------------
//                   GND           --------->      GND 
//                   VIN           --------->   + 5 Volts
//                    CS           --------->   Pin 0 CS1
//                   MISO          --------->   Pin 1 MISO1
//                   MOSI          --------->   Pin 26 MOSI1
//                   SCK           --------->   Pin 27 SCK1
//              Autres broches     --------->   Non connectées

// Encodeur rotatif optique GrayHill 
// ---------------GrayHill------------------------Teensy 4.1------------------------
//                  1     --------------------->   GND
//                  2     --------------------->   GND
//                  3 SW  --------------------->   Pin 35 Bouton central
//                  4 B   --------------------->   Pin 36 Phase B
//                  5 A   --------------------->   Pin 37 Phase A
//                  6 Vcc --------------------->   3.3 v (courant : qq mA)

// Le transceiver CAN MCP 2562 EP (CAN 2.0) est connecté au bus CAN3, avec la library Flexcan_T4 de Teensyduino
//-------------- MCP 2562 ------------------------ Teensy 4.1 ------------------------
//             Pin1 TXD ------------------------> Pin 31 CTX3
//             Pin2 GND ------------------------> GND
//             Pin3 VDD ------------------------> + 5V
//             Pin4 RXD ------------------------> Pin 30 CRX3
//             Pin5 Vio ------------------------> + 3.3V
//             Pin6 CAN LOW --------------------> sortie CanBus
//             Pin7 CAN HIGH -------------------> Sortie CanBus
//             Pin8 Standby --------------------> GND

// Capteur de pression absolue pour la PA : Adafruit MPRLS
//----------------MPRLS ------------------------- Teensy 4.1 -----------------------
//                 Vin --------------------------> Vin
//                 GND --------------------------> GND
//                 SCL --------------------------> Pin 24 (SCL2)
//                 SDA --------------------------> Pin 25 (SDA2)
//            Autres broches --------------------> Non connectées

// Valeurs analogiques mesurées
//-------------- Valeur ------------------------- Teensy 4.1 ------------------------
//            Bus 3,3volts --------------------------> A9 
//               CHT2 -------------------------------> A5
//               CHT3 -------------------------------> A6
//        Température huile -------------------------> A4
//         Pression huile ---------------------------> A3
//               AFR --------------------------------> A0
//               EGT3 -------------------------------> A17
//               EGT4 -------------------------------> A16
//         Bus 14 volts -----------------------------> A8
//     Courant pos. 3 unidir. -----------------------> A2  (Courant appelé par le bus principal)
//     Courant pos. 2 bidir. ------------------------> A1  (Courant de charge/décharge de la batterie)

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                  Inclusions des bibliothèques et fichiers externes
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

#include <EEPROM.h>  
#include <SPI.h>      
#include <RA8875.h> 
#include <SD.h>  
#include <Wire.h>
#include "Adafruit_MPRLS.h"    
#include <Adafruit_GFX.h>  
// Pour les fontes, il faut télécharger cette library https://github.com/mjs513/ILI9341_fonts et l'installer dans le dossier des libraries Arduino.
#include <font_Arial.h>
#include <font_ArialBold.h>
#include "fontesLCD.h" // Une police personnalisée pour l'affichage rapide et lisible des paramètres moteur
#include <FlexCAN_T4.h> 
#include <TimeLib.h> 
#include <QuadEncoder.h> 
#include <TeensyTimerTool.h> 
using namespace TeensyTimerTool; 

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Création des objets
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

RA8875 tft = RA8875(0, 255, 26, 27, 1); 
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN_Module_EMS; // Crée l'objet Flexan_T4 "CAN_Module_EMS" en mode CAN 2.0.
QuadEncoder encodeur(1, 37, 36, 1);  // Canal 1, Phase A (pin37), Phase B(pin36), pullups nécessaire avec l'encodeur GrayHill(1).
PeriodicTimer TimerSendDataToRecord; 
File fichier; 
#define RESET_PIN  -1  // no hard-reset pin
#define EOC_PIN    -1  // no end-of-conversion pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             Déclarations des variables et constantes globales
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

// ************************************ Déclaration du typedef struct, puis initialisation d'un tableau d'éléments de cette structure, définissant l'arborescence des menus ***************************************************
typedef struct
{
  char* label;              
  uint32_t numero;          
  char action;
  uint8_t nbOptionsSoeurs;  
  uint8_t indexOfOm;        
} OptionMenu;     

OptionMenu menu[] = 
{
  {"   GEN", 10, 'S'},
    {" BACK",  100, 'B'},
    {"  Lum",  101, 'P'},
    {"Loc-utc",  102, 'P'},
    {"  QUIT", 103, 'Q'},
  {" DatLog", 11, 'S'},
     {" BACK",  110, 'B'},
     {"  Start", 111, 'A'},
     {"  Stop", 112, 'A'},
     {"  QUIT", 113, 'Q'},    
  {"  QUIT", 12, 'Q'},

  {" ", 1000000, '\0'} 
};

//********************************************************************* Autres variables Menus **************************************************************************
bool Menu_Ouvert = false; 
bool SetVal = false; 
uint8_t indexOptionMenuEnCours = 0;
int16_t * ptrGen = NULL;
uint16_t abscisseMenu=0, ordonneeMenu = 451;
uint8_t hauteurCase = 28, largeurCase = 100;
char buf[25];
#define BckgrndMenus RA8875_BLUE

//******************************************************************** Variables Encodeur ***********************************************************************
int ValeurPositionEncodeur;
int ValeurPositionEncodeurOld = 0;
int ValeurPositionEncodeurOldSetVal = 0;
byte Pin_Bouton = 35;
const uint16_t debounceTime = 500;
volatile unsigned long lastButtonPress;
bool Bouton_Clic = false;

//******************************************************* Déclaration de 3 variables pour la gestion de l'EEPROM **********************************************
#define EMSeepromAdresse 22
uint16_t  OffsetEMSeepromAdresse = 0;
uint16_t testEeprom;

// ******************************************************************************** Variables EMS ********************************************************
float Vbus3_3volts, Vbus14volts=13.0, Ibus=0.0, Ibat=0.0;
float CHT2=40.0, CHT3=40.0, Thuile=40.0, Phuile=0.0;
float AFR=10.0, EGT3=10.0, EGT4=10.0; 
float valVbus14voltsFiltreePrecedente=13, valIbusFiltreePrecedente=0, valIbatFiltreePrecedente=0;
float valCHT2FiltreePrecedente=40, valCHT3FiltreePrecedente=40, valThuileFiltreePrecedente=40, valPhuileFiltreePrecedente=0.0;
float valAFRFiltreePrecedente=10.0, valEGT3FiltreePrecedente=10.0, valEGT4FiltreePrecedente=10.0;
float offsetCHT2=0.62, offsetCHT3=0.62, offsetThuile=0.62, offsetPhuile=0.48;
float gainCHT2=1.6, gainCHT3=1.6, gainThuile=1.6, gainPhuile=1.64;
float IP3unidir,IP2bidir;
float pressionAdmission=5.0, valPAfiltreePrecedente=5.0;
#define RpontTemp 91
#define RpontPhuile 50
#define correctionIbus 0.53 // L'emplacement du shunt unidir en position 3 sur F-PBCF ne tient pas compte de la consommation du MicroEMS ni de celle du relais contact général, soit 0.53 ampère au total, à rajouter à IBus

// ******************************************* Affectation des broches analogiques ********************************************************
#define pinVbus3_3volts A9 
#define pinCHT2 A5
#define pinCHT3 A6
#define pinThuile A4
#define pinPhuile A3
#define pinAFR A0
#define pinEGT3 A17
#define pinEGT4 A16
#define pinVbus14volts A8
#define pinIP3unidir A2
#define pinIP2bidir A1

// **************************************************** Variables pour la réception des données reçues par le CAN Bus *********************************************************************
CAN_message_t msg;
uint8_t borneHauteNiveauEssence, borneBasseNiveauEssence;
float vBat = 12.6, calcFuelLevel, fuelFlow;
uint16_t rpm=0;
float oat=18, altitudePression=5250, altitudeDensite=6320;
uint16_t anneeGNSS=0;
uint8_t moisGNSS=0, jourGNSS=0, heureGNSS=0, minuteGNSS=0, secondeGNSS=0;
int16_t vitessePropre=235;
time_t heureActuelle;

// ************************************************************ Variables utilisées pour le chronométrage de la boucle principale **********************************************************
int nombreBoucleLoop = 0;
int dureeNdbBoucles = 0;
float Ndb = 10.0;
unsigned long topHoraire;

//****************************************************** Variables utilisées pour l'affichage des caractères de la police perso LCD et leurs couleurs ************************************************************************
int16_t x =0,y =0;
uint8_t largeurChar, hauteurChar;
uint16_t charColor, backgroundColor;
char str[10];
int16_t entier;

// ******************************************************************** Variables gérant les enregistrements de données par le flight data recorder externe ***********************************************************
bool okSendDataToRecord = false;
bool recordStarted = false;

//**************************************************************** Variables diverses ************************************************************************
int16_t luminosite;
int16_t correctionHeureLocale=0;
bool clockStarted = false;
time_t topStartChrono;
time_t elapsedTime;
uint8_t cptBlink = 0;

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             SETUP
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() 
{
// *************************************************************************************** Initialisation de la carte microSD **************************************************************************************************** 
  bool result = (SD.begin(BUILTIN_SDCARD)); 

// ************************************************************************ Initialisation du contrôleur graphique et écran  d'introduction ************************************************************************************
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  SPI1.setMISO(1);
  SPI1.setCS(0);
  SPI1.begin();
  tft.begin(Adafruit_800x480);
  delay(500);
  tft.brightness(luminosite*16-1);
  tft.useLayers(true);
  tft.setTransparentColor(RA8875_PINK);
  tft.writeTo(L1);
  tft.layerEffect(LAYER1);
  tft.fillRect(0, 0, 800, 480, RA8875_BLACK);
  sablier(tft.width() / 2, tft.height() / 2);
  tft.writeTo(L2);
  tft.fillWindow(RA8875_LIGHT_GREY);
  photo("ROTAX912.out",410,320);
  tft.setCursor(134,50); 
  tft.setFont(Arial_48_Bold); 
  tft.setTextColor(RA8875_WHITE); 
  tft.println("EMS-DUINO v 1.5");
  tft.setCursor(130,50); 
  tft.setFont(Arial_48_Bold); 
  tft.setTextColor(RA8875_BLACK); 
  tft.println("EMS-DUINO v 1.5");
  tft.layerEffect(LAYER2);
  tft.writeTo(L1); 
  tft.fillWindow(RA8875_PINK);
  tft.layerEffect(TRANSPARENT);
  tft.writeTo(L2);
  tft.setFont(Arial_20_Bold);
  tft.setTextColor(RA8875_BLACK);
  tft.setCursor(0,140);
  if (result) tft.println("Carte micro SD : OK");
  else 
  {
    tft.setTextColor(RA8875_RED);
    tft.println("Carte micro SD non disponible");
  }
  delay(251);
  tft.setTextColor(RA8875_BLACK);
  tft.println("LCD                    : OK");


// ********************************************************************************* Initialisation de la voie série USB *************************************************************  
  Serial.begin(115200);
  tft.println("Serial USB         : OK");
  delay(251);

// *********************************************************************** Initialisation du capteur de pression d'admission Adafruit MPRLS *************************************************
if (! mpr.begin(0x18, &Wire2)) 
  {
    tft.println("Probleme capteur PA");
  }
  else tft.println("Capteur PA        : OK ");
  delay(251);
  
// ********************************************************************************* Initialisation des variables stockées en EEPROM *************************************************************  

 EEPROM.get(EMSeepromAdresse + 0, testEeprom);
 if (testEeprom != 3333) // Si la valeur uint16_t stockée à l'offset 0 n'est pas égale à 3333, c'est que l'EEPROM n'a encore jamais été utilisée à l'adresse EMSeepromAdresse.
                         // Il faut donc initialiser toutes les variables, puis les inscrire ensuite dans l'EEPROM
 {
  testEeprom = 3333;
  luminosite = 8;
  correctionHeureLocale = 1;
  EEPROM.put(EMSeepromAdresse + 0, testEeprom);
  EEPROM.put(EMSeepromAdresse + 2, luminosite);
  EEPROM.put(EMSeepromAdresse + 4, correctionHeureLocale);
 } 
 else
 {                                                                                                
  EEPROM.get(EMSeepromAdresse + 0,testEeprom);
  EEPROM.get(EMSeepromAdresse + 2,luminosite);
  EEPROM.get(EMSeepromAdresse + 4,correctionHeureLocale);
 }
  tft.println("EEPROM            : OK");
  delay(251);

// ***************************************************************************************** Initialisation du CAN bus  *****************************************************************************************************
  CAN_Module_EMS.begin();
  CAN_Module_EMS.setBaudRate(500000);
  CAN_Module_EMS.setMaxMB(16);
  CAN_Module_EMS.enableFIFO();
  CAN_Module_EMS.enableFIFOInterrupt();
  CAN_Module_EMS.onReceive(FIFO,canSniff);
  CAN_Module_EMS.mailboxStatus();
  tft.println("CAN Bus            : OK");
  delay(251);

// *************************************************************************** Initialisation de l'encodeur rotatif ********************************************************************************************
  pinMode(Pin_Bouton, INPUT_PULLUP);
  attachInterrupt(Pin_Bouton, Bouton_ISR, RISING); 
  encodeur.setInitConfig();
  encodeur.EncConfig.IndexTrigger = ENABLE;
  encodeur.EncConfig.INDEXTriggerMode = RISING_EDGE;
  encodeur.init();
  Bouton_Clic = false;
  tft.println("Encodeur          : OK");
  delay(251);

// ************************************************************************* Initialisation des timers *********************************************************************************************************************
  TimerSendDataToRecord.begin(SendDataToRecord, 200ms);
  tft.println("Timers               : OK");
  delay(251);
  
// **************************************************************************************** Initialisation et tri des options des menus  **********************************************************************************************************************
// Pour faciliter la gestion (ajouts, suppressions, modifications) des options de menu, ces options sont affichées pendant le setup sur le terminal série, avant et après tri.
  uint8_t nb = NbOptionsMenu(); // Calcul du nombre de lignes du tableau
  Serial.print ("Nombre de lignes utiles du tableau : ");
  Serial.println(nb); Serial.println();
  Serial.println("**********Tableau avant tri et initialisation des index***********");
  Serial.println("Index------Label--------------Numero--Action---NbOptS--IndexOfOm--");
  for (byte n=1; n<=nb+1; n++)
  {
    sprintf(buf,"%2d",(n-1));
    Serial.print(buf); Serial.print(char(9)); 
    sprintf(buf,"%-20s",(menu[n-1].label));
    Serial.print (buf); Serial.print(char(9)); 
    sprintf (buf,"%4d",(menu[n-1].numero));
    Serial.print (buf); Serial.print (char(9));
    Serial.print(menu[n-1].action); Serial.print (char(9)); 
    sprintf(buf,"%2d",(menu[n-1].nbOptionsSoeurs));
    Serial.print(buf); Serial.print(char(9));       
    sprintf(buf,"%2d",(menu[n-1].indexOfOm));
    Serial.print(buf); Serial.println();   
  }
  triBulleTableau(); // on trie le tableau dans l'ordre du champ "numero"
  initTable();       // Initialisation des champs "nbOptionsSoeurs" et indexOfOm"
  
  Serial.println();
  Serial.println("**********Tableau après tri et initialisation des index***********");
  Serial.println("Index------Label--------------Numero--Action---NbOptS--IndexOfOm--");  
  for (byte n=1; n<=nb+1; n++)
  {
    indexOptionMenuEnCours = n-1;
    sprintf(buf,"%2d",(n-1));
    Serial.print(buf); Serial.print(char(9)); 
    sprintf(buf,"%-20s",(menu[n-1].label));
    Serial.print (buf); Serial.print(char(9)); 
    sprintf (buf,"%4d",(menu[n-1].numero));
    Serial.print (buf); Serial.print (char(9));
    Serial.print(menu[n-1].action); Serial.print (char(9)); 
    sprintf(buf,"%2d",(menu[n-1].nbOptionsSoeurs));
    Serial.print(buf); Serial.print(char(9));       
    sprintf(buf,"%2d",(menu[n-1].indexOfOm));
    Serial.println(buf); 
  }
  tft.println("Menus               : OK");
  delay(251);

// ****************************************************************************** Initialisation des broches analogiques  *****************************************************************************************************
  pinMode(pinVbus3_3volts, INPUT_DISABLE);
  pinMode(pinCHT2, INPUT_DISABLE);
  pinMode(pinCHT3, INPUT_DISABLE);
  pinMode(pinThuile, INPUT_DISABLE);
  pinMode(pinPhuile, INPUT_DISABLE);
  pinMode(pinAFR, INPUT_DISABLE);
  pinMode(pinEGT3, INPUT_DISABLE);
  pinMode(pinEGT4, INPUT_DISABLE);
  pinMode(pinVbus14volts, INPUT_DISABLE);
  pinMode(pinIP3unidir, INPUT_DISABLE);
  pinMode(pinIP2bidir, INPUT_DISABLE);

  tft.println("ADC                   : OK");
  delay(500);

// ************************************************** Lancement de l'écran de l'EMS à la fin des initialisations, affichages des vignettes textes constantes ****************************************************************************
 tft.fillWindow(RA8875_BLACK);
 tft.setFont(Arial_40_Bold); 
 tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
 tft.setCursor(5,8);          tft.print("CHT2 :");
 tft.setCursor(5,61);         tft.print("CHT3 :");
 tft.setCursor(5,114);        tft.print("T.OIL :");
 tft.setCursor(5,167);        tft.print("P.OIL :");
 tft.setCursor(5,220);        tft.print("MAP  :"); 
 tft.setFont(Arial_40_Bold);
 tft.setCursor(410,10);       tft.print("AFR   :");
 tft.setCursor(410,71);       tft.print("EGT3 :"); 
 tft.setCursor(410,132);      tft.print("EGT4 :");
 tft.setFont(Arial_32_Bold);
 tft.setCursor(8,273);        tft.print("V "); 
 tft.setFont(Arial_24_Bold);  tft.print("eBus");  
 tft.setFont(Arial_32_Bold);  tft.print("    :"); 
 tft.setCursor(8,312);        tft.print("V "); 
 tft.setFont(Arial_24_Bold);  tft.print("Bat");  
 tft.setFont(Arial_32_Bold);  tft.print("      :"); 
 tft.setCursor(5,351);        tft.print (" I bus     :"); 
 tft.setCursor(5,390);        tft.print (" I bat      : ");
 tft.setCursor(5,440);        tft.print ("Chrono : ");
 tft.setFont(Arial_40_Bold);
 tft.setCursor(410,207);      tft.print("RPM   :");
 tft.setFont(Arial_32_Bold);
 tft.setCursor(465,275);     tft.print("FUEL LEVEL");
 tft.setFont(Arial_28_Bold);
 tft.setCursor(405,340);     tft.print("Calc. "); 
 tft.setCursor(620,320);     tft.print("Min");
 tft.setCursor(620,365);     tft.print("Max"); 
 tft.setFont(Arial_40_Bold);
 tft.setCursor(410,425);     tft.print("F.Flow : ");  

 // ***************************************************************************************** Affichages des cadres constants  *****************************************************************************************************
 tft.drawRect(0,0,390,265, RA8875_WHITE);
 tft.drawRect(399,0,400,190, RA8875_WHITE);
 tft.drawRect(0,267,390,212, RA8875_WHITE);
 tft.drawRect(399,193,400,69, RA8875_WHITE); 
 tft.drawRect(399,265,400,145, RA8875_WHITE);
 tft.drawRect(399,415,400,64, RA8875_WHITE);
 tft.drawFastHLine(0,429,390,RA8875_WHITE);

// ***************************************************************************************** Initialisations diverses  *****************************************************************************************************
  topHoraire = millis();
  Bouton_Clic = false;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                             LOOP
// --------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() 
{
// ************************************************************** Ajustage éventuel de la luminosité si elle a été modifiée dans le menu ****************************************************************  
tft.brightness(luminosite*16-1); 

// ************************************************************************ Affichage du chronomètre de temps moteur ****************************************************************************************
 if(!clockStarted)
 {
  if(Phuile>0.5) // démarrage du chronomètre à la mise en marche du moteur, dès que la pression d'huile dépasse 0.5 bar
  {
    clockStarted=true;
    topStartChrono=now();
  }
 }
 else 
 {
  if(Phuile>0.5) // Le chrono va s'incrémenter tant que la pression d'huile est supérieure à 0.5
  {
    elapsedTime = now()-topStartChrono;
    tft.setCursor(190,430); 
    tft.setFontDefault(); 
    tft.setFontScale (2); 
    tft.setTextColor(RA8875_WHITE, RA8875_BLACK);
    printDigits (hour(elapsedTime)); 
    tft.print(":"); 
    printDigits (minute(elapsedTime)); 
    tft.print(":"); 
    printDigits (second(elapsedTime)); 
  }
  else
  {
    clockStarted=false; // Le chrono s'arrête si le moteur s'arrête. Il redémarrera à 0 si le moteur est à nouveau lancé
  }
 }

//******************************************************************************* Acquisition de la pression d'admission *********************************************************************************
  pressionAdmission = mpr.readPressure()*0.02953; // PA acquise en hPa, puis convertie en Inch Hg
  pressionAdmission = filtrageRII (valPAfiltreePrecedente, pressionAdmission, 0.03);
  valPAfiltreePrecedente = pressionAdmission;

// ************************************************************************ Acquisition des broches analogique, calcul et filtrage des valeurs  ************************************************************
  
 int valeurDigitale = analogRead(pinVbus3_3volts);
 Vbus3_3volts = (valeurDigitale * 3.3/1023.0);
 
 valeurDigitale = analogRead(pinCHT2);
 float VpinCHT2 = (valeurDigitale*3.3/1023.0);
 float VpontCHT2 = (VpinCHT2/gainCHT2)+offsetCHT2;
 float RcaptCHT2 = (VpontCHT2*RpontTemp)/(3.3-VpontCHT2);
 CHT2 = 3.193*pow(log(RcaptCHT2),2)-61.72*log(RcaptCHT2)+302.2;
 CHT2 = filtrageRII (valCHT2FiltreePrecedente, CHT2, 0.03); 
 valCHT2FiltreePrecedente = CHT2; 

 valeurDigitale = analogRead(pinCHT3);
 float VpinCHT3 = (valeurDigitale*3.3/1023.0);
 float VpontCHT3 = (VpinCHT3/gainCHT3)+offsetCHT3;
 float RcaptCHT3 = (VpontCHT3*RpontTemp)/(3.3-VpontCHT3);
 CHT3 = 3.193*pow(log(RcaptCHT3),2)-61.72*log(RcaptCHT3)+302.2;
 CHT3 = filtrageRII (valCHT3FiltreePrecedente, CHT3, 0.03); 
 valCHT3FiltreePrecedente = CHT3; 

 valeurDigitale = analogRead(pinThuile);
 float VpinThuile = (valeurDigitale*3.3/1023.0);
 float VpontThuile = (VpinThuile/gainThuile)+offsetThuile;
 float RcaptThuile = (VpontThuile*RpontTemp)/(3.3-VpontThuile);
 Thuile = 3.193*pow(log(RcaptThuile),2)-61.72*log(RcaptThuile)+302.2;
 Thuile = filtrageRII (valThuileFiltreePrecedente, Thuile, 0.03); 
 valThuileFiltreePrecedente = Thuile; 

 valeurDigitale = analogRead(pinPhuile);
 float VpinPhuile = (valeurDigitale*3.3/1023.0);
 float VpontPhuile = (VpinPhuile/gainPhuile)+offsetPhuile;
 float RcaptPhuile = (VpontPhuile*RpontPhuile)/(3.3-VpontPhuile);
 Phuile = 0.1418*pow(RcaptPhuile,2)+38.806*RcaptPhuile-383.25;
 Phuile = filtrageRII (valPhuileFiltreePrecedente, Phuile, 0.03); 
 valPhuileFiltreePrecedente = Phuile; 
 
 valeurDigitale = analogRead(pinAFR);
 float VpinAFR = (valeurDigitale*3.3/1023.0);
 AFR = (5*VpinAFR*2/3.3)+10;
 AFR = filtrageRII (valAFRFiltreePrecedente, AFR, 0.03); 
 valAFRFiltreePrecedente = AFR; 

 valeurDigitale = analogRead(pinEGT3);
 EGT3 = (valeurDigitale*5/1023.0)/0.005;
 EGT3 = filtrageRII (valEGT3FiltreePrecedente, EGT3, 0.03); 
 valEGT3FiltreePrecedente = EGT3; 

 valeurDigitale = analogRead(pinEGT4);
 EGT4 = ((valeurDigitale*5)/1023.0)/0.005;
 EGT4 = filtrageRII (valEGT4FiltreePrecedente, EGT4, 0.03); 
 valEGT4FiltreePrecedente = EGT4; 

 valeurDigitale = analogRead(pinVbus14volts);
 Vbus14volts = (valeurDigitale*3.3/1023.0)*5.5257;
 Vbus14volts = filtrageRII (valVbus14voltsFiltreePrecedente, Vbus14volts, 0.03); 
 valVbus14voltsFiltreePrecedente = Vbus14volts; 

 valeurDigitale = analogRead(pinIP3unidir);
 float VpinIP3unidir = valeurDigitale*3.3/1023.0;
 Ibus=(VpinIP3unidir*20.625)/3.3; //0 volts -> 0A et 3.2 volts -> 20A d'où 3.3 volts -> 20.625A
 Ibus = filtrageRII (valIbusFiltreePrecedente, Ibus, 0.03); 
 valIbusFiltreePrecedente = Ibus; 

 valeurDigitale = analogRead(pinIP2bidir);
 float VpinIP2bidir = valeurDigitale*3.3/1023.0;
 Ibat=((VpinIP2bidir-1.65)*21.29*2)/3.3; //3.2 volts -> +20A et 1.65 volts -> 0A, d'où 0.1 volts -> -20A, et 0 volts -> -21,29A, et 3.3 volts -> 21,29A
 Ibat = filtrageRII (valIbatFiltreePrecedente, Ibat, 0.03); 
 valIbatFiltreePrecedente = Ibat; 

// ************************************************************************** Affichage de toutes les valeurs avec la police rapide *************************************************************************************

 setWarningCouleur(CHT2, 50, 90, 110, 130); 
 entier=int16_t(CHT2+0.5);
 sprintf (str,"%3d",entier); 
 corrigeStr(); 
 x=215; y=8; 
 printString (str, 40,x,y,charColor, backgroundColor);

 setWarningCouleur(CHT3, 50, 90, 110, 130); 
 entier=int16_t(CHT3+0.5);
 sprintf (str,"%3d",entier); 
 corrigeStr(); 
 x=215; y=61; 
 printString (str, 40,x,y,charColor, backgroundColor);

 setWarningCouleur(Thuile, 50, 90, 110, 140); 
 entier=int16_t(Thuile+0.5);
 sprintf (str,"%3d",entier); 
 corrigeStr(); 
 x=215; y=114; 
 printString (str, 40,x,y,charColor, backgroundColor); 
 
 Phuile=Phuile/1000.0;
 setWarningCouleur(Phuile, 1.5, 2, 5, 7); 
 floatPosToString(Phuile); 
 x=215; y=167; 
 printString (str, 40,x,y,charColor, backgroundColor); 

 floatPosToString(pressionAdmission); 
 x=215; y=220; 
 printString (str, 40,x,y,RA8875_WHITE, RA8875_BLACK); 

 floatPosToString(AFR); 
 x=600; y=10; 
 printString (str, 40,x,y,RA8875_WHITE, RA8875_BLACK); 
 
 if (EGT3>880) 
 {
  charColor=RA8875_RED; 
  backgroundColor = RA8875_YELLOW;
 } 
 else 
 {
  charColor= RA8875_GREEN; 
  backgroundColor = RA8875_BLACK; 
 }
 entier=int16_t(EGT3+0.5);
 sprintf (str,"%3d",entier); 
 corrigeStr(); 
 x=600; y=70; 
 printString (str, 40,x,y,charColor, backgroundColor);  
 
 if (EGT4>880) 
 {
  charColor=RA8875_RED; 
  backgroundColor = RA8875_YELLOW;
 } 
 else 
 {
  charColor= RA8875_GREEN; 
  backgroundColor = RA8875_BLACK; 
 } 
 entier=int16_t(EGT4+0.5);
 sprintf (str,"%3d",entier); 
 corrigeStr(); 
 x=600; y=135; 
 printString (str, 40,x,y,charColor, backgroundColor);  
 
 setWarningCouleur(Vbus14volts, 12.8, 13.3, 14.1, 14.8); 
 floatPosToString(Vbus14volts); 
 x=225; y=273; 
 printString (str, 32,x,y,charColor, backgroundColor); 

 //setWarningCouleur(tensionBus/10.0, 12.8, 13.3, 14.5, 15); 
 //floatPosToString(tensionBus/10.0);
 setWarningCouleur(vBat, 12.8, 13.3, 14.5, 15); 
 floatPosToString(vBat); 
 x=225; y=312; 
 printString (str, 32,x,y,charColor, backgroundColor); 

 floatPosToString(Ibus+correctionIbus); 
 x=225; y=351; 
 printString (str, 32,x,y,RA8875_WHITE, RA8875_BLACK); 
 
 char str2[10]=";"; // Ibat est la seule valeur succeptible d'être négative, un formattage particulier est nécessaire, pour placer au début soit un espace (en fait un ';'), soit le signe '-'
 if (Ibat<0)str2[0]='-';
 floatPosToString(Ibat); 
 strcat(str2, str);
 x=199; y=390;
 printString (str2, 32,x,y,RA8875_WHITE, RA8875_BLACK); 

 setWarningCouleur(rpm, 1400, 3500, 5500, 5800); 
 entier=int16_t(rpm+0.5);
 sprintf (str,"%4d",entier); 
 corrigeStr(); 
 x=615; y=205; 
 printString (str, 40,x,y,charColor, backgroundColor);  

 floatPosToString(calcFuelLevel);
 x=510; y=335; 
 printString (str, 32,x,y,RA8875_WHITE, RA8875_BLACK);  
  
 entier=(int16_t)borneBasseNiveauEssence;
 sprintf (str,"%2d",entier); 
 corrigeStr(); 
 x=720; y=320; 
 printString (str, 32,x,y,RA8875_WHITE, RA8875_BLACK);  
 
 entier=(int16_t)borneHauteNiveauEssence;
 sprintf (str,"%2d",entier); 
 corrigeStr(); 
 x=720; y=365; 
 printString (str, 32,x,y,RA8875_WHITE, RA8875_BLACK);

 floatPosToString(fuelFlow);
 x=650; y=425; 
 printString (str, 40,x,y,RA8875_WHITE, RA8875_BLACK); 

 delay(25); // Ce delay(25) donne une itération de loop() environ toutes les 50 ms, en cohérence avec les coefficients de filtrage des différentes valeurs

// ************************************************************************* Envoi éventuel de tous les paramètres sur la voie série USB pour enregistrement *************************************************************
// L'EMS ne synchronise ni son horloge ni le RTC de la Teensy avec l'heure UTC. 
// Il reçoit par le CAN 5 fois/sec l'heure UTC, et s'en sert (après correction pour l'heure locale lors de la réception CAN) pour l'horodatage des enregistrements.
  
if (okSendDataToRecord && recordStarted)
{
  okSendDataToRecord=false;
  if(cptBlink == 0) tft.fillCircle(365,400,17,RA8875_RED);
  if(cptBlink == 2) tft.fillCircle(365,400,17,RA8875_BLACK);
  Serial.print (day(heureActuelle)); 
  Serial.print('/');
  Serial.print (month(heureActuelle)); 
  Serial.print('/');
  Serial.print (year(heureActuelle)); 
  Serial.print(';');
  Serial.print (hour(heureActuelle)); 
  Serial.print(':');
  Serial.print (minute(heureActuelle)); 
  Serial.print(':');
  Serial.print (second(heureActuelle)); 
  Serial.print(';');
  Serial.print (CHT2,0);                  Serial.print(';');
  Serial.print (CHT3,0);                  Serial.print(';');
  Serial.print (Thuile,0);                Serial.print(';');
  Serial.print (Phuile,1);                Serial.print(';');
  Serial.print (pressionAdmission,1);     Serial.print(';');
  Serial.print (Vbus14volts,1);           Serial.print(';');

  Serial.print (vBat,1);                  Serial.print(';');
  Serial.print (Ibus+correctionIbus,1);   Serial.print(';');
  Serial.print (Ibat,1);                  Serial.print(';');
  Serial.print (AFR,1);                   Serial.print(';');
  Serial.print (EGT3,0);                  Serial.print(';');
  Serial.print (EGT4,0);                  Serial.print(';');
  Serial.print (rpm);                     Serial.print(';');

  Serial.print (calcFuelLevel,1);         Serial.print(';');
  Serial.print (borneBasseNiveauEssence); Serial.print(';');
  Serial.print (borneHauteNiveauEssence); Serial.print(';');

  Serial.print (fuelFlow,1);                 Serial.print(';');
  Serial.print (oat,0);                   Serial.print(';');
  Serial.print (vitessePropre);           Serial.print(';');
  Serial.print (altitudePression,0);      Serial.print(';');
  Serial.print (altitudeDensite,0);       Serial.println(); 

  cptBlink ++; if (cptBlink==4) cptBlink = 0;
}

//  ***************************************************************************** Traitement d'un clic sur le bouton de l'encodeur *******************************************************************************************
  if (Bouton_Clic == true)
  {
    if (Menu_Ouvert == false)
    { 
      Menu_Ouvert = true;
      OuvreMenu(0);
    }
    else
    {
      if (SetVal == false)
       {
         switch (menu[indexOptionMenuEnCours].action)
          {
            case 'Q':
               FermeMenu (); 
               break;
    
            case 'S':
               OuvreMenu(menu[indexOptionMenuEnCours].indexOfOm);
               break;
    
            case 'B': 
               OuvreMenu(menu[indexOptionMenuEnCours].indexOfOm);
               break;
    
            case 'P': 
              SetVal = true; 
              encodeur.write(0);
              ValeurPositionEncodeurOld = ValeurPositionEncodeur;
              ValeurPositionEncodeurOldSetVal = 0;
              Bouton_Clic = false;
              switch (menu[indexOptionMenuEnCours].numero)
                { 
                  case 101  : ptrGen = &luminosite;                
                              OffsetEMSeepromAdresse =  2;   
                              break; 
                  case 102  : ptrGen = &correctionHeureLocale;     
                              OffsetEMSeepromAdresse =  4;   
                              break;                
                  default   : break;
                }
              tft.writeTo(L1);
              tft.fillRect((((menu[indexOptionMenuEnCours].numero)%10)*largeurCase), (ordonneeMenu-hauteurCase+3),largeurCase, hauteurCase-3, RA8875_RED);
              tft.setCursor(((menu[indexOptionMenuEnCours].numero)%10)*largeurCase +(largeurCase/4), ordonneeMenu+4-hauteurCase);
              tft.setTextColor(RA8875_WHITE, RA8875_RED);
              tft.setFont(Arial_10);
              tft.print(*ptrGen); 
              tft.setFont(Arial_18);
              tft.writeTo(L2);
              break;
    
            case 'A' :
              switch (menu[indexOptionMenuEnCours].numero)
                { 
                  case 111 : serialPrintNomsChamps(); 
                             recordStarted=true; 
                             FermeMenu(); 
                             break;
                  case 112 : tft.fillCircle(365,400,17,RA8875_BLACK); 
                             recordStarted=false; 
                             FermeMenu(); 
                             break;
                } 
              break;
              
          }
       } //
       
       else
       {
         if (OffsetEMSeepromAdresse != 1000) EEPROM.put(EMSeepromAdresse + OffsetEMSeepromAdresse, *ptrGen);
         SetVal = false;
         ptrGen = NULL;
         FermeMenu();
       }
    }// fin du premier else
    Bouton_Clic = false;
  }

// ************************************************************************* Polling de la position de l'encodeur à chaque passage dans Loop ************************************************************************************
  if (Menu_Ouvert == true)
  {
    if (SetVal == false)
     {
        ValeurPositionEncodeur = encodeur.read();
        tft.setTextColor (RA8875_WHITE, RA8875_PINK);
        if (ValeurPositionEncodeur != ValeurPositionEncodeurOld)
         { 
          
           tft.writeTo(L1);
           
           uint16_t mini = menu[indexOptionMenuEnCours].numero;
           mini = mini%10;
           mini= indexOptionMenuEnCours - mini;
           uint8_t maxi = mini + menu[mini].nbOptionsSoeurs -1;
           if ((ValeurPositionEncodeur > maxi-mini) || (ValeurPositionEncodeur < 0)) 
             {
               encodeur.write(0);
               ValeurPositionEncodeur = encodeur.read();
             }
           indexOptionMenuEnCours = ValeurPositionEncodeur + mini;
           tft.drawRect ((ValeurPositionEncodeurOld * largeurCase), ordonneeMenu,largeurCase,hauteurCase, BckgrndMenus);
           tft.drawRect ((ValeurPositionEncodeur * largeurCase), ordonneeMenu,largeurCase,hauteurCase, RA8875_WHITE);
           tft.writeTo(L2);
         }
        ValeurPositionEncodeurOld = ValeurPositionEncodeur;
     }
     else
     {
      ValeurPositionEncodeur = encodeur.read();
      if (ValeurPositionEncodeur != ValeurPositionEncodeurOldSetVal)
        {
          tft.writeTo(L1);
          int8_t sens = encodeur.getHoldDifference();
          *ptrGen = *ptrGen + sens;
          verificationValeurs();
          tft.fillRect((((menu[indexOptionMenuEnCours].numero)%10)*largeurCase), (ordonneeMenu-hauteurCase+3),largeurCase, hauteurCase-3, RA8875_RED);
          tft.setCursor(((menu[indexOptionMenuEnCours].numero)%10)*largeurCase +(largeurCase/4), ordonneeMenu+4-hauteurCase);
          tft.setTextColor(RA8875_WHITE, RA8875_RED);
          tft.setFont(Arial_10);
          tft.print(*ptrGen);
          tft.setFont(Arial_18); 
          tft.writeTo(L2);       
        }
      ValeurPositionEncodeurOldSetVal = ValeurPositionEncodeur;
     }
  }

// ********************************************************* Mesure et affichage de la durée de la boucle loop() et autres affichages de debogage************************************************************************************

if (nombreBoucleLoop >=Ndb)
  {
    dureeNdbBoucles = millis() - topHoraire;
    tft.setFontDefault(); 
    tft.setFontScale (0);
    tft.setTextColor (RA8875_WHITE, RA8875_BLACK);
    tft.setCursor(780,460);  
    tft.print (float(dureeNdbBoucles/Ndb),0); 
    topHoraire = millis();
    nombreBoucleLoop = 0;
    /*
    tft.setCursor(735,10);  tft.print("oat:");
    tft.setCursor(735,25);  tft.print (oat,0); tft.print(" ");
    tft.setCursor(735,50);  tft.print("TAS:");
    tft.setCursor(735,65);  tft.print (vitessePropre,DEC); tft.print(" ");
    tft.setCursor(735,90);  tft.print("altP:");
    tft.setCursor(735,105); tft.print (altitudePression,0); tft.print("  ");   
    tft.setCursor(735,130); tft.print("altD:");
    tft.setCursor(735,145); tft.print (altitudeDensite,0); tft.print("  ");
    */
  }
  else
  {
    nombreBoucleLoop++;
  }
}

//***************************************************************************************************** Fin de la boucle infinie Loop ***********************************************************************************************

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                       ISR autorisant l'envoi de données au flight recorder
////-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void SendDataToRecord()
{ 
  okSendDataToRecord = true;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                   ISR du CAN Bus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// L'EMS reçoit de l'EFIS 5 fois par seconde quelques paramètres pour faciliter l'analyse des enregistrements : l'heure UTC, les altitudes pression et densité, et la vitesse propre.
// Il reçoit la température extérieure du module distant magnétomètre/température.
// Il reçoit du Micro-EMS les bornes hautes et basse du niveau carburant mesuré, le RPM, la tension de la batterie, le fuel-flow instantané et le niveau de carburant calculé
void canSniff(const CAN_message_t &msg) 
{
  switch (msg.id)
      {
        case 30 :
          {
            borneBasseNiveauEssence = msg.buf[0];
            borneHauteNiveauEssence = msg.buf[1];
            rpm = *(int16_t*)(msg.buf+2);
            vBat = *(float*)(msg.buf+4);
            break;   
          }
        case 31 : 
          {
            fuelFlow = *(float*)(msg.buf);   
            calcFuelLevel = *(float*)(msg.buf+4);
            break;
          }
        case 42 :
          {
            oat = *(float*)(msg.buf);
            break;    
          }
        case 62 :
          {
            altitudeDensite = *(float*)(msg.buf);            
            altitudePression = *(float*)(msg.buf+4);
            break;
          }
        case 63 :
          {
            heureActuelle = *(int32_t*)(msg.buf);
            heureActuelle = heureActuelle + correctionHeureLocale*3600;
            vitessePropre = *(int16_t*)(msg.buf+4);            
            break;
          }
        default:
            break;
      }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonction utilisée pour le filtrage des données brutes issues des capteurs
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// Fonction de filtre à Réponse Impulsionnelle Infinie (RII)
float filtrageRII (float valeurFiltreePrecedente, float valeurCourante , float coeffFiltrageRII)
{
  return valeurFiltreePrecedente  * (1-coeffFiltrageRII) + valeurCourante * coeffFiltrageRII ;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                          Fonctions liées à l'encodeur rotatif et aux menus 
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void verificationValeurs() // Cette fonction vérifie si certains paramètres modifiés dans les menus restent dans les limites autorisées
{
  if (luminosite>16) luminosite = 16;       
  if (luminosite<1) luminosite = 1;
  if(correctionHeureLocale < -12) correctionHeureLocale = -12;
  if(correctionHeureLocale > 12) correctionHeureLocale = 12;
}

// ************************************************************************* Routine d'interruption liée au traitement d'une pression sur le bouton de l'encodeur *******************************************************************************
void Bouton_ISR() 
{
  
  if ((millis() - lastButtonPress) >= debounceTime)
  {
    lastButtonPress = millis ();
    Bouton_Clic = true;
  }
}
// ***************************************************************************************** Procédure d'ouverture du menu ******************************************************************************************************

void OuvreMenu (uint8_t ndx)
{
  tft.writeTo(L1); 
  indexOptionMenuEnCours = ndx;
  tft.fillRect (abscisseMenu, ordonneeMenu-hauteurCase+2, tft.width()-abscisseMenu, hauteurCase*2-2, BckgrndMenus);
  tft.drawLine(abscisseMenu+1, ordonneeMenu-hauteurCase+2, tft.width()-abscisseMenu, ordonneeMenu-hauteurCase+2, RA8875_WHITE);
  encodeur.write(0);
  ValeurPositionEncodeur = 0;
  ValeurPositionEncodeurOld = 0;
  tft.setFont(Arial_18);
  tft.setTextColor (RA8875_WHITE, BckgrndMenus);
  tft.setCursor(abscisseMenu+1, ordonneeMenu+5-hauteurCase);
  if(indexOptionMenuEnCours==0)
  {
    tft.print("MENU GENERAL");
  }
  else
  {
    int8_t i=0;
    tft.setTextColor (RA8875_WHITE, BckgrndMenus);
    while(menu[indexOptionMenuEnCours+i].numero != (menu[indexOptionMenuEnCours].numero)/10) i--;
    tft.print(menu[indexOptionMenuEnCours+i].label);
    indexOptionMenuEnCours ++;
    ValeurPositionEncodeur ++; 
    encodeur.write(ValeurPositionEncodeur); 
    ValeurPositionEncodeurOld = ValeurPositionEncodeur;
  }
  for (byte n = ndx; n < ndx + menu[ndx].nbOptionsSoeurs; n++)
  {
    tft.setCursor ((largeurCase*(n-ndx))+2, ordonneeMenu+3);
    tft.print(menu[n].label);
  } 
   tft.drawRect ((ValeurPositionEncodeur*largeurCase),ordonneeMenu, largeurCase,hauteurCase, 0xFFFF);
   tft.writeTo(L2); 
}

// ********************************************************************************************* Procédure de fermeture du menu **************************************************************************************************
void FermeMenu()
{
  Menu_Ouvert = false;
  Bouton_Clic = false; 
  tft.writeTo(L1);
  tft.fillRect (abscisseMenu, ordonneeMenu-hauteurCase, tft.width()-abscisseMenu, hauteurCase*2, RA8875_PINK);
  tft.setFontDefault(); 
  tft.setFontScale (0);
  tft.writeTo(L2);
}

// ************************************************ Cette fonction retourne, à partir d'un index donné quelconque, l'index de la première option soeur (numéro terminant par un zéro) **********************************************
uint8_t indexRetour(uint8_t indexQuelconque)
{
  int8_t i = 0;
  while(menu[indexQuelconque+i].numero > ((menu[indexQuelconque].numero)/10)*10) i--;
  return (indexQuelconque+i);
}

// ***************************** Cette fonction envoie sur la voie série USB à chaque début d'enregistrement les noms des champs à enregistrer par le flight data recorder ************************************************

void serialPrintNomsChamps()
{
  Serial.print ("Date"); Serial.print(';');
  Serial.print ("Heure"); Serial.print(';');
  Serial.print ("CHT2"); Serial.print(';');
  Serial.print ("CHT3"); Serial.print(';');
  Serial.print ("Thuile"); Serial.print(';');
  Serial.print ("Phuile"); Serial.print(';');
  Serial.print ("MAP"); Serial.print(';');
  Serial.print ("VeBus"); Serial.print(';');
  Serial.print ("VBus"); Serial.print(';');
  Serial.print ("Ibus"); Serial.print(';');
  Serial.print ("IBat"); Serial.print(';');
  Serial.print ("AFR"); Serial.print(';');
  Serial.print ("EGT3"); Serial.print(';');
  Serial.print ("EGT4"); Serial.print(';');
  Serial.print ("RPM"); Serial.print(';');
  Serial.print ("NivCalc"); Serial.print(';');
  Serial.print ("NivMin"); Serial.print(';');
  Serial.print ("NivMax"); Serial.print(';');
  Serial.print ("Fflow"); Serial.print(';');
  Serial.print ("OAT"); Serial.print(';');
  Serial.print ("TAS"); Serial.print(';');
  Serial.print ("AltP"); Serial.print(';');
  Serial.print ("AltD"); Serial.println();  
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                Fonctions utilisées lors de l'initialisation des options des menus
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void initTable()
{
  for (uint8_t i=0; i<NbOptionsMenu(); i++)
  {
    uint16_t n = menu[i].numero;
    int8_t j = 0;
    if ((n/10)*10 == n)
    {
       while (menu[i+j].numero < n+10) j++;
       for (uint8_t k = 0; k<j; k++) menu[i+k].nbOptionsSoeurs = j;
       j = 0;
    }
    
    if(menu[i].action == 'S')
    {
       while(menu[i+j].numero != n*10) j++;
       menu[i].indexOfOm = i+j;
       j=0;
    }
    
    if(menu[i].action == 'B')
    {
       while(menu[i+j].numero != (n/100)*10) j--;
       menu[i].indexOfOm = i+j;
    }
  }
}

void triBulleTableau() 
{
  char * texte; 
  uint32_t nombre; 
  char lettre;
  for (byte i = NbOptionsMenu()-1; i>0; i--)
  {
    for (byte j = 1; j <= i; j++)
    {
      if (menu[j-1].numero > menu[j].numero)
      {
        texte =   menu[j].label;
        nombre = menu[j].numero;
        lettre =  menu[j].action;
        menu[j].label        = menu[j-1].label;
        menu[j].numero       = menu[j-1].numero;
        menu[j].action       = menu[j-1].action;
        menu[j-1].label        = texte;
        menu[j-1].numero       = nombre;
        menu[j-1].action       = lettre;
      }
    }
  }
}

uint8_t NbOptionsMenu()
{
  uint8_t i=0;
  while (menu[i].numero != 1000000) { i=i+1;}
  return i;
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                          Fonctions liées à l'utilisation de la police LCD à affichage rapide
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void traceLcdChar(uint8_t * tab, uint8_t hauteurChar, uint16_t x, uint16_t y, uint16_t couleurChar, uint16_t couleurFond)
{
  largeurChar = tab[0];
  tft.fillRect(x,y,largeurChar, hauteurChar,couleurFond);
  uint8_t nbRect = tab[1];
  for (uint8_t i = 2; i<(4*nbRect)+2; i+=4) 
  {
    tft.fillRect(x+tab[i],y+tab[i+1], tab[i+2], tab[i+3], couleurChar);
  }
}

void printString (char * str, uint8_t hauteur, uint16_t x, uint16_t y, uint16_t couleurChar, uint16_t couleurFond)
{
 switch (hauteur)
   { 
    case 40 : 
      for (byte n=0; n<strlen(str); n++)
      {
        traceLcdChar (fontLcd40[(byte(str[n]))-45],hauteur,x,y,couleurChar, couleurFond);
        x=x+largeurChar;
      }  
      break;
  
    default : 
      for (byte n=0; n<strlen(str); n++)
      {
        traceLcdChar (fontLcd32[(byte(str[n]))-45],hauteur,x,y,couleurChar, couleurFond);
        x=x+largeurChar;
      }      
      break;
   }
}
void corrigeStr()// remplace tous les espaces de la chaine de caractères str par des ';' et ajoute un ';' à la fin
{
  for (byte n=0; n<strlen(str); n++) if (str[n]== ' ') str[n]=';';
  strcat(str,";");
}
void floatPosToString(float val)//Cette fonction convertit un float quelconque en une "string variable", après inversion du signe si la valeur est négative et  arrondi à une seule décimale après la virgule.
{
  //char * tmpSign = (val < 0) ? "-" : " ";
  float tmpVal = (val < 0) ? -val : val;
  int tmpInt1 = tmpVal;
  float tmpFrac = (tmpVal - tmpInt1)+0.05; 
  int tmpInt2 = (tmpFrac * 10);
  if (tmpInt2==10) 
  {
    tmpInt2=0; 
    tmpInt1++;
  }
  sprintf (str, "%2d.%1d", tmpInt1, tmpInt2);
  for (byte n=0; n<strlen(str); n++) if (str[n]== ' ') str[n]=';'; // remplace tous les espaces de la chaine de caractères str par des ; 
  strcat(str,";");// et ajoute un ; à la fin
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//                                                                                                  Fonctions diverses
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setWarningCouleur (float val, float rouge, float jaune, float vert, float jaunehaut)
{
  if ((val <= rouge) | (val > jaunehaut)) {charColor = RA8875_RED;    backgroundColor = RA8875_YELLOW; return;}
  if ((val <= jaune) & (val > rouge))     {charColor = RA8875_YELLOW; backgroundColor = RA8875_BLACK; return;}
  if ((val <= vert) & (val > jaune))      {charColor = RA8875_GREEN;  backgroundColor = RA8875_BLACK; return;}
  if ((val <= jaunehaut) & (val > vert))  {charColor = RA8875_YELLOW; backgroundColor = RA8875_BLACK;return;}
}

void printDigits(int digits) // formate l'heure à afficher
{
  if (digits < 10) tft.print('0'); 
  tft.print(digits);
}

void sablier(uint16_t x, uint16_t y) // affiche un sablier
{
  tft.fillRect (x - 20, y - 26, 40, 2, RA8875_WHITE);
  tft.fillRect (x - 20, y + 25, 40, 2, RA8875_WHITE);
  tft.drawLine (x - 18, y - 21, x - 18, y + 20, RA8875_WHITE);
  tft.drawLine (x + 18, y - 21, x + 18, y + 20, RA8875_WHITE);
  tft.drawArc (x, y - 14, 14, 1, 195, 270, RA8875_WHITE);
  tft.drawArc (x, y - 14, 14, 1, 90, 165, RA8875_WHITE);
  tft.drawArc (x, y + 14, 14, 1, 270, 345, RA8875_WHITE);
  tft.drawArc (x, y + 14, 14, 1, 15, 90, RA8875_WHITE);
  tft.drawArc (x, y + 9, 32, 1, 335, 25, RA8875_WHITE);
  tft.drawArc (x, y - 9, 32, 1, 155, 205, RA8875_WHITE);
  tft.drawLine(x - 13, y - 14, x - 13, y - 19, RA8875_WHITE);
  tft.drawLine(x - 13, y + 14, x - 13, y + 19, RA8875_WHITE);
  tft.drawLine(x + 13, y - 14, x + 13, y - 19, RA8875_WHITE);
  tft.drawLine(x + 13, y + 14, x + 13, y + 19, RA8875_WHITE);
  tft.drawArc (x, y - 14, 12, 13, 90, 270, RA8875_WHITE);
  tft.drawLine (x - 1, y - 14, x + 1, y - 14, RA8875_WHITE);
  tft.fillRect (x - 1, y - 3, 3, 12, RA8875_WHITE);
  tft.fillTriangle(x, y + 7, x + 8, y + 20, x - 8, y + 20, RA8875_WHITE);
}

void photo(char str[], uint16_t largeur, uint16_t hauteur) 
{
fichier = SD.open(str, FILE_READ);
uint16_t xOrig, yOrig;
xOrig = (tft.width()- largeur)/2+120;
yOrig = (tft.height()- hauteur)/2+30;
if(fichier)
  {
    unsigned char LSB, MSB;
    uint16_t couleur;
    for (uint16_t j=1; j<=hauteur; j++)
      {
        for (uint16_t i = 1; i<=largeur; i++)
          {
            LSB = fichier.read();
            MSB = fichier.read();
            couleur = LSB + (MSB<<8);
            tft.drawPixel(i+xOrig, j+yOrig, couleur);
          }
      }
    fichier.close();
  } 
else {tft.print("Erreur a l'ouverture du fichier "); tft.println(str); delay(2000);}
}
