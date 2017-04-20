/*
  Sortie Digit LED
  0,1,2,3,4: Blanches
  7,8,11,12,13: Bleues

  Sorties Digit PWM
  5: Eclairage Blanc (couvercle + barre immergée)
  6: Eclairage Hydroponique (couvercle + barre immergée)

  // play the pitch:
  tone(9, thisPitch, 10);
  delay(1);        // delay in between reads for stability

*/

/////Cycle d'eclairage en SECONDES
#define DAILY_CYCLE   		  86400            	// 24h 24*60*60 secondes
////BLEU
//#define DUREE_STBY_BLEU   2
//#define DUREE_MONTEE_BLEU 60*60           // 1h 60*60 secondes
//#define DUREE_RUN_BLEU    12*60*60        // 12h 12*60*60 secondes
//#define DUREE_DESC_BLEU   60*60           // 1h 60*60 secondes
//BLEU
#define DUREE_STBY_BLEU     1
#define DUREE_MONTEE_BLEU   30
#define DUREE_RUN_BLEU      5
#define DUREE_DESC_BLEU     30
//BLANC
#define DUREE_STBY_BLANC    (DUREE_STBY_BLEU + (DUREE_MONTEE_BLEU/2))
#define DUREE_MONTEE_BLANC  DUREE_MONTEE_BLEU/2
#define DUREE_RUN_BLANC		  DUREE_RUN_BLEU
#define DUREE_DESC_BLANC    DUREE_DESC_BLEU/2
unsigned long LastTime 		  = 0;              //Temps du dernier passage dans la loop

/////STATE
const unsigned long StateBleu[2][5] = {
  {0, 1, 2, 3, 4},	                          //{"STBY", "MONTEE", "RUN", "DESC", "DAILY_CYCLE"}
  {DUREE_STBY_BLEU, DUREE_MONTEE_BLEU, DUREE_RUN_BLEU, DUREE_DESC_BLEU, DAILY_CYCLE}
};
const unsigned long StateBlanc[2][5] = {
  {0, 1, 2, 3, 4},                            //{"STBY", "MONTEE", "RUN", "DESC", "DAILY_CYCLE"}
  {DUREE_STBY_BLANC, DUREE_MONTEE_BLANC, DUREE_RUN_BLANC, DUREE_DESC_BLANC, DAILY_CYCLE}
};
int CurrentStateBleu 		    = 0;
unsigned long Time_in_StateBleu = 0;
int CurrentStateBlanc 		  = 0;
unsigned long Time_in_StateBlanc = 0;
int CurrentStateBleuSAV		  = 0;
unsigned long Time_in_StateBleuSAV = 0;
int CurrentStateBlancSAV 	  = 0;
unsigned long Time_in_StateBlancSAV = 0;

//Registre a Decalage
#define SER_Pin  8   //pin 14 on the 75HC595
#define RCLK_Pin  9  //pin 12 on the 75HC595
#define SRCLK_Pin  10 //pin 11 on the 75HC595
#define NUMBER_OF_74HC595S 1  //How many of the shift registers - change this
#define NUMOFREGISTERPINS NUMBER_OF_74HC595S * 8       //do not touch
boolean registers[NUMOFREGISTERPINS];


//LEDs de visualisations
#define LED_COUNT     		  4
int LedBleu_Pins[]     = { 0, 1, 2, 3};
int LedBlanc_Pins[] 	 = { 4, 5, 6, 7 };

//RAMPES D'ECLAIRAGE
int Rampe_Blanc 		  = 5;
int Rampe_Bleu		    = 6;

// BUTTON:
#define   MODE_AUTO  		  0
#define   MODE_ALLHIGH  	  1
#define   MODE_ALLLOW  		  2
#define   BUTTONPIN 		    11
int LastBPMode 				      = MODE_AUTO;    // 0:Normal 1:ALL HIGH 2:ALL LOW

//Buzzer
#define   BUZZERPIN 		    7


///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  delay (1000);                                //donner du temps a l'ouverture du port serie

  // Regsitre a decalage
  pinMode(SER_Pin, OUTPUT);
  pinMode(RCLK_Pin, OUTPUT);
  pinMode(SRCLK_Pin, OUTPUT);

  //reset all register pins
  clearRegisters();
  writeRegisters();

  // declare pins to be an output:
  pinMode(Rampe_Blanc, OUTPUT);
  pinMode(Rampe_Bleu, OUTPUT);
  pinMode(BUTTONPIN, INPUT);
  pinMode(BUZZERPIN, OUTPUT);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////



void loop() {
  unsigned long CurrentTime = 0;
  unsigned long DiffTime = 0;

  //Gestion Bouton
  //Button_Management (digitalRead(BUTTONPIN));

  //Calcul Temps depuis derniere boucle
  CurrentTime = millis() / 1000;
  DiffTime = CurrentTime - LastTime;
  LastTime = CurrentTime;

  //Gestion Etat
  Etat_Management(&Time_in_StateBleu, &CurrentStateBleu, StateBleu[1][CurrentStateBleu], DiffTime);
  Etat_Management(&Time_in_StateBlanc, &CurrentStateBlanc, StateBlanc[1][CurrentStateBlanc], DiffTime);

  //Gestion des LED + Rampes
  LightON_Management (CurrentStateBleu, Time_in_StateBleu, Rampe_Bleu, LedBleu_Pins, StateBleu);
  //LightON_Management (CurrentStateBlanc, Time_in_StateBlanc, Rampe_Blanc, LedBlanc_Pins, StateBlanc);

  delay (500);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void LightON_Management (int CurrentState,
                         unsigned long Time_in_State,
                         int Rampe,
                         int LEDTab[],
                         unsigned long State[2][5])
{
  int ledLevel = 0;
  int rampeBrightness = 0;

  Serial.println("---------------------------------------------------------");
  Serial.print("State:"); Serial.print(CurrentState); Serial.print(" Time_in_State:"); Serial.println(Time_in_State);

  switch (State[0][CurrentState]) {
    case 0:
      ledLevel = 0;
      rampeBrightness = 0;
      break;

    case 1:
      ledLevel = map(Time_in_State, 0, State[1][CurrentState], 0, LED_COUNT);
      rampeBrightness = mapincexp(Time_in_State, 0, State[1][CurrentState], 0, 255);
      break;

    case 2:
      ledLevel = LED_COUNT ;
      rampeBrightness = 255;
      break;

    case 3:
      ledLevel = map(Time_in_State,  0, State[1][CurrentState], LED_COUNT, 0 );
      rampeBrightness = mapdecexp(Time_in_State,  0, State[1][CurrentState], 255, 0);
      break;

    case 4:
      ledLevel = 0;
      rampeBrightness = 0;
      break;

  }
  Serial.print(" ledLevel="); Serial.print(ledLevel); Serial.print(" RampeBrightness="); Serial.println(rampeBrightness);

  LED_allumage (LEDTab, ledLevel);
  analogWrite(Rampe, rampeBrightness);
}

void LED_allumage (int LEDTab[], int ledLevel) {
  for (int thisLed = 0; thisLed < LED_COUNT; thisLed++) {
    if (thisLed < ledLevel) {
      setRegisterPin(LEDTab[thisLed], HIGH);
    } else {
      setRegisterPin(LEDTab[thisLed], LOW);
    }
  }
  writeRegisters();  //MUST BE CALLED TO DISPLAY CHANGES
  //Only call once after the values are set how you need.
}

void Etat_Management (unsigned long *Time_in_State, int *CurrState, unsigned long StateDuration, unsigned long DiffTime)  {
  *Time_in_State = *Time_in_State + DiffTime;
  if (*Time_in_State >= StateDuration) {
    (*CurrState)++;
    *Time_in_State = 0;
  }
}

void Button_Management (int buttonState) {
  if (buttonState == HIGH) {

    switch (LastBPMode) {
      case MODE_AUTO:
        CurrentStateBleuSAV = CurrentStateBleu;
        Time_in_StateBleuSAV = Time_in_StateBleu;
        CurrentStateBlancSAV = CurrentStateBlanc;
        Time_in_StateBlancSAV = Time_in_StateBlanc;
        CurrentStateBleu = 2; //RUN
        Time_in_StateBleu = 0;
        CurrentStateBlanc = 2; //RUN
        Time_in_StateBlanc = 0;
        LastBPMode = MODE_ALLHIGH;
        tone(BUZZERPIN, 2000, 200);
        break;

      case MODE_ALLHIGH:
        CurrentStateBleu = 4; //ETEINT
        Time_in_StateBleu = 0;
        CurrentStateBlanc = 4; //ETEINT
        Time_in_StateBlanc = 0;
        LastBPMode = MODE_ALLLOW;
        tone(BUZZERPIN, 2500, 200); delay(300); tone(BUZZERPIN, 2500, 200);
        break;

      case MODE_ALLLOW:
        Serial.print(" RESTORE: ");
        CurrentStateBleu = CurrentStateBleuSAV;
        Time_in_StateBleu = Time_in_StateBleuSAV;
        CurrentStateBlanc = CurrentStateBlancSAV;
        Time_in_StateBlanc = Time_in_StateBlancSAV;
        LastBPMode = MODE_AUTO;
        tone(BUZZERPIN, 1500, 1000);
        break;
    }
  }
}

//set all register pins to LOW
void clearRegisters() {
  for (int i = NUMOFREGISTERPINS - 1; i >=  0; i--) {
    registers[i] = LOW;
  }
}

//Set and display registers
//Only call AFTER all values are set how you would like (slow otherwise)
void writeRegisters() {

  digitalWrite(RCLK_Pin, LOW);

  for (int i = NUMOFREGISTERPINS - 1; i >=  0; i--) {
    digitalWrite(SRCLK_Pin, LOW);

    int val = registers[i];

    digitalWrite(SER_Pin, val);
    digitalWrite(SRCLK_Pin, HIGH);

  }
  digitalWrite(RCLK_Pin, HIGH);

}

//set an individual pin HIGH or LOW
void setRegisterPin(int index, int value) {
  registers[index] = value;
}

long mapincexp(long x, long in_min, long in_max, long out_min, long out_max)
{
  return pow((x - in_min), 2) * (out_max - out_min) / pow((in_max - in_min), 2) + out_min;
}
long mapdecexp(long x, long in_min, long in_max, long out_min, long out_max)
{
  return sqrt(x - in_min) * (out_max - out_min) / sqrt(in_max - in_min) + out_min;
}



