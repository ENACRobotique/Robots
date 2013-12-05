#include "lib_us.h"
#include "lib_line.h"
#include "lib_move.h"

// needs a Wire.begin() in setup()

// This function gets data via I²C from the ADS7828-based line detection board
uint16_t getIntensity(uint8_t ch) {
  // config command definitions
  uint8_t addr = 0x48;
  static uint8_t _ch2cfg[] = {0<<4, 4<<4, 1<<4, 5<<4, 2<<4, 6<<4, 3<<4, 7<<4};
#define ADC_ON (1<<2)
#define CH(ch) _ch2cfg[(ch)&0x7]
#define SINGLE_ENDED (1<<7)

  // data definitions
  uint16_t intensity;
  static uint16_t _choffset[] = {0, 63, 722, 464, 30, 537, 489, 1038};
#define CH_OFFSET(ch) _choffset[ch]
#define MAX_OFFSET 1038

  // send config command
  Wire.beginTransmission(addr);
  Wire.write(ADC_ON | CH(ch) | SINGLE_ENDED);      // Send Command Byte
  if(Wire.endTransmission())
    return uint16_t(-1);

  // ask data
  Wire.requestFrom(addr, uint8_t(2));

  // wait for it
  while(Wire.available() < 2);

  // read, prepare and return data
  intensity = Wire.read() << 8;
  intensity |= Wire.read();
  if(intensity >= 3700)
    return 4095+MAX_OFFSET;
  else
    return intensity + CH_OFFSET(ch);
}

//uint8_t decodeLines(uint8_t *c /* capteurs */, uint8_t *n /* nombre , uint8_t *p, uint8_t *d) {

//uint8_t getLines(uint8_t *c, 


void asserLine()
	{
	uint8_t tab[8];
	//Serial.println("asserLine");
	//Détection de la ligne noir ou non par les capteurs
	int i=0;
	for(i=0;i<8;i++)
		{
		if(getIntensity(i)>4500) //ligne noir
			{
			tab[i]=1;
			}
		else //autre couleur
			{
			tab[i]=0;
			}
		}

	/*//Autre methode en cour
	int nbLine=0;

for(int i=0;i<8;i++)
	{
	if(tab[i]==1); //line détection

	}
	 */

	//Proportion gauche/droite
	int Sgauche=tab[4]+tab[5]+tab[6]+tab[7];
	int Sdroite=tab[0]+tab[1]+tab[2]+tab[3];
	int diff=Sdroite-Sgauche;

	#ifdef DEBUG
		Serial.println(diff);
	#endif

		if(diff!=0) //Garde en mémoire si plus de ligne ou au milieu
			{
			move(100,diff*10); //Attention max 250
			}
	}

