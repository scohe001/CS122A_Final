#include <avr/io.h>

signed short xAxis, yAxis;
enum Positions {None, Right, Left, Up, Down};
enum Positions joyPos;

void joystickInit() {
	ADCSRA |= (1 << ADEN) | (1 << ADSC);
	xAxis=0;
	yAxis=0;
}

void joystickSample() {
    //Code given by Cody Simons (Thanks cody!<3)
	ADCSRA|=(1<<ADSC);
	while(ADCSRA&0x40);
	xAxis=ADC;
	ADMUX|=0x01;
	ADCSRA|=(1<<ADSC);
	while(ADCSRA&0x40);
	yAxis=ADC;
	ADMUX&=0xFE;
    //</Cody>
	
	//Adjust system so center is (0,0)
	//Values found empirically
	xAxis -= 496;
	yAxis -= 524;
	
	//If we're under threshold, found using x^2 + y^2 < r^2
	if(((signed long long)xAxis * xAxis) + ((signed long long)yAxis * yAxis) < 65536) { joyPos = None; return; }
	enum Positions possibilities[4] = {Right, Left, Up, Down};
	signed short projections[4] = {xAxis, -xAxis, yAxis, -yAxis};
	unsigned char greatest = 0;
	for(int x=1; x<4; x++) {
		if(projections[x] > projections[greatest]) greatest = x;
	}
	joyPos = possibilities[greatest];
}