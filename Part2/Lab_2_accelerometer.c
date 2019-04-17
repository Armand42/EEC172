int x = 63;
int y = 63;
//initial position of ball
radius = 4;
char x_acc = acceleration in x direction;
char y_acc = acceleration in y direction;
//range of x_acc and y_acc is 0-255
//8-bit two's complement
//two's complement so the first bit is positive or negative
bool negative = 0; //if first bit is 1, we know it is positive
if (x_acc > 127){
	x_acc = x_acc ^ 0xFF;
	x_acc = x_acc + 1;
}
if (y_acc > 127){
	y_acc = y_acc ^ 0xFF;
	y_acc = y_acc + 1;
}
//the circle should go from left to right in two seconds
//We do not entirely know entire pipeline length
//Assume updates 10 times a second or something
//0-128 in 2 seconds
//64 pixels per second
//10 updates per second
//at max speed 127/20
//6.35 pixels per update
int speed = 20;
x = x + x_acc/speed;
y = y + y_acc/speed;
if (x < 0)
	x = 0;
if (y < 0)
	y = 0;
if (x > 127)
	x = 127;
if (y > 127)
	y = 127;
draw_circle(x, y, radius, BLUE);
//MAP_UtilsDelay()