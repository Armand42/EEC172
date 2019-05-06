//Up here we are given a key press

//VARIABLES
int key_press = -1 through 11
int old_key = -1;
int size = 0; //where we are on transmit array //same as size
char current = which character was pressed 'a', 'd', 'g', 'j', 'm', 'p', 't', 'w'
int dup = 0; //Number of times we have cycled through the digit


if (key_press > -1) // valid key press
{

if (key_press == 11){ //SEND
	//send message (index size)
	//delete old message on screen
	int j = 0;
	for (j = 0; j < index; j++){
		MAP_UARTCharPut(UARTA1_BASE, tran_arr[j]);
	}
	drawRect(0, 0, 120, 20, BLACK);
	//set cursor back to starting position
	size = 0;
}
else if (key_press == 0){ //SPACE
	//add a space to the char array
	//Probably just put a rectangle
	drawRect(10*size, 0, 10, 20, BLACK);
	tran_arr[size++] = ' ';
	//move index over one position
}
else if (key_press == 10 && size){ //BACKSPACE/DELETE
	//If there are characters to delete
	//delete element from char array
	size--;
	drawRect(10*size, 0, 10, 20, BLACK);
	//move index to the left
}
else if (g_ulRefTimerInts < 2000 && (key_press == old_key)){ //If duplicate
	//need to change letter being displayed
	//alter in display
	//alter in character array
	//no change in size	

	current = tran_arr[size];
	if (key_press == 7 || key_press == 9){ // 4 options
		if (dup == 3){ //wrap around
			current = current - 3;
			dup = 0;
		}
		else{
			current++;
			dup++;
		}
	}
	else{ // 3 options
		if (dup == 2){ //wrap around
			current = current - 2;
			dup = 0;
		}
		else{
			current++;
			dup++;
		}
	}
	//insert
	drawChar(size*10, 0, current, GREEN, BLACK, 2);
	tran_arr[size] = current;

}
else { //Regular operation
	//add character to array
	//add character to display
	//Move cursor to next line
	//We are using 16 characters
	//So one line would be 128/16 = 8 pixels per character
	//Could do two lines with 16 pixels, we will see
	drawChar(size*10, 0, current, GREEN, BLACK, 2);
	tran_arr[size++] = current;
	old_key = key_press;
}
//set key_press to previous so we can check if duplicate
//restart timer counter
}