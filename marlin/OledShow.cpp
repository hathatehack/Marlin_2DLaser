#include "Arduino.h"
#include "OledShow.h"
#include "bmp.h"
 
void oled_init(void)
{
    pinMode(12, INPUT_PU);
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    oled.OLED_Init();
}
 
void display(void)
{
    oled.Draw_Rectangle(0,7,0,128);
    oled.Asc6_8(43,2,(unsigned char*)"NUOSHAN");  //(x,y,str)
    delay(1000);
    oled.Asc8_16(30,4,(unsigned char*)"Laser Box "); //(x,y,str)
    delay(500);
    oled.Fill_RAM(0x00);

    for(int i = 0; i < 2; i++){
        oled.Set_Entire_Display(0xA5);
        delay(70);
        oled.Set_Entire_Display(0xA4);
        delay(70);
    }
    oled.displayoff();
    oled.Show_Pattern(BMP1,0x00,0x07,1,128);
    delay(1000);
    oled.displayon();
    // Scrolling (Full Screen)
    oled.Vertical_Scroll(0x00,0x01,20);     // Upward
    delay(1000);
    oled.Vertical_Scroll(0x01,0x01,20); // Downward

    // Fade In/Out (Full Screen)
    oled.Fade_Out();
    delay(100);
    oled.Fade_In();
    delay(100);
    oled.Fade_Out();
    delay(100);
    oled.Fade_In();
//    delay(1000);

//    oled.Fill_RAM(0x00);
}
