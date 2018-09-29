#include "Arduino.h"
#include "configuration.h"
#include "pins.h"
#include "fastio.h"
#include "Oled.h"
#include "Ascii.h"

#define OLED_MODE 0
#define SIZE 16
#define XLevelL     0x02
#define XLevelH     0x10
#define XLevel      ((XLevelH&0x0F)*16+XLevelL)
#define Max_Column  128
#define Max_Row     64
#define Brightness  0xFF
#define X_WIDTH     128
#define Y_WIDTH     64
//����������
#define RIGHT   0x26
#define LEFT    0x27

#define OLED_CS_Clr()   OUT(OLED_PINS_CS, 0)
#define OLED_CS_Set()   OUT(OLED_PINS_CS, 1)

#define OLED_RST_Clr() OUT(OLED_PINS_RST, 0)//RES
#define OLED_RST_Set() OUT(OLED_PINS_RST, 1)

#define OLED_DC_Clr() OUT(OLED_PINS_DC, 0)//DC
#define OLED_DC_Set() OUT(OLED_PINS_DC, 1)

#define OLED_SCLK_Clr() OUT(OLED_PINS_CLK, 0)//CLK
#define OLED_SCLK_Set() OUT(OLED_PINS_CLK, 1)

#define OLED_SDIN_Clr() OUT(OLED_PINS_DIN, 0)//DIN
#define OLED_SDIN_Set() OUT(OLED_PINS_DIN, 1)

#define OLED_CMD  0 //д����
#define OLED_DATA 1 //д����

Oled oled(OLED_PINS_CS, OLED_PINS_RST, OLED_PINS_DC, OLED_PINS_CLK, OLED_PINS_DIN);

Oled::Oled(int cs,int rst,int dc,int clk,int din)
{
  _cs = cs;
  _rst = rst;
  _dc = dc;
  _clk = clk;
  _din = din;
}

void Oled::OLED_Init(void)
{
        //OLED��ʼ��
    pinMode(_cs, OUTPUT);
    digitalWrite( _cs , HIGH);
    pinMode(_dc,OUTPUT);
    digitalWrite( _dc , HIGH);
    pinMode(_clk,OUTPUT);
    digitalWrite( _clk , HIGH);
    pinMode(_din,OUTPUT);
    digitalWrite( _din , HIGH);
    pinMode(_rst,OUTPUT);
    digitalWrite( _rst , HIGH);
	delay(100);
	digitalWrite( _rst , LOW);
	delay(100);
	digitalWrite( _rst , HIGH);

    Set_Command_Lock(0x12);         // Unlock Driver IC (0x12/0x16)
    Set_Display_On_Off(0xAE);       // Display Off (0xAE/0xAF)
    Set_Display_Clock(0xA0);        // Set Clock as 116 Frames/Sec
    Set_Multiplex_Ratio(0x3F);      // 1/64 Duty (0x0F~0x3F)
    Set_Display_Offset(0x00);       // Shift Mapping RAM Counter (0x00~0x3F)
    Set_Start_Line(0x00);           // Set Mapping RAM Display Start Line (0x00~0x3F)
    Set_Addressing_Mode(0x02);      // Set Page Addressing Mode (0x00/0x01/0x02)
    Set_Segment_Remap(0xA1);        // Set SEG/Column Mapping (0xA0/0xA1)
    Set_Common_Remap(0xC8);         // Set COM/Row Scan Direction (0xC0/0xC8)
    Set_Common_Config(0x12);        // Set Alternative Configuration (0x02/0x12)
    Set_Contrast_Control(Brightness);   // Set SEG Output Current
    Set_Precharge_Period(0x82);     // Set Pre-Charge as 8 Clocks & Discharge as 2 Clocks
    Set_VCOMH(0x34);            // Set VCOM Deselect Level
    Set_Entire_Display(0xA4);       // Disable Entire Display On (0xA4/0xA5)
    Set_Inverse_Display(0xA6);      // Disable Inverse Display On (0xA6/0xA7)

    Fill_RAM(0x00);             // Clear Screen
    Set_Display_On_Off(0xAF);       // Display On (0xAE/0xAF)
}
/*******OLED�������ú�����www.lcdsoc.com*****************/

void Oled::Set_Start_Column(unsigned char d)  //����ҳģʽ��ʼ��
{
    Write_Command(0x00+d%16);       // Set Lower Column Start Address for Page Addressing Mode
                        //   Default => 0x00
    Write_Command(0x10+d/16);       // Set Higher Column Start Address for Page Addressing Mode
                        //   Default => 0x10
}


void Oled::Set_Addressing_Mode(unsigned char d) //�����Դ�Ѱַģʽ
{
    Write_Command(0x20);            // Set Memory Addressing Mode
    Write_Command(d);           //   Default => 0x02
                        //     0x00 => Horizontal Addressing Mode  ˮƽѰַģʽ    ����β��ҳ��ַ�Զ���1 �д�ͷ��ʼ
                        //     0x01 => Vertical Addressing Mode   ��ֱѰַģʽ     ��ҳβ���е�ַ�Զ���1 ҳ��ͷ��ʼ
                        //     0x02 => Page Addressing Mode       ҳѰַģʽ  8��Ϊ1ҳ �е�ַָ���Զ���1
}


void Oled::Set_Column_Address(unsigned char a, unsigned char b)  //�����е�ַ   ˮƽѰַģʽ��ʹ��
{
    Write_Command(0x21);            // Set Column Address
    Write_Command(a);           //   Default => 0x00 (Column Start Address)
    Write_Command(b);           //   Default => 0x7F (Column End Address)
}


void Oled::Set_Page_Address(unsigned char a, unsigned char b) //����ҳ��ַ  ��ֱѰַģʽ��ʹ��
{
    Write_Command(0x22);            // Set Page Address
    Write_Command(a);           //   Default => 0x00 (Page Start Address)
    Write_Command(b);           //   Default => 0x07 (Page End Address)
}


void Oled::Set_Start_Line(unsigned char d) //��ʾ��ʼ�ж�Ӧ�Դ�ĵ�ַ
{
    Write_Command(0x40|d);          // Set Display Start Line
                        //   Default => 0x40 (0x00)
}


void Oled::Set_Contrast_Control(unsigned char d)  //���öԱȶ� 0x00~0xff��������
{
    Write_Command(0x81);            // Set Contrast Control for Bank 0
    Write_Command(d);           //   Default => 0x7F
}


void Oled::Set_Segment_Remap(unsigned char d)  //�е�ַ��ӳ��
{
    Write_Command(d);           // Set Segment Re-Map
                        //   Default => 0xA0
                        //     0xA0 => Column Address 0 Mapped to SEG0
                        //     0xA1 => Column Address 0 Mapped to SEG127
}


void Oled::Set_Entire_Display(unsigned char d) //�����Դ���ʾ������ʾǿ��ȫ��
{
    Write_Command(d);           // Set Entire Display On / Off
                        //   Default => 0xA4
                        //     0xA4 => Normal Display
                        //     0xA5 => Entire Display On ��ʾǿ��ȫ��
}


void Oled::Set_Inverse_Display(unsigned char d) //���÷�����ʾ������RAM 1��ʾ������
{
    Write_Command(d);           // Set Inverse Display On/Off
                        //   Default => 0xA6
                        //     0xA6 => Normal Display
                        //     0xA7 => Inverse Display On
}


void Oled::Set_Multiplex_Ratio(unsigned char d)    //����ռ�ձ�
{
    Write_Command(0xA8);            // Set Multiplex Ratio
    Write_Command(d);           //   Default => 0x3F (1/64 Duty)
}


void Oled::Set_Display_On_Off(unsigned char d) //���ÿ�����ʾ���غ�OLED����˯��ģʽ
{
    Write_Command(d);           // Set Display On/Off
                        //   Default => 0xAE
                        //     0xAE => Display Off
                        //     0xAF => Display On
}


void Oled::Set_Start_Page(unsigned char d)  //����ҳģʽ�µ�ҳ��ʼ��ַ
{
    Write_Command(0xB0|d);          // Set Page Start Address for Page Addressing Mode
                        //   Default => 0xB0 (0x00)
}


void Oled::Set_Common_Remap(unsigned char d)  //��������ӳ��
{
    Write_Command(d);           // Set COM Output Scan Direction
                        //   Default => 0xC0
                        //     0xC0 => Scan from COM0 to 63
                        //     0xC8 => Scan from COM63 to 0
}


void Oled::Set_Display_Offset(unsigned char d)//������ƫ���� Ĭ��0
{
    Write_Command(0xD3);            // Set Display Offset
    Write_Command(d);           //   Default => 0x00
}


void Oled::Set_Display_Clock(unsigned char d)   //������ʾʱ��
{
    Write_Command(0xD5);            // Set Display Clock Divide Ratio / Oscillator Frequency
    Write_Command(d);           //   Default => 0x70
                        //     D[3:0] => Display Clock Divider
                        //     D[7:4] => Oscillator Frequency
}


void Oled::Set_Precharge_Period(unsigned char d) //����Ԥ���ʱ��
{
    Write_Command(0xD9);            // Set Pre-Charge Period
    Write_Command(d);           //   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
                        //     D[3:0] => Phase 1 Period in 1~15 Display Clocks
                        //     D[7:4] => Phase 2 Period in 1~15 Display Clocks
}


void Oled::Set_Common_Config(unsigned char d)   //����com pin��Ӳ������
{
    Write_Command(0xDA);            // Set COM Pins Hardware Configuration
    Write_Command(d);           //   Default => 0x12
                        //     Alternative COM Pin Configuration
                        //     Disable COM Left/Right Re-Map
}


void Oled::Set_VCOMH(unsigned char d)  //����VCOMH��ѹ����ѹ
{
    Write_Command(0xDB);            // Set VCOMH Deselect Level
    Write_Command(d);           //   Default => 0x34 (0.78*VCC)
}

 /*
void Oled::Set_NOP() //��ָ��
{
    Write_Command(0xE3);            // Command for No Operation
}
   */

void Oled::Set_Command_Lock(unsigned char d) //���������� 0x16����Ӧָ�� ����Ϊ 0x12
{
    Write_Command(0xFD);            // Set Command Lock
    Write_Data(d);              //   Default => 0x12
                        //     0x12 => Driver IC interface is unlocked from entering command.
                        //     0x16 => All Commands are locked except 0xFD.
}


/*IO��ģ��SPI�ӿ�ͨѶ���� 2��www.lcdsoc.com*/
/*************************************/
//ģ��SPI�ӿ���OLEDд���� 
void Oled::Write_Command(unsigned char Data)
{
unsigned char i;

    OLED_CS_Clr();
    OLED_DC_Clr();

    for(i=0;i<8;i++)
    {
        OLED_SCLK_Clr();
        if(Data&0x80)
           OLED_SDIN_Set();
        else
           OLED_SDIN_Clr();
        Data = Data << 1;
        OLED_SCLK_Set();
    }
    OLED_DC_Set();
    OLED_CS_Set();

}

//ģ��SPI�ӿ���OLEDд����
void Oled::Write_Data(unsigned char Data)
{
unsigned char i;

    OLED_CS_Clr();
    OLED_DC_Set();

    for(i=0;i<8;i++)
    {
        OLED_SCLK_Clr();
        if(Data&0x80)
           OLED_SDIN_Set();
        else
           OLED_SDIN_Clr();
        Data = Data << 1;
        OLED_SCLK_Set();
    }
    OLED_DC_Set();
    OLED_CS_Set();
}
/****************************************************/




/******************OLED��ʾӦ�ú�����*www.lcdsoc.com***********************/


//-------------------------------------------
//������ʾλ�� ��ַ����ΪPAGEѰַģʽ��
//��д����λ��ַ��д����λ��ַ  
//XΪ��ʼ��   YΪPAGEֵ[0---7]
//********************************************
void Oled::Set_Pos(unsigned char x, unsigned char y)
{ 
Set_Start_Page(y) ;             //����PAGE
Set_Start_Column(x);
}

//==============================================================
//��������������һ���㣨x,y��
//��������ʵ����ֵ(x,y),x�ķ�Χ0��127��y�ķ�Χ0��64
//���أ���
//==============================================================
void Oled::Set_Pixel(unsigned char x,unsigned char y)
{
    unsigned char data1;  //data1��ǰ�������
    data1 = 0x01<<(y%8);
    Set_Pos(x,(y>>3));
    Write_Data(data1);
}

//==============================================================
//����������д��һ���׼ASCII�ַ���   6x8
//��������ʾ��λ�ã�x,y����yΪҳ��Χ0��7��Ҫ��ʾ���ַ���
//���أ���
//==============================================================  
void Oled::Asc6_8(unsigned char x,unsigned char y,unsigned char ch[])
{
   unsigned char c=0;
   unsigned char i=0;
   unsigned char j=0;
   while(ch[j]!='\0') 
   {
    c=ch[j]-32;
    if(x>126) 
    {
     x=0;
     y++;
    }
    Set_Pos(x,y);
    for(i=0;i<6;i++)
      Write_Data(F6x8[c][i]);
    x+=6;
    j++;
   }
   
}


//==============================================================
//����������д��һ���׼ASCII�ַ���    8x16
//��������ʾ��λ�ã�x,y����yΪҳ��Χ0��7��Ҫ��ʾ���ַ���
//���أ���
//==============================================================  
void Oled::Asc8_16(unsigned char x,unsigned char y,unsigned char ch[])
{
  unsigned char c=0,i=0,j=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;  //���ַ��ڿ���λ��
    if(x>120)
       {x=0;y=y+2;} //����
    Set_Pos(x,y);  //��������
    for(i=0;i<8;i++)
      Write_Data(ASC8X16[c*16+i]);  //ʹ��Asc8x16.h�ֿ�
    Set_Pos(x,y+1);
    for(i=8;i<16;i++)
      Write_Data(ASC8X16[c*16+i]);
    x+=8; //�¸��ַ���x����
    j++;  //�¸��ַ�
  }
}


/********************************************************
   ��one page ��Χ�ڻ�����www.lcdsoc.com
  a: Start Page   ��ʼҳ 0~��8-1��
  b: Start Column  ��ʼ�� ��Χ 2~��131-16��
  c: End Column    ������
  Date�� ����ֵ   ��SSD1309����� Figure 10-2
*********************************************************/
void Oled::Show_Line( unsigned char a, unsigned char b, unsigned char c,unsigned char Data)
{
unsigned char j;

    Set_Start_Page(a); //����Ϊ��ǰҳ
    Set_Start_Column(b);

    for(j=b;j<c;j++)
    {
        Write_Data(Data);
    }
}

/*****************************************************
  �����ο�  www.lcdsoc.com
 p1:start page  ��ʼҳ
 p2:end page     ����ҳ
 x1:start column  ��ʼ��
 x2:end column    ������
******************************************************/
void Oled::Draw_Rectangle(unsigned char p1,unsigned char p2,unsigned char x1, unsigned char x2)
{
unsigned char i,j;

    Set_Start_Page(p1);
    Set_Start_Column(x1);

    for(i=0;i<x2;i++)
    {
        Write_Data(0x01);
    }

    Set_Start_Page(p2);
    Set_Start_Column(x1);

    for(i=0;i<x2;i++)
    {
        Write_Data(0x80);
    }

    for(i=p1;i<p2+1;i++)
    {
        Set_Start_Page(i);

        for(j=0;j<x2;j+=(x2-1))
        {
            Set_Start_Column(x1+j);

            Write_Data(0xFF);
        }
    }
}

/***************************************************************
//  ��ʾ12*16������ �ִ�С 12x12 2015-05���²���ͨ��
//  ȡģ��ʽΪ������ʽ������
//   num���������ֿ��е�λ��
//   x: Start Column  ��ʼ�� ��Χ 0~��127-12��
//   y: Start Page   ��ʼҳ 0~��7-1��
***************************************************************/
//void Oled::HZ12_16( unsigned char x, unsigned char y, unsigned char num)
//{
//    unsigned char j ;
//
//    Set_Pos(x,y);  //������ʼ����
//    for(j=0;j<12;j++)
//    {
//        Write_Data(HZK12X16[24*num+j]);  //д��һҳ��12������
//    }
//    Set_Pos(x,y+1);//������ʼ����Ϊ��һҳ
//    for(j=12;j<24;j++)
//    {
//        Write_Data(HZK12X16[24*num+j]); //д��һҳҳ��12������
//    }
//
//}

//****************************************************
//   д��һ��12*16���� www.lcdsoc.com
//    num1,num2���������ֿ��е�λ��    ��num1��ʾ��num2
//    x: Start Column  ��ʼ�� ��Χ 0~��127-12��
//    y: Start Page   ��ʼҳ 0~��7-1��
//*****************************************************
void Oled::Show_HZ12_16(unsigned char  x,unsigned char  y,unsigned char num1,unsigned char num2)
{
  unsigned char  i;
  for(i=num1;i<num2+1;i++)
  {
  HZ12_16(x,y,i);
  x=x+12;
  }
}


/***************************************************************
//  ��ʾ16*16������ 2015-0424���²���ͨ��
//  ȡģ��ʽΪ������ʽ������
//   num���������ֿ��е�λ��
//   x: Start Column  ��ʼ�� ��Χ 0~��127-16��
//   y: Start Page   ��ʼҳ 0~��7-1��
***************************************************************/
//void Oled::HZ16_16( unsigned char x, unsigned char y, unsigned char num)
//{
//    unsigned char j ;
//
//    Set_Pos(x,y);
//    for(j=0;j<16;j++)
//    {
//        Write_Data(Hzk[2*num][j]);
//    }
//    Set_Pos(x,y+1);
//    for(j=0;j<16;j++)
//    {
//        Write_Data(Hzk[2*num+1][j]);
//    }
//
//}

//****************************************************
//   д��һ��16*16���� www.lcdsoc.com
//    num1,num2���������ֿ��е�λ��    ��num1��ʾ��num2
//    x: Start Column  ��ʼ�� ��Χ 0~��127-16��
//    y: Start Page   ��ʼҳ 0~��7-1��
//*****************************************************
void Oled::Show_HZ16_16(unsigned char  x,unsigned char  y,unsigned char num1,unsigned char num2)
{
  unsigned char  i;
  for(i=num1;i<num2+1;i++)
  {
  HZ16_16(x,y,i);
  x=x+16;
  }
}




//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  ȫ�����www.lcdsoc.com
//Data��Ҫ�������� 0x00����0xff
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Fill_RAM(unsigned char Data)
{
unsigned char i,j;

    for(i=0;i<8;i++)
    {
        Set_Start_Page(i);
        Set_Start_Column(0x00);

        for(j=0;j<128;j++)
        {
            Write_Data(Data);
        }
    }
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  ���ֻ���ȫ�����www.lcdsoc.com
//   Data��Ҫ�������� 0x00����0xff
//    a: Start Page ��ʼҳ
//    b: End Page    ����ҳ
//    c: Start Column  ��ʼ��
//    d: Total Columns  ������
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Fill_Block(unsigned char Data, unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
unsigned char i,j;

    for(i=a;i<(b+1);i++)
    {
        Set_Start_Page(i);
        Set_Start_Column(c);

        for(j=0;j<d;j++)
        {
            Write_Data(Data);
        }
    }
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  ���ֻ���ȫ����ʾͼƬ www.lcdsoc.com
//  *Data_Pointer ����ָ��ָ��ͼƬ����
//    a: Start Page  ��ʼҳ
//    b: End Page     ����ҳ
//    c: Start Column   ��ʼ��
//    d: Total Columns   ������
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Show_Pattern(unsigned char *Data_Pointer, unsigned char a, unsigned char b, unsigned char c, unsigned char d)
{
unsigned char i,j;

    for(i=a; i<=b; i++)
    {
        Set_Start_Page(i);
        Set_Start_Column(c);

        for(j=0;j<d;j++)
        {
            Write_Data(*Data_Pointer);
            Data_Pointer++;
        }
    }
}

 
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  ��ֱ����Vertical /www.lcdsoc.com Fade Scrolling (Full Screen)
//    a: Scrolling Direction
//       "0x00" (Upward)
//       "0x01" (Downward)
//    b: Set Numbers of Row Scroll per Step
//    c: Set Time Interval between Each Scroll Step
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c)
{
unsigned int i,j;

    switch(a)
    {
        case 0:
            for(i=0;i<Max_Row;i+=b)
            {
                Set_Start_Line(i);
                for(j=0;j<c;j++)
                {
                    delay(1);
                }
            }
            break;
        case 1:
            for(i=0;i<Max_Row;i+=b)
            {
                Set_Start_Line(Max_Row-i);
                for(j=0;j<c;j++)
                {
                    delay(1);
                }
            }
            break;
    }
    Set_Start_Line(0x00);
}



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  ˮƽ���� www.lcdsoc.com
//  DIRECTION:��������
//  p1,p2:��ʼҳ������ҳ
//  t��ʱ��   0:5֡ 1:64֡ 2:128֡ 3:256֡�������� 4:2֡ 5:3֡ 6:4֡ 7:1֡����죩
//  c1,c2:��ʼ�У�������
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Horizontal_Scroll(unsigned char DIRECTION,unsigned char p1, unsigned char p2, unsigned char t, unsigned char c1,unsigned char c2)
{

    if (DIRECTION==RIGHT)
        {
            Write_Command(0x26);            //  ���ҹ���
            Write_Command(0x00);
            Write_Command(p1);
            Write_Command(t);
            Write_Command(p2);
            Write_Command(0x00);
            Write_Command(c1);
            Write_Command(c2);
            Write_Command(0x2F);            // �������

        }
    else
        {
            Write_Command(0x27);            // �������
            Write_Command(0x00);
            Write_Command(p1);
            Write_Command(t);
            Write_Command(p2);
            Write_Command(0x00);
            Write_Command(c1);
            Write_Command(c2);
            Write_Command(0x2F);            // Activate Scrolling

        }

}



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  ֹͣˮƽ���ߴ�ֱ������ֹͣRAM���ݱ�����дwww.lcdsoc.com
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Deactivate_Scroll()
{
    Write_Command(0x2E);            // Deactivate Scrolling
}




//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade In (Full Screen www.lcdsoc.com) ����
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Fade_In()
{
unsigned int i;

//    Set_Display_On_Off(0xAF);
    for(i=0; i<=Brightness; i++)
    {
        Set_Contrast_Control(i);
        delay(10);
    }
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Fade Out (Full Screen www.lcdsoc.com)����
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Fade_Out()
{
int i;

    for(i=Brightness; i>-1; i--)
    {
        Set_Contrast_Control(i);
        delay(10);
    }
//    Set_Display_On_Off(0xAE);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//  Sleep Mode www.lcdsoc.com ˯��ģʽ
//
//    "0x00" Enter Sleep Mode
//    "0x01" Exit Sleep Mode www.lcdsoc.com
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void Oled::Sleep(unsigned char a)
{
    switch(a)
    {
        case 0:
            Set_Display_On_Off(0xAE);
            Set_Entire_Display(0xA5);
            break;
        case 1:
            Set_Entire_Display(0xA4);
            Set_Display_On_Off(0xAF);
            break;
    }
}

void Oled::displayoff(void)
{
    Set_Display_On_Off(0xAE);
}

void Oled::displayon(void)
{
    Set_Display_On_Off(0xAF);
}
