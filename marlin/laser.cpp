#include "laser.h"

#define X(v)    (0x0000 | (0x800 - (v)))
#define Y(v)    (0x8000 | (0x800 - (v)))
#define TT      2 //ps256: 610
#define TC      60
#define TD      0

enum dir{L2R = 0, R2L};

/* Laser beam */
void laser_init(void)
{
 GPIO_InitTypeDef  GPIO_InitStructure;

 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 GPIO_ResetBits(GPIOA,GPIO_Pin_3);
}
/* Laser beam */

/* Galvo */
void SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2,  ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_15);

    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPI_Cmd(SPI2, ENABLE);
}

void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    SPI2->CR1&=0XFFC7;
    SPI2->CR1|=SPI_BaudRatePrescaler;
    SPI_Cmd(SPI2,ENABLE);
}
void SPI2_ReadWriteByte(u16 TxData)
{
    u8 retry=0;

    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
        if(++retry>200) return;
    SPI_I2S_SendData(SPI2, TxData);
}

void servo_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_14);

    SPI2_Init();
    SPI2_SetSpeed(SPI_BaudRatePrescaler_2);
}

void point_xy(AxisEnum axis, unsigned short data)
{
    unsigned short channel;

    channel = axis == X_AXIS ? 0x0000 : 0x8000;

    SYNC = 0;
    SPI2_ReadWriteByte(channel | data);
    delayMicroseconds(1);
    SYNC = 1;
}

            //F,0   0,0
            //8,8
            //F,F   0,F

void point(int x, int y)
{
    if(x > 2047) x = 2047;
    if(x < -2047) x = -2047;
    if(y > 2047) y = 2047;
    if(y < -2047) y = -2047;

    SYNC = 0;
    //delay_ms(1);
    SPI2_ReadWriteByte(X(x));//(0x0800);//
    delayMicroseconds(1);//delay_da(TT);//delay_ms(1);//
    SYNC = 1;
    //delay_ms(1);
    SYNC = 0;
    //delay_ms(1);
    SPI2_ReadWriteByte(Y(y));//(0x8800);//y
    delayMicroseconds(1);//delay_da(TT);//delay_ms(1);//
    SYNC = 1;

    //delay_ms(1);
    LDAC = 0;
    //delay_ms(1);//delay_da(TC);//
    LDAC = 1;

//    delayMicroseconds(600);//there is a tail if 500 or less then, because the mirror turning not fast enough!!
//    LASER = 1;
}

void line(unsigned short g, char m, unsigned int n)
{
    int ad = 0, i = 0, t;

    if(n)
    {
        LASER = 0;
        t = 600;//there is a tail if 500 or less then, because the mirror turning not fast enough!!
        n++;
        while(n)
        {
            ad += i;
            if(ad >= 0xFFF)
            {
             ad = 0xFFF;
                i = -g;
                n--;
     //           t = 10;
            }
            else if(ad <= 0)
            {
             ad = 0;
                i = g;
                n--;
     //           t = 10;
            }

            SYNC = 0;
            SPI2_ReadWriteByte(0x0000+ad);//(0x4000+ad);//
            delayMicroseconds(1);//delay_da(TT);//delay_us(1);//delay_ms(1);//
            SYNC = 1;
            //delay_ms(1);
            SYNC = 0;
            SPI2_ReadWriteByte(0x8000+ (m==R2L ? ad : 0xFFF-ad));//(0x8000+(0xFFF-ad));//(0xC000+ad);//
            delayMicroseconds(1);//delay_da(TT);//delay_us(1);//delay_ms(1);//    //ps256: L2R 615   R2L 665 7
            SYNC = 1;

    //        delay_ms(1);
            LDAC = 0;
            //delay_us(1);//delay_da(TC);//delay_ms(1);//
            LDAC = 1;

            delayMicroseconds(t);//(600);//there is a tail if 500 or less then, because the mirror turning not fast enough!!
           LASER = 1;
        }
        LASER = 0;
        //delay_ms(1);//delay_da(50000);
    }
}

void area(unsigned short g)
{
    int a[2] = {2047, 2047}, b, n , i, j;

    point(2047, 2047);
    delayMicroseconds(600);
    LASER = 1;
    for(i = 0; i < 6; i++)
    {
        if(i <= 3)
        {
            n = i % 2;
            b = i >> 1 ? g : -g;
            for(j = (4095+g)/g; j ; j--)
            {
               // printf("%d %d   ", a[0], a[1]);
                point(a[0], a[1]);
                delayMicroseconds(600);
                a[n] += b;
            }
            //printf("\r\n---------------------------------------------------------------------------------------\r\n");
        }
        else if(i == 4)
        {
            line(g, R2L, 1);
        }
        else if(i == 5)
        {
            line(g, L2R, 1);
        }
    }
}

void action(mode mode, unsigned short g, unsigned int num)
{
    switch(mode)
    {
        case POINT:
            while(num--)
            {
//                LASER = 0;
//                point(1024, 1024);
//                delayMicroseconds(600);//there is a tail if 500 or less then, because the mirror turning not fast enough!!
//                LASER = 1;
//                 delay(50);
//                LASER = 0;
//                point(-1024, 1024);
//                delayMicroseconds(600);//there is a tail if 500 or less then, because the mirror turning not fast enough!!
//                LASER = 1;
//                 delay(50);
                LASER = 0;
               point(0, 0);
               delayMicroseconds(10);
               LASER = 1;
                delay(100);
            }
            LASER = 0;
        break;

        case LINE:
            while(num--)
            {
               line(g, L2R, 1);
               line(g, R2L, 1);
            }
        break;

        case AREA:
            while(num--)
               area(g);//(1365);
        break;
    }
}
/* Galvo */
