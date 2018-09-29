#ifndef Oled_h
#define Oled_h

class Oled
{
  public:
    Oled(int cs, int rst,int dc,int clk,int din);
    //OLED��ʼ������
    void OLED_Init(void);

    //OLED�������ú�����
    void Set_Start_Column(unsigned char d);//����ҳģʽ��ʼ��
    void Set_Addressing_Mode(unsigned char d); //�����Դ�Ѱַģʽ
    void Set_Column_Address(unsigned char a, unsigned char b);  //�����е�ַ  ˮƽѰַģʽ��ʹ��
    void Set_Page_Address(unsigned char a, unsigned char b);    //����ҳ��ַ  ��ֱѰַģʽ��ʹ��
    void Set_Start_Line(unsigned char d); //��ʾ��ʼ�ж�Ӧ�Դ�ĵ�ַ
    void Set_Contrast_Control(unsigned char d); //���öԱȶ� 0x00~0xff��������
    void Set_Segment_Remap(unsigned char d);     //�е�ַ��ӳ��
    void Set_Entire_Display(unsigned char d); //�����Դ���ʾ������ʾǿ��ȫ��
    void Set_Inverse_Display(unsigned char d); //���÷�����ʾ������RAM 1��ʾ������
    void Set_Multiplex_Ratio(unsigned char d);   //����ռ�ձ�
    void Set_Display_On_Off(unsigned char d); //���ÿ�����ʾ���غ�OLED����˯��ģʽ
    void Set_Start_Page(unsigned char d);  //����ҳģʽ�µ�ҳ��ʼ��ַ
    void Set_Common_Remap(unsigned char d); //��������ӳ��
    void Set_Display_Offset(unsigned char d);//������ƫ���� Ĭ��0
    void Set_Display_Clock(unsigned char d);      //������ʾʱ��
    void Set_Precharge_Period(unsigned char d); //����Ԥ���ʱ��
    void Set_Common_Config(unsigned char d) ;  //����com pin��Ӳ������
    void Set_VCOMH(unsigned char d) ; //����VCOMH��ѹ����ѹ
    void Set_Command_Lock(unsigned char d); //���������� 0x16����Ӧָ�� ����Ϊ 0x12

    /*IO��ģ��SPI�ӿ�ͨѶ���� www.lcdsoc.com 2��*/
    void Write_Command(unsigned char Data);//ģ��SPI�ӿ���OLEDд����
    void Write_Data(unsigned char Data);//ģ��SPI�ӿ���OLEDд����

    /******************OLED��ʾӦ�ú�����*www.lcdsoc.com***********************/
    void Set_Pos(unsigned char x, unsigned char y);//������ʾλ��
    void Set_Pixel(unsigned char x,unsigned char y);//��һ����
    void Asc6_8(unsigned char x,unsigned char y,unsigned char ch[]);//д��һ���׼ASCII�ַ��� 6*8
    void Asc8_16(unsigned char x,unsigned char y,unsigned char ch[]);//д��һ���׼ASCII�ַ��� 8*16
    void Show_Line( unsigned char a, unsigned char b, unsigned char c,unsigned char Data);// ��one page ��Χ�ڻ�����
    void Draw_Rectangle(unsigned char p1,unsigned char p2,unsigned char x1, unsigned char x2);//  �����ο�
    void HZ16_16( unsigned char x, unsigned char y, unsigned char num);//��ʾ16*16������
    void Show_HZ16_16(unsigned char  x,unsigned char  y,unsigned char num1,unsigned char num2);//д��һ��16*16����
    void HZ12_16( unsigned char x, unsigned char y, unsigned char num); //  ��ʾ12*16������
    void Show_HZ12_16(unsigned char  x,unsigned char  y,unsigned char num1,unsigned char num2);// д��һ��12*16����
    void Fill_RAM(unsigned char Data);//ȫ�����
    void Fill_Block(unsigned char Data, unsigned char a, unsigned char b, unsigned char c, unsigned char d);//���ֻ���ȫ�����
    void Show_Pattern(unsigned char *Data_Pointer, unsigned char a, unsigned char b, unsigned char c, unsigned char d);// ���ֻ���ȫ����ʾͼƬ
    void Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c);//��ֱ����
    void Horizontal_Scroll(unsigned char DIRECTION,unsigned char p1, unsigned char p2, unsigned char t, unsigned char c1,unsigned char c2);//ˮƽ����
    void Deactivate_Scroll(void);//  ֹͣˮƽ���ߴ�ֱ������ֹͣRAM���ݱ�����д
    void Fade_In(void);//����
    void Fade_Out(void);//����
    void Sleep(unsigned char a);//˯��ģʽ
    void displayoff(void);
    void displayon(void);

  private:
    int _cs;
    int _rst;
    int _dc;
    int _clk;
    int _din;
};

 #endif
