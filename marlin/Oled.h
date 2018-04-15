#ifndef Oled_h
#define Oled_h

class Oled
{
  public:
    Oled(int cs, int rst,int dc,int clk,int din);
    //OLED初始化函数
    void OLED_Init(void);

    //OLED命令设置函数集
    void Set_Start_Column(unsigned char d);//设置页模式开始列
    void Set_Addressing_Mode(unsigned char d); //设置显存寻址模式
    void Set_Column_Address(unsigned char a, unsigned char b);  //设置列地址  水平寻址模式中使用
    void Set_Page_Address(unsigned char a, unsigned char b);    //设置页地址  垂直寻址模式中使用
    void Set_Start_Line(unsigned char d); //显示起始行对应显存的地址
    void Set_Contrast_Control(unsigned char d); //设置对比度 0x00~0xff（最亮）
    void Set_Segment_Remap(unsigned char d);     //列地址重映射
    void Set_Entire_Display(unsigned char d); //设置显存显示或者显示强制全亮
    void Set_Inverse_Display(unsigned char d); //设置反向显示，正常RAM 1表示像素亮
    void Set_Multiplex_Ratio(unsigned char d);   //设置占空比
    void Set_Display_On_Off(unsigned char d); //设置开关显示，关后OLED进入睡眠模式
    void Set_Start_Page(unsigned char d);  //设置页模式下的页开始地址
    void Set_Common_Remap(unsigned char d); //设置行重映射
    void Set_Display_Offset(unsigned char d);//设置行偏移量 默认0
    void Set_Display_Clock(unsigned char d);      //设置显示时钟
    void Set_Precharge_Period(unsigned char d); //设置预充电时间
    void Set_Common_Config(unsigned char d) ;  //行与com pin的硬件连接
    void Set_VCOMH(unsigned char d) ; //设置VCOMH稳压器电压
    void Set_Command_Lock(unsigned char d); //设置命令锁 0x16不响应指令 解锁为 0x12

    /*IO口模拟SPI接口通讯函数 www.lcdsoc.com 2个*/
    void Write_Command(unsigned char Data);//模拟SPI接口向OLED写命令
    void Write_Data(unsigned char Data);//模拟SPI接口向OLED写数据

    /******************OLED显示应用函数集*www.lcdsoc.com***********************/
    void Set_Pos(unsigned char x, unsigned char y);//设置显示位置
    void Set_Pixel(unsigned char x,unsigned char y);//置一个点
    void Asc6_8(unsigned char x,unsigned char y,unsigned char ch[]);//写入一组标准ASCII字符串 6*8
    void Asc8_16(unsigned char x,unsigned char y,unsigned char ch[]);//写入一组标准ASCII字符串 8*16
    void Show_Line( unsigned char a, unsigned char b, unsigned char c,unsigned char Data);// 在one page 范围内画横线
    void Draw_Rectangle(unsigned char p1,unsigned char p2,unsigned char x1, unsigned char x2);//  画矩形框
    void HZ16_16( unsigned char x, unsigned char y, unsigned char num);//显示16*16点阵汉字
    void Show_HZ16_16(unsigned char  x,unsigned char  y,unsigned char num1,unsigned char num2);//写入一串16*16汉字
    void HZ12_16( unsigned char x, unsigned char y, unsigned char num); //  显示12*16点阵汉字
    void Show_HZ12_16(unsigned char  x,unsigned char  y,unsigned char num1,unsigned char num2);// 写入一串12*16汉字
    void Fill_RAM(unsigned char Data);//全屏填充
    void Fill_Block(unsigned char Data, unsigned char a, unsigned char b, unsigned char c, unsigned char d);//部分或者全屏填充
    void Show_Pattern(unsigned char *Data_Pointer, unsigned char a, unsigned char b, unsigned char c, unsigned char d);// 部分或者全屏显示图片
    void Vertical_Scroll(unsigned char a, unsigned char b, unsigned char c);//垂直滚动
    void Horizontal_Scroll(unsigned char DIRECTION,unsigned char p1, unsigned char p2, unsigned char t, unsigned char c1,unsigned char c2);//水平滚动
    void Deactivate_Scroll(void);//  停止水平或者垂直滚动，停止RAM数据必须重写
    void Fade_In(void);//淡入
    void Fade_Out(void);//淡出
    void Sleep(unsigned char a);//睡眠模式
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
