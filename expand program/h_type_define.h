#ifndef _TYPE_DEFINE_H_
#define _TYPE_DEFINE_H_
//------------------------------------------------------------------------------
//变量类型定义声明
typedef             char      char_t;   

typedef   signed    char      int8_t;
typedef   signed    short     int16_t;
typedef   signed    long      int32_t;

typedef unsigned    char      uint8_t;
typedef unsigned    short     uint16_t;
typedef unsigned    long      uint32_t;
//------------------------------------------------------------------------------
typedef union
{
    int16_t sword;       //按有符号字访问
    struct
    {
        int8_t low;
        int8_t high;
    }byte;               //按有符号字节访问
    
    uint16_t uword;      //按无符号字访问
    struct
    {
        uint8_t low;
        uint8_t high;
    }ubyte;              //按无符号字节访问
}word_type;
//------------------------------------------------------------------------------
typedef union
{
   int32_t  lword;       //按有符号长字访问
   struct 
   {
      int16_t high;
      int16_t low;
   }sword;               //按有符号字访问
   struct 
   {
      int8_t HH;
      int8_t HL;
      int8_t LH;
      int8_t LL;      
   }byte;                //按有符号字节访问
   uint32_t ulword;      //按无符号长字访问
   struct 
   {
      uint16_t high;
      uint16_t low;
   }uword;               //按无符号字访问
   struct 
   {
      uint8_t HH;
      uint8_t HL;
      uint8_t LH;
      uint8_t LL;     
   }ubyte;               //按无符号字节访问
}lword_type;             //定义长字结构体的类型
//------------------------------------------------------------------------------
typedef union flag
{
    uint16_t  uword;
    struct
    {
        uint8_t  bit0:1;
        uint8_t  bit1:1;
        uint8_t  bit2:1;
        uint8_t  bit3:1;
        uint8_t  bit4:1;
        uint8_t  bit5:1;
        uint8_t  bit6:1;
        uint8_t  bit7:1;
        uint8_t  bit8:1;
        uint8_t  bit9:1;
        uint8_t  bit10:1;
        uint8_t  bit11:1;
        uint8_t  bit12:1;
        uint8_t  bit13:1;
        uint8_t  bit14:1;
        uint8_t  bit15:1;
    }bits;
} flag_type;
//------------------------------------------------------------------------------
//宏定义
          #define   SET0      0x01      //00000001b
          #define   SET1      0x02      //00000010b
          #define   SET2      0x04      //00000100b
          #define   SET3      0x08      //00001000b
          #define   SET4      0x10      //00010000b
          #define   SET5      0x20      //00100000b
          #define   SET6      0x40      //01000000b
          #define   SET7      0x80      //10000000b
          #define   CLR0      0xfe      //11111110b
          #define   CLR1      0xfd      //11111101b
          #define   CLR2      0xfb      //11111011b
          #define   CLR3      0xf7      //11110111b
          #define   CLR4      0xef      //11101111b
          #define   CLR5      0xdf      //11011111b
          #define   CLR6      0xbf      //10111111b
          #define   CLR7      0x7f      //01111111b
//------------------------------------------------------------------------------
//外部变量声明
#define   val_row        0
#define   min_row        1
#define   max_row        2
#define   editrate_row   3
#define   disprate_row   4
//------------------------------------------------------------------------------
#define   not_dot        0
#define   dot_0d1        1
#define   dot_0d01       2
#define   dot_0d001      3
#define   dot_0d0001     4
//-------------------------------------------------------------------------------
#endif

