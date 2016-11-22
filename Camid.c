typedef unsigned char       uint8;  /*  8 bits */
typedef unsigned short int  uint16; /* 16 bits */
typedef unsigned long int   uint32; /* 32 bits */
typedef unsigned long long  uint64; /* 64 bits */
typedef char                int8;   /*  8 bits */
typedef short int           int16;  /* 16 bits */
typedef long  int           int32;  /* 32 bits */
typedef long  long          int64;  /* 64 bits */

#include <stdio.h>
#include <string.h>

#define N (i-1)*160+(j-1) //二维坐标转换为一维数组对应数据 
#define Img_Col 180 //图像宽度
#define White_Line_Min 10   //最小赛道宽度
#define White_Line_Max 160   //最大赛道宽度
 
int Fit_Middleline[161];
void get_centerline(uint8 img[19200])    //  提取黑线
{
   uint8 Left_Black,Left_Black_Old;
   uint8 Right_Black,Right_Black_Old;
   uint8 Middleline;  
   uint8 Left_Black_Flag=0; 
   uint8 Right_Black_Flag=0;
   uint8 i,j;
   
   for(i=120;i>1;i--)  //整幅图像的每一行都进行扫描（效率低）
   {
   	
    for(j=80;j>1;j--)  // 从中间向左边搜索，寻找黑点
    {
      
      
      if(img[N]==0 && img[N-1]==1)
      {

        Left_Black=j;       // 找到左边黑点
        Left_Black_Old=Left_Black;  // 保存上一次的值，当下一次未找到黑点时，将上一次的黑点作为本次的黑点
        Left_Black_Flag++;  //找到左黑线则Left_Black_Flag加1 ，用于后面路径类型的判断

        break;
      }
      else
      {
      	
        Left_Black=j;  // 未找到左边黑点
        Left_Black_Flag=0;
      }
    }
    
    for(j=81;j<160;j++)          // 从中间向右边搜索，寻找黑点
    {
      

      if(img[N]==0 && img[N+1]==1)
      {

        Right_Black=j;         //找到右边黑点
        Right_Black_Old=Right_Black;    // 保存上一次的值，当下一次未找到黑点时，将上一次的黑点作为本次的黑点
        Right_Black_Flag++;
        break; 
      }
      else
      {
        Right_Black=j;   //未找到右边黑点
        Right_Black_Flag=0;
      }
      
    }

      
    //////////////////////  道路判断    -------------///////////////////////
     ////////////          进入直道    ///////////////////
    if(Left_Black_Flag==1 && Right_Black_Flag==1)    //找到双边黑线, 车在直道上
    {
      if((Right_Black-Left_Black>=White_Line_Min)&& (Right_Black-Left_Black<=White_Line_Max)) //使黑线宽度在规定的范围内，即限定赛道的宽度，因为采集的总共有80列的，可知赛道的两边差值应在0~80之间
       {                                                                                  
         Middleline=(Left_Black + Right_Black)/2;
        Fit_Middleline[i]=Middleline;
        
      }
    }
    /////////////   直道结束     /////////////////////
    /////////////   进入左弯道   /////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==1)   //只找到右侧黑线，证明黑线向左弯
    {
      if((Right_Black >=White_Line_Min) && (Right_Black <=White_Line_Max))
       {
         Middleline=Right_Black/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   左弯道结束   //////////////////// 
    /////////////   进入右弯道   /////////////////////
    else if(Left_Black_Flag==1 && Right_Black_Flag==0)   //只找到左侧黑线，证明黑线向右弯
    {
      if((Img_Col -Left_Black >=White_Line_Min) && (Img_Col -Left_Black <=White_Line_Max))
       {
         Middleline=(Left_Black+Img_Col)/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   右弯道结束   ////////////////////
    ////////////    进入十字路口或者丢失两边黑线   /////////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==0)   //两边都没找到黑线，则舍弃该行，沿用上一行的值
    {
     
         Left_Black=Left_Black_Old;
         Right_Black=Right_Black_Old;
         Middleline=(Left_Black + Right_Black)/2;
         Fit_Middleline[i]=Middleline;
    }  
    img[(i-1)*160+(Middleline-1)]=1;

 }
}
 


void main()
{
	uint8 data[19201];
	uint8 img[19200];
	FILE *fp=fopen("C:\\Users\\HK\\Desktop\\danhang.txt","r");
	if(!fp)
	{
		printf("can't open file\n");
		
	}

	fgets(data,19202,fp);
    
	fclose(fp);
	
	int i;
	 //转换成采集解压后的数据 
	for(i=0;i<19200;i++)  
	{
		img[i]=data[i]-'0';
		
	}
		
	get_centerline(img);
	for(i=0;i<19200;i++)
	{
		if(i%160==0)
			printf("\n");
		printf("%d",img[i]);
		
	}
		
	
	
	
	
	
	
	
	
	
} 
