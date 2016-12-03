﻿typedef unsigned char       uint8;  /*  8 bits */
typedef unsigned short int  uint16; /* 16 bits */
typedef unsigned long int   uint32; /* 32 bits */
typedef unsigned long long  uint64; /* 64 bits */
typedef char                int8;   /*  8 bits */
typedef short int           int16;  /* 16 bits */
typedef long  int           int32;  /* 32 bits */
typedef long  long          int64;  /* 64 bits */


#include <stdio.h>
#include <string.h>
#define File "dd.txt"

#define img_top 41 //图像上部 
#define img_base 100 //图像下部 
#define img_high img_base-img_top+1 //图像高度 
#define Img_Col 160 //图像宽度

#define servo_pwm_middle  3400    // 舵机中值
#define servo_pwm_max  4200      // 舵机偏转最大值
#define servo_pwm_min  2600     //舵机偏转最小值
#define White_Line_Min 20   //最小赛道宽度
#define White_Line_Max 160   //最大赛道宽度
#define N (i-1)*160+(j-1) //二维坐标转换为一维数组对应数据  
#define Xi i-img_top+1 //实际行数转换到从0开始的行数 
 
float KP=23;//舵机方向比例系数
float KD=0.08; //5.0;//舵机方向微分系数
uint16 Fit_Middleline[img_high+1];

//修正数组
int xz[60]={25,27,29,30,31,33,34,35,36,38,38,40,42,43,44,46,47,49,49,51,52,53,55,56,57,59,60,61,63,64,65,66,68,69,70,72,73,74,76,77,79,79,81,81,84,84,86,87,88,89,89,91,93,93,95,96,97,98,99,101};

uint8 tenflag=0;
int get_centerline(uint8 img[19200])    //  提取黑线
{

   uint8 Left_Black[img_high+1];
   uint8 Right_Black[img_high+1];
   uint8 Middleline=80;  
   int16 i,j;

   ////////////////////////////////////////搜索前三行,判断是否是有效图像/////////////////////////////
    
   for(i=img_base;i>=img_base-2;i--)  //从中线开始搜索前三行 
   {   	
    for(j=Img_Col/2;j>1;j--)  // 从中间向左边搜索，寻找黑点
    {      
      if(img[N]==254 && img[N-1]==0 )
      {     	
        Left_Black[Xi]=j;       // 找到左边黑点
        break;
      }
      else
      	Left_Black[Xi]=1;	  
    }
    
    for(j=(Img_Col/2);j<160;j++)          // 从中间向右边搜索，寻找黑点
    {      
      if(img[N]==254 && img[N+1]==0)
      {
        Right_Black[Xi]=j;         //找到右边黑点
        break; 
      }	        
	  else
	  	Right_Black[Xi]=255;	    
    }
    //最近三行有全黑行则舍弃 
    if(Left_Black[Xi]==1 && Right_Black[Xi]==255 && img[(i-1)*160+(80-1)]!=0)
    	return 0;
    else //不舍弃则补线 
    {
    	if(Left_Black[Xi]==1 && Right_Black[Xi]!=255)
			Left_Black[Xi]=Right_Black[Xi]-xz[Xi-1];
		else if(Left_Black[Xi]!=1 && Right_Black[Xi]==255)	
			Right_Black[Xi]=Left_Black[Xi]+xz[Xi-1];
	}
   
 //   if(Fit_Middleline[i]!=0)
//    	img[(i-1)*160+(Fit_Middleline[i]-1)]=0; //画黑线    

 }
 
  /////////////////////////////图像有效,继续搜索 ////////////////////////////////////////
   for(i=img_base-3;i>=img_top;i--)  //边沿寻点 
   {   
   
   	
   	uint8 oldlb=Left_Black[Xi+1];

	for(j=oldlb+6;j>=oldlb-6;j--)  //从上次的点搜索 
	{    	  
	  	if(img[N]==254 && img[N-1]==0 )
	   	{     	
	    	Left_Black[Xi]=j;       
	        break;
    	} 	    	    		
	}
	if(j==oldlb-7) //边沿寻点不成，改为朴素的选点 
	{
		
		for(j=Img_Col/2;j>1;j--)  // 从中间向左边搜索，寻找黑点
	    {      
	      if(img[N]==254 && img[N-1]==0 )
	      {     	
	        Left_Black[Xi]=j;       // 找到左边黑点
	        break;
	      }
	      else
	      	Left_Black[Xi]=1;	  
	    }
	}
	          
	uint8 oldrb=Right_Black[Xi+1];
	for(j=oldrb-6;j<=oldrb+6;j++)          // 从上次的点搜索 
    {      
	 		      			 
	 	 if(img[N]==254 && img[N+1]==0)
	      {
	        Right_Black[Xi]=j;         
	        break; 
	      }	        
    }
    if(j==oldrb+7) //边沿寻点不成，改为朴素的选点 
    {
    	
    	for(j=(Img_Col/2);j<160;j++)          // 从中间向右边搜索，寻找黑点
	    {      
	      if(img[N]==254 && img[N+1]==0)
	      {
	        Right_Black[Xi]=j;         //找到右边黑点
	        break; 
	      }	        
		  else
		  	Right_Black[Xi]=255;	    
	    }	
	}
	

    
    //十字路口标记 
    if(Left_Black[Xi]==1 && Right_Black[Xi]==255 && img[(i-1)*160+(80-1)]!=0)
    	tenflag++;

	//非十字路口补线 （弯道补线） 
	else if(tenflag<=3) 
	{
		if(Left_Black[Xi]==1 && Right_Black[Xi]!=255)
			Left_Black[Xi]=Right_Black[Xi]-xz[Xi-1];
		else if(Left_Black[Xi]!=1 && Right_Black[Xi]==255)	 
			Right_Black[Xi]=Left_Black[Xi]+xz[Xi-1];
	}
    		
 }
 //////////////////////////////////////////赛道左右边线数组处理以及画中线 ///////////////////////////////////////
	uint8 n;
	uint8 discon=0;
	uint8 LeftZJ,RightZJ; //出十字的标志 
	
	Fit_Middleline[img_high]=(Right_Black[img_high]+Left_Black[img_high])/2; //最后一行中线 
	
	
	for(n=img_high-1;n>=1;n--) 
	{
		/////////////十字路口 /////////////////
	 	if(tenflag>=3) 
	 	{
	 		
	 		
	 		if(!(Left_Black[n]-Left_Black[n+1]>=0 && Left_Black[n]-Left_Black[n+1]<=2) && !LeftZJ) //检测到直角，标记为出十字 
	 			LeftZJ=Left_Black[n+1]; 
								
	 		if (!(Right_Black[n]-Right_Black[n+1]<=0 && Right_Black[n]-Right_Black[n+1]>=-2) && !RightZJ)

	 			RightZJ=Right_Black[n+1];

			
	 					
	 		if(!LeftZJ && !RightZJ) //非十字区 
				Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
			
			else if(LeftZJ || RightZJ)	//进入十字区，用刚出十字的中线拟合 
				Fit_Middleline[n]=Fit_Middleline[n+1];
				
			
			if(LeftZJ && RightZJ) //出十字路口 
				if((Left_Black[n]>LeftZJ) && (Right_Black[n]<RightZJ) )
				{
					Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
					printf("sdsadsa\n\n\n");
				}
						
				
					
			
			
			
			
					
		}
		/////////////弯道（已经补线）和直道 ///////////////////
		else 
		{
			
			int Midd=(Right_Black[n]+Left_Black[n])/2; //当前行的拟合中线 
			if(Midd-Fit_Middleline[n+1]<=3 && Midd-Fit_Middleline[n+1]>=-3)
				Fit_Middleline[n]=Midd;	
			else if(Midd-Fit_Middleline[n+2]<=3 && Midd-Fit_Middleline[n+2]>=-3)		//与底下一行不连续，则搜索底下的底下一行	
				Fit_Middleline[n]=Midd;							
			else if(Midd-Fit_Middleline[n+3]<=4 && Midd-Fit_Middleline[n+3]>=-4)			
				Fit_Middleline[n]=Midd;							
			else			
				Fit_Middleline[n]=0;				
			
//			//赛道连续差值6以内，一旦不连续则终止中线拟合 
//			if((Left_Black[n]-Left_Black[n+1]>=-6 && Left_Black[n]-Left_Black[n+1]<=6) && Right_Black[n]-Right_Black[n+1]<=6 && Right_Black[n]-Right_Black[n+1]>=-6 && discon==0)
//	 		{
//	 			Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
//	 			
//			}
//			else
//			{
//				Fit_Middleline[n]=0;
//				discon=1;
//			}
//				
		}
		
	if(Fit_Middleline[n]!=0)  //画中线 
		img[((n+img_top-1)-1)*160+(Fit_Middleline[n]-1)]=0;	
				
	}
	
	for(n=img_high;n>=1;n--) 
		printf("%d:%d %d\n",n,Left_Black[n],Right_Black[n]);
	
	
	  return 8;
}
 


//舵机控制
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   int Nozero=0;
   
   for(i=img_high;i>=1;i--)  //取有效行的平均值
   {
   	if(Fit_Middleline[i]!=0)
   	{
   		
   		SteerSum=SteerSum+(Fit_Middleline[i]-Img_Col/2);
   		Nozero++;
   		
   		
	}
   } 
   
   Error=-(int)(SteerSum/Nozero);

  Servo_PWM=KP*Error+KD*(Error-LastError);
  Servo_PWM=Servo_PWM + servo_pwm_middle;
  LastError=Error;
  
  if(Servo_PWM > servo_pwm_max)  //限定舵机打角范围，防止超过两个轮子所能转过的范围
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if((Error<2) && (Error>-2))    //偏差太小就不改变舵机角度
     Servo_PWM=servo_pwm_middle;    //使用原来舵机的值 
   
  
   
  return Servo_PWM;

}



void main()
{
	uint8 data[19201];
	uint8 img[19200];
	char sa[100]="C:\\Users\\HK\\Desktop\\Desktop\\";
	FILE *fp=fopen(strcat(sa,File),"r");
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
		if(data[i]=='1')
			img[i]=254;
		else if(data[i]=='0')
			img[i]=0;
		
	}
		
	get_centerline(img);
	for(i=(img_top-1)*160;i<img_base*160;i++)
	{
		if(i%160==0)
			printf("\n%3d:",((i)/160-img_top+1)+1);
		if(img[i]==254)
			printf("1");
		else if(img[i]==0)
			printf("0");
		
	}
	
	printf("\n%d",servo_control());
	
	


	
	
	
	
} 
