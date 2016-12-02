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
#define File "dd3.txt"

#define img_high img_base-img_top+1
#define img_top 41
#define img_base 100
#define Img_Col 160 //图像宽度
#define servo_freq 50
#define servo_FTM FTM0
#define servo_CH FTM_CH0
#define servo_pwm_middle  3400    // 舵机中值
#define servo_pwm_max  4200      // 舵机偏转最大值
#define servo_pwm_min  2600 
#define I i-36 
#define White_Line_Min 20   //最小赛道宽度
#define White_Line_Max 160   //最大赛道宽度
#define N (i-1)*160+(j-1) //二维坐标转换为一维数组对应数据  

float KP=23;//舵机方向比例系数
float KD=0.08; //5.0;//舵机方向微分系数
uint16 Fit_Middleline[161];

//修正数组
int xz[85]={10,11,11,12,12,12,13,14,15,15,16,17,17,18,19,19,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,34,34,35,36,36,37,38,38,39,39,40,40,42,42,43,43,44,44,44,45,46,46,47,48,48,49,49,50,50,51,51,52,53,53,53,54,54,55,55,56,56,56,57,58,58,59,59,59};

uint8 tenflag=0;
int get_centerline(uint8 img[19200])    //  提取黑线
{

   uint8 Left_Black[img_high+1];
   uint8 Right_Black[img_high+1];
   uint8 Middleline=80;  
   uint8 First_lost=1;
   uint8 i,j;
   uint8 C=0; //偏移系数
   
   for(i=img_base;i>=img_top;i--)  //整幅图像的每一行都进行扫描（效率低）
   {
   	
    for(j=Middleline-C;j>1;j--)  // 从中间向左边搜索，寻找黑点
    {
      
      
      if(img[N]==1 && img[N-1]==0 )
      {
      	

        Left_Black[i-40]=j;       // 找到左边黑点

        break;
      }
      else
      {
      	Left_Black[i-40]=1;
	  }
    }
    
    for(j=(Middleline+1+C);j<160;j++)          // 从中间向右边搜索，寻找黑点
    {
      

      if(img[N]==1 && img[N+1]==0)
      {

        Right_Black[i-40]=j;         //找到右边黑点
        break; 
      }	        
	  else
	  {
	  	Right_Black[i-40]=255;
	  }  
    }
    
    if(Left_Black[i-40]==1 && Right_Black[i-40]==255)
    	tenflag++;
    
    //最近三行不全为黑色舍弃该场 
    if((Right_Black[i-40]==0 && Left_Black[i-40]==0) && (i==img_base || i==img_base-1 || i==img_base-2))
    {
    	return 0;
	}

   
 //   if(Fit_Middleline[i]!=0)
//    	img[(i-1)*160+(Fit_Middleline[i]-1)]=0; //画黑线 
    
 //   C=(Right_Black-Left_Black)/2-5;
	
 }
uint8 n;
uint8 F[4][2]; //F[1][1]左下  F[2][1]右下  F[3][1]左上 F[4][1]右上 
for(n=img_high-1;n>=1;n--) 
{
 	if(tenflag>=3) //十字路口 
 	{
 		if(!(Left_Black[n]-Left_Black[n+1]>=0 && Left_Black[n]-Left_Black[n+1]<=2))
 		{
 			if(F[1][1]==0) //左下 
 			{
 				F[1][0]=n+1;
 				F[1][1]=Left_Black[n+1];
 				
			}
 				
 						
		}
 		if (!(Right_Black[n]-Right_Black[n+1]<=0 && Right_Black[n]-Right_Black[n+1]>=-2))
 		{
 			if(F[2][1]==0) //右下 
 			{
 				F[2][0]=n+1;
 				F[2][1]=Right_Black[n+1];
			}
 				
 			
		}	
		
				
	}		
}

 printf("%d:%d\n%d:%d\n",F[1][0],F[1][1],F[2][0],F[2][1]);

 	
 
 
  return 8;
}
 


//舵机控制

int Servo_Control_PWM;
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   
   for(i=120;i>100;i--)  //仅对近处的50行取平均值
    SteerSum+=Fit_Middleline[i]-Img_Col/2;
   for(i=100;i>75;i--) //远25加大权重
    SteerSum+=(Fit_Middleline[i]-Img_Col/2)*2.5;
   Error=-(int)(SteerSum/45);

  Servo_PWM=KP*Error+KD*(Error-LastError);
  Servo_PWM=Servo_PWM + servo_pwm_middle;
  LastError=Error;
  
  if(Servo_PWM > servo_pwm_max)  //限定舵机打角范围，防止超过两个轮子所能转过的范//围，servo_pwm_max是轮子最大的转角，可调节占空比使舵机打角后轮子所能转的角度对//应的最大占空比，这样便可得出轮子所能转过的最大占空比和最小占空比。
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if((Error<2) && (Error>-2))    //偏差太小就不改变舵机角度
     Servo_PWM=servo_pwm_middle;    //使用原来舵机的值 
   
  Servo_Control_PWM=Servo_PWM;
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
		img[i]=data[i]-'0';
		
	}
		
	get_centerline(img);
	for(i=(img_top-1)*160;i<img_base*160;i++)
	{
		if(i%160==0)
			printf("\n%3d:",((i)/160-img_top+1)+1);
		printf("%d",img[i]);
		
	}
	
		
	
	


	
	
	
	
} 
