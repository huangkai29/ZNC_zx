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
#define F "wd14.txt"


#define Img_Col 160 //ͼ����
#define servo_freq 50
#define servo_FTM FTM0
#define servo_CH FTM_CH0
#define servo_pwm_middle  3400    // �����ֵ
#define servo_pwm_max  4200      // ���ƫת���ֵ
#define servo_pwm_min  2600 
#define I i-36 
#define White_Line_Min 20   //��С�������
#define White_Line_Max 160   //����������
#define N (i-1)*160+(j-1) //��ά����ת��Ϊһά�����Ӧ����  

float KP=23;//����������ϵ��
float KD=0.08; //5.0;//�������΢��ϵ��
uint16 Fit_Middleline[161];

//��������
int xz[85]={10,11,11,12,12,12,13,14,15,15,16,17,17,18,19,19,20,21,21,22,23,23,24,24,25,26,26,27,28,28,29,30,30,31,32,32,33,34,34,35,36,36,37,38,38,39,39,40,40,42,42,43,43,44,44,44,45,46,46,47,48,48,49,49,50,50,51,51,52,53,53,53,54,54,55,55,56,56,56,57,58,58,59,59,59};


int get_centerline(uint8 img[19200])    //  ��ȡ����
{

   uint8 Left_Black;
   uint8 Right_Black;
   uint8 Middleline=80;  
   uint8 Left_Black_Flag=0; 
   uint8 Right_Black_Flag=0;
   uint8 First_lost=1;
   uint8 i,j;
   uint8 C=0; //ƫ��ϵ��
   
   for(i=120;i>=1;i--)  //����ͼ���ÿһ�ж�����ɨ�裨Ч�ʵͣ�
   {
   	
    for(j=Middleline-C;j>1;j--)  // ���м������������Ѱ�Һڵ�
    {
      
      
      if(img[N]==1 && img[N-1]==0)
      {

        Left_Black=j;       // �ҵ���ߺڵ�

        Left_Black_Flag++;  //�ҵ��������Left_Black_Flag��1 �����ں���·�����͵��ж�

        break;
      }
      else
      {
      	
        Left_Black=j;  // δ�ҵ���ߺڵ�
        Left_Black_Flag=0;
      }
    }
    
    for(j=(Middleline+1+C);j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
    {
      

      if(img[N]==1 && img[N+1]==0)
      {

        Right_Black=j;         //�ҵ��ұߺڵ�

        Right_Black_Flag++;
        break; 
      }
      else
      {
        Right_Black=j;   //δ�ҵ��ұߺڵ�
        Right_Black_Flag=0;
      }
      	          
    }
    //������в�ȫΪ��ɫ�����ó� 
    if((Right_Black_Flag==0 && Left_Black_Flag==0) && (i==120 || i==119 || i==118 || i==117 || i==116 || i==115))
    {
    	return 0;
	}

      
    //////////////////////  ��·�ж�    -------------///////////////////////
     ////////////          ����ֱ��    ///////////////////
    if(Left_Black_Flag==1 && Right_Black_Flag==1)    //�ҵ�˫�ߺ���, ����ֱ����
    {
      if((Right_Black-Left_Black>=White_Line_Min)&& (Right_Black-Left_Black<=White_Line_Max)) //ʹ���߿���ڹ涨�ķ�Χ�ڣ����޶������Ŀ�ȣ���Ϊ�ɼ����ܹ���80�еģ���֪���������߲�ֵӦ��0~80֮��
       {                                                                                  
         Middleline=(Left_Black + Right_Black)/2;
        Fit_Middleline[i]=Middleline;
        
      }
    }
    /////////////   ֱ������     /////////////////////
    /////////////   ���������   /////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==1)   //ֻ�ҵ��Ҳ���ߣ�֤������������
    {
      if((Right_Black >=White_Line_Min) && (Right_Black <=White_Line_Max))
       {
         Middleline=Right_Black-xz[I];
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   //////////////////// 
    /////////////   ���������   /////////////////////
    else if(Left_Black_Flag==1 && Right_Black_Flag==0)   //ֻ�ҵ������ߣ�֤������������
    {
      if((Img_Col -Left_Black >=White_Line_Min) && (Img_Col -Left_Black <=White_Line_Max))
       {
         Middleline=Left_Black+xz[I];
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   ////////////////////
    ////////////    ����ʮ��·�ڻ��߶�ʧ���ߺ���   /////////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==0)   //���߶�û�ҵ����ߣ����������У�������һ�е�ֵ
    {
    	
     	if(First_lost==1)
     	{
     		
     		Middleline=Fit_Middleline[i+6];
     		Fit_Middleline[i]=Middleline;
     		First_lost=0;
     		
		}
     	else
     	{
     //		Left_Black=Left_Black_Old;
      //   	Right_Black=Right_Black_Old;
         	Middleline=Fit_Middleline[i+1];
         	Fit_Middleline[i]=Middleline;
		}
         
    }  
    if(Fit_Middleline[i]!=0)
    	img[(i-1)*160+(Fit_Middleline[i]-1)]=0; //������ 
    
 //   C=(Right_Black-Left_Black)/2-5;
	printf("%d ",Right_Black-Left_Black);
 }
  return 8;
}
 


//�������

int Servo_Control_PWM;
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   
   for(i=120;i>100;i--)  //���Խ�����50��ȡƽ��ֵ
    SteerSum+=Fit_Middleline[i]-Img_Col/2;
   for(i=100;i>75;i--) //Զ25�Ӵ�Ȩ��
    SteerSum+=(Fit_Middleline[i]-Img_Col/2)*2.5;
   Error=-(int)(SteerSum/45);

  Servo_PWM=KP*Error+KD*(Error-LastError);
  Servo_PWM=Servo_PWM + servo_pwm_middle;
  LastError=Error;
  
  if(Servo_PWM > servo_pwm_max)  //�޶������Ƿ�Χ����ֹ����������������ת���ķ�//Χ��servo_pwm_max����������ת�ǣ��ɵ���ռ�ձ�ʹ�����Ǻ���������ת�ĽǶȶ�//Ӧ�����ռ�ձȣ�������ɵó���������ת�������ռ�ձȺ���Сռ�ձȡ�
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if((Error<2) && (Error>-2))    //ƫ��̫С�Ͳ��ı����Ƕ�
     Servo_PWM=servo_pwm_middle;    //ʹ��ԭ�������ֵ 
   
  Servo_Control_PWM=Servo_PWM;
  return Servo_PWM;
 
}
void main()
{
	uint8 data[19201];
	uint8 img[19200];
	char sa[100]="C:\\Users\\HK\\Desktop\\Desktop\\";
	FILE *fp=fopen(strcat(sa,F),"r");
	if(!fp)
	{
		printf("can't open file\n");
		
	}

	fgets(data,19202,fp);
    
	fclose(fp);
	
	int i;
	 //ת���ɲɼ���ѹ������� 
	for(i=0;i<19200;i++)  
	{
		img[i]=data[i]-'0';
		
	}
		
	get_centerline(img);
	for(i=6400;i<16000;i++)
	{
		if(i%160==0)
			printf("\n%3d:",(i/160)+1);
		printf("%d",img[i]);
		
	}
		
	
	


	
	
	
	
} 
