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
#define File "wd14.txt"

#define img_high img_base-img_top+1
#define img_top 41
#define img_base 100
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
int xz[60]={25,27,29,30,31,33,34,35,36,38,38,40,42,43,44,46,47,49,49,51,52,53,55,56,57,59,60,61,63,64,65,66,68,69,70,72,73,74,76,77,79,79,81,81,84,84,86,87,88,89,89,91,93,93,95,96,97,98,99,101};

uint8 tenflag=0;
int get_centerline(uint8 img[19200])    //  ��ȡ����
{

   uint8 Left_Black[img_high+1];
   uint8 Right_Black[img_high+1];
   uint8 Middleline=80;  
   int16 i,j;

   ////////////////////////////////////////����ǰ����,�ж��Ƿ�����Чͼ��/////////////////////////////
    
   for(i=img_base;i>=img_base-2;i--)  //�����߿�ʼ����ǰ���� 
   {   	
    for(j=Img_Col/2;j>1;j--)  // ���м������������Ѱ�Һڵ�
    {      
      if(img[N]==1 && img[N-1]==0 )
      {     	
        Left_Black[i-40]=j;       // �ҵ���ߺڵ�
        break;
      }
      else
      	Left_Black[i-40]=1;	  
    }
    
    for(j=(Img_Col/2);j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
    {      
      if(img[N]==1 && img[N+1]==0)
      {
        Right_Black[i-40]=j;         //�ҵ��ұߺڵ�
        break; 
      }	        
	  else
	  	Right_Black[i-40]=255;	    
    }
    //���������ȫ���������� 
    if(Left_Black[i-40]==1 && Right_Black[i-40]==255 && img[(i-1)*160+(80-1)]!=0)
    	return 0;
    else //���������� 
    {
    	if(Left_Black[i-40]==1 && Right_Black[i-40]!=255)
			Left_Black[i-40]=Right_Black[i-40]-xz[i-40-1];
		else if(Left_Black[i-40]!=1 && Right_Black[i-40]==255)	
			Right_Black[i-40]=Left_Black[i-40]+xz[i-40-1];
	}
   
 //   if(Fit_Middleline[i]!=0)
//    	img[(i-1)*160+(Fit_Middleline[i]-1)]=0; //������    

 }
 
  /////////////////////////////ͼ����Ч,�������� ////////////////////////////////////////
   for(i=img_base-3;i>=img_top;i--)  //����Ѱ�� 
   {   
   
   	
   	uint8 oldlb=Left_Black[i-40+1];

	for(j=oldlb+6;j>=oldlb-6;j--)  //���ϴεĵ����� 
	{    	  
	  	if(img[N]==1 && img[N-1]==0 )
	   	{     	
	    	Left_Black[i-40]=j;       
	        break;
    	} 	    	    		
	}
	if(j==oldlb-7) //����Ѱ�㲻�ɣ���Ϊ���ص�ѡ�� 
	{
		
		for(j=Img_Col/2;j>1;j--)  // ���м������������Ѱ�Һڵ�
	    {      
	      if(img[N]==1 && img[N-1]==0 )
	      {     	
	        Left_Black[i-40]=j;       // �ҵ���ߺڵ�
	        break;
	      }
	      else
	      	Left_Black[i-40]=1;	  
	    }
	}
	          
	uint8 oldrb=Right_Black[i-40+1];
	for(j=oldrb-6;j<=oldrb+6;j++)          // ���ϴεĵ����� 
    {      
	 		      			 
	 	 if(img[N]==1 && img[N+1]==0)
	      {
	        Right_Black[i-40]=j;         
	        break; 
	      }	        
    }
    if(j==oldrb+7) //����Ѱ�㲻�ɣ���Ϊ���ص�ѡ�� 
    {
    	
    	for(j=(Img_Col/2);j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
	    {      
	      if(img[N]==1 && img[N+1]==0)
	      {
	        Right_Black[i-40]=j;         //�ҵ��ұߺڵ�
	        break; 
	      }	        
		  else
		  	Right_Black[i-40]=255;	    
	    }	
	}
	

    
    //ʮ��·�ڱ�� 
    if(Left_Black[i-40]==1 && Right_Black[i-40]==255 && img[(i-1)*160+(80-1)]!=0)
    	tenflag++;

	//��ʮ��·�ڲ��� 
	else
	{
		if(Left_Black[i-40]==1 && Right_Black[i-40]!=255)
			Left_Black[i-40]=Right_Black[i-40]-xz[i-40-1];
		else if(Left_Black[i-40]!=1 && Right_Black[i-40]==255)	
			Right_Black[i-40]=Left_Black[i-40]+xz[i-40-1];
	}
    		
 }
 //////////////////////////////////////////�������ұ������鴦���Լ������� ///////////////////////////////////////
	uint8 n;
	uint8 zzFlag=0;
	uint8 discon=0;
	
	for(n=img_high-1;n>=1;n--) 
	{
		
	 	if(tenflag>=3) //ʮ��·�� 
	 	{
	 		
	 		
	 		if(!(Left_Black[n]-Left_Black[n+1]>=0 && Left_Black[n]-Left_Black[n+1]<=2))
	 			zzFlag=1;
	 				 								
	 		if (!(Right_Black[n]-Right_Black[n+1]<=0 && Right_Black[n]-Right_Black[n+1]>=-2))
	 			zzFlag=1;		
	 		if(zzFlag==0)
			{
				Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
			}
			else if(zzFlag==1)
			{
				Fit_Middleline[n]=Fit_Middleline[n+1];
			}
			else
				Fit_Middleline[n]=0;
			
			
			
					
		}
		
		else //������Ѿ����ߣ���ֱ�� 
		{
			//����������ֵ6���ڣ�һ������������ֹ������� 
			if((Left_Black[n]-Left_Black[n+1]>=-6 && Left_Black[n]-Left_Black[n+1]<=6) && Right_Black[n]-Right_Black[n+1]<=6 && Right_Black[n]-Right_Black[n+1]>=-6 && discon==0)
	 		{
	 			Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
	 			
			}
			else
			{
				Fit_Middleline[n]=0;
				discon=1;
			}
				
		}
		
	if(Fit_Middleline[n]!=0) 
		img[(n+40-1)*160+(Fit_Middleline[n]-1)]=0;			
	}
	
	for(n=img_high;n>=1;n--) 
	{
		printf("%d:%d %d\n",n,Left_Black[n],Right_Black[n]);
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
	FILE *fp=fopen(strcat(sa,File),"r");
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
	for(i=(img_top-1)*160;i<img_base*160;i++)
	{
		if(i%160==0)
			printf("\n%3d:",((i)/160-img_top+1)+1);
		printf("%d",img[i]);
		
	}
	
		
	
	


	
	
	
	
} 
