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
#define File "w2.txt"




#define img_top 41 //ͼ���ϲ� 
#define img_base 100 //ͼ���²� 
#define img_high img_base-img_top+1 //ͼ��߶� 
#define Img_Col 160 //ͼ����

#define servo_pwm_middle  3400    // �����ֵ
#define servo_pwm_max  4200      // ���ƫת���ֵ
#define servo_pwm_min  2600     //���ƫת��Сֵ
#define White_Line_Min 20   //��С�������
#define White_Line_Max 160   //����������
#define N (i-1)*Img_Col+(j-1) //��ά����ת��Ϊһά�����Ӧ����  
#define Xi i-img_top+1 //ʵ������ת������0��ʼ������ 
 
uint8 plotmid=1; //�Ƿ�����  

float KP=30;//����������ϵ��
float KD=0.08; //5.0;//�������΢��ϵ��
uint16 Fit_Middleline[img_high+1];

//��������
int xz[60]={27,29,30,30,32,32,34,36,37,38,40,41,42,43,44,46,48,49,50,52,53,54,56,56,58,60,61,63,63,65,66,67,69,71,71,73,75,76,77,78,80,81,82,83,84,85,87,87,89,91,91,93,93,95,95,97,98,99,100,101};

   int16 Left_Black[img_high+1];
   int16 Right_Black[img_high+1];
int get_centerline(uint8 img[19200])    //  ��ȡ����
{
   uint8 tenflag=0;
   uint8 tenflag2=0;
   uint8 Middleline=80;  
   int16 i,j;

   ////////////////////////////////////////����ǰ����,�ж��Ƿ�����Чͼ��/////////////////////////////
    
   for(i=img_base;i>=img_base-2;i--)  //�����߿�ʼ����ǰ���� 
   {   	
    for(j=Img_Col/2;j>1;j--)  // ���м������������Ѱ�Һڵ�
    {      
      if(img[N]==254 && img[N-1]==0 )
      {     	
        Left_Black[Xi]=j;       // �ҵ���ߺڵ�
        break;
      }
      else
      	Left_Black[Xi]=1;	  
    }
    
    for(j=(Img_Col/2);j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
    {      
      if(img[N]==254 && img[N+1]==0)
      {
        Right_Black[Xi]=j;         //�ҵ��ұߺڵ�
        break; 
      }	        
	  else
	  	Right_Black[Xi]=255;	    
    }
    //���������ȫ���������� 
    if(img[(i-1)*160+(80-1)]==0)
        return 1;
    if((Left_Black[Xi]==1 && Right_Black[Xi]==255) )
      tenflag2++;
    
    	
    else //���������� 
    {
    	if(Left_Black[Xi]==1 && Right_Black[Xi]!=255)
			Left_Black[Xi]=Right_Black[Xi]-xz[Xi-1];
		else if(Left_Black[Xi]!=1 && Right_Black[Xi]==255)	
			Right_Black[Xi]=Left_Black[Xi]+xz[Xi-1];
	}
   
 

 }
  /////////////////////////////ͼ����Ч,�������� ////////////////////////////////////////
   for(i=img_base-3;i>=img_top;i--)  //����Ѱ�� 
   {   
   
   	
   	uint8 oldlb=Left_Black[Xi+1];

	for(j=oldlb+6;j>=oldlb-6;j--)  //���ϴεĵ����� 
	{    	  
	  	if(img[N]==254 && img[N-1]==0 )
	   	{     	
	    	Left_Black[Xi]=j;       
	        break;
    	} 	    	    		
	}
	if(j==oldlb-7) //����Ѱ�㲻�ɣ���Ϊ���ص�ѡ�� 
	{
		
		for(j=Img_Col/2;j>1;j--)  // ���м������������Ѱ�Һڵ�
	    {      
	      if(img[N]==254 && img[N-1]==0 )
	      {     	
	        Left_Black[Xi]=j;       // �ҵ���ߺڵ�
	        break;
	      }
	      else
	      	Left_Black[Xi]=1;	  
	    }
	}
	          
	uint8 oldrb=Right_Black[Xi+1];
	for(j=oldrb-6;j<=oldrb+6;j++)          // ���ϴεĵ����� 
    {      
	 		      			 
	 	 if(img[N]==254 && img[N+1]==0)
	      {
	        Right_Black[Xi]=j;         
	        break; 
	      }	        
    }
    if(j==oldrb+7) //����Ѱ�㲻�ɣ���Ϊ���ص�ѡ�� 
    {
    	
    	for(j=(Img_Col/2);j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
	    {      
	      if(img[N]==254 && img[N+1]==0)
	      {
	        Right_Black[Xi]=j;         //�ҵ��ұߺڵ�
	        break; 
	      }	        
		  else
		  	Right_Black[Xi]=255;	    
	    }	
	}
	

    
    //ʮ��·�ڱ�� 
    uint8 Firmid=(Right_Black[img_base]-Left_Black[img_base])/2;
    if(Left_Black[Xi]==1 && Right_Black[Xi]==255  && i>=img_top+20)
    	if(img[(i-1)*160+(Firmid-1)]==0) //����ͨ· 
    		;
    	else
    		tenflag++;

	//��ʮ��·�ڲ��� ��������ߣ� 
	else  
	{
		if(Left_Black[Xi]==1 && Right_Black[Xi]!=255)
			Left_Black[Xi]=Right_Black[Xi]-xz[Xi-1];
		else if(Left_Black[Xi]!=1 && Right_Black[Xi]==255)	 
			Right_Black[Xi]=Left_Black[Xi]+xz[Xi-1];
	}
    		
 }
 //////////////////////////////////////////�������ұ������鴦���Լ������� ///////////////////////////////////////
 
 	/////////////////ʮ��·�ڶ�ʧ����///////////// 
 	uint8 n;
	uint8 discon=0;
	uint8 LeftZJ=0,RightZJ=0; //��ʮ�ֵı�־ 
 	if(tenflag2>=2)
 	{
 		Fit_Middleline[1]=(Right_Black[1]+Left_Black[1])/2; //��һ������
 		for(n=2;n<=img_high;n++) 
		{
			/////////////ʮ��·�� /////////////////
				
		 		if(!(Left_Black[n]-Left_Black[n-1]<=3 && Left_Black[n]-Left_Black[n-1]>=-3) && LeftZJ==0) //��⵽ֱ�ǣ����Ϊ��ʮ�� 
		 			LeftZJ=Left_Black[n-1]; 
									
		 		if (!(Right_Black[n]-Right_Black[n-1]>=-3 && Right_Black[n]-Right_Black[n-1]<=3) && RightZJ==0)
	
		 			RightZJ=Right_Black[n-1];
	
				
		 					
		 		if(!LeftZJ && !RightZJ) //��ʮ���� 
					Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
				
				else if(LeftZJ || RightZJ)	//����ʮ�������øճ�ʮ�ֵ�������� 
					Fit_Middleline[n]=Fit_Middleline[n-1];
					
				if(Fit_Middleline[n]!=0 && plotmid==1)  //������ 
					img[((n+img_top-1)-1)*160+(Fit_Middleline[n]-1)]=0;	
										
		}
		
 		
	}
 	else
 	{
		/////////////ʮ��·�� /////////////////
		
		Fit_Middleline[img_high]=(Right_Black[img_high]+Left_Black[img_high])/2; //���һ������ 
		
		for(n=img_high-1;n>=1;n--) 
		{
			
		 	if(tenflag>=10) 
		 	{
		 		
		 		
		 		if(!(Left_Black[n]-Left_Black[n+1]>=0 && Left_Black[n]-Left_Black[n+1]<=2) && !LeftZJ) //��⵽ֱ�ǣ����Ϊ��ʮ�� 
		 			LeftZJ=Left_Black[n+1]; 
									
		 		if (!(Right_Black[n]-Right_Black[n+1]<=0 && Right_Black[n]-Right_Black[n+1]>=-2) && !RightZJ)
	
		 			RightZJ=Right_Black[n+1];
	
				
		 					
		 		if(!LeftZJ && !RightZJ) //��ʮ���� 
					Fit_Middleline[n]=(Right_Black[n]+Left_Black[n])/2;
				
				else if(LeftZJ || RightZJ)	//����ʮ�������øճ�ʮ�ֵ�������� 
					Fit_Middleline[n]=Fit_Middleline[n+1];
					
				
				if(LeftZJ && RightZJ) //��ʮ��·�� 
					if((Left_Black[n]>LeftZJ) && (Right_Black[n]<RightZJ) )
					{
						int Midd=(Right_Black[n]+Left_Black[n])/2; //��ǰ�е��������  ��ֵ�ڿ������ 
						if(Midd-Fit_Middleline[n+1]<=10 && Midd-Fit_Middleline[n+1]>=-10 && Midd<=Img_Col && Midd>=0 )
							Fit_Middleline[n]=Midd;		
					}
										
			}
			/////////////������Ѿ����ߣ���ֱ�� ///////////////////
			else 
			{
				
				int Midd=(Right_Black[n]+Left_Black[n])/2; //��ǰ�е��������  ��ֵ�ڿ������ 
				if(Midd-Fit_Middleline[n+1]<=11 && Midd-Fit_Middleline[n+1]>=-11 && Midd<=Img_Col && Midd>=0 )
					Fit_Middleline[n]=Midd;	
	//			else if(Midd-Fit_Middleline[n+4]<=3 && Midd-Fit_Middleline[n+2]>=-3)		//�����һ�в����������������µĵ���һ��	
	//				Fit_Middleline[n]=Midd;							
	//			else if(Midd-Fit_Middleline[n+5]<=4 && Midd-Fit_Middleline[n+3]>=-4)			
	//				Fit_Middleline[n]=Midd;							
				else			
					Fit_Middleline[n]=0;				
				
	//			//����������ֵ6���ڣ�һ������������ֹ������� 
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
		
	if(Fit_Middleline[n]!=0 && plotmid==1)  //������ 
		img[((n+img_top-1)-1)*160+(Fit_Middleline[n]-1)]=0;	
				
	}
 		
	 }

	
//	for(n=img_high;n>=1;n--) 
//		printf("%d:%d,%d\n",n,Right_Black[n],Left_Black[n]);
	
	
	  return 8;
}
 


//�������
int Error,LastError=0;
int servo_control(void)
{
   
   
   int i;
   int SteerSum=0; 
   int Servo_PWM;
   int Nozero=0;
   
   for(i=img_high;i>=1;i--)  //ȡ��Ч�е�ƽ��ֵ
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
  
  if(Servo_PWM > servo_pwm_max)  //�޶������Ƿ�Χ����ֹ����������������ת���ķ�Χ
    Servo_PWM = servo_pwm_max;
  
  if(Servo_PWM < servo_pwm_min)
    Servo_PWM = servo_pwm_min;
  
  if((Error<2) && (Error>-2))    //ƫ��̫С�Ͳ��ı����Ƕ�
     Servo_PWM=servo_pwm_middle;    //ʹ��ԭ�������ֵ 
   
  
   
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
	
//	printf("\n%d",servo_control());
	
	


	
	
	
	
} 

