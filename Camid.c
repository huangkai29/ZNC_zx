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

#define N (i-1)*160+(j-1) //��ά����ת��Ϊһά�����Ӧ���� 
#define Img_Col 180 //ͼ����
#define White_Line_Min 10   //��С�������
#define White_Line_Max 160   //����������
 
int Fit_Middleline[161];
void get_centerline(uint8 img[19200])    //  ��ȡ����
{
   uint8 Left_Black,Left_Black_Old;
   uint8 Right_Black,Right_Black_Old;
   uint8 Middleline;  
   uint8 Left_Black_Flag=0; 
   uint8 Right_Black_Flag=0;
   uint8 i,j;
   
   for(i=120;i>1;i--)  //����ͼ���ÿһ�ж�����ɨ�裨Ч�ʵͣ�
   {
   	
    for(j=80;j>1;j--)  // ���м������������Ѱ�Һڵ�
    {
      
      
      if(img[N]==0 && img[N-1]==1)
      {

        Left_Black=j;       // �ҵ���ߺڵ�
        Left_Black_Old=Left_Black;  // ������һ�ε�ֵ������һ��δ�ҵ��ڵ�ʱ������һ�εĺڵ���Ϊ���εĺڵ�
        Left_Black_Flag++;  //�ҵ��������Left_Black_Flag��1 �����ں���·�����͵��ж�

        break;
      }
      else
      {
      	
        Left_Black=j;  // δ�ҵ���ߺڵ�
        Left_Black_Flag=0;
      }
    }
    
    for(j=81;j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
    {
      

      if(img[N]==0 && img[N+1]==1)
      {

        Right_Black=j;         //�ҵ��ұߺڵ�
        Right_Black_Old=Right_Black;    // ������һ�ε�ֵ������һ��δ�ҵ��ڵ�ʱ������һ�εĺڵ���Ϊ���εĺڵ�
        Right_Black_Flag++;
        break; 
      }
      else
      {
        Right_Black=j;   //δ�ҵ��ұߺڵ�
        Right_Black_Flag=0;
      }
      
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
         Middleline=Right_Black/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   //////////////////// 
    /////////////   ���������   /////////////////////
    else if(Left_Black_Flag==1 && Right_Black_Flag==0)   //ֻ�ҵ������ߣ�֤������������
    {
      if((Img_Col -Left_Black >=White_Line_Min) && (Img_Col -Left_Black <=White_Line_Max))
       {
         Middleline=(Left_Black+Img_Col)/2;
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   ////////////////////
    ////////////    ����ʮ��·�ڻ��߶�ʧ���ߺ���   /////////////////////////
    else if(Left_Black_Flag==0 && Right_Black_Flag==0)   //���߶�û�ҵ����ߣ����������У�������һ�е�ֵ
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
	 //ת���ɲɼ���ѹ������� 
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
