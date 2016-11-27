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
 
int xz[75]={10,10,11,12,13,13,14,15,15,16,17,17,18,19,20,20,21,22,23,24,24,25,26,27,27,28,29,30,30,31,32,32,33,34,35,35,36,37,37,38,39,40,40,41,42,42,43,43,44,45,45,46,47,48,48,49,50,50,51,52,52,53,54,54,55,55,56,56,57,57,58,58,59,60,60};
int Fit_Middleline[161];
void get_centerline(uint8 img[19200])    //  ��ȡ����
{
   uint8 Left_Black,Left_Black_Old;
   uint8 Right_Black,Right_Black_Old;
   uint8 Middleline=80;  
   uint8 Left_Black_Flag=0; 
   uint8 Right_Black_Flag=0;
   uint8 i,j;
   uint8 C=0; //ƫ��ϵ��
   
   for(i=120;i>1;i--)  //����ͼ���ÿһ�ж�����ɨ�裨Ч�ʵͣ�
   {
   	
    for(j=Middleline-C;j>1;j--)  // ���м������������Ѱ�Һڵ�
    {
      
      
      if(img[N]==1 && img[N-1]==0)
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
    
    for(j=(Middleline+1+C);j<160;j++)          // ���м����ұ�������Ѱ�Һڵ�
    {
      

      if(img[N]==1 && img[N+1]==0)
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
         Middleline=Right_Black-xz[i-46];
         Fit_Middleline[i]=Middleline;
       }
    }
    /////////////   ���������   //////////////////// 
    /////////////   ���������   /////////////////////
    else if(Left_Black_Flag==1 && Right_Black_Flag==0)   //ֻ�ҵ������ߣ�֤������������
    {
      if((Img_Col -Left_Black >=White_Line_Min) && (Img_Col -Left_Black <=White_Line_Max))
       {
         Middleline=Left_Black+xz[i-46];
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
    img[(i-1)*160+(Fit_Middleline[i]-1)]=0; //������ 
    
 //   C=(Right_Black-Left_Black)/2-5;

 }
}
 


void main()
{
	uint8 data[19201];
	uint8 img[19200];
	FILE *fp=fopen("C:\\Users\\HK\\Desktop\\Desktop\\dd1.txt","r");
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
