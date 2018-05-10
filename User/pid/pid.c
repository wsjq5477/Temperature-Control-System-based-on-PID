#include "bsp_adc.h"
#include "bsp_usart.h"
#include "./generaltim/bsp_generaltim.h"
#include "./led/bsp_led.h"

/**************************宏定义********************************/
#define T 1         //PID时间常数 
#define Q 0.05		//系统噪声
#define R 1			//测量噪声
#define x0 25		//状态值初始值
#define p0 0.5		//协方差初始值
#define room_temperture 23
/**************************引用声明********************************/
extern __IO uint16_t ADC_ConvertedValue;	// ADC1转换的电压值通过MDA方式传到SRAM

/**************************全局变量定义********************************/
float Kp = 1.79;  
float Ki = 0.032;
float Kd = 6;

float temperature_data[10];
float temperature_real[3];
float temperature_filter;
float temperature_setting = 35;	
int count = 0;
int tem_flag = 0;
signed int PWM;
signed int R_PWM;			//室温引起的偏移量

/**************************函数声明********************************/
void sort_rise(void);
void temperature_sampling(void);
void temperature_control(void);
void temperature_PID(void);
double KalmanFilter(const float ResrcData);

/*******************************************************
	函数名称:temperature_sampling();
	函数功能:AD采样,并进行反解与滤波得到温度数据
	说明:无
********************************************************/
void temperature_sampling()
{	 
	float ADC_ConvertedValueLocal; 		
	float Rt;
	//float Rt_sim = 110;//测试
	int temperture_mcu;
	
	
	ADC_ConvertedValueLocal = (float)ADC_ConvertedValue/4096*3.3; // 读取转换的AD值,并补偿失调电压
	Rt = 10000/(0.9784736+ADC_ConvertedValueLocal/276.914)-10000-0.5;
	temperature_data[count] = (Rt-100)/0.385;
	
	if(count!=9)
	{
		count++;
	}
	else
	{
		count = 0;
		sort_rise();			//将十次数据升序排列
		temperature_filter = (temperature_data[3]+temperature_data[4]+temperature_data[5]+temperature_data[6])/4;	//取中间4次数据的平均值	
		temperture_mcu = (int)(KalmanFilter(temperature_filter)*10000);	//卡尔曼滤波
		//temperture_mcu = (int)((temperature_filter)*10000);	//不使用卡尔曼滤波
		//printf("%d\n",temperture_mcu);
		if(tem_flag==0)
		{
			temperature_real[0] = 0;							//更新温度采集数据至2
			temperature_real[1] = 0;
			temperature_real[2] = KalmanFilter(temperature_filter);
			if((KalmanFilter(temperature_filter)-room_temperture)<10&&(KalmanFilter(temperature_filter)-room_temperture)>-10)  //数据稳定
			{
				tem_flag = 1;
				printf("%d\n",temperture_mcu);
			}
		}
		else 
		{		
			if(tem_flag==1)
			{
				temperature_real[0] = temperature_real[1];		//更新温度采集数据至1
				temperature_real[1] = temperature_real[2];
				temperature_real[2] = KalmanFilter(temperature_filter);
				R_PWM= (temperature_setting-room_temperture)*25;
				PWM = R_PWM+100*Kp*(temperature_setting-KalmanFilter(temperature_filter));
				tem_flag = 2;
				printf("%d\n",temperture_mcu);
			}
			if(tem_flag==2)
			{
				temperature_real[0] = temperature_real[1];		//更新温度采集数据完毕
				temperature_real[1] = temperature_real[2];
				temperature_real[2] = KalmanFilter(temperature_filter);
				printf("%d",temperture_mcu);	
				temperature_control();	
			}
		}
		/*******************数据发送至串口显示*************************
		printf("Uo = %fV\r\n",ADC_ConvertedValueLocal);
		printf("Rt = %f\r\n",Rt);
		printf("Temperture = %f\r\n",temperature_filter);
		****/
	}
}

/*******************************************************
	函数名称:temperature_control();
	函数功能:用PID算法进行温度控制
	入口参数:无
	出口参数:无
********************************************************/
void temperature_control()
{
	if(temperature_setting-temperature_real[2]>10)
	{
		TIM3->CCR1 = 1800;
		GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
		GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		//printf("升温拉满\r\n");
		printf("0");
		printf("1800\n");
	}
	else if(temperature_real[2]-temperature_setting>10)
	{
		TIM3->CCR1 = 1800;
		GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
		GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		printf("1");
		printf("1800\n");
		//printf("降温拉满\r\n");
	}
	else 
	{
		temperature_PID();	
	}
}

/*************************************************************
	函数名称:temperature_PID();
	函数功能:完成温度的PID控制
	说明:1.首先使Ki=Kd=0,从小到大调节Kp
		 2.当调节到合适值时,令Kp=Kp*5/6,由小到大整定Ki
		 3.达到稳定后调节Kd,一般(1/3-1/4)Ki;
*************************************************************/ 
void temperature_PID()
{
    int D_PWM;
	int temp_PWM;
    float A,B,C;
    
    A = Kp*(1+Ki+Kd);
    B = Kp*(1+2*Kd);
    C = Kp*(Kd);

	/***************************************积分分离PID算法*********************************************/
	if(temperature_setting-temperature_real[2]>0.5||temperature_setting-temperature_real[2]<-0.5)	//温差大于3℃使用PD控制
	{
		PWM = 100*(Kp*(temperature_setting-temperature_real[2])+Kd*(temperature_real[2]-temperature_real[1]))+R_PWM;
	}
	else
	{
		//D_PWM = (signed int)((A*(temperature_setting-temperature_real[2])-B*(temperature_setting-temperature_real[1])+C*(temperature_setting-temperature_real[0]))*100);
		PWM = 100*Kp*(temperature_setting-temperature_real[2])+R_PWM;
	}
	
	/********************************************限幅***************************************************/
	if(temperature_setting-temperature_real[2]>0)
	{
		/*
		if(temperature_setting-temperature_real[2]<4)
		{
			PWM = 100*(Kp*(temperature_setting-temperature_real[2]))+R_PWM;
			temp_PWM = R_PWM;
			TIM3->CCR1 = temp_PWM;
			GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
			GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		}
		*/
		if(PWM>1800)
		{
			temp_PWM = 1800;
			TIM3->CCR1 = temp_PWM;
			GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
			GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		}
		else if(PWM>0)
		{
			temp_PWM = PWM;
			TIM3->CCR1 = PWM;
			GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
			GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		}
		printf("0");
		printf("%d\n",temp_PWM);
	}
	else
	{
		//PWM = 100*(Kp*(temperature_setting-temperature_real[2]))+R_PWM;
		if(PWM<-1800)
		{		
			temp_PWM = 1800;
			TIM3->CCR1 = temp_PWM;
			GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
			GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		}
		else if(PWM<0)
		{
			temp_PWM = -PWM;
			TIM3->CCR1 = -PWM;
			GPIO_ResetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
			GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		}	
		printf("1");
		printf("%d\n",temp_PWM);
	}	
}

/*******************************************************
	函数名称:sort_rise();
	函数功能:升序排序，由小到大
	入口参数:无
	出口参数:无
********************************************************/
void sort_rise()
{
	int i,j;
	float temp;
	
	for(i=0;i<10;i++)
	{
		for(j=0;j<10-i-1;j++)
		{
			if(temperature_data[j]>temperature_data[j+1])
			{
				temp = temperature_data[j];		
				temperature_data[j] = temperature_data[j+1];
				temperature_data[j+1] = temp;
			}
		}
	}
}
/*******************************************************
	函数名称:KalmanFilter(const double ResrcData);
	函数功能:卡尔曼滤波
	入口参数:系统测量值
	出口参数:卡尔曼滤波预测值
********************************************************/
double KalmanFilter(const float ResrcData)
{
		double temp = (double)ResrcData;
        static double x_last = x0;		//状态值
        double x_mid;		
        double x_now;		
	
		static double p_last = p0;		//协方差	
        double p_mid ;				
        double p_now;	
	
        double kg;   				//卡尔曼增益    

        x_mid=x_last; 						//x_last=x(k-1|k-1),x_mid=x(k|k-1)
        p_mid=p_last+Q; 					//p_mid=p(k|k-1),p_last=p(k-1|k-1),Q为噪声
        
		/************Kalman滤波五公式***************/
		kg=p_mid/(p_mid+R); 				//kg为kalman filter,R为噪声
        x_now=x_mid+kg*(temp-x_mid);		//估计的最优值
        p_now=(1-kg)*p_mid;					//最优值对应协方差
        p_last = p_now; 					//更新协方差
        x_last = x_now; 					//更新状态值

        return x_now;               
}
