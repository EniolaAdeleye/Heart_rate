/********************************************************************************************************************
                                         main Implementation
*********************************************************************************************************************
*
*           AUTHOR: Anass KAZIZ (kaziz.anas@gmail.com)
*           FILENAME: main.cpp
*           LAST MODIFIED: 02/01/2018
*           TARGET: MK64FN1M0VDC12 (CPU: ARM® Cortex®-M4 32-bit core)
*           COMPILER: MBED
*   
*           The algorithm of Heart rate calculation has been taken from MAX30102 code, which was developed by Maxim Inc.
*
*********************************************************************************************************************/

#include "mbed.h"
#include "algorithm.h"
#include "MAX30101.h"
#include "Hexi_OLED_SSD1351.h"
#include "string.h"
#include "stdio.h"

Serial pc(USBTX, USBRX);

#define MAX_BRIGHTNESS 255

uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;



DigitalOut led1(LED1);

SSD1351 oled(PTB22,PTB21,PTC13,PTB20,PTE6, PTD15);

MAX30101 max30101(PTB1,PTB0);
I2C i2c_(PTB1,PTB0);

DigitalOut powerEN1 (PTA29); // Power Enable MAX30101 Sensor
DigitalOut powerEN2 (PTB12); // BOOST_EN
DigitalOut powerEN3 (PTC13); // 3V3B_EN


DigitalIn INT(PTB18);

char text[20];
const uint8_t *image1;

int main() { 

    powerEN1 = 1;  
    powerEN2 = 0;         
    powerEN3 = 1; 

    uint32_t un_min, un_max, un_prev_data;  //variables to calculate the on-board LED brightness that reflects the heartbeats
    int i;
    int32_t n_brightness;
    float f_temp;
    
    oled_text_properties_t textProperties = {0};
    oled.GetTextProperties(&textProperties);    

    oled.DimScreenON();
        
    oled.FillScreen(COLOR_BLACK);
    
    //image1  = Heartimage;

    /* Turn on the backlight of the OLED Display */
    // oled.DimScreenON();
    
    /* Fill 96px by 96px Screen with 96px by 96px NXP Image starting at x=0,y=0 */
    //oled.DrawImage(image1,0,0);
    
    strcpy((char *) text,"Heart Rate Test");
    oled.Label((uint8_t *)text,5,10);    
    
    
    max30101.maxim_max30101_reset(); //resets the max30101

    wait(1);
    
    //read and clear status register
    max30101.maxim_max30101_read_reg(0x00,&uch_dummy);
    
    //uch_dummy=getchar();
    
    max30101.maxim_max30101_init();  //initializes the max30101       
        
    n_brightness=0;
    un_min=0x3FFFF;
    un_max=0;
  
    n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
    
    //read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(INT.read()==1);   //wait until the interrupt pin asserts
        
        max30101.maxim_max30101_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from max30101 FIFO
            
        if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];    //update signal min
        if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];    //update signal max

    }
    un_prev_data=aun_red_buffer[i];
    
    
    //calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    
    //Continuously taking samples from max30101.  Heart rate and SpO2 are calculated every 1 second
    while(1)
    {
        led1 = !led1;
        
        i=0;
        un_min=0x3FFFF;
        un_max=0;
        
        //dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            
            //update the signal min and max
            if(un_min>aun_red_buffer[i])
            un_min=aun_red_buffer[i];
            if(un_max<aun_red_buffer[i])
            un_max=aun_red_buffer[i];
        }
        
        //take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(INT.read()==1);
            max30101.maxim_max30101_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
        
            if(aun_red_buffer[i]>un_prev_data)
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(un_max-un_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }
//#if defined(TARGET_KL25Z) || defined(TARGET_MAX32600MBED)
//            led.write(1-(float)n_brightness/256);
//#endif
            
            textProperties.fontColor= COLOR_WHITE;
            oled.SetTextProperties(&textProperties);
  
              strcpy(text,"HR=");
              oled.Label((uint8_t *)text,15,40);

              strcpy(text,"bpm");
              oled.Label((uint8_t *)text,65,40);
                
              sprintf(text,"%i",n_heart_rate);
              oled.TextBox((uint8_t *)text,35,40,30,15);                            
                
                pc.printf("Heart Rate : %i\n", n_heart_rate);
               
                
                 

        }
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
    }
}
 