#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_watchdog.h"
#include "iot_gpio.h"
#include "hi_time.h"
#include "hi_io.h"
#include "iot_gpio_ex.h"
#include "iot_pwm.h"
#include "hi_timer.h"
#include "iot_errno.h"
#include "hi_errno.h"
#include "pca9555.h"
#include "hi_i2c.h"
#include "oled_ssd1306.h"
#include "iot_uart.h"
#include "hi_uart.h"


#define DELAY_US20    20
#define DELAY_MS10    10
#define  COUNT   10
#define  FREQ_TIME    20000
#define IOT_PWM_PORT_PWM0   0
#define IOT_PWM_PORT_PWM1   1
#define IOT_PWM_PORT_PWM2   2
#define IOT_PWM_PORT_PWM3   3
#define IOT_FREQ            60000
#define IOT_DUTYR            28
#define IOT_DUTYL            36                                                                                                                       
#define DELAY_US    20
#define IOT_I2C_IDX_BAUDRATE (400 * 1000)
#define CW2015_I2C_IDX 0
#define CW2015_READ_ADDR     (0xC5)
#define CW2015_WRITE_ADDR    (0xC4)
#define WRITELEN  2
#define CW2015_HIGHT_REGISTER 0x02
#define CW2015_LOW_REGISTER   0x03
#define CW2015_WAKE_REGISTER  0x0A
#define DELYA_US20            20


static volatile int g_buttonState = 0;
static volatile int g_buttonPressed = 0;

uint32_t Cw20_WriteRead(uint8_t reg_high_8bit_cmd, uint8_t send_len, uint8_t read_len)
{
    uint32_t status = 0;
    uint32_t ret = 0;
    uint8_t recvData[888] = { 0 };
    hi_i2c_data i2c_write_cmd_addr = { 0 };
    uint8_t send_user_cmd[1] = {reg_high_8bit_cmd};
    memset(recvData, 0x0, sizeof(recvData));
    i2c_write_cmd_addr.send_buf = send_user_cmd;
    i2c_write_cmd_addr.send_len = send_len;

    i2c_write_cmd_addr.receive_buf = recvData;
    i2c_write_cmd_addr.receive_len = read_len;

    status = hi_i2c_writeread(CW2015_I2C_IDX, CW2015_READ_ADDR, &i2c_write_cmd_addr);
    if (status != IOT_SUCCESS) {
        printf("I2cRead() failed, %0X!\n", status);
        return status;
    }
    ret = recvData[0];
    return ret;
}

uint32_t Cw20_Write(uint8_t addr, uint8_t writedata, uint32_t buffLen)
{
    uint8_t buffer[2] = {addr, writedata};
    uint32_t retval = IoTI2cWrite(CW2015_I2C_IDX, CW2015_WRITE_ADDR, buffer, buffLen);
    if (retval != IOT_SUCCESS) {
        printf("IoTI2cWrite(%02X) failed, %0X!\n", buffer[0], retval);
        return retval;
    }
    printf("IoTI2cWrite(%02X)\r\n", buffer[0]);
    return IOT_SUCCESS;
}

void CW2015Init(void)
{
    /*
     * 初始化I2C设备0，并指定波特率为400k
     * Initialize I2C device 0 and specify the baud rate as 400k
     */
    IoTI2cInit(CW2015_I2C_IDX, IOT_I2C_IDX_BAUDRATE);
    /*
     * 设置I2C设备0的波特率为400k
     * Set the baud rate of I2C device 0 to 400k
     */
    IoTI2cSetBaudrate(CW2015_I2C_IDX, IOT_I2C_IDX_BAUDRATE);
    /*
     * 设置GPIO13的管脚复用关系为I2C0_SDA
     * Set the pin reuse relationship of GPIO13 to I2C0_ SDA
     */
    IoSetFunc(IOT_IO_NAME_GPIO_13, IOT_IO_FUNC_GPIO_13_I2C0_SDA);
    /*
     * 设置GPIO14的管脚复用关系为I2C0_SCL
     * Set the pin reuse relationship of GPIO14 to I2C0_ SCL
     */
    IoSetFunc(IOT_IO_NAME_GPIO_14, IOT_IO_FUNC_GPIO_14_I2C0_SCL);
    /* 使电量检测模块从sleep mode变为wake up mode,0x00代表唤醒,0x11代表沉睡,2代表2bit控制 */
    /* Change the power detection module from sleep mode to wake up mode. */
    /* 0x00 represents wake-up, 0x11 represents deep sleep, and 2 bit control */
    Cw20_Write(CW2015_WAKE_REGISTER, 0x00, 2);
}

float GetVoltage(void)
{
    uint8_t buff[WRITELEN] = {0};
    float voltage = 0;
    uint32_t temp = 0;
    // 读取电压的前6位 Read the first 6 bits of voltage
    buff[0] = Cw20_WriteRead(CW2015_HIGHT_REGISTER, 1, 1);
    // 读取电压的后8位 Read the last 8 bits of voltage
    buff[1] = Cw20_WriteRead(CW2015_LOW_REGISTER, 1, 1);
    /* 通过位运算最后得到14位的A/D测量值 */
    /* The final 14 bit A/D measurement value is obtained through bit operation */
    /* 将buf[0]左移8位与buf[1]组成最终电压值 */
    /* Move buf [0] to the left by 8 bits to form the final voltage value with buf [1] */
    temp = (buff[0] << 8) | buff[1];
    /* 通过计算得到最终的电压值 （CW2015的电压分辨率为305.0uV,转换1uv = 1 / 1000000） */
    /* The final voltage value is obtained through calculation */
    /* the voltage resolution of CW2015 is 305.0uV, and the conversion 1uv=1/1000000) */
    voltage = temp * 305.0 / 1000000;
    return voltage;
}

void FeedFood(void)
{   
    printf("1");
    static char line1[32] = {0};
    static char line2[32] = {0};
    float voltage = 0.0;
    float v = 0.0;

   
    voltage = GetVoltage();
                
    int ret1 = snprintf(line1, sizeof(line1), "Welcome!");
       
    OledShowString(10, 3, line1, 2);
    v=100*voltage/4.10;
    int ret2 = snprintf(line2, sizeof(line2), "Battery:%.2f", v);
    OledShowString(10, 5, line2, 1);
    usleep(DELYA_US20);
    TaskMsleep(2000);          
    OledInit();
    OledFillScreen(0); 
}

void TimerThread(const char *arg)
{
    (void)arg;
    osTimerId_t id;
    osStatus_t status;
    osTimerId_t id1;
    osStatus_t s;
    id1 = osTimerNew((osTimerFunc_t)FeedFood, osTimerOnce, NULL, NULL);
    s = osTimerStart(id1, 200);
    osDelay(200);
    GetFunKeyState();
}

void OnFuncKeyPressed(char *arg)
{
    (void) arg;
    g_buttonState = 1;
}


void FuncKeyInit(void)
{
    /*
     * 使能GPIO11的中断功能, OnFuncKeyPressed 为中断的回调函数
     * Enable the interrupt function of GPIO11. OnFuncKeyPressed is the interrupt callback function
     */
    IoTGpioRegisterIsrFunc(IOT_IO_NAME_GPIO_11, IOT_INT_TYPE_EDGE,
                           IOT_GPIO_EDGE_FALL_LEVEL_LOW, OnFuncKeyPressed, NULL);
    /*
     * S3:IO0_2,S4:IO0_3,S5:IO0_4 0001 1100 => 0x1c 将IO0_2,IO0_3,IO0_4方向设置为输入，1为输入，0位输出
     * S3:IO0_ 2,S4:IO0_ 3,S5:IO0_ 4 0001 1100=>0x1c Change IO0_ 2,IO0_ 3,IO0_ 4 direction is set as
     * input, 1 is input, and 0 bit is output
     */
    SetPCA9555GpioValue(PCA9555_PART0_IODIR, 0x1c);
}

void GetFunKeyState(void)
{
    
    uint8_t ext_io_state = 0;
    uint8_t ext_io_state_d = 0;
    uint8_t status;
     while(1)
     {
        if (g_buttonState == 1)
            {
            uint8_t diff;
            status = PCA9555I2CReadByte(&ext_io_state);
            if (status != IOT_SUCCESS) {
                printf("i2c error!\r\n");
                ext_io_state = 0;
                ext_io_state_d = 0;
                g_buttonState = 0;
                continue;
            }
            
            diff = ext_io_state ^ ext_io_state_d;
            if (diff == 0) 
            {
                printf("diff = 0! state:%0X, %0X\r\n", ext_io_state, ext_io_state_d);
            }
            if ((diff & 0x04) && ((ext_io_state & 0x04) == 0)) 
             {
                printf("button3 pressed,\r\n");
                g_buttonPressed = 3;
                Hcsr04Init();
                Hcsr04SampleTask();
             } 

            else if ((diff & 0x08) && ((ext_io_state & 0x08) == 0)) 
             {
                printf("button2 pressed \r\n");
                g_buttonPressed = 2;
                Hcsr04Init();
                Hcsr04SampleTask();
             }

             else if ((diff & 0x10) && ((ext_io_state & 0x10) == 0)) 
             {
                printf("button1 pressed \r\n");
                g_buttonPressed = 1;
                Hcsr04Init();
                Hcsr04SampleTask();
             }

            status = PCA9555I2CReadByte(&ext_io_state);
            ext_io_state_d = ext_io_state;
            g_buttonState = 0;
            }
        usleep(DELAY_US);
     }
}

void LeftWheelForward(void)
{
    printf("Left wheel forward!\n");
    IoTPwmStart(IOT_PWM_PORT_PWM2, IOT_DUTYL, IOT_FREQ);
}

void LeftWheelBackward(void)
{
    printf("Left wheel backward!\n");
    IoTPwmStart(IOT_PWM_PORT_PWM3, IOT_DUTYL, IOT_FREQ);
}

void LeftWheelStop(void)
{
    printf("Left wheel stop!\n");
    IoTPwmStop(IOT_PWM_PORT_PWM2);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
}

void RightWheelForward(void)
{
    printf("Right wheel forward!\n");
    IoTPwmStart(IOT_PWM_PORT_PWM1, IOT_DUTYR, IOT_FREQ);
}

void RightWheelBackward(void)
{
    printf("Right wheel backward!\n");
    IoTPwmStart(IOT_PWM_PORT_PWM0, IOT_DUTYR, IOT_FREQ);
}

void RightWheelStop(void)
{
    printf("Right wheel stop!\n");
    IoTPwmStop(IOT_PWM_PORT_PWM0);
    IoTPwmStop(IOT_PWM_PORT_PWM1);
}

void Hcsr04Init(void)
{
// 左电机GPIO5,GPIO6初始化 Initialization of left motor GPIO5 and GPIO6
    IoTGpioInit(IOT_IO_NAME_GPIO_5);
    IoTGpioInit(IOT_IO_NAME_GPIO_6);
    // 右电机GPIO9, GPIO10初始化 Right motor GPIO9, GPIO10 initialization
    IoTGpioInit(IOT_IO_NAME_GPIO_9);
    IoTGpioInit(IOT_IO_NAME_GPIO_10);

    // 设置GPIO5的管脚复用关系为PWM2输出 Set the pin multiplexing relationship of GPIO5 to PWM2 output
    IoSetFunc(IOT_IO_NAME_GPIO_5, IOT_IO_FUNC_GPIO_5_PWM2_OUT);
    // 设置GPIO6的管脚复用关系为PWM3输出 Set the pin multiplexing relationship of GPIO6 to PWM3 output
    IoSetFunc(IOT_IO_NAME_GPIO_6, IOT_IO_FUNC_GPIO_6_PWM3_OUT);
    // 设置GPIO9的管脚复用关系为PWM0输出 Set the pin multiplexing relationship of GPIO9 to PWM0 output
    IoSetFunc(IOT_IO_NAME_GPIO_9, IOT_IO_FUNC_GPIO_9_PWM0_OUT);
    // 设置GPIO10的管脚复用关系为PWM01输出 Set the pin multiplexing relationship of GPIO10 to PWM01 output
    IoSetFunc(IOT_IO_NAME_GPIO_10, IOT_IO_FUNC_GPIO_10_PWM1_OUT);

    // GPIO5方向设置为输出 GPIO5 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_5, IOT_GPIO_DIR_OUT);
    // GPIO6方向设置为输出 GPIO6 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_6, IOT_GPIO_DIR_OUT);
    // GPIO9方向设置为输出 GPIO9 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_9, IOT_GPIO_DIR_OUT);
    // GPIO10方向设置为输出 GPIO10 direction set to output
    IoTGpioSetDir(IOT_IO_NAME_GPIO_10, IOT_GPIO_DIR_OUT);
    // 初始化PWM2 Initialize PWM2
    IoTPwmInit(IOT_PWM_PORT_PWM2);
    // 初始化PWM3 Initialize PWM3
    IoTPwmInit(IOT_PWM_PORT_PWM3);
    // 初始化PWM0 Initialize PWM0
    IoTPwmInit(IOT_PWM_PORT_PWM0);
    // 初始化PWM1 Initialize PWM1
    IoTPwmInit(IOT_PWM_PORT_PWM1);
    // 先使两个电机处于停止状态 motors stop
    RightWheelStop();
    LeftWheelStop();
}

void BuzzerInit()
{
    IoTGpioInit(IOT_IO_NAME_GPIO_12);
    IoSetFunc(IOT_IO_NAME_GPIO_12, IOT_IO_FUNC_GPIO_12_PWM3_OUT);
    IoTGpioSetDir(IOT_IO_NAME_GPIO_12, IOT_GPIO_DIR_OUT);
    IoTPwmInit(IOT_PWM_PORT_PWM3);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
}
//串口收发部分
void Uart1GpioInit(void)
{
    IoTGpioInit(IOT_IO_NAME_GPIO_0);
    // 设置GPIO0的管脚复用关系为UART1_TX Set the pin reuse relationship of GPIO0 to UART1_ TX
    IoSetFunc(IOT_IO_NAME_GPIO_0, IOT_IO_FUNC_GPIO_0_UART1_TXD);
    IoTGpioInit(IOT_IO_NAME_GPIO_1);
    // 设置GPIO1的管脚复用关系为UART1_RX Set the pin reuse relationship of GPIO1 to UART1_ RX
    IoSetFunc(IOT_IO_NAME_GPIO_1, IOT_IO_FUNC_GPIO_1_UART1_RXD);
}

void Uart1Config(void)
{
    uint32_t ret;
    /* 初始化UART配置，波特率 9600，数据bit为8,停止位1，奇偶校验为NONE */
    /* Initialize UART configuration, baud rate is 9600, data bit is 8, stop bit is 1, parity is NONE */
    IotUartAttribute uart_attr = {
        .baudRate = 115200,
        .dataBits = 8,
        .stopBits = 1,
        .parity = 0,
    };
    ret = IoTUartInit(HI_UART_IDX_1, &uart_attr);
    if (ret != IOT_SUCCESS) {
        printf("Init Uart1 Falied Error No : %d\n", ret);
        return;
    }
}
void L610Initial()
{
    const char *DataSend13 ="AT+MIPCALL=1\r\n";
    const char *DataSend14 ="AT+TCDEVINFOSET=1,\"V6GLXWI4LJ\",\"car\",\"X28irPoT/TC76ffHBcP+Hw==\"\r\n";
    const char *DataSend15 ="AT+TCMQTTCONN=1,20000,240,1,1\r\n";
    const char *DataSend16 ="AT+TCMQTTSUB=\"$thing/down/property/V6GLXWI4LJ/car\",1\r\n";
    const char *DataSend17 ="AT+TCMQTTSUB=\"$thing/down/property/V6GLXWI4LJ/car\",1\r\n";
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend13, strlen(DataSend13));
    TaskMsleep(1000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend14, strlen(DataSend14));
    TaskMsleep(1000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend15, strlen(DataSend15));
    TaskMsleep(1000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend16, strlen(DataSend16));
    TaskMsleep(1000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend17, strlen(DataSend17));
    TaskMsleep(1000);

}
void Route1()
{
    static char line[32] = {0};
    printf("Following Route 1...\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":0}}\"\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":0}}\"\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":0}}\"\r\n");
    const char *DataSend1 = "AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":1}}\"\r\n";
    const char *DataSend2 = "AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":0}}\"\r\n";
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend1, strlen(DataSend1));
    printf("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":1}}\"\r\n");
    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(600);
    IoTPwmStop(IOT_PWM_PORT_PWM3);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(2000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelBackward();
    RightWheelForward();
    TaskMsleep(660);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);//turn left

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(1000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(600);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);//turn right

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(2000); // Move forward for 2 seconds
    LeftWheelStop();
    RightWheelStop();
    CW2015Init();
    OledInit();
    OledFillScreen(0);

    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(100);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
    TaskMsleep(100);
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(100);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
  
    int ret = snprintf(line, sizeof(line), "Please submit the documents!");
    OledShowString(10, 3, line, 2);
    TaskMsleep(10000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend2, strlen(DataSend2));
    printf("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":0}}\"\r\n");

    OledInit();
    OledFillScreen(0);
    TaskMsleep(100);
    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(1180);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(1800); // Move forward for 2 seconds
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelBackward();
    RightWheelForward();
    TaskMsleep(630);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(1020); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(600);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(2000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(600);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
}

void Route2()
{
    printf("Following Route 2...\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":0}}\"\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":0}}\"\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":0}}\"\r\n");
    const char *DataSend1 = "AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":1}}\"\r\n";
    const char *DataSend2 = "AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":0}}\"\r\n";
    static char line[32] = {0};
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend1, strlen(DataSend1));
    printf("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":1}}\"\r\n");
    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(600);
    IoTPwmStop(IOT_PWM_PORT_PWM3);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(4000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);
    CW2015Init();
    OledInit();
    OledFillScreen(0);

    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(100);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
    TaskMsleep(100);
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(100);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
  
    int ret = snprintf(line, sizeof(line), "Please submit the documents!");
    OledShowString(10, 3, line, 2);
    TaskMsleep(10000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend2, strlen(DataSend2));
    printf("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":0}}\"\r\n");

    OledInit();
    OledFillScreen(0);
    TaskMsleep(100);
    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(1300);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(4000); // Move forward for 2 seconds
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(600);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
}

void Route3()
{
    printf("Following Route 3...\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff1\\\":0}}\"\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff2\\\":0}}\"\r\n");
    ("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":0}}\"\r\n");
    const char *DataSend1 = "AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":1}}\"\r\n";
    const char *DataSend2 = "AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":0}}\"\r\n";
    static char line[32] = {0};
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend1, strlen(DataSend1));
    printf("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":1}}\"\r\n");
    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(600);
    IoTPwmStop(IOT_PWM_PORT_PWM3);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(2000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(630);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(1000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelBackward();
    RightWheelForward();
    TaskMsleep(630);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(2000); // Move forward for 2 seconds
    LeftWheelStop();
    RightWheelStop();
    CW2015Init();
    OledInit();
    OledFillScreen(0);

    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(100);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
    TaskMsleep(100);
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(100);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
  
    int ret = snprintf(line, sizeof(line), "Please submit the documents!");
    OledShowString(10, 3, line, 2);
    TaskMsleep(10000);
    IoTUartWrite(HI_UART_IDX_1, (unsigned char*)DataSend2, strlen(DataSend2));
    printf("AT+TCMQTTPUB=\"$thing/up/property/V6GLXWI4LJ/car\",1,\"{\\\"method\\\":\\\"report\\\",\\\"clientToken\\\":\\\"123\\\",\\\"params\\\":{\\\"staff3\\\":0}}\"\r\n");

    OledInit();
    OledFillScreen(0);
    TaskMsleep(100);
    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(1100);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(1800); // Move forward for 2 seconds
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    RightWheelBackward();
    LeftWheelForward();
    TaskMsleep(630);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);//turn left

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(1020); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelBackward();
    RightWheelForward();
    TaskMsleep(630);
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    LeftWheelForward();
    RightWheelForward();
    TaskMsleep(2000); 
    LeftWheelStop();
    RightWheelStop();
    TaskMsleep(500);

    BuzzerInit();
    IoTPwmStart(IOT_PWM_PORT_PWM3, 50,3000);
    TaskMsleep(600);
    IoTPwmStop(IOT_PWM_PORT_PWM3);
}

void FollowRoute(int route)
{
    switch (route)
    {
    case 1:
        
        Route1();
        break;
    case 2:
        
        Route2();
        break;
    case 3:
        
        Route3();
        break;
    default:
        break;
    }
}


void Hcsr04SampleTask(void)
{
    printf("Hcsr04SampleTask init\r\n");
    while (1) {
       
       if (g_buttonPressed != 0)
        {
            FollowRoute(g_buttonPressed);
            g_buttonPressed = 0;
        }
        osDelay(DELAY_MS10);
    }
}


void Hcsr04SampleEntry(void)
{
      // 对UART1的一些初始化 Some initialization of UART1
    Uart1GpioInit();
    // 对UART1参数的一些配置 Some configurations of UART1 parameters
    Uart1Config();
    L610Initial();
    PCA9555Init();
    SetPCA9555GpioValue(PCA9555_PART1_IODIR, PCA9555_OUTPUT);
    CW2015Init();
    OledInit();
    OledFillScreen(0);
    FuncKeyInit();
    osThreadAttr_t attr;
    IoTWatchDogDisable();
    attr.name = "Hcsr04SampleTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = 1024 * 5; // 堆栈大小为1024*5 stack size 5*1024
    attr.priority = osPriorityNormal;
    if (osThreadNew((osThreadFunc_t)TimerThread, NULL, &attr) == NULL) {
        printf("[Timer_demo] osThreadNew Falied to create TimerThread!\n");
    }
   
   
}

APP_FEATURE_INIT(Hcsr04SampleEntry);