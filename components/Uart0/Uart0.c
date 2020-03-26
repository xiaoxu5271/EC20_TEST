#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "Uart0.h"

#define UART0_TXD (UART_PIN_NO_CHANGE)
#define UART0_RXD (UART_PIN_NO_CHANGE)
#define UART0_RTS (UART_PIN_NO_CHANGE)
#define UART0_CTS (UART_PIN_NO_CHANGE)

#define UART1_TXD (GPIO_NUM_21)
#define UART1_RXD (GPIO_NUM_22)

#define UART2_TXD (GPIO_NUM_17)
#define UART2_RXD (GPIO_NUM_16)
#define UART2_RTS (UART_PIN_NO_CHANGE)
#define UART2_CTS (UART_PIN_NO_CHANGE)

static const char *TAG = "EC20";
SemaphoreHandle_t xMutex_uart2_sw = NULL;

#define EC20_SW 25
#define BUF_SIZE 1024

char ICCID[24] = {0};
char EC20_RECV[BUF_SIZE];

static void Uart0_Task(void *arg);
char *AT_Cmd_Send(char *cmd_buf, char *check_buff, uint16_t time_out, uint8_t try_num);
uint8_t EC20_Http_CFG(void);
uint8_t EC20_Active(void);

void Uart_Init(void)
{
    uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, UART1_TXD, UART1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    // //uart2 switch io
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1 << EC20_SW);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    // gpio_set_level(EC20_SW, 1); //
    // vTaskDelay(200 / portTICK_PERIOD_MS);
    // gpio_set_level(EC20_SW, 0); //

    // //创建切换uart2 互斥信号
    // xMutex_uart2_sw = xSemaphoreCreateMutex();
    // //串口0 接收解析
    xTaskCreate(Uart0_Task, "Uart0_Task", 4096, NULL, 9, NULL);
}

char *Send_AT_CMD(char *cmd, char *check_buff, uint8_t cmd_len, uint16_t time_out)
{
    char *ret;
    char *send_buff;
    send_buff = (char *)malloc(cmd_len + 2);
    sprintf(send_buff, "%s\r\n", cmd);
    ret = AT_Cmd_Send(send_buff, check_buff, time_out, 10);
    free(send_buff);
    return ret;
}

void Uart0_read(void)
{
    uint8_t data_u0[BUF_SIZE] = {0};

    int len0 = uart_read_bytes(UART_NUM_1, data_u0, BUF_SIZE, 500 / portTICK_RATE_MS);
    if (len0 != 0) //读取到按键数据
    {
        len0 = 0;
        printf("\n%s\n", data_u0);
        // ParseTcpUartCmd((char *)data_u0);
        // bzero(data_u0, sizeof(data_u0));
    }
}

void Uart0_Task(void *arg)
{
    uint8_t ret;
    ret = EC20_Init();
    if (ret == 0)
    {
        goto end;
    }
    ret = EC20_Http_CFG();
    if (ret == 0)
    {
        goto end;
    }

    ret = EC20_Active();
    if (ret == 0)
    {
        goto end;
    }

end:
    // AT_Cmd_Send("AT+QHTTPSTOP\r\n", "OK", 5000, 1);

    vTaskDelete(NULL);
}

/**********************************************************/

/*******************************************************************************
//Check AT Command Respon result，
// 
*******************************************************************************/
char *AT_Cmd_Send(char *cmd_buf, char *check_buff, uint16_t time_out, uint8_t try_num)
{
    char *rst_val = NULL;
    int len0;
    uint8_t i, j;

    for (i = 0; i < try_num; i++)
    {
        uart_flush(UART_NUM_1);
        uart_flush_input(UART_NUM_1);
        memset(EC20_RECV, 0, BUF_SIZE);
        uart_write_bytes(UART_NUM_1, cmd_buf, strlen(cmd_buf));

        for (j = 0; j < time_out; j++)
        {
            memset(EC20_RECV, 0, BUF_SIZE);
            len0 = uart_read_bytes(UART_NUM_1, (uint8_t *)EC20_RECV, BUF_SIZE, 20 / portTICK_RATE_MS);
            if (len0 > 0) //
            {
                ESP_LOGI(TAG, "%s\n", EC20_RECV);
                rst_val = strstr(EC20_RECV, check_buff); //
                if (rst_val != NULL)
                {
                    break;
                }
                // break;
            }
        }
        if (rst_val != NULL)
        {
            break;
        }
    }

    return rst_val; //
}

//EC20 init
uint8_t EC20_Init(void)
{
    char *ret;
    //开机
    gpio_set_level(EC20_SW, 1); //
    vTaskDelay(200 / portTICK_PERIOD_MS);
    gpio_set_level(EC20_SW, 0); //
    vTaskDelay(8000 / portTICK_PERIOD_MS);

    ret = AT_Cmd_Send("AT\r\n", "OK", 100, 10);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "AT  ");
        return 0;
    }

    // ret = AT_Cmd_Send("ATE0\r\n", "OK", 100, 5);//回显
    // if (ret == NULL)
    // {
    //     ESP_LOGE(TAG, "ATE0  ");
    //     return 0;
    // }

    ret = AT_Cmd_Send("AT+IPR=115200\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "AT+IPR=115200  ");
        return 0;
    }

    ret = AT_Cmd_Send("AT+CPIN?\r\n", "READY", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "AT+CPIN?  ");
        return 0;
    }

    ret = AT_Cmd_Send("AT+QCCID\r\n", "+QCCID:", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "AT+QCCID  ");
        return 0;
    }
    else
    {
        memcpy(ICCID, ret + 8, 20);
        ESP_LOGI(TAG, "ICCID=%s", ICCID);
    }

    ret = AT_Cmd_Send("AT+CGATT?\r\n", "+CGATT: 1", 100, 100);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "AT+QCCID  ");
        return 0;
    }

    return 1;
}

uint8_t EC20_Http_CFG(void)
{
    char *ret;
    ret = AT_Cmd_Send("AT+QHTTPCFG=\"contextid\",1\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }

    ret = AT_Cmd_Send("AT+QHTTPCFG=\"responseheader\",0\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }

    ret = AT_Cmd_Send("AT+QHTTPCFG=\"closewaittime\",0\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }

    ret = AT_Cmd_Send("AT+QIACT?\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }
    else
    {
        ret = strstr(EC20_RECV, "+QIACT: 1,1"); //
        if (ret != NULL)
        {
            ESP_LOGI(TAG, "QIACT ok! %d", __LINE__);
            return 1;
        }
    }

    ret = AT_Cmd_Send("AT+QICSGP=1,1,\"CMNET\",\"\",\"\" ,1\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }

    ret = AT_Cmd_Send("AT+QIACT=1\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }

    ret = AT_Cmd_Send("AT+QIACT?\r\n", "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Http_CFG %d", __LINE__);
        return 0;
    }

    return 1;
}

uint8_t EC20_Active(void)
{
    char *ret;
    char *cmd_buf;
    char *active_url;
    uint8_t active_len;
    active_url = (char *)malloc(80);
    cmd_buf = (char *)malloc(24);
    memset(cmd_buf, 0, 24);
    memset(active_url, 0, 80);
    sprintf(active_url, "http://api.ubibot.cn/products/ubibot-sp1/devices/AAAA0004SP1/activate\r\n");
    active_len = strlen(active_url);
    sprintf(cmd_buf, "AT+QHTTPURL=%d,10\r\n", 69);
    ret = AT_Cmd_Send(cmd_buf, "CONNECT", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }
    ret = AT_Cmd_Send(active_url, "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }

    ret = AT_Cmd_Send("AT+QHTTPGET=60\r\n", "+QHTTPGET:", 6000, 1);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }

    ret = AT_Cmd_Send("AT+QHTTPREAD=60\r\n", "+QHTTPREAD: 0", 100, 1);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }

end:
    free(active_url);
    free(cmd_buf);
    if (ret == NULL)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

uint8_t EC20_Post_Data(void)
{
    char *ret;
    char *cmd_buf;
    char *active_url;
    uint8_t active_len;
    active_url = (char *)malloc(80);
    cmd_buf = (char *)malloc(24);
    memset(cmd_buf, 0, 24);
    memset(active_url, 0, 80);
    sprintf(active_url, "http://api.ubibot.cn/products/ubibot-sp1/devices/AAAA0004SP1/activate\r\n");
    active_len = strlen(active_url);
    sprintf(cmd_buf, "AT+QHTTPURL=%d,10\r\n", 69);
    ret = AT_Cmd_Send(cmd_buf, "CONNECT", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }
    ret = AT_Cmd_Send(active_url, "OK", 100, 5);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }

    ret = AT_Cmd_Send("AT+QHTTPGET=60\r\n", "+QHTTPGET:", 6000, 1);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }

    ret = AT_Cmd_Send("AT+QHTTPREAD=60\r\n", "+QHTTPREAD: 0", 100, 1);
    if (ret == NULL)
    {
        ESP_LOGE(TAG, "EC20_Active %d", __LINE__);
        goto end;
    }

end:
    free(active_url);
    free(cmd_buf);
    if (ret == NULL)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}