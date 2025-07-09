/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-4-1     Dream       first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "wlan_dev.h"
#include "drv_gpio.h"
#include <cJSON.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <board.h>
#include "mqttclient.h"
#include <gps_rmc.h>
#include <pid.h>

#define GPS_RMC_SAMPLE_UART_NAME "uart5"

struct rt_semaphore rx_sem;
static rt_device_t uart;

struct gps_info info_data = {0};
    gps_info_t info = &info_data;


struct rt_wlan_device *wlan_dev;
uint16_t buffer_wifi[30];
mqtt_client_t *client = NULL;
//rt_mutex_t info_mutex = RT_NULL;

/*void data(double latitude, double longitude, char *buffer, size_t buffer_size)
        {
           rt_snprintf(buffer, buffer_size, "[{\"latitude\":%.6lf,\"longitude\":%.6lf}]", latitude, longitude);
            rt_snprintf(buffer, buffer_size, "{\"latitude\":35.3042,\"longitude\":113.9512}");
        }*/
void connect_wifi(void)
{

    // 定义并初始化结构体，存储 Wi-Fi 连接信息
    struct rt_sta_info sta_info;
    memset(&sta_info, 0, sizeof(struct rt_sta_info)); // 将结构体清零

    // **Wi-Fi SSID 配置**
    strncpy(sta_info.ssid.val, "Hong", RT_WLAN_SSID_MAX_LENGTH); // Wi-Fi 名称（SSID）
    sta_info.ssid.len = strlen("Hong");

    // **Wi-Fi 密码配置**
    strncpy(sta_info.key.val, "12345678", RT_WLAN_PASSWORD_MAX_LENGTH); // Wi-Fi 密码
    sta_info.key.len = strlen("12345678");

    // **Wi-Fi 安全类型设定**
    sta_info.security = SECURITY_WPA2_AES_PSK; // 设置为 WPA2 安全模式

    // **设置频率通道号（可选）**
    sta_info.channel = 0; // 如果为 0，则表示让系统自动选择通道

    // **BSSID（MAC地址）配置**
    // 如果不限定目标设备的 BSSID，则可以将其置 0
    memset(sta_info.bssid, 0, sizeof(sta_info.bssid)); // 不限制特定设备


    // 查找 STA 设备句柄
    wlan_dev= rt_device_find("wlan0"); // STA 模式设备通常为 "wlan0"

    if (wlan_dev != NULL)
    {
        rt_thread_mdelay(1000);       //如果重连还是无法连接，疑似是手机问题
        // 调用 wlan_join 连接 Wi-Fi
        rt_err_t result = wlan_join(wlan_dev, &sta_info);
        if (result == RT_EOK)
        {
            rt_kprintf("成功发送 Wi-Fi 连接请求！\n");
        }
        else
        {
            rt_kprintf("Wi-Fi 连接失败，请检查配置！\n");
            rt_thread_mdelay(1000);
            result = wlan_join(wlan_dev, &sta_info);
        }
    }
}



char* gps_to_json(gps_info_t info) {
    cJSON *root = cJSON_CreateObject();


    // 添加数据
    cJSON_AddNumberToObject(root, "latitude", info->coord.location.latitude.value);//
    cJSON_AddNumberToObject(root, "longitude", info->coord.location.longitude.value);//
    cJSON_AddNumberToObject(root, "温度1", read_temperature());
    cJSON_AddNumberToObject(root, "温度2", read_temperature2());
    cJSON_AddNumberToObject(root, "湿度1", read_humi());
    cJSON_AddNumberToObject(root, "湿度2", read_humi2());

    // 生成JSON
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); // 释放

    return json_str;
}


/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}



void gps_rmc_sample_entry(void *p)
{
    char buff[128] = {0}, *buff_p = buff, ch = 0;

    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(uart, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        if (ch == '\n')
        {
            if (rt_strstr((const char *)buff, "RMC"))
            {
                if (gps_rmc_parse(info, buff))
                {
                    //rt_kprintf("GPS解析成功，正在上传云平台\n");
                    //gps_print_info(info);
                    //rt_kprintf("%s\n", gps_to_json(info));
                    rt_thread_delay(3000);

                }

                //rt_kprintf("%s\n %s\n", (char*)info->coord.location.latitude.string,(char*)info->coord.location.longitude.string);
                                  // generate_test_gps_data(info);
            }
            rt_memset(buff, 0, 128);
            rt_memset(info, 0, sizeof(struct gps_info));
            buff_p = buff;
            continue;
        }
        *buff_p++ = ch;

    }

}

int gps_rmc_sample_entry_init(void)
{

    uart = rt_device_find(GPS_RMC_SAMPLE_UART_NAME);
    if (uart == RT_NULL)
    {
        rt_kprintf("Not find %s device.\r\n", GPS_RMC_SAMPLE_UART_NAME);
        return RT_ERROR;
    }

    /* 初始化信号量 */
    rt_sem_init(&rx_sem, GPS_RMC_SAMPLE_UART_NAME"_rx", 0, RT_IPC_FLAG_FIFO);

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_9600;
    rt_device_control(uart, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_open(uart, RT_DEVICE_FLAG_INT_RX);
    const char cmd[] = "$PCAS03,0,0,0,0,1,0,0,0,0,0,,,1,*32\r\n";
    rt_device_write(uart, 0, cmd, strlen(cmd));
    rt_device_set_rx_indicate(uart, uart_input);

    rt_thread_t t = rt_thread_create(
        "gps_rmc_p", gps_rmc_sample_entry, RT_NULL,
        2048, 16, 10
    );
    if (t == RT_NULL)
    {
        rt_kprintf("Failde to create gps rmc info procees thread.\r\n");
        return RT_ERROR;
    }
    if (rt_thread_startup(t) != RT_EOK)
    {
        rt_kprintf("Failde to startup gps rmc info procees thread.\r\n");
        rt_thread_delete(t);
        return RT_ERROR;
    }
    return RT_EOK;
}

static void sub_topic_handle1(void* client, message_data_t* msg)
{
    (void) client;
    KAWAII_MQTT_LOG_I("-----------------------------------------------------------------------------------");
    KAWAII_MQTT_LOG_I("%s:%d %s()...\ntopic: %s\nmessage:%s", __FILE__, __LINE__, __FUNCTION__, msg->topic_name, (char*)msg->message->payload);
    KAWAII_MQTT_LOG_I("-----------------------------------------------------------------------------------");
}

int mqtt_publish_handle1(mqtt_client_t *client)
{
    //generate_test_gps_data(info1);
    char *json_payload = gps_to_json(info);
    rt_kprintf("%s\n", gps_to_json(info));
    mqtt_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.qos = QOS1;
    msg.payload = gps_to_json(info);
    msg.payloadlen = strlen(gps_to_json(info));

    mqtt_publish(client, "$iot/cold/user/location", &msg);
    cJSON_free(json_payload);
}


static void kawaii_mqtt_demo(void)
{

    rt_thread_delay(6000);

     mqtt_log_init();

    client = mqtt_lease();

    mqtt_set_host(client, "aykaswl.iot.gz.baidubce.com");
    mqtt_set_port(client, "1883");
    mqtt_set_user_name(client, "thingidp@aykaswl|cold|0|MD5");
    mqtt_set_password(client, "fe0c581a7559b3d96dbf529b5222a57d");
    mqtt_set_client_id(client, "cold");
    mqtt_set_clean_session(client, 1);
    KAWAII_MQTT_LOG_I("The ID of the Kawaii client is: %s ", KAWAII_MQTT_CLIENTID);

    mqtt_connect(client);

    //mqtt_subscribe(client, KAWAII_MQTT_SUBTOPIC, QOS0, sub_topic_handle1);

     while (1) {

               mqtt_publish_handle1(client);

                    mqtt_sleep_ms(3 * 1000);


    }
}

int ka_mqtt(void)
{
    rt_thread_t tid_mqtt;

    tid_mqtt = rt_thread_create("kawaii_demo", kawaii_mqtt_demo, RT_NULL, 2048, 5, 10);
    if (tid_mqtt == RT_NULL) {
        return -RT_ERROR;
    }

    rt_thread_startup(tid_mqtt);

    return RT_EOK;
}
MSH_CMD_EXPORT(ka_mqtt, Kawaii MQTT client test program);




int main(void)
{
    connect_wifi();
    rt_thread_mdelay(3000);
    ka_mqtt();
    gps_rmc_sample_entry_init();
    pid_pwm_init();



    return RT_EOK;

}


