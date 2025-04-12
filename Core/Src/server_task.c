#include "server_task.h"
#include "lwip/api.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include "cmsis_os.h"  // Ensure this is v1

#define SERVER_PORT  3400

void ServerTask(void const *argument) {
    struct netconn *conn, *newconn;
    err_t err, accept_err;

    conn = netconn_new(NETCONN_TCP);
    if (conn == NULL) {
        printf("Failed to create netconn\n");
        vTaskDelete(NULL); // terminate task if out of mem
    }

    err = netconn_bind(conn, IP_ADDR_ANY, SERVER_PORT);
    if (err != ERR_OK) {
        printf("Bind error %d\n", err);
        netconn_delete(conn);
        vTaskDelete(NULL);
    }

    netconn_listen(conn);
    printf("JSON Server listening on port %d...\n", SERVER_PORT);

    for (;;) {
        accept_err = netconn_accept(conn, &newconn);
        if (accept_err == ERR_OK) {
            struct netbuf *buf;
            void *data;
            u16_t len;

            while ((err = netconn_recv(newconn, &buf)) == ERR_OK) {
                netbuf_data(buf, &data, &len);

                char json_buf[256] = {0};
                if (len < sizeof(json_buf)) {
                    memcpy(json_buf, data, len);
                    json_buf[len] = '\0';
                } else {
                    memcpy(json_buf, data, sizeof(json_buf)-1);
                    json_buf[sizeof(json_buf)-1] = '\0';
                }

                printf("Received: %s\n", json_buf);

                cJSON *root = cJSON_Parse(json_buf);
                cJSON *response_json = NULL;
                char *response_str = NULL;

                if (root) {
                    cJSON *val1_item = cJSON_GetObjectItemCaseSensitive(root, "val1");
                    cJSON *val2_item = cJSON_GetObjectItemCaseSensitive(root, "val2");
                    double val1 = 0.0;
                    int val2 = 0;
                    if (cJSON_IsNumber(val1_item)) val1 = val1_item->valuedouble;
                    if (cJSON_IsNumber(val2_item)) val2 = val2_item->valueint;

                    double result = val1 + val2;

                    response_json = cJSON_CreateObject();
                    cJSON_AddStringToObject(response_json, "status", "OK");
                    cJSON_AddNumberToObject(response_json, "result", result);

                    response_str = cJSON_PrintUnformatted(response_json);
                    if (response_str) {
                        netconn_write(newconn, response_str, strlen(response_str), NETCONN_COPY);
                        printf("Sent: %s\n", response_str);
                    }
                } else {
                    response_json = cJSON_CreateObject();
                    cJSON_AddStringToObject(response_json, "status", "ERROR");
                    cJSON_AddStringToObject(response_json, "message", "Invalid JSON");
                    response_str = cJSON_PrintUnformatted(response_json);
                    netconn_write(newconn, response_str, strlen(response_str), NETCONN_COPY);
                }

                if (root) cJSON_Delete(root);
                if (response_json) cJSON_Delete(response_json);
                if (response_str) cJSON_free(response_str);

                netbuf_delete(buf);
            }

            netconn_close(newconn);
            netconn_delete(newconn);
        }
    }
}

void ServerTask_Start(void) {
    osThreadDef(serverTask, ServerTask, osPriorityNormal, 0, 512);
    osThreadCreate(osThread(serverTask), NULL);
}
