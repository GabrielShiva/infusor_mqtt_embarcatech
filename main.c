#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalh3ar com os pinos GPIO do Raspberry Pi Pico

#include "lib/ssd1306.h"
#include "lib/font.h"

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#define WIFI_SSID "JR TELECOM-LAR" // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "Rama2000"   // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "192.168.1.5"  // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "gabrielshiva" // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "100936"     // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

// Definição de macros para o protocolo I2C (SSD1306)
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define SSD1306_ADDRESS 0x3C

// Inicializa instância do display
ssd1306_t ssd;

// Armazena o texto que será exibido no display OLED
char display_text[40] = {0};

// Define os pinos dos botões
#define BUZZER_PIN 21 // Buzzer de alerta quando o valor de infusão for superior ao definido
#define INFUSION_RATE_PIN 27 // Representa a taxa de infusão atual (ml/H)
#define VOL_PIN 26 // Representa o volume total atual
#define LED_RED 12 // led de alerta quando o valor de infusão for superior ao definido

#define MAX_INFUSION_RATE 90.0f

// Flag que define se a taxa de infusão está acima do permitido
volatile bool is_over_limit = false;
volatile bool is_infusion_active = false;

// Define variável para debounce do botão
volatile uint32_t last_time_btn_press = 0;

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

//Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

// Publicar dados da infusão
static void infusion_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t infusion_worker = { .do_work = infusion_worker_fn };

// Inicialização do protocolo I2C para comunicação com o display OLED
static void i2c_setup(uint baud_in_kilo);

// Inicializa o display OLED
static void ssd1306_setup(ssd1306_t *ssd_ptr);

// Faz a transformação de 0-4095 para a faixa especificada
float adc_transformation(uint16_t adc_val, float min, float max);

// Realiza a leitura dos valores via joystick
static float read_infusion_values(uint input_selection);

// Publicar valores lidos pelo adc
static void publish_infusion_values(MQTT_CLIENT_DATA_T *state);

int main() {
    stdio_init_all();
    printf("Iniciando cliente mqtt\n");

    adc_init();
    adc_gpio_init(INFUSION_RATE_PIN);
    adc_gpio_init(VOL_PIN);

    gpio_init(LED_RED);
    gpio_set_dir(LED_RED, GPIO_OUT);
    gpio_put(LED_RED, 0);

    // Inicialização do protocolo I2C com 400Khz e inicialização do display
    i2c_setup(400);
    ssd1306_setup(&ssd);

    bool color = true;
    ssd1306_fill(&ssd, !color);
    ssd1306_draw_string(&ssd, "Inicializando", 5, 20);
    ssd1306_draw_string(&ssd, "MQTT", 5, 30);
    ssd1306_send_data(&ssd);
    sleep_ms(500);

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    state.mqtt_client_info.client_id = "pico_gabriel_uxH";
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S;
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;

    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;

    #if LWIP_ALTCP && LWIP_ALTCP_TLS
        // TLS enabled
        #ifdef MQTT_CERT_INC
            static const uint8_t ca_cert[] = TLS_ROOT_CERT;
            static const uint8_t client_key[] = TLS_CLIENT_KEY;
            static const uint8_t client_cert[] = TLS_CLIENT_CERT;
            // This confirms the indentity of the server and the client
            state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
                    client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
            #if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
                printf("Warning: tls without verification is insecure\n");
            #endif
        #else
            state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
            printf("Warning: tls without a certificate is insecure\n");
        #endif
    #endif

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        ssd1306_fill(&ssd, !color);
        ssd1306_draw_string(&ssd, "Falha ao", 5, 20);
        ssd1306_draw_string(&ssd, "Inicializar", 5, 30);
        ssd1306_draw_string(&ssd, "CYW43", 5, 40);
        ssd1306_send_data(&ssd);

        panic("Falha ao inicializar o CYW43");
    }

    ssd1306_fill(&ssd, !color);
    ssd1306_draw_string(&ssd, "Conectando em", 5, 20);
    snprintf(display_text, sizeof(display_text), "%s", WIFI_SSID);
    ssd1306_draw_string(&ssd, display_text, 5, 30);
    ssd1306_send_data(&ssd);
    sleep_ms(500);

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        ssd1306_fill(&ssd, !color);
        ssd1306_draw_string(&ssd, "Falha ao", 5, 20);
        ssd1306_draw_string(&ssd, "Conectar na", 5, 30);
        ssd1306_draw_string(&ssd, "Rede WiFi", 5, 40);
        ssd1306_send_data(&ssd);

        panic("Falha ao conectar na rede");
    }

    ssd1306_fill(&ssd, !color);
    ssd1306_draw_string(&ssd, "Conectado", 5, 20);
    ssd1306_draw_string(&ssd, "Com sucesso", 5, 30);
    ssd1306_send_data(&ssd);
    sleep_ms(500);

    printf("\nConectou ao Wifi\n");

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        panic("Requisição DNS falhou");
    }

    ssd1306_fill(&ssd, !color);
    ssd1306_send_data(&ssd);

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));

        if (is_infusion_active) {
            ssd1306_fill(&ssd, !color);
            ssd1306_draw_string(&ssd, "INFUSAO", 5, 20);
            ssd1306_draw_string(&ssd, "ATIVADA", 5, 30);
            ssd1306_send_data(&ssd);

             if (is_over_limit) {
                gpio_put(LED_RED, 1);
            } else {
                gpio_put(LED_RED, 0);
            }
        }

        if (!is_infusion_active) {
            ssd1306_fill(&ssd, !color);
            ssd1306_draw_string(&ssd, "INFUSAO", 5, 20);
            ssd1306_draw_string(&ssd, "DESATIVADA", 5, 30);
            ssd1306_send_data(&ssd);

            gpio_put(LED_RED, 0);
        }
    }

    printf("Cliente mqtt pico saindo\n");
    return 0;
}

// Faz a transformação de 0-4095 para a faixa especificada
float adc_transformation(uint16_t adc_val, float min, float max) {
    return min + (adc_val / 4095.0f) * (max - min);
}

// Realiza a leitura dos valores via joystick
static float read_infusion_values(uint input_selection) {
    if (input_selection == 0) {
        adc_select_input(0);
        return adc_transformation(adc_read(), 1.0, 100.0);
    }

    if (input_selection == 1) {
        adc_select_input(1);
        return adc_transformation(adc_read(), 5.0, 500.0);
    }

    return -1.0f;
}

// Publicar valores lidos pelo adc
static void publish_infusion_values(MQTT_CLIENT_DATA_T *state) {
    static float old_rate;
    static float old_vol;

    // Define os tópicos
    const char *infusion_rate_key = full_topic(state, "/infusion/rate");
    const char *infusion_volume_key = full_topic(state, "/infusion/volume");

    // Realiza as leituras dos valores pelo joystick
    float rate = read_infusion_values(0);
    float vol = read_infusion_values(1);

    if (rate > MAX_INFUSION_RATE) {
        is_over_limit = true;
    } else {
        is_over_limit = false;
    }

    // Verifica se os valores foram atualizados
    if (rate != old_rate || vol != old_vol) {
        old_rate = rate;
        old_vol = vol;

        // Publica os valores obtidos via MQTT
        char payload_str[16];

        snprintf(payload_str, sizeof(payload_str), "%.2f", rate);
        printf("Publicando %s para %s\n", payload_str, infusion_rate_key);
        mqtt_publish(state->mqtt_client_inst, infusion_rate_key, payload_str, strlen(payload_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);

        snprintf(payload_str, sizeof(payload_str), "%.2f", vol);
        printf("Publicando %s para %s\n", payload_str, infusion_volume_key);
        mqtt_publish(state->mqtt_client_inst, infusion_volume_key, payload_str, strlen(payload_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

// Inicializa o protocolo I2C
static void i2c_setup(uint baud_in_kilo) {
  i2c_init(I2C_PORT, baud_in_kilo * 1000);

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);
}

// Inicializa o display OLED
static void ssd1306_setup(ssd1306_t *ssd_ptr) {
  ssd1306_init(ssd_ptr, WIDTH, HEIGHT, false, SSD1306_ADDRESS, I2C_PORT); // Inicializa o display
  ssd1306_config(ssd_ptr);                                                // Configura o display
  ssd1306_send_data(ssd_ptr);                                             // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(ssd_ptr, false);
  ssd1306_send_data(ssd_ptr);
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        printf("pub_request_cb failed %d", err);
    }
}

// Definição de tópico para protocolo MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
    #if MQTT_UNIQUE_TOPIC
        static char full_topic[MQTT_TOPIC_LEN];
        snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
        return full_topic;
    #else
        return name;
    #endif
}

// Requisição de assinatura - subscribe
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;

    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/infusion/state"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Função de callback que trata as mensagens recebidas do broker
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;

    #if MQTT_UNIQUE_TOPIC
        const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
    #else
        const char *basic_topic = state->topic;
    #endif

    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    bool color  = true;

    printf("Topic: %s, Message: %s\n", state->topic, state->data);

    if (strcmp(basic_topic, "/infusion/state") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0) {
            is_infusion_active = true;
        } else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0) {
            is_infusion_active = false;
        }
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // Cancela todas as inscrições do cliente
    }
}

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar dados lidos de forma periódica
static void infusion_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;

    if (is_infusion_active) {
        publish_infusion_values(state);
    }

    async_context_add_at_time_worker_in_ms(context, worker, 1 * 1000);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;

    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true; // Define que a conexão foi feita
        sub_unsub_topics(state, true); // Se inscreve nos tópicos de interesse;

        // Indica que o cliente está online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publica os dados a cada 8 segundos se nada tiver sido alterado
        infusion_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &infusion_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            panic("Falha ao conectar ao servidor");
        }
    } else {
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
    // Define a porta que será utilizada e informa se está utilizando TLS ou não
    #if LWIP_ALTCP && LWIP_ALTCP_TLS
        const int port = MQTT_TLS_PORT;
        printf("Using TLS\n");
    #else
        const int port = MQTT_PORT;
        printf("Warning: Not using TLS\n");
    #endif

    // Cria a instância para o cliente mqtt
    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        panic("MQTT client instance creation error");
    }

    // Exibe o endereço IP do pico e o servidor (broker) que está tentando conectar
    printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();

    // Tenta realizar uma conexão com o broker mqtt
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        panic("MQTT broker connection error");
    }

    #if LWIP_ALTCP && LWIP_ALTCP_TLS
        // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
        mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
    #endif

    // Define função callback que será chamada quando uma mensagem for publicada nos tópicos em que o cliente está inscrito
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        panic("dns request failed");
    }
}
