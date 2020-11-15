//Código desenvolvido por Luis Felipe
//Última atualização neste código: 15/11/2020

/*Este código pode/deve ser gravado em todas as ESP's. 
A partir dele, o dispositivo móvel pode conectar-se a qualquer uma das ESP's por Bluetooth Classic(BT),
e enviar um comando para envio de mensagens via ESP-NOW para as outras ESP's.

Acrescido nesta versão: Será enviado pelo BT uma sequencia com 3 inteiros, para ligar o LED em determinada cor
O protocolo para determinar a cor, faz o envio de uma String com um número de 10 dígitos,
Sendo: {0}{123}{456}{789}{0}.
O primeiro dígito {0}, refere-se ao comando que deve ser executado. No exemplo, 0 refere-se aos LEDS.
Os dígitos {123}, referem-se a cor vermelha do LED e deve ser um número entre 0 e 255.
Os dígitos {456}, referem-se a cor verde do LED e deve ser um número entre 0 e 255.
Os dígitos {789}, referem-se a cor azul do LED e deve ser um número entre 0 e 255.
O ultimo dígito {0}, refere-se a qual dispositivo deve ser enviado o comando, sendo '0' o envio em broadcast(para todos),
e de '1' a '6' para as respectivas ESPs cadastradas em broadcastAddress.
*/

#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <Ultrasonic.h>

//Variáveis dos pinos dos LED's
int RGB_RED = 21;
int RGB_GREEN = 22;
int RGB_BLUE = 23;
int CHANNEL_RED = 0;
int CHANNEL_GREEN = 1;
int CHANNEL_BLUE = 2;

//Variáveis do Sensor Ultrassônico
const int trigPin = 18;
const int echoPin = 19;

Ultrasonic ultrasonic(trigPin, echoPin); //INICIALIZANDO OS PINOS DO ARDUINO

int distancia;

//Variável que define para qual esp vai ser enviada a mensagem
int destino;

//MAC Address's para comunicação ESP NOW
uint8_t broadcastAddress[][6] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  //TODO MUNDO
    {0x30, 0xAE, 0xA4, 0xFE, 0x69, 0x90},  //ESP 1
    {0xCC, 0x50, 0xE3, 0x96, 0x2C, 0xC8},  //ESP 2
    {0xCC, 0x50, 0xE3, 0x95, 0xD7, 0xFC}}; //ESP 3

esp_now_peer_info_t peerInfo;

//Variáveis de mensagens enviadas/recebidas
typedef struct struct_data
{
    unsigned long time;
    int comando;
    int red = 50;
    int green = 25;
    int blue = 0;
} struct_data;

struct_data msg;

//Variáveis auxiliares para mensagens enviadas/recebidas
struct_data enviado;
struct_data recebido;

//Variável da comunicação Bluetooth
BluetoothSerial SerialBT;

//Escopo de Funções
void start_espnow();
void callbackBT(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void onReceiveData(const uint8_t *mac, const uint8_t *data, int len);
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status);
void envia_mensagem();
void define_mensagem();
String leStringSerial();
String leBTSerial();

void config_led(int pino, int channel_pwm);
void cor_led(int c_red, int c_green, int c_blue, int red, int green, int blue);

void start_ultrasonic();
int dist_ultrasonic();

void comando();

void setup()
{
    //Inicializa Serial
    Serial.begin(115200);
    //Inicializa BluetoothSerial
    SerialBT.register_callback(callbackBT);
    SerialBT.begin("ESP32");
    

    //Define modo do Wi-Fi em Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Serial.println();
    Serial.println(WiFi.macAddress());

    //Inicializa a comunicação esp_now e adiciona os pares definidos em broadcastAddress
    start_espnow();

    //Registra o Callback de recebimento de dados
    esp_now_register_recv_cb(onReceiveData);
    //Registra o Callback de status de envio
    esp_now_register_send_cb(OnDataSent);

    //Configurando 1 Led RGB
    config_led(21, 0);
    config_led(22, 1);
    config_led(23, 2);

    //Configura o Sensor Ultrassônico
    start_ultrasonic();
}

void loop()
{
    define_mensagem();
}

void start_espnow()
{
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    int slavesCount = sizeof(broadcastAddress) / 6 / sizeof(uint8_t);

    //Para cada slave
    for (int i = 0; i < slavesCount; i++)
    {
        //Criamos uma variável que irá guardar as informações do slave
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        //Copia o endereço do array para a estrutura
        memcpy(peerInfo.peer_addr, broadcastAddress[i], sizeof(broadcastAddress[i]));
        esp_now_add_peer(&peerInfo);
    }
}

void callbackBT(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if (event == ESP_SPP_SRV_OPEN_EVT)
    {
        Serial.println("Cliente conectado!");
    }

    if (event == ESP_SPP_CLOSE_EVT)
    {
        Serial.println("Cliente desconectado!");
    }
}

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len)
{
    char macStr[18];
    Serial.print("Pacote recebido de: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(macStr);

    memcpy(&msg, data, sizeof(msg));

    recebido = msg;
    Serial.println("Dados recebidos: ");
    Serial.print("Comando: ");
    Serial.println(recebido.comando);
    Serial.print("red: ");
    Serial.println(recebido.red);
    Serial.print("green: ");
    Serial.println(recebido.green);
    Serial.print("blue: ");
    Serial.println(recebido.blue);
    Serial.println("\n\n");

    comando();
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
    char macStr[18];
    //Copiamos o Mac Address destino para uma string
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    //Mostramos o Mac Address que foi destino da mensagem
    Serial.print("Enviado para: ");
    Serial.println(macStr);
    //Mostramos se o status do envio foi bem sucedido ou não
    Serial.print("Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
    Serial.println("\n\n");
}

void envia_mensagem()
{

    msg = enviado;
    esp_err_t result = esp_now_send(broadcastAddress[destino], (uint8_t *)&msg, sizeof(msg));

    if (result == ESP_OK)
    {
        Serial.println("Dados enviados: ");
        Serial.print("Comando: ");
        Serial.println(enviado.comando);
        Serial.print("red: ");
        Serial.println(enviado.red);
        Serial.print("green: ");
        Serial.println(enviado.green);
        Serial.print("blue: ");
        Serial.println(enviado.blue);
    }
    else
    {
        Serial.println("Erro no envio da mensagem");
    }
}

void define_mensagem()
{
    if (SerialBT.available() > 0)
    {
        String recebido_BT = leBTSerial();

        if (recebido_BT[0] == '0')
        {
            String numb = "";
            for (int i = 1; i <= 3; i++)
            {
                numb.concat(recebido_BT[i]);
                if (i == 3) 
                {
                    enviado.red = numb.toInt();
                    numb = "";
                }

            }
            for (int i = 4; i <= 6; i++)
            {
                numb.concat(recebido_BT[i]);
                if (i == 6)
                {
                    enviado.green = numb.toInt();
                    numb = "";
                }
            }
            for (int i = 7; i <= 9; i++)
            {
                numb.concat(recebido_BT[i]);
                if (i == 9)
                {
                    enviado.blue = numb.toInt();
                    numb = "";
                }
            }
            for (int i = 10; i <= 11; i++)
            {
                numb.concat(recebido_BT[i]);
                if (i == 11)
                {
                    destino = numb.toInt();
                    numb = "";
                }
            }
            envia_mensagem();
        }
    }
}

String leBTSerial()
{
    String conteudo = "";
    String conteudo_final = "";

    // Enquanto receber algo pela serial
    while (SerialBT.available() > 0)
    {
        conteudo = SerialBT.readString();
        for (int i = 0; i <= conteudo.length(); i++)
        {
            if (i >= 5)
                conteudo_final.concat(conteudo[i]);
        }
    }
    Serial.print("Recebido pelo BT: ");
    Serial.println(conteudo_final);
    return conteudo_final;
}

void config_led(int pino, int channel_pwm)
{
    pinMode(pino, OUTPUT);
    ledcAttachPin(pino, channel_pwm);
    ledcSetup(channel_pwm, 1000, 8);
}

void cor_led(int c_red, int c_green, int c_blue, int red, int green, int blue)
{
    ledcWrite(c_red, red);
    ledcWrite(c_green, green);
    ledcWrite(c_blue, blue);
}

void start_ultrasonic()
{
    pinMode(echoPin, INPUT);  //DEFINE O PINO COMO ENTRADA (RECEBE)
    pinMode(trigPin, OUTPUT); //DEFINE O PINO COMO SAÍDA (ENVIA)
}

int dist_ultrasonic()
{
    distancia = ultrasonic.distanceRead();
    Serial.print("Distancia: ");
    Serial.println(distancia);
    return distancia;
}

void comando()
{
    if(recebido.comando == 0)
    {
        Serial.println("Ligar led");
        cor_led(CHANNEL_RED, CHANNEL_GREEN, CHANNEL_BLUE, recebido.red, recebido.green, recebido.blue);
    }
}
