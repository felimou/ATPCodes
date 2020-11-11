#include <esp_now.h>
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <HCSR04.h>

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
    String mensagem;
    unsigned long time;
} struct_data;

struct_data msg;

//Variáveis auxiliares para mensagens enviadas/recebidas
String enviado;
String recebido;

//Variável da comunicação Bluetooth
BluetoothSerial SerialBT;

//Escopo de Funções
void start_espnow();
void onReceiveData(const uint8_t *mac, const uint8_t *data, int len);
void OnDataSent(const uint8_t *mac, esp_now_send_status_t status);
void envia_mensagem();
void define_mensagem();
String leStringSerial();
int leBTSerial();

void setup()
{
    //Inicializa Serial
    Serial.begin(115200);
    //Inicializa BluetoothSerial
    SerialBT.begin("ESP32");

    //Define modo do Wi-Fi em Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    //Printa no Serial o Endereço MAC da ESP32
    Serial.println();
    Serial.println(WiFi.macAddress());

    //Inicializa a comunicação esp_now e adiciona os pares definidos em broadcastAddress
    start_espnow();

    //Registra o Callback de recebimento de dados
    esp_now_register_recv_cb(onReceiveData);
    //Registra o Callback de status de envio
    esp_now_register_send_cb(OnDataSent);
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

void onReceiveData(const uint8_t *mac, const uint8_t *data, int len)
{
    char macStr[18];
    Serial.print("Pacote recebido de: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println(macStr);

    memcpy(&msg, data, sizeof(msg));

    recebido = msg.mensagem;
    Serial.println("Dados recebidos: ");
    Serial.print("Mensagem: ");
    Serial.println(recebido);
    Serial.println("\n\n");
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
    msg.mensagem = enviado;
    esp_err_t result = esp_now_send(broadcastAddress[destino], (uint8_t *)&msg, sizeof(msg));

    if (result == ESP_OK)
    {
        Serial.println("Dados enviados: ");
        Serial.print("Mensagem: ");
        Serial.println(msg.mensagem);
    }
    else
    {
        Serial.println("Erro no envio da mensagem");
    }
}

void define_mensagem()
{
    if (Serial.available() > 0)
    {
        // Lê toda string recebida
        String recebido_Serial = leStringSerial();

        if (recebido_Serial == "C0")
        {
            destino = 0;
            envia_mensagem();
        }

        if (recebido_Serial == "C1")
        {
            destino = 1;
            envia_mensagem();
        }

        if (recebido_Serial == "C2")
        {
            destino = 2;
            envia_mensagem();
        }

        if (recebido_Serial == "C3")
        {
            destino = 3;
            envia_mensagem();
        }
    }

    if (SerialBT.available() > 0)
    {
        int recebido_BT = leBTSerial();

        if (recebido_BT == 48) //0 em decimal
        {
            destino = 0;
            envia_mensagem();
        }

        if (recebido_BT == 49) //1 em decimal
        {
            destino = 1;
            envia_mensagem();
        }

        if (recebido_BT == 50) //2 em decimal
        {
            destino = 2;
            envia_mensagem();
        }

        if (recebido_BT == 51) //3 em decimal
        {
            destino = 3;
            envia_mensagem();
        }

        if (recebido_BT == 52) //4 em decimal
        {
        }
    }
}

String leStringSerial()
{
    String conteudo = "";
    char caractere;

    // Enquanto receber algo pela serial
    while (Serial.available() > 0)
    {
        // Lê byte da serial
        caractere = Serial.read();
        // Ignora caractere de quebra de linha
        if (caractere != '\n')
        {
            // Concatena valores
            conteudo.concat(caractere);
        }
        // Aguarda buffer serial ler próximo caractere
        delay(3);
    }
    Serial.print("Recebido pelo Serial: ");
    Serial.println(conteudo);
    return conteudo;
}

int leBTSerial()
{
    int caractere;
    int conteudo;
    if (SerialBT.available() > 0)
    {
        caractere = SerialBT.read();
        if (caractere != 10 && caractere != 13)
        {
            conteudo = caractere;
        }
        //Serial.print("Recebido pelo BT: ");
        //Serial.println(conteudo);
    }
    return conteudo;
}
