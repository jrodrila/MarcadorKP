/*
 Name:		    maestro_ESP32_test_esp_now.ino
 Description:   Test comunicacion ESP8266 con ESP32
 Created:	    29/10/2022
 Author:	    Juan Carlos Rodriguez Lara
 Mail:          jrodrila@gmail.com
 Version:       0.1
*/

// Librerias
#ifdef ESP32
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif

#include <esp_now.h>
#include <Arduino.h>
#include <iostream>



// Definiciones pines ESP32C3
#define LIBRE_PIN_0     0    //(IO0 / ADC1_CH0 / XTAL_32K_N)
#define BOT_CH          1    //(IO1 / ADC1_CH1 / XTAL_32K_N)
#define BOT_UP          2    //(IO2 / ADC1_CH2 / FSPIQ)
#define PIN_RED         3    // ADC1_CH3 / (IO03 / ADC1_CH3)
#define PIN_GREEN       4    // ADC1_CH4 / (IO04 / ADC1_CH4 / FSPIHD / MTMS)
#define PIN_BLUE        5    // ADC2_CH0 / (IO05 / ADC2_CH0 / FSPIWP / MTDI)
#define I2C_SDA         6    //I2C_SDA (IO6 / FSPICLK / MTCK)  pin 7 de J3 _0x3C
#define I2C_SCL         7    //I2C_SCL (IO7 / FSPID / MTDO)    pn 8 de J3
#define LIBRE_PIN_8     8    //(IO8)
#define BUTTON_1        9    // IO09 / Boton integrado en PCB
#define LIBRE_PIN_10    10   //(IO10 / FSPICSO)
#define LIBRE_PIN_12    12   //(IO12 / SPIHD)
#define LIBRE_PIN_14    14   //(IO14 / SPICS0)
#define LIBRE_PIN_15    15   //(IO15 / SPICLK)
#define LIBRE_PIN_16    16   //(IO16 / SPID)
#define LIBRE_PIN_17    17   //(IO17 / SPIQ)
#define LED_PIN_1       18   // IO18 Led Naranja Integrado
#define LED_PIN_2       19   // IO19 Led Blanco Integrado
#define U0RX            20   // RX0 (IO20 / RX0)
#define U0TX            21   // TX0 (IO21 / TX0)




//Variables ESP-NOW
int id_pcb = 111;                   //ID de MCU

String success;                     //Varible para saber que el mensaje se ha entregado
uint8_t broadcastAddress1[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  //Direccion MAC donde queremos mandar los datos { 0xC8, 0xC9, 0xA3, 0x60, 0xFA, 0x67 }

//34:B4:72:4E:32:8C - esp32c3_1
//34:B4:72:4E:2A:84 - esp32c3_2
//30:C6:F7:29:01:28 - esp32-wroom-32d
//C8:C9:A3:60:FA:67 - lolin D1 mini

//Estructura para enviar datos
typedef struct struct_message {
    int id;          //ID de MCU
    bool rst;        //resetear
    bool aut;        //autoreset
    int cnt;         //contador
    int set_tiempo;  //ajustar tiempo
    int set_aviso;   //ajustar aviso
    bool upt_master;//Flag actualizar master
    bool upt_slave;//Flag actualizar slave
    float vsense; //Valor de tension del master
    bool pau;
} struct_message;

struct_message datos_master;   //creamos estructura para MANDAR datos al esclavo
struct_message datos_slave;  //creamos estructura para RECIBIR los datos del esclavo

esp_now_peer_info_t peerInfo;//No funciona con espnow.h

//FUNCIONES
/*ESP - NOW Callback when data is sent*/
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery SL OK" : "Fallo Entrega en ESCLAVO");
}


/*ESP - NOW Callback when data is received*/
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&datos_slave, incomingData, sizeof(datos_slave));
    
    Serial.print("Bytes on SLAVE: ");
    Serial.println(len);
    Serial.print(datos_slave.id);
    Serial.print("--> rst: ");
    Serial.print(datos_slave.rst);
    Serial.print(" - aut: ");
    Serial.print(datos_slave.aut);
    Serial.print(" - tiempo: ");
    Serial.print(datos_slave.set_tiempo);
    Serial.print(" - aviso: ");
    Serial.print(datos_slave.set_aviso);
    Serial.print(" - cnt: ");
    Serial.print(datos_slave.cnt);
    Serial.print(" - actualiza_slave: ");
    Serial.println(datos_slave.upt_slave);
    Serial.print(" - actualiza_master: ");
    Serial.println(datos_slave.upt_master);
    Serial.print(" - Tension master: ");
    Serial.println(datos_slave.vsense);
    Serial.print(" Pausado master: ");
    Serial.println(datos_slave.pau);
}

void setup()
{
    /*Iniciamos monitor serie*/
    Serial.begin(115200);
    /*Inicializamos a WIFI*/
    WiFi.mode(WIFI_STA);

    /*******************Init ESP-NOW***********************/
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializando ESP-NOW");
        return;//Esto estaba comentado
    }
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    // Preparamos info para registrar esclavo
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    // Añadimos esclavo
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Fallo añadiendo peer");
        return;
    }
    // Registramos funcion callback que sera llamada cuando recibimos datos
    esp_now_register_recv_cb(OnDataRecv);
    /*******************FIN Init ESP-NOW********************/

}

/*####################### BUCLE PRINCIPAL ######################*/
void loop() {

    /*Enviamos info ESP-NOW*/
    //Actualizar datos de envio
    delay(1500);

    datos_master.id = id_pcb;
    datos_master.cnt = 0;
    datos_master.rst = false;
    datos_master.aut = false;
    datos_master.set_tiempo = 20;
    datos_master.set_aviso = 10;
    datos_master.upt_master = false; //Activamos actualizacion del master si esta activo
    datos_master.pau = false;

    // Enviamos mensaje ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t*)&datos_master, sizeof(datos_master));
    if (result == ESP_OK) {
        Serial.print("Envio a esclavo desde ");
        Serial.print(datos_master.id);
        Serial.print(" ");
        Serial.print(millis());
        Serial.println(" OK");
    }
    else {
        Serial.println("Envio a esclavo NOK");
    }

    /*FIN Enviamos info ESP-NOW*/


}

