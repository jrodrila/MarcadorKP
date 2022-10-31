/*
 Name:		    esclavo_ESP8266_test_esp_now.ino
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

#include <espnow.h>
#include <Arduino.h>
#include <iostream>



// Definiciones pines LOLIN D1 mini ESP8266EX
#define BOT_CH          0    //IO0
#define BOT_UP          2    //IO2
#define I2C_SDA         4    //IO4 / I2C_SDA 
#define I2C_SCL         5    //IO5 / I2C_SCL 
#define LIBRE_PIN_10    A0   //A0
#define LIBRE_PIN_12    12   //IO12 / MISO
#define LIBRE_PIN_14    14   //IO14 / SCK
#define LIBRE_PIN_15    15   //IO15
#define LIBRE_PIN_16    16   //IO16
#define LIBRE_PIN_13    13   //IO13 / MOSI
#define U0RX            RX   // RX
#define U0TX            TX   // TX




//Variables ESP-NOW
int id_pcb = 121;                   //ID de MCU


String success;                     //Varible para saber que el mensaje se ha entregado
uint8_t broadcastAddress1[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  //Direccion MAC donde queremos mandar los datos

//34:B4:72:4E:32:8C - esp32c3_2 
//34:B4:72:4E:2A:84 - esp32c3_1
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

struct_message datos_slave;   //creamos estructura para MANDAR datos del esclavo
struct_message datos_master;  //creamos estructura para RECIBIR los datos del maestro



//FUNCIONES
// ESP-NOW Callback when data is sent
void OnDataSent(uint8_t* mac_addr, uint8_t sendStatus) {
    
    if (sendStatus == 0) {
        Serial.print("Envio a maestro desde ");
        Serial.print(datos_slave.id);
        Serial.print(" ");
        Serial.print(millis());
        Serial.println(" OK");
    }
    else {
        Serial.println("Delivery fail to Master");
    }
}
// ESP-NOW Callback when data is received
void OnDataRecv(uint8_t* mac, uint8_t *incomingData, uint8_t len) {
    memcpy(&datos_master, incomingData, sizeof(datos_master));
    Serial.print("Bytes on SLAVE: ");
    Serial.println(len);
 
    Serial.print(datos_master.id);
    Serial.print("--> rst: ");
    Serial.print(datos_master.rst);
    Serial.print(" - aut: ");
    Serial.print(datos_master.aut);
    Serial.print(" - tiempo: ");
    Serial.print(datos_master.set_tiempo);
    Serial.print(" - aviso: ");
    Serial.print(datos_master.set_aviso);
    Serial.print(" - cnt: ");
    Serial.print(datos_master.cnt);
    Serial.print(" - actualiza_slave: ");
    Serial.println(datos_master.upt_slave);
    Serial.print(" - actualiza_master: ");
    Serial.println(datos_master.upt_master);
    Serial.print(" - Tension master: ");
    Serial.println(datos_master.vsense);
    Serial.print(" Pausado master: ");
    Serial.println(datos_master.pau);
}



void setup()
{
    /*Iniciamos monitor serie*/
    Serial.begin(115200);
    /*Inicializamos a WIFI*/
    WiFi.mode(WIFI_STA);

    /*******************Init ESP-NOW***********************/
    if (esp_now_init() != 0) {
        Serial.println("Error initializando ESP-NOW");
        return;
    }

    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);//Exclusivo espnow.h

    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

    // Registramos funcion callback que sera llamada cuandop recibimos datos
    esp_now_register_recv_cb(OnDataRecv);
    /*******************FIN Init ESP-NOW********************/

}

/*####################### BUCLE PRINCIPAL ######################*/
void loop() {


    delay(1500);

        /*Enviamos info ESP-NOW*/
        //Actualizar datos de envio
        datos_slave.id = id_pcb;
        datos_slave.cnt = 10;
        datos_slave.rst = false;
        datos_slave.aut = false;
        datos_slave.set_tiempo = 30;
        datos_slave.set_aviso = 10;
        datos_slave.upt_master = false; //Activamos actualizacion del master si esta activo
        datos_slave.pau = false;
        // Enviamos mensaje ESP-NOW

        // Send message via ESP-NOW
        esp_now_send(broadcastAddress1, (uint8_t*)&datos_slave, sizeof(datos_slave));

        /*FIN Enviamos info ESP-NOW*/


}


