/*
 Name:		    esclavo_ESP8266_test_esp_now.ino
 Description:   Test comunicacion ESP8266 con ESP32
 Created:	    29/10/2022
 Author:	    Juan Carlos Rodriguez Lara
 Mail:          jrodrila@gmail.com
 Version:       0.1
*/

// Librerias
#include <ESP8266WiFi.h>
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
bool resetear = 0;                  //Para mandar un reset
bool auto_setting = 0;              //Para cambiar parámetro de autoreset
int tiempo_setting = 0;             //Para cambiar el parámetro de tiempo
int aviso_setting = 0;              //Para cambiar el parámetro de aviso
int contador_master = 0;            //Para guardar el valor del contador del maestro


String success;                     //Varible para saber que el mensaje se ha entregado
uint8_t broadcastAddress1[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };  //Direccion MAC donde queremos mandar los datos

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

struct_message datos_slave;   //creamos estructura para MANDAR datos del esclavo
struct_message datos_master;  //creamos estructura para RECIBIR los datos del maestro

//esp_now_peer_info_t peerInfo;//No funciona con espnow.h




//FUNCIONES
// ESP-NOW Callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    //Serial.print("\r\nLast:\t");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery MS OK" : "Fallo Entrega en MAESTRO");
    if (status == 0) {
        success = "Envio a Maestro OK :)";
    }
    else {
        success = "Envio a Maestro NOK :(";
    }
}
// ESP-NOW Callback when data is received
void OnDataRecv(uint8_t* mac, uint8_t* incomingData, uint8_t len) {
    memcpy(&datos_master, incomingData, sizeof(datos_master));
    //Serial.print("Bytes on SLAVE: ");
    //Serial.println(len);
    //resetear = datos_master.rst;
    actualizar = 1;
    tiempo_setting = datos_master.set_tiempo;
    aviso_setting = datos_master.set_aviso;
    contador_master = datos_master.cnt;
    param_upt_remote_esp = datos_master.upt_slave; // Actualizamos datos en el remoto si esta activo
    vsense_value = datos_master.vsense; //Actualizamosd el valor de tension del master
    contador = contador_master;

    Serial.print(datos_master.id);
    Serial.print("--> rst: ");
    Serial.print(datos_master.rst);
    Serial.print(" - aut: ");
    Serial.print(datos_master.aut;);
    Serial.print(" - tiempo: ");
    Serial.print(tiempo_setting);
    Serial.print(" - aviso: ");
    Serial.print(aviso_setting);
    Serial.print(" - cnt: ");
    Serial.print(contador_master);
    Serial.print(" - actualiza_slave: ");
    Serial.println(param_upt_remote_esp);
    Serial.print(" - Tension master: ");
    Serial.println(vsense_value);
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
    // Registramos funcion callback que sera llamada cuandop recibimos datos
    esp_now_register_recv_cb(OnDataRecv);
    /*******************FIN Init ESP-NOW********************/

}

/*####################### BUCLE PRINCIPAL ######################*/
void loop() {


    if (actualizar == 1)
    {

        /*Enviamos info ESP-NOW*/
        //Actualizar datos de envio
        aviso_setting = segundos_aviso;
        tiempo_setting = contador_max;
        auto_setting = autoreset;
        datos_slave.id = id_pcb;
        datos_slave.cnt = contador;
        datos_slave.rst = resetear;
        datos_slave.aut = auto_setting;
        datos_slave.set_tiempo = tiempo_setting;
        datos_slave.set_aviso = aviso_setting;
        datos_slave.upt_master = param_upt_local_esp; //Activamos actualizacion del master si esta activo
        datos_slave.pau = pausar_esp;
        // Enviamos mensaje ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t*)&datos_slave, sizeof(datos_slave));
        if (result == ESP_OK) {
            Serial.println("Envio a maestro OK");
            param_upt_local_esp = 0;
        }
        else {
            Serial.println("Envio a maestro NOK");
        }
        resetear = 0;
        actualizar = 0;
        /*FIN Enviamos info ESP-NOW*/



    }
}


