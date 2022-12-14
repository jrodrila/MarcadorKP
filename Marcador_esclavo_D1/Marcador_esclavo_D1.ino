/*
 Name:		    Marcador_maestro.ino
 Description:   MARCADOR POSESION KAYAK POLO
 Created:	    19/03/2022 13:01:25
 Author:	    Juan Carlos Rodriguez Lara
 Mail:          jrodrila@gmail.com
 Version:       0.1 - Slave
*/

// Librerias
#include <Wire.h>  //Libreria para que funcione el I2C de la pantalla OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Arduino.h>
#include <iostream>


// Definiciones Pantalla_OLED
#define SCREEN_WIDTH 128  // OLED width,  in pixels
#define SCREEN_HEIGHT 32  // OLED height, in pixels

// Definiciones pines LOLIN D1 mini ESP8266EX
#define BOT_CH          0    //IO0
#define BOT_UP          2    //IO2//LED Interno
#define I2C_SDA         4    //IO4 / I2C_SDA 
#define I2C_SCL         5    //IO5 / I2C_SCL 
#define LIBRE_PIN_10    A0   //A0
#define LIBRE_PIN_12    12   //IO12 / MISO
#define LIBRE_PIN_14    14   //IO14 / SCK
#define LIBRE_PIN_15          15   //IO15
#define LIBRE_PIN_16    16   //IO16
#define LIBRE_PIN_13    13   //IO13 / MOSI
#define U0RX            RX   // RX
#define U0TX            TX   // TX

// Definiciones
#define MAX_DISPLAY     61          // Valor m?ximo por defecto
#define AUTORESET       false        // Autoreset por defecto activado
#define AVISO           20           //Tiempo de aviso de posesi?n
#define mS_TO_S_FACTOR  1000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP   10         /* Time ESP32 will go to sleep (in seconds) */



//Variables timer
unsigned long intervalo = 1000;  //milisegundos a contar
unsigned long tiempo_previo_boton = 0;
unsigned long tiempo_previo = 0;
unsigned long tiempo_pausa_on = 0;
unsigned long tiempo_pausa_off = 0;
unsigned long tiempo_pausa = 2000; //Tiempo manteniendo el botoin de cambio, para qeu se pause el marcador


//Variables microprocesador
uint32_t frecuencia = 0;
uint32_t cpu_freq_mhz = 160;     //Menos de 80 MHz no va bien la WIFI ni el puerto serie



//Variables marcador

const int timeThreshold = 500;  //Tiempo de filtro de rebote para los botones
long contador_max = MAX_DISPLAY;
long contador_max_temp = MAX_DISPLAY;
long contador = contador_max;
bool autoreset = AUTORESET;
bool autoreset_temp = AUTORESET;
int decenas;
int unidades;
int segundos_aviso = AVISO;
int segundos_aviso_temp = AVISO;
bool estadoLED = 0;  //SOLO Esclavo
bool estadoBOT_CH = 0;
bool estadoBOT_UP = 0;
float vsense_value = 0;
bool actualizar = 0;      //Flag para actualizar datos
bool param_upt_local = 0;           //Flag para actualizar los par?metros de los menus cuando volvemos al menu principal
bool pausar = false;

//Variables menu
int numScreen = 0;  //N?mero de pantalla


//Variables ESP-NOW
int id_pcb = 111;                   //ID de MCU
bool resetear = 0;                  //Para mandar un reset
bool auto_setting = 0;              //Para cambiar par?metro de autoreset
int tiempo_setting = 0;             //Para cambiar el par?metro de tiempo
int aviso_setting = 0;              //Para cambiar el par?metro de aviso
int contador_master = 0;            //Para guardar el valor del contador del maestro
bool param_upt_local_esp = 1;       //Flag para actualizar los par?metros del master
bool param_upt_remote_esp = 0;      //Flag para actualizar los par?metros del slave
bool pausar_esp = false;

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


//Objetos
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  //Pantalla OLED 0.91

//FUNCIONES
// ESP-NOW Callback when data is sent
void OnDataSent(uint8_t* mac_addr, uint8_t status) {
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
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
    memcpy(&datos_master, incomingData, sizeof(datos_master));
    //Serial.print("Bytes on SLAVE: ");
    //Serial.println(len);
    //resetear = datos_master.rst;
    actualizar = 1;
    auto_setting = datos_master.aut;
    tiempo_setting = datos_master.set_tiempo;
    aviso_setting = datos_master.set_aviso;
    contador_master = datos_master.cnt;
    param_upt_remote_esp = datos_master.upt_slave; // Actualizamos datos en el remoto si esta activo
    vsense_value = datos_master.vsense; //Actualizamosd el valor de tension del master
    contador = contador_master;
    if (param_upt_remote_esp == 1) {
        autoreset = auto_setting;
        contador_max = tiempo_setting;
        segundos_aviso = aviso_setting;
    }
    Serial.print(datos_master.id);
    Serial.print("--> rst: ");
    Serial.print(datos_master.rst);
    Serial.print(" - aut: ");
    Serial.print(auto_setting);
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

//Funcion para actualizar el OLED
void actualizarOLED(int menu)
{
    oled.clearDisplay();  // clear display
    if (menu == 0) {
        if (param_upt_local == 1) {
            contador_max = contador_max_temp;//Actualizamos el contador m?ximo
            segundos_aviso = segundos_aviso_temp;//Actualizamos el aviso
            autoreset = autoreset_temp;//Actualizamos autoreset

            param_upt_local = 0;
            param_upt_local_esp = 1;
        }
        oled.setTextSize(4);
        oled.setCursor(35, 5);           // set position to display
        oled.println(String(decenas));   // set text
        oled.setCursor(60, 5);           // set position to display
        oled.println(String(unidades));  // set text
        oled.setTextSize(1);
        oled.setCursor(90, 0);  // set position to display
        oled.println("T: ");    // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);                  // set position to display
        oled.println(String(contador_max - 1));  // set text
        oled.setTextSize(1);
        oled.setCursor(90, 18);  // set position to display
        oled.println("A: ");     // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);               // set position to display
        oled.println(String(segundos_aviso));  // set text
        if (autoreset) {
            oled.setTextSize(1);
            oled.setCursor(0, 25);  // set position to display
            oled.println("AUTO");   // set text
        }
    }
    if (menu == 1) {
        if (estadoBOT_UP) {
            contador_max_temp += 10;
            if (contador_max_temp > 99) {
                contador_max_temp = 11;
            }
            param_upt_local = 1;
            estadoBOT_UP = 0;
        }
        oled.setTextSize(1);
        oled.setCursor(90, 0);  // set position to display
        oled.println("T: ");    // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);                       // set position to display
        oled.println(String(contador_max_temp - 1));  // set text
    }
    if (menu == 2) {
        if (estadoBOT_UP) {
            segundos_aviso_temp += 5;
            if (segundos_aviso_temp > contador_max) {
                segundos_aviso_temp = 0;
            }
            param_upt_local = 1;
            estadoBOT_UP = 0;
        }
        oled.setTextSize(1);
        oled.setCursor(90, 18);  // set position to display
        oled.println("A: ");     // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);               // set position to display
        oled.println(String(segundos_aviso_temp));  // set text
    }
    if (menu == 3) {
        if (estadoBOT_UP) {
            autoreset_temp = !autoreset_temp;
            param_upt_local = 1;
            estadoBOT_UP = 0;
        }
        oled.setTextSize(2);
        oled.setCursor(50, 18);  // set position to display
        if (autoreset_temp) {
            oled.println("ON");  // set text
            oled.setTextSize(1);
            oled.setCursor(0, 25);  // set position to display
            oled.println("AUTO");   // set text
        }
        else {
            oled.println("OFF");  // set text
        }
    }
    if (menu == 4) {
        oled.setTextSize(1);
        //oled.setCursor(6, 0);
        //oled.print("-IP:");
        //oled.println(WiFi.localIP());
        oled.setCursor(0, 8);
        oled.print(frecuencia);
        oled.println(" MHz");
        oled.setCursor(0, 16);
        oled.print(vsense_value);
        oled.println(" Volts on Master");
    }
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(String((menu + 1)));
    oled.println("-SL");
    oled.display();  // display on OLED
}





//Interrupciones
void IRAM_ATTR ISR_button1()  //Resetear el contador#####NO FUNCIONA PRINTLN (resetea el micro)
{
    estadoLED = !estadoLED;
  
    //Serial.println("Estadoboton: ON " + String(estadoLED));
    resetear = 1;
    actualizar = 1;
    //power_saving = 0; //desactivamos power saving
}

void IRAM_ATTR ISR_BOT_CH()  //Boton cambio de funcion #####NO FUNCIONA PRINTLN (resetea el micro)
{
    tiempo_pausa_on = millis();
    if (millis() - tiempo_previo_boton > timeThreshold) {
        estadoBOT_CH = 1;
        actualizar = 1;
        if (numScreen < 4) {
            numScreen++;
        }
        else {
            numScreen = 0;
        }
        tiempo_previo_boton = millis();
        //power_saving = 0; //desactivamos power saving
    }
}

void IRAM_ATTR ISR_BOT_CH_RS()  //Boton cambio de funcion #####NO FUNCIONA PRINTLN (resetea el micro)
{
    tiempo_pausa_off = millis();

    if (tiempo_pausa_off - tiempo_pausa_on > tiempo_pausa) {
        pausar = true;
        actualizar = 1;
        numScreen = 0;
    }
}
void IRAM_ATTR ISR_BOT_UP()  //Boton cambio de opcion/Resetear el contador#####NO FUNCIONA PRINTLN (resetea el micro)
{
    if (millis() - tiempo_previo_boton > timeThreshold) {
        actualizar = 1;
        estadoBOT_UP = 1;
        resetear = 1;
        tiempo_previo_boton = millis();
        //power_saving = 0; //desactivamos power saving
    }
}


void setup()
{
    /*Iniciamos monitor serie*/
    Serial.begin(115200);
    /*Definimos frecuencia de trabajo del micro*/
    //setCpuFrequencyMhz(cpu_freq_mhz);

    /*Inicializar libreria Preferences*/
   
    

    autoreset = AUTORESET;
    autoreset_temp = autoreset;
    Serial.printf("Autoreset inicio: %u\n", autoreset);

    contador_max = MAX_DISPLAY;
    contador_max_temp = contador_max;
    Serial.printf("Contador max inicio: %u\n", contador_max);

    segundos_aviso = AVISO;
    segundos_aviso_temp = segundos_aviso;
    Serial.printf("Aviso inicio: %u\n", segundos_aviso);




    /*I2C Inicializacion*/
    Wire.begin(I2C_SDA, I2C_SCL);
    delay(500);//500


    /*Pines botones*/
    // HIGH interruptor abierto - LOW interruptor cerrado.

    pinMode(BOT_CH, INPUT_PULLUP);//Boton cambio menu
    pinMode(BOT_UP, INPUT_PULLUP);//Botion cambio opcion



    /*Inicializamos a WIFI*/
    WiFi.mode(WIFI_STA);


    /*******************Init ESP-NOW***********************/
    if (esp_now_init() != ERR_OK) {
        Serial.println("Error initializando ESP-NOW");
        return;//Esto estaba comentado
    }
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    // Preparamos info para registrar esclavo
    esp_now_add_peer(broadcastAddress1, ESP_NOW_ROLE_COMBO, 0, NULL, 0);


    // Set ESP-NOW Role
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);//Exclusivo espnow.h


    // Registramos funcion callback que sera llamada cuandop recibimos datos
    esp_now_register_recv_cb(OnDataRecv);
    /*******************FIN Init ESP-NOW********************/

    /*Inicializamos interrupciones*/
    
    attachInterrupt(BOT_CH, ISR_BOT_CH, FALLING);//Boton para cambio de funcion
    //attachInterrupt(BOT_CH, ISR_BOT_CH_RS, RISING);//Interrupcion para detectar la caida del boton, y pausar el timer
    attachInterrupt(BOT_UP, ISR_BOT_UP, FALLING);////Boton para resetear el tiempo / cambio de opcion
    //Serial.print(bootCount);
    //Serial.println(" inicios");

    /*********************Inicializamos OLED*****************/
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("failed to start SSD1306 OLED"));
        while (1);
    }
    oled.clearDisplay();          // clear display
    oled.setTextSize(2);         // set text size
    oled.setTextColor(WHITE);    // set text color

    //if (bootCount == 0) {
    oled.clearDisplay();          // clear display
    oled.setTextSize(2);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 0);         // set position to display
    oled.println(" Marcador ");   // set text
    oled.setCursor(0, 17);        // set position to display
    oled.println(" v0.1 SL");       // set text
    oled.display();               // display on OLED
    delay(1000);
    //   }
   /*********************FIN Inicializamos OLED*****************/
   /* DEBUGGER */
    frecuencia = 0;  // In MHz
    Serial.print(frecuencia);
    Serial.println(" MHz");



}

/*####################### BUCLE PRINCIPAL ######################*/
void loop() {

    //if (millis() - tiempo_previo >= power_saving_time) {
     //   oled.clearDisplay();
     //   oled.display(); 
     //   esp_sleep_enable_ext0_wakeup(GPIO_NUM_02, 1);

    //}

    if (actualizar == 1)
    {
        decenas = (contador - (contador % 10)) / 10;
        unidades = contador - decenas * 10;
        //if (millis() - tiempo_actual > power_saving_time) {
        //    power_saving = 1;
        //    oled.clearDisplay();
        //    oled.display();               
        //}

        //if (power_saving == 0) {
        actualizarOLED(numScreen);

        //}

        if (pausar == true) {
            pausar_esp = !pausar_esp;
            pausar = false;
        }

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
      
        if (esp_now_send(broadcastAddress1, (uint8_t*)&datos_slave, sizeof(datos_slave)) == ERR_OK) {
            Serial.println("Envio a maestro OK");
            param_upt_local_esp = 0;
        }
        else {
            Serial.println("Envio a maestro NOK");
        }
        resetear = 0;
        actualizar = 0;
        /*FIN Enviamos info ESP-NOW*/

        //eval_RGB();

       // esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // ESP32 wakes up every 0.8 seconds
        //Serial.println("Going to light-sleep now");
        //Serial.flush();
        //esp_deep_sleep_start();

        Serial.print("Pausar: ");
        Serial.println(pausar_esp);

    }
}
