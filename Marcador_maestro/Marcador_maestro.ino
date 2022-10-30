/*
 Name:		    Marcador_maestro.ino
 Description:   MARCADOR POSESION KAYAK POLO
 Created:	    19/03/2022 13:01:25
 Author:	    Juan Carlos Rodriguez Lara
 Mail:          jrodrila@gmail.com
 Version:       0.1 - Master
*/

// Librerias
#include <dummy.h>
#include <Wire.h> //Libreria para que funcione el I2C de la pantalla OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Arduino.h>
#include <iostream>
#include <Preferences.h>

// Definiciones Pantalla_OLED
#define SCREEN_WIDTH        128   // OLED width,  in pixels
#define SCREEN_HEIGHT       32    // OLED height, in pixels
#define TIEMPO_AVISO        150   // Tiempo activo de la bocina durante el aviso
#define TIEMPO_FINAL        400   // Tiempo activo de la bocina durante el final
#define TIEMPO_DELAY_INT_ON 300   // Tiempo de seguridad para volver a activar interrupciones

// Definiciones pines ESP32-DevKit v4
#define LIBRE_IO36    36      // IO36 / I   /   GPIO36, ADC1_CH0, S_VP                    -> J1 - Pin3 (no pull-up resistors)
#define LIBRE_IO39    39      // IO39 / I   /   GPIO39, ADC1_CH3, S_VN                    -> J1 - Pin4 (no pull-up resistors)   
#define VSENSE        34      // IO34 / I   /   GPIO34, ADC1_CH6, VDET_1                  -> J1 - Pin5 (no pull-up resistors)
#define LIBRE_IO35    35      // IO35 / I   /   GPIO35, ADC1_CH7, VDET_2                  -> J1 - Pin6 (no pull-up resistors)  
#define SEG_DC_A      32      // IO32 / IO  /   GPIO32, ADC1_CH4, TOUCH_CH9, XTAL_32K_P   -> J1 - Pin7 
#define SEG_DC_B      33      // IO33 / IO  /   GPIO33, ADC1_CH5, TOUCH_CH8, XTAL_32K_N   -> J1 - Pin8 
#define SEG_DC_C      25      // IO25 / IO  /   GPIO25, ADC1_CH8, DAC_1                   -> J1 - Pin9
#define SEG_DC_D      26      // IO26 / IO  /   GPIO26, ADC2_CH9, DAC_2                   -> J1 - Pin10
#define SEG_UN_A      27      // IO27 / IO  /   GPIO27, ADC2_CH7, TOUCH_CH7               -> J1 - Pin11
#define SEG_UN_B      14      // IO14 / IO  /   GPIO14, ADC2_CH6, TOUCH_CH6, MTMS         -> J1 - Pin12
#define SEG_UN_C      12      // IO12 / IO  /   GPIO12, ADC2_CH5, TOUCH_CH5, MTDI         -> J1 - Pin13 
#define SEG_UN_D      13      // IO13 / IO  /   GPIO13, ADC2_CH4, TOUCH_CH4, MTCK         -> J1 - Pin15 
#define D2            9       // IO09 / IO  /   GPIO9, D2                                 -> J1 - Pin16 (no usar: flash)
#define D3            10      // IO10 / IO  /   GPIO10, D3                                -> J1 - Pin17 (no usar: flash)
#define CMD           11      // IO11 / IO  /   GPIO11, CMD                               -> J1 - Pin18 (no usar: flash)
#define SEG_UN_dot    23      // IO23 / IO  /   GPIO23                                    -> J3 - Pin2
#define I2C_SCL       22      // IO22 / IO  /   GPIO22                                    -> J3 - Pin3
#define U0TXD         1       // IO01 / IO  /   GPIO1, U0TXD                              -> J3 - Pin4
#define U0RXD         3       // IO03 / IO  /   GPIO3, U0RXD                              -> J3 - Pin5
#define I2C_SDA       21      // IO21 / IO  /   GPIO21                                    -> J3 - Pin6
#define SEG_DC_dot    19      // IO19 / IO  /   GPIO19                                    -> J3 - Pin8
#define BOT_CH        18      // IO18 / IO  /   GPIO18                                    -> J3 - Pin9
#define BOT_UP        5       // IO05 / IO  /   GPIO5                                     -> J3 - Pin10
#define LIBRE_IO17    17      // IO17 / IO  /   GPIO17                                    -> J3 - Pin11
#define LIBRE_IO16    16      // IO16 / IO  /   GPIO16                                    -> J3 - Pin12
#define LIBRE_IO04    4       // IO04 / IO  /   GPIO4, ADC2_CH0, TOUCH_CH0                -> J3 - Pin13
#define BUTTON_1      0       // IO00 / IO  /   GPIO0, ADC2_CH1, TOUCH_CH1, Boot          -> J3 - Pin14 (boton integrado en PCB)
#define LIBRE_IO02    2       // IO02 / IO  /   GPIO2, ADC2_CH2, TOUCH_CH2                -> J3 - Pin15
#define BOCINA        15      // IO15 / IO  /   GPIO15, ADC2_CH3, TOUCH_CH3, MTDO         -> J3 - Pin16
#define D1            8       // IO08 / IO  /   GPIO8, D1                                 -> J3 - Pin17 (no usar: flash)
#define D0            7       // IO07 / IO  /   GPIO7, D0                                 -> J3 - Pin18 (no usar: flash)
#define SCK           6       // IO06 / IO  /   GPIO6, SCK                                -> J3 - Pin19 (no usar: flash)


// Definiciones
#define MAX_DISPLAY   60      // Valor máximo por defecto
#define AUTORESET     false   // Autoreset por defecto activado
#define AVISO         20       //Tiempo de aviso de posesión


//Variables timer
unsigned long intervalo = 1000; //milisegundos a contar
unsigned long tiempo_previo = 0;
unsigned long tiempo_actual = 0;
unsigned long tiempo_previo_boton = 0;

//Variables microprocesador
uint32_t frecuencia = 0;
uint32_t cpu_freq_mhz = 160;     //Menos de 80 MHz no va bien la WIFI ni el puerto serie

//Variables marcador
Preferences preferences; // Objeto para guardar datos en la NVS
const int timeThreshold = 500;  //Tiempo de filtro de rebote para los botones
long contador_max = MAX_DISPLAY;
long contador_max_temp = MAX_DISPLAY;
long contador = contador_max;
long tiempo_bocina;
long tiempo_bocina_int_on;
bool autoreset = AUTORESET;
bool autoreset_temp = AUTORESET;
int decenas;
int unidades;
int segundos_aviso = AVISO;
int segundos_aviso_temp = AVISO;
int activar_aviso = 0;
int activar_final = 0;
bool final_activado = false; // Variable para saber si ya ha sonado la bocina, cuando no hay autoreset
bool estadoBOT_CH = 0;
bool estadoBOT_UP = 0;
float vsense_value = 0;
bool numero_bcd[10][4] = {
    {0,0,0,0},//0
    {0,0,0,1},//1
    {0,0,1,0},//2
    {0,0,1,1},//3
    {0,1,0,0},//4
    {0,1,0,1},//5
    {0,1,1,0},//6
    {0,1,1,1},//7
    {1,0,0,0},//8
    {1,0,0,1},//9
};
bool param_upt_local = 0;           //Flag para actualizar los parámetros de los menus cuando volvemos al menu principal
bool pausar = false;                //Pausar el marcador


//Variables menu
int numScreen = 0; //Número de pantalla


//Variables ESP-NOW
int id_pcb = 100;
bool resetear = 0;                  //Para mandar un reset
bool auto_setting = 0;              //Para cambiar parámetro de autoreset
int tiempo_setting = 0;             //Para cambiar el parámetro de tiempo
int aviso_setting = 0;              //Para cambiar el parámetro de aviso
int contador_slave = 0;             //Para guardar el valor del contador del esclavo
bool param_upt_local_esp = 0;       //Flag para actualizar los parámetros del master
bool param_upt_remote_esp = 1;      //Flag para actualizar los parámetros del slave. Por defecto mandamos configuracion del master
bool pausar_esp = false;
String success;                     //Varible para saber que el mensaje se ha entregado
uint8_t broadcastAddress1[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };//Direccion MAC donde queremos mandar los datos

//Estructura para enviar datos
typedef struct struct_message {
    int id;
    bool rst;//resetear
    bool aut;//autoreset
    int cnt;//contador
    int set_tiempo;//ajustar tiempo
    int set_aviso;//ajustar aviso
    bool upt_master;//Flag actualizar master
    bool upt_slave;//Flag actualizar slave
    float vsense;
    bool pau;
} struct_message;

struct_message datos_slave;//creamos estructura para RECIBIR datos del esclavo
struct_message datos_master;//creamos estructura para MANDAR los datos del maestro

esp_now_peer_info_t peerInfo;

//Objetos
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);//Pantalla OLED 0.91

//Interrupciones
void IRAM_ATTR ISR_BOT_CH() //Boton cambio de funcion #####NO FUNCIONA PRINTLN (resetea el micro) 
{
    if (millis() - tiempo_previo_boton > timeThreshold)
    {
        pausar_esp = !pausar_esp; //Temporal
        /*estadoBOT_CH = 1;
        if (numScreen < 4) {
            numScreen++;
        }
        else {
            numScreen = 0;
        }*/
        tiempo_previo_boton = millis();
    }


}
void IRAM_ATTR ISR_BOT_UP() //Boton cambio de opcion/Resetear el contador#####NO FUNCIONA PRINTLN (resetea el micro) 
{
    if (millis() - tiempo_previo_boton > timeThreshold)
    {
        estadoBOT_UP = 1;
        tiempo_previo_boton = millis();
    }

}

//FUNCIONES
// ESP-NOW Funcion Callback cuando mandamos datos
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    //Serial.print("\r\nLast:\t");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery SL OK" : "Fallo Entrega en ESCLAVO");
    if (status == 0) {
        success = "Envio a Esclavo OK :)";
    }
    else {
        success = "Envio a Esclavo NOK :(";
    }
}
// ESP-NOW Funcion Callback cuando recibimos datos
void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
    memcpy(&datos_slave, incomingData, sizeof(datos_slave));
    //Serial.print("Bytes on MASTER: ");
    //Serial.println(len);
    resetear = datos_slave.rst;
    auto_setting = datos_slave.aut;
    tiempo_setting = datos_slave.set_tiempo;
    aviso_setting = datos_slave.set_aviso;
    contador_slave = datos_slave.cnt;
    param_upt_local_esp = datos_slave.upt_master; // Actualizamos datos en el master si esta activo
    if (param_upt_local_esp == 1) {
        autoreset = auto_setting;
        contador_max = tiempo_setting;
        segundos_aviso = aviso_setting;
    }
    //pausar_esp = datos_slave.pau;
    //DEBUGGER
    Serial.print(datos_slave.id);
    Serial.print("--> rst: ");
    Serial.print(resetear);
    Serial.print(" - aut: ");
    Serial.print(auto_setting);
    Serial.print(" - tiempo: ");
    Serial.print(tiempo_setting);
    Serial.print(" - aviso: ");
    Serial.print(aviso_setting);
    Serial.print(" - cnt: ");
    Serial.print(contador_slave);
    Serial.print(" - actualiza_master: ");
    Serial.println(param_upt_local_esp);
    Serial.print(" Pausar master: ");
    Serial.println(pausar_esp);
}

//Funcion para actualizar el OLED
void actualizarOLED(int menu)
{
    oled.clearDisplay(); // clear display
    if (menu == 0)
    {
        if (param_upt_local == 1) {
            contador_max = contador_max_temp;//Actualizamos el contador máximo
            segundos_aviso = segundos_aviso_temp;//Actualizamos el aviso
            autoreset = autoreset_temp;//Actualizamos autoreset

            preferences.begin("my-app", false);
            preferences.putBool("autoreset", autoreset); // Store the counter to the Preferences
            preferences.putLong("contador_max", contador_max);
            preferences.putInt("segundos_aviso", segundos_aviso);
            preferences.end();   // Close the Preferences

            param_upt_local = 0;
            param_upt_remote_esp = 1;
        }
        oled.setTextSize(4);
        oled.setCursor(35, 5);       // set position to display
        oled.println(String(decenas)); // set text
        oled.setCursor(60, 5);       // set position to display
        oled.println(String(unidades)); // set text 
        oled.setTextSize(1);
        oled.setCursor(90, 0);       // set position to display
        oled.println("T: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);       // set position to display
        oled.println(String(contador_max - 1)); // set text
        oled.setTextSize(1);
        oled.setCursor(90, 18);       // set position to display
        oled.println("A: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);       // set position to display
        oled.println(String(segundos_aviso)); // set text
        if (autoreset) {
            oled.setTextSize(1);
            oled.setCursor(0, 25);       // set position to display
            oled.println("AUTO"); // set text
        }
    }
    if (menu == 1)
    {
        if (estadoBOT_UP) {
            contador_max_temp += 10;
            if (contador_max_temp > 99) {
                contador_max_temp = 11;
            }
            param_upt_local = 1;
            estadoBOT_UP = 0;
            
        }
        oled.setTextSize(1);
        oled.setCursor(90, 0);       // set position to display
        oled.println("T: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 0);       // set position to display
        oled.println(String(contador_max_temp - 1)); // set text
    }
    if (menu == 2)
    {
        if (estadoBOT_UP) {
            segundos_aviso_temp += 5;
            if (segundos_aviso_temp > contador_max) {
                segundos_aviso_temp = 0;
            }
            param_upt_local = 1;
            estadoBOT_UP = 0;
            
        }
        oled.setTextSize(1);
        oled.setCursor(90, 18);       // set position to display
        oled.println("A: "); // set text
        oled.setTextSize(2);
        oled.setCursor(100, 18);       // set position to display
        oled.println(String(segundos_aviso_temp)); // set text
    }
    if (menu == 3)
    {
        if (estadoBOT_UP) {
            autoreset_temp = !autoreset_temp;
            param_upt_local = 1;
            estadoBOT_UP = 0;
            
        }

        oled.setTextSize(2);
        oled.setCursor(50, 18);       // set position to display
        if (autoreset_temp) {
            oled.println("ON"); // set text
            oled.setTextSize(1);
            oled.setCursor(0, 25);       // set position to display
            oled.println("AUTO"); // set text
        }
        else {
            oled.println("OFF"); // set text
        }

    }
    if (menu == 4)
    {
        oled.setTextSize(1);
        //oled.setCursor(6, 0);
        //oled.print("-IP:");
        //oled.println(WiFi.localIP());
        oled.setCursor(0,8);
        oled.print(frecuencia);
        oled.println(" MHz");
        oled.setCursor(0,16);
        oled.print(vsense_value);
        oled.println(" Volts on Master");
    }
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(String((menu + 1)));
    oled.println("-MS");
    oled.display();              // display on OLED
}

//Funcion para verificar si activamos la bocina  
void eval_bocina()
{
    if (contador == segundos_aviso)
    {
        //Serial.println("Aviso 5 segundos");
        activar_aviso = 2;
    }
    if (contador == 0)
    {
        //Serial.println("Piiiiiii");
        if (final_activado == false) {
            activar_final = 2; // Activamos bocina final
        }
        
    }
}
//Funcion para activar la bocina
void activar_bocina()
{
    if (tiempo_bocina <= tiempo_actual) // Desactivamos bocina
    {
        digitalWrite(BOCINA, 1);
    }
    if (tiempo_bocina_int_on <= tiempo_actual) // Activamos interrupciones
    {
        attachInterrupt(BOT_CH, ISR_BOT_CH, FALLING);
        attachInterrupt(BOT_UP, ISR_BOT_UP, FALLING);

    }
    if (activar_aviso == 2)
    {
        detachInterrupt(BOT_UP);
        detachInterrupt(BOT_CH);
        if (vsense_value > 10) { //Activamos bocina si superamos el voltage minimo de trabajo, para proteger bateria
            digitalWrite(BOCINA, 0); }
        tiempo_bocina = tiempo_actual + TIEMPO_AVISO;
        tiempo_bocina_int_on = tiempo_actual + TIEMPO_AVISO + TIEMPO_DELAY_INT_ON;
        activar_aviso = 0;

    }
    if (activar_final == 2)
    {
        detachInterrupt(BOT_UP);
        detachInterrupt(BOT_CH);
        if (vsense_value > 10) { //Activamos bocina si superamos el voltage minimo de trabajo, para proteger bateria
            digitalWrite(BOCINA, 0); }
        tiempo_bocina = tiempo_actual + TIEMPO_FINAL;
        tiempo_bocina_int_on = tiempo_actual + +TIEMPO_FINAL + TIEMPO_DELAY_INT_ON;
        final_activado = true;
        activar_final = 0;
    }
}

//Funcion autoreset
void eval_autoreset()
{
    if (contador == 0 && autoreset == true)
    {
        contador = contador_max;
        final_activado = false;
    }
    else if (contador == 0 && autoreset == false)
    {
        contador = 1;
    }
}
//Funcion actualizar display
void display(int dec, int unds)
{
    bool display_unds[4] = { numero_bcd[unds][0],numero_bcd[unds][1],numero_bcd[unds][2],numero_bcd[unds][3]};
    bool display_dec[4] = { numero_bcd[dec][0],numero_bcd[dec][1],numero_bcd[dec][2],numero_bcd[dec][3] };

    digitalWrite(SEG_UN_A, display_unds[3]);
    digitalWrite(SEG_UN_B, display_unds[2]);
    digitalWrite(SEG_UN_C, display_unds[1]);
    digitalWrite(SEG_UN_D, display_unds[0]);

    digitalWrite(SEG_DC_A, display_dec[3]);
    digitalWrite(SEG_DC_B, display_dec[2]);
    digitalWrite(SEG_DC_C, display_dec[1]);
    digitalWrite(SEG_DC_D, display_dec[0]);
}





void setup()
{
    /*Iniciamos monitor serie*/
    Serial.begin(115200);
    /*Definimos frecuencia de trabajo del micro*/
    setCpuFrequencyMhz(cpu_freq_mhz);

    /*Inicializar libreria Preferences*/
    preferences.begin("my-app", false); // my-app es el nombre del namespace --> preferences.clear(); para limpiar todo 
    unsigned int counter = preferences.getUInt("counter", 0); //Obtener el contador, si no existe asigna 0 por defecto

    autoreset = preferences.getBool("autoreset", AUTORESET);
    autoreset_temp = autoreset;
    Serial.printf("Autoreset inicio: %u\n", autoreset);

    contador_max = preferences.getLong("contador_max", MAX_DISPLAY);
    contador_max_temp = contador_max;
    Serial.printf("Contador max inicio: %u\n", contador_max);

    segundos_aviso = preferences.getInt("segundos_aviso", AVISO);
    segundos_aviso_temp = segundos_aviso;
    Serial.printf("Aviso inicio: %u\n", segundos_aviso);


    counter++; //incrementamos el contador de inicios en 1
    Serial.printf("Contador de inicio: %u\n", counter); // Imprimimos valor actual
    preferences.putUInt("counter", counter); // Store the counter to the Preferences
    preferences.end();   // Close the Preferences

    /*Inicializamos Reles*/

    pinMode(SEG_UN_A, OUTPUT);
    pinMode(SEG_UN_B, OUTPUT);
    pinMode(SEG_UN_C, OUTPUT);
    pinMode(SEG_UN_D, OUTPUT);
    pinMode(SEG_UN_dot, OUTPUT);
    pinMode(SEG_DC_A, OUTPUT);
    pinMode(SEG_DC_B, OUTPUT);
    pinMode(SEG_DC_C, OUTPUT);
    pinMode(SEG_DC_D, OUTPUT);
    pinMode(SEG_DC_dot, OUTPUT);

    pinMode(BOCINA, OUTPUT);//Bocina
    digitalWrite(BOCINA, 1);

    /*Pines botones*/
    pinMode(BOT_CH, INPUT_PULLUP);//Boton cambio menu
    pinMode(BOT_UP, INPUT_PULLUP);//Botion cambio opcion


    /*Conectamos a WIFI*/
    WiFi.mode(WIFI_STA);//Debemos inicializar WIFI antes de ESP-NOW
    //WiFi.begin(ssid, password);
    //if (WiFi.waitForConnectResult() != WL_CONNECTED)
    // {
    //   Serial.printf("WiFi Failed!\n");
    //   //return;
    // }
    // Serial.print("IP: ");
    // Serial.println(WiFi.localIP());

    /*******************Init ESP-NOW***********************/
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializando ESP-NOW");
        //return;
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

    /*Inicializamos interrupciones*/
    attachInterrupt(BOT_CH, ISR_BOT_CH, FALLING);//Boton para cambio de funcion
    attachInterrupt(BOT_UP, ISR_BOT_UP, FALLING);//Boton para resetear el tiempo / cambio de opcion

    /*********************Inicializamos OLED***************/
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("failed to start SSD1306 OLED"));
        while (1);
    }
    delay(1000);                  // wait two seconds for initializing
    oled.clearDisplay();          // clear display
    delay(200);
    oled.setTextSize(2);         // set text size
    oled.setTextColor(WHITE);    // set text color
    oled.setCursor(0, 0);         // set position to display
    oled.println(" Marcador ");   // set text
    oled.setCursor(0, 17);        // set position to display
    oled.println(" v0.1 MS");       // set text
    oled.display();               // display on OLED
    delay(1000);
    /*********************FIN Inicializamos OLED*****************/

     /* DEBUGGER */
    frecuencia = getCpuFrequencyMhz();  // In MHz  
    Serial.print(frecuencia);
    Serial.println(" MHz");
}

/*####################### BUCLE PRINCIPAL ######################*/
void loop()
{
    tiempo_actual = millis();
    activar_bocina();

    if (tiempo_actual - tiempo_previo >= intervalo)
    {
        tiempo_previo = tiempo_actual;

        //Reseteamos el tiempo de forma local    
        if (numScreen == 0 && estadoBOT_UP == 1) {
            contador = contador_max;
            final_activado = false;

            Serial.print(final_activado);
            Serial.println(" final activado local");
            
            estadoBOT_UP = 0;
        }
        //Reseteamos el tiempo de forma remota   
        if (resetear == 1) {
            contador = contador_max;
            final_activado = false;
            Serial.print(final_activado);
            Serial.println(" final activado remoto");
            resetear = 0;
        }

        Serial.print("pausar_esp: ");
        Serial.println(pausar_esp);

        if (pausar_esp == false) {
            contador--;
            Serial.print("contador: ");
            Serial.println(contador);
        }
           

        decenas = (contador - (contador % 10)) / 10;
        unidades = contador - decenas * 10;

        display(decenas, unidades);//Actualizar LCD
        actualizarOLED(numScreen);


        /*Enviamos info ESP-NOW*/
        //Actualizar datos de envio
        aviso_setting = segundos_aviso;
        tiempo_setting = contador_max;
        auto_setting = autoreset;
        pausar = pausar_esp;

        datos_master.id = id_pcb;
        datos_master.cnt = contador;
        datos_master.rst = resetear;
        datos_master.aut = auto_setting;
        datos_master.set_tiempo = tiempo_setting;
        datos_master.set_aviso = aviso_setting;
        datos_master.upt_slave = param_upt_remote_esp; //Activamos actualizacion de los esclavos si esta activo
        datos_master.vsense = vsense_value; //Valor de la tension del master
        //datos_master.pau = pausar;
        // Enviamos mensaje ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t*)&datos_master, sizeof(datos_master));
        if (result == ESP_OK) {
            Serial.println("Envio a esclavo OK");
            param_upt_remote_esp = 0; //Enviada peticion de actualizacion, reseteamos la variable
        }
        else {
            Serial.println("Envio a esclavo NOK");
        }
        /*FIN Enviamos info ESP-NOW*/
        eval_bocina();
        eval_autoreset();
        vsense_value = analogRead(VSENSE);
        vsense_value = ((vsense_value)*6)/1000;
        /* DEBUGGER */
        Serial.print(vsense_value);
        Serial.println(" Volts");
    }

}

