#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <string.h>
#include <iostream>
#include <string>
#include <stdlib.h>

#ifdef __AVR__
#include <avr/power.h>
#endif

// GPIO0 -> D3
// GPIO1 -> TX
// GPIO2 -> D4
// GPIO3 -> RX
// GPIO4 -> D2
// GPIO5 -> D1
// GPIO12 -> D6
// GPIO13 -> D7
// GPIO14 -> D5
// GPIO15 -> D8
// GPIO16 -> D0

#define STRIPEPIN 0 //GPIO0 -> Pin D3
#define MQTT_CONN_KEEPALIVE 300 // Some MQTT Broker might require lower keepalive timings
#define NUM_LEDS 72

const char* host = "TestMqttClient01"; // REPLACE UNIQUE ID! MUST BE CHANGED FOR EVERY CLIENT!
String Topic = "TopicLevel1/TopicLevel2/"; // Adjust Topics for your own needs
const int port = 80; // Since no other https server is running port 80 can be used 
const char* updatePath = "/firmware"; // Can be left empty, since no other https server is running
const char* updateUsername = "fw_user";
const char* updatePassword = "fw_pass";
String BuildDateTime = __TIMESTAMP__;

const char* mqttServer = "192.168.0.1"; // REPLACE with your own MQTT server address
uint16_t mqttPort = 1883; // REPLACE with your own MQTT server port

ESP8266WebServer httpServer(port); 
ESP8266HTTPUpdateServer httpUpdater;

uint16_t ledCount = NUM_LEDS; // REPLACE with your actual number of leds
Adafruit_NeoPixel strip = Adafruit_NeoPixel(ledCount, STRIPEPIN, NEO_GRB + NEO_KHZ800);

const char* ssid = "wifi_ssid"; // REPLACE with your WiFi SSID
const char* password = "wifi_pass"; // REPLACE with your WiFi password

WiFiClient client;

PubSubClient pubSubClient(client);

unsigned long preMillis = 0;

uint16_t H;
uint8_t S;
uint8_t V;
uint8_t lastV;

bool publishHSV = true;

enum Mode { None, Balls, Rainbow, Fire, Random };
Mode CurrentMode = None;

int FunctionState[] = { 0, 0, 0 };
long stateC1 = 0;
int stateC2 = 0;
int stateC3 = 0;
uint updateInterval = 0;
uint32_t Color = strip.Color(50, 50, 50);

// Bouncing Balls variables
const int BallCount = 3;
float Gravity = -9.81;
int StartHeight = 5;
float Height[BallCount];
float ImpactVelocityStart = sqrt(-2 * Gravity * StartHeight);
float ImpactVelocity[BallCount];
float TimeSinceLastBounce[BallCount];
int   Position[BallCount];
long  ClockTimeSinceLastBounce[BallCount];
float Dampening[BallCount];
// Changed to HSV Colors, whereby HUE is in the range of 0-65535. For V the global V will be used.
uint16_t Colors[3][2] = { {0, 255}, // HUE: 0 -> Red, full saturation
            {21845, 255}, // HUE: 65535 / 3 = 21845 -> Green, full saturation
            {43690, 255} }; // HUE: 65535 / 3 * 2 = 43690 -> Blue, full saturation


void setup() {
    Serial.begin(115200);
    delay(10);

    manageWifi();

    // Print the addresses
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());

    httpUpdater.setup(&httpServer, updatePath, updateUsername, updatePassword);
    httpServer.begin();

    String updateSite = "http://" + WiFi.localIP().toString() + ":" + port + updatePath;

    Serial.print("HTTPUpdateServer ready! Open ");
    Serial.print(updateSite);
    Serial.print(" with username '");
    Serial.print(updateUsername);
    Serial.print("' and password '");
    Serial.print(updatePassword);
    Serial.println("'");

    pubSubClient.setServer(mqttServer, mqttPort);
    pubSubClient.setCallback(callback);
    pubSubConnect();

    // Initialize the LED strip
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'

    // Starting HSV values shall result in a warm, white light
    H = 5000;
    S = 220;
    V = 0;
    lastV = 50;

    pubSubClient.publish((Topic + "FirmwarePath").c_str(), updateSite.c_str(), true);
    pubSubClient.publish((Topic + "User").c_str(), updateUsername, true);
    pubSubClient.publish((Topic + "Password").c_str(), updatePassword, true); // ATTENTION: Clear text passwords should only be sent over secured connections!
    pubSubClient.publish((Topic + "IP").c_str(), WiFi.localIP().toString().c_str(), true);
    pubSubClient.publish((Topic + "Availability").c_str(), "ONLINE", true);
    pubSubClient.publish((Topic + "Build").c_str(), BuildDateTime.c_str(), true);

    digitalWrite(2, HIGH); //prevents the blue status led from lighting up
}

void loop() {
    manageWifi();

    httpServer.handleClient();

    pubSubConnect();

    pubSubStats();

    timeServant();
}

void timeServant()
{
    if (CurrentMode != None)
    {
        if (millis() - preMillis >= updateInterval)
        {
            if (CurrentMode == Balls)
            {
                BouncingBalls();
            }
            else if (CurrentMode == Rainbow)
            {
                rainbow();
            }
            else if (CurrentMode == Fire)
            {
                fire(40, 80);
            }
            else if (CurrentMode == Random)
            {
                randomStripe();
            }
            preMillis = millis();
            preMillis = millis();
        }
    }
}

void manageWifi()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        // Connect to WiFi network
        Serial.println();
        Serial.println();
        Serial.print("Connecting to ");
        Serial.println(ssid);

        WiFi.mode(WIFI_STA);
        //WiFi.mode(WIFI_AP_STA);

        WiFi.begin(ssid, password);

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        delay(1000);
    }
}

void BouncingBalls()
{
    for (int i = 0; i < BallCount; i++) {
        TimeSinceLastBounce[i] = millis() - ClockTimeSinceLastBounce[i];
        Height[i] = 0.5 * Gravity * pow(TimeSinceLastBounce[i] / 1000, 2.0) + ImpactVelocity[i] * TimeSinceLastBounce[i] / 1000;

        if (Height[i] < 0) {
            Height[i] = 0;
            ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
            ClockTimeSinceLastBounce[i] = millis();

            if (ImpactVelocity[i] < 0.01) {
                ImpactVelocity[i] = ImpactVelocityStart;
            }
        }
        Position[i] = round(Height[i] * (NUM_LEDS - 1) / StartHeight);
    }
    for (int i = 0; i < BallCount; i++) {
        strip.setPixelColor((Position[i] - NUM_LEDS + 1) * -1, strip.ColorHSV(Colors[i][0], Colors[i][1], V));
    }

    strip.show();
    strip.clear();
}

void fire(int Cooling, int Sparking)
{
    static byte heat[NUM_LEDS];
    int cooldown;

    // Step 1.  Cool down every cell a little
    for (int i = 0; i < NUM_LEDS; i++) {
        cooldown = random(0, ((Cooling * 10) / NUM_LEDS) + 2);

        if (cooldown > heat[i]) {
            heat[i] = 0;
        }
        else {
            heat[i] = heat[i] - cooldown;
        }
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for (int k = NUM_LEDS - 1; k >= 2; k--) {
        heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' near the bottom
    if (random(255) < Sparking) {
        int y = random(7);
        heat[y] = heat[y] + random(160, 255);
        //heat[y] = random(160,255);
    }

    // Step 4.  Convert heat to LED colors
    for (int j = 0; j < NUM_LEDS; j++) {
        setPixelHeatColor((j - NUM_LEDS + 1) * -1, heat[j]);
    }

    strip.show();
}

void setPixelHeatColor(int Pixel, byte temperature)
{
    // Scale 'heat' down from 0-255 to 0-191
    byte t192 = round((temperature / 255.0) * V);

    // calculate ramp up from
    byte heatramp = t192 & 0x3F; // 0..63
    heatramp <<= 2; // scale up to 0..252

    // figure out which third of the spectrum we're in:
    if (t192 > 0x80) {                     // hottest
        strip.setPixelColor(Pixel, 255, 255, heatramp);
    }
    else if (t192 > 0x40) {             // middle
        strip.setPixelColor(Pixel, 255, heatramp, 0);
    }
    else {                               // coolest
        strip.setPixelColor(Pixel, heatramp, 0, 0);
    }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow()
{
    if (stateC1 >= 5 * 65536) stateC1 = 0;
    while (stateC2 < ledCount)
    {
        int pixelHue = stateC1 + (stateC2 * 65536L / strip.numPixels());
        strip.setPixelColor(stateC2, strip.ColorHSV(pixelHue, 255, V));
        stateC2++;
    }
    strip.show();
    stateC2 = 0;
    stateC1 += 256;
}

void randomStripe()
{
    strip.setPixelColor(random(NUM_LEDS), random(255), random(255), random(255));
    strip.setBrightness(V);
    strip.show();
}

void pixelCorrection(uint16_t h, uint8_t s, uint8_t v, uint16_t n = ledCount + 1)
{
    if (n >= ledCount)
    {
        for (int i = 0; i < ledCount; i++)
        {
            if (v >= 10 || i % 10 <= v || CurrentMode != None)
            {
                strip.setPixelColor(i, strip.ColorHSV(h, s, v));
            }
            else
            {
                strip.setPixelColor(i, 0, 0, 0);
            }
        }
    }
    else
    {
        strip.setPixelColor(n, strip.ColorHSV(h, s, v));
    }
    strip.show();
}

void pubSubConnect()
{
    if (!pubSubClient.connected())
    {
        // Loop until we're reconnected
        while (!pubSubClient.connected())
        {
            Serial.print("Attempting MQTT connection...");
            // Attempt to connect
            if (pubSubClient.connect(host, (Topic + "Availability").c_str(), 0, true, "OFFLINE")) //ATTENTION: UNIQUE ID! MUST BE CHANGED FOR EACH NEW CLIENT!!!
            {
                pubSubClient.subscribe((Topic + "set").c_str());
                Serial.println("connected");
            }
            else
            {
                Serial.print("failed, rc=");
                Serial.print(pubSubClient.state());
                Serial.println(" try again in 5 seconds");
                // Wait 5 seconds before retrying
                delay(5000);
            }
        }
    }
    pubSubClient.loop();
}

void callback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, (Topic + "set").c_str()) == 0)
    {
        Serial.print("Received set hsv cmd: ");
        Serial.print((char*)payload);
        Serial.print(" (length ");
        Serial.print(length);
        Serial.println(")");

        std::string payloadString = "";
        for (int i = 0; i < length; i++) {
            payloadString += (char)payload[i];
        }

        std::transform(payloadString.begin(), payloadString.end(), payloadString.begin(),
            [](unsigned char c) { return std::tolower(c); });

        std::string delimiter = " ";
        size_t pos = payloadString.find(delimiter);
        std::string key = payloadString.substr(0, pos);
        int value = atoi(payloadString.substr(pos + delimiter.length(), length).c_str());
        Serial.println((key + " Received").c_str());

        if (key == "hue")
        {
            H = value;
            CurrentMode = None;
        }
        else if (key == "sat")
        {
            S = value;
            CurrentMode = None;
        }
        else if (key == "bri")
        {
            if (value == 0) lastV = V;
            V = value;
        }
        else if (key == "rainbow")
        {
            CurrentMode = Rainbow;
            updateInterval = value;
            if (V == 0) V = lastV;
        }
        else if (key == "balls")
        {
            CurrentMode = Balls;
            updateInterval = 0;
            StartHeight = value;
            if (V == 0) V = lastV;
            for (int i = 0; i < BallCount; i++) {
                ClockTimeSinceLastBounce[i] = millis();
                Height[i] = StartHeight;
                Position[i] = 0;
                ImpactVelocity[i] = ImpactVelocityStart;
                TimeSinceLastBounce[i] = 0;
                Dampening[i] = 0.90 - float(i) / pow(BallCount, 2);
            }
        }
        else if (key == "fire")
        {
            CurrentMode = Fire;
            updateInterval = value;
            if (V == 0) V = lastV;
        }
        else if (key == "none")
        {
            if (V == 0)
            {
                V = lastV;
            }
            CurrentMode = None;
        }
        else if (key == "random")
        {
            CurrentMode = Random;
            updateInterval = value;
            if (V == 0) V = lastV;
        }
        else if (key == "on")
        {
            if (V == 0)
            {
                V = lastV;
                CurrentMode = None;
            }
        }
        else if (payloadString == "off")
        {
            lastV = V;
            V = 0;
            CurrentMode = None;
        }

        publishHSV = true;

        pixelCorrection(H, S, V, ledCount + 1);
    }
}

void pubSubStats()
{
    if (publishHSV)
    {
        char hVal[4];
        sprintf(hVal, "%d", H);
        char sVal[4];
        sprintf(sVal, "%d", S);
        char vVal[4];
        sprintf(vVal, "%d", V);

        char hsvVal[12];
        strcpy(hsvVal, hVal);
        strcat(hsvVal, ",");
        strcat(hsvVal, sVal);
        strcat(hsvVal, ",");
        strcat(hsvVal, vVal);

        if (pubSubClient.publish((Topic + "hsv").c_str(), hsvVal))
        {
            Serial.println("HSV values published");
            publishHSV = false;
        }
        else
        {
            Serial.println("Could not publish HSV values");
        }

        String mode = "None";
        if (CurrentMode == Balls) mode = "Balls";
        else if (CurrentMode == Rainbow) mode = "Rainbow";
        else if (CurrentMode == Fire) mode = "Fire";
        else if (CurrentMode == Fire) mode = "Random";

        if (pubSubClient.publish((Topic + "mode").c_str(), mode.c_str()))
        {
            Serial.println("Mode published");
        }
        else
        {
            Serial.println("Could not publish Mode");
        }

        if (V > 0)
        {
            pubSubClient.publish((Topic + "state").c_str(), "on");
        }
        else
        {
            pubSubClient.publish((Topic + "state").c_str(), "off");
        }
    }
}
