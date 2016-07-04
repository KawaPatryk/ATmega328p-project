#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <DHT.h>

int motorolaRX = 3; //podlaczam jacka czerwony kabel
int motorolaTX = 2; //podlaczam jacka zielony
int arudinoLedSmsPin = 13;
int hallPin = 10;
int hallLEDPin = 7;
int hallVCC = 8;
int servoPin = 11;
int dhtPin = 4;

String textSmsa = "Wrzucono list do skrzynki - otwarcie nr ";
int iloscOtwarc = 0;
Servo myservo; //stworzenie obiektu myservo
DHT dht(dhtPin, DHT11); //stworzenie obiektu dht
SoftwareSerial motorolaSerial(motorolaRX, motorolaTX); // Obiekt sluzacy do komunikacji z komorka przez port szeregowy (RX,TX)

/*resetuje czujnik halla
 * Czujnik halla jest zatrzaskowy, jak zostanie wlaczony potrzebuje resetu by znowu
 * wykryc zblizanie sie magnesu.
 * resetuje sie go odlaczajac i podlaczajac mu zasilanie
 */
void resetHall() {
	digitalWrite(hallVCC, LOW);
	delay(5);
	digitalWrite(hallVCC, HIGH);
}

// odczytuje długi tekst, który przychodzi na SoftwareSerial;
// (bez tej metody SoftwareSerial potrafi buforować tylko 64 bajty)
String motorolaSerialReadLongString() {
	static const long MAX_WAITING_TIME_FOR_NEXT_CHAR = 2000; //  maksymalny czas oczekiwania na następny znak [ms]
	String buffer = "";

	long lastReadTime = millis();

	while (millis() - lastReadTime < MAX_WAITING_TIME_FOR_NEXT_CHAR) {
		if (motorolaSerial.available()) {
			char ch = motorolaSerial.read();
			buffer += ch;

			lastReadTime = millis();
		}
	}

	return buffer;
}

// zczytuje smsy zamienia je na jeden dlugi string i zwraca (dodatkowo wypisuje na serial monitor)
String readAllSMS() {
	motorolaSerial.println("AT"); // Sends AT command to wake up cell phone
	delay(500);
	motorolaSerial.println("AT+CMGF=1"); // Puts phone into SMS mode
	delay(1000);
	motorolaSerial.readString();
	motorolaSerial.println("AT+CMGL=\"ALL\"");
	String wynik = motorolaSerialReadLongString();
	Serial.println(wynik);
	return wynik;
}

void deleteSMS() {
	motorolaSerial.println("AT"); // Sends AT command to wake up cell phone
	delay(500);
	motorolaSerial.println("AT+CMGF=1"); // Puts phone into SMS mode
	delay(1000);
	motorolaSerial.println("AT+CMGD=1"); // Deletes message at index of 1
	delay(5000); // Wait a second
}

void sendSMS(String tekst) {
	digitalWrite(arudinoLedSmsPin, HIGH); // Turn LED on.
//	motorolaSerial.println("AT"); // Sends AT command to wake up cell phone
//	delay(500);
//	motorolaSerial.println("AT+CMGF=1"); // Puts phone into SMS mode
//	delay(1000);
//	motorolaSerial.println("AT+CMGD=1"); // Deletes message at index of 1
//	delay(5000); // Wait a second
	motorolaSerial.println("AT"); // Sends AT command to wake up cell phone
	delay(500);
	motorolaSerial.println("AT+CMGW=\"+48792818853\""); // YOUR NUMBER HERE; Starting saving sms into memory of cell phone on first free position
	delay(1000);
	motorolaSerial.print(tekst); // saving message contest on free place in memory of cell phone
	delay(1000);
	motorolaSerial.write(byte(26)); // (signals end of message)
	delay(1000);
	motorolaSerial.println("AT+CMSS=1"); // Sends message at index of 1
	digitalWrite(arudinoLedSmsPin, LOW); // Turn LED off
	delay(250);
	digitalWrite(arudinoLedSmsPin, HIGH); // Turn LED on.
	delay(10000); // Give the phone time to send the SMS
	motorolaSerial.println("AT"); // Sends AT command to wake up cell phone
	delay(500);
	motorolaSerial.println("AT+CMGD=1"); // Deletes message at index of 1
	delay(5000);
	digitalWrite(arudinoLedSmsPin, LOW); // Turn LED off.
}
//dostaje wszystkie smsy w stringu, sprawdza czy ktorys jest zapytaniem o #wilg albo #temp
//jezeli jest to zwraca stringa #temp albo #wilg
//a jezeli nie ma to zwraca stringa Blad
String getHashCommand(String motorolaSMSs) {

	int pozycjaHasha; //pozycja hasha

	pozycjaHasha = motorolaSMSs.indexOf("#"); //znajdywanie pozycji na ktorej stoi hash i zapisywanie jej do zmiennej

	//szukanie czy tekstem jest wilg czy temp od pozycji hasla
	if (motorolaSMSs.substring(pozycjaHasha, pozycjaHasha + 5) == "#wilg") {
		//motorolaSMSs = "#wilg"; //zapisanie i zwrocenie ze chodzi o wilgotnosc
		return "#wilg";
	} else if (motorolaSMSs.substring(pozycjaHasha, pozycjaHasha + 5)
			== "#temp") {
		//motorolaSMSs = "#temp"; //zapisanie i zwrocenie ze chodzi o temperature
		return "#temp";
	} else {
		return "Blad"; //jezeli nie ma ani temp ani wilg zwraca blad
	}
}
//zwraca numer pierwszej wiadomosci sms ze stringa ktorego jej sie przekazuje
String getFirstSMSNumber(String motorolaSMSs) {

	int pozycjaTekstu = motorolaSMSs.indexOf("+CMGL: ");
	int pozycjaPrzecinka = motorolaSMSs.indexOf(",", pozycjaTekstu);
	if ((pozycjaTekstu == -1) || (pozycjaPrzecinka == -1)) {
		return "Blad";
	} else {
		return motorolaSMSs.substring(pozycjaTekstu + 7, pozycjaPrzecinka);
	}
}

void setup() {
//inicjalizacja czujnika dht11
	dht.begin();

//inicjalizacja serial monitora (na potrzeby testowania)
	Serial.begin(9600);

// inicjalizacja modulu wysylania SMSow
	pinMode(arudinoLedSmsPin, OUTPUT); // Initialize pin ArudinoLEDSMSPIN as digital out (LED)
	motorolaSerial.begin(4800); // Open serial connection at baud rate of 4800

//inicjalizacja modlulu halla
	pinMode(hallPin, INPUT); //ustawienie pinu halla na inputowy (arduino przyjmuje informacje)
	pinMode(hallLEDPin, OUTPUT); //pin LED(podlaczona dioda) informuje czy czujnik halla jest ustawiony na High czy Low
	digitalWrite(hallLEDPin, LOW);
	pinMode(hallVCC, OUTPUT); //zasilanie czujnika halla
	digitalWrite(hallVCC, HIGH);
	delay(1000);
	resetHall();

//inicjalizacja serwomechanizmu
	myservo.attach(servoPin);
	myservo.write(0); //kat poczatkowy 0 stopni przygotowanie do podniesienia choragiewki
	delay(2000);

	//wyswietlWszystkieSMS();
}

void loop() {
	Serial.println("Waiting...");
	delay(1000);
	Serial.println("Proceding...");
	//modul sprawdzajacy czujnik halla czy ktos wrzucil list do skrzynki
	Serial.println("Now the system is operating with hall sensor");
	int hallState = digitalRead(hallPin);

	if (hallState == HIGH) {
		digitalWrite(hallLEDPin, HIGH);
		iloscOtwarc = iloscOtwarc + 1;
		sendSMS(textSmsa + iloscOtwarc); //metoda wysylajaca smsa
		resetHall();
		myservo.write(90); //kat koncowz 90 stopni choragiewka podniesiona
		delay(2000);
		myservo.write(0); //powrot do 0 stopni, przygotowanie do kolejnego podniesienia
	} else if (hallState == LOW) {
		digitalWrite(hallLEDPin, LOW);
	}

	//sprawdza czy przyszedl sms z pytaniem o temp lub wilg
	Serial.println("Now system is responsing to sms commands");
	String command = getHashCommand(readAllSMS());

	if (command == "#temp") {
		Serial.print("sending sms with temperature ");

			float x = dht.readTemperature();
			char charBuf[6];
			dtostrf(x, 5, 2, charBuf);

			Serial.println(charBuf);

			deleteSMS();

			sendSMS(charBuf);
	} else if (command == "#wilg") {
		Serial.print("sending sms with humidity ");

		float x = dht.readHumidity();
		char charBuf[6];
		dtostrf(x, 5, 2, charBuf);

		Serial.println(charBuf);

		deleteSMS();

		sendSMS(charBuf);
	}

	//sukcesywnie usuwa wiadomosci
	/*
	 Serial.println("Now system is deleting messages, the first message");
	 delay(3000);
	 String numerSMSa = getFirstSMSNumber(readAllSMS());

	 if (numerSMSa != "Blad") {
	 motorolaSerial.println("AT"); // Sends AT command to wake up cell phone
	 delay(500);
	 motorolaSerial.println("AT+CMGF=1"); // Puts phone into SMS mode
	 delay(1000);
	 motorolaSerial.println("AT+CMGD=" + numerSMSa); // Deletes message at index of numerSMSa
	 delay(2000); // Wait a second
	 }
	 */
}
