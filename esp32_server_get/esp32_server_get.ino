#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

/* Credenciales de nuestra red WiFi */
const char* ssid = "Megacable_2.4G_B46C";
const char* password = "9fdvCAE3";


WebServer server(80);

int valor = 0;

void handleRoot() {
  String paginaHTML = "";
  paginaHTML += "<html><body>";
  paginaHTML += "<h1>Ejemplo ESP32 - Textbox Numérico</h1>";
  paginaHTML += "<form action='/guardar' method='get'>";
  paginaHTML += "<label for='valor'>Ingrese un valor:</label>";
  paginaHTML += "<input type='number' id='valor' name='valor'>";
  paginaHTML += "<input type='submit' value='Enviar'>";
  paginaHTML += "</form>";
  paginaHTML += "</body></html>";
  server.send(200, "text/html", paginaHTML);
}

/* Cuando se presiona el boton de Enviar del formulario */
void handleGuardar() {
  valor = server.arg("valor").toInt();
  String mensaje = "Valor recibido: " + String(valor);
  server.send(200, "text/plain", mensaje);
  Serial.println(mensaje);
}

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print('.');
  }
  Serial.println("OK!");

  // Esta es la dirección IP de nuestro servidor, en el dispositivo donde se accedera al servidor, 
  // debemos de estar conectados a la misma red WiFi que nuestro servidor
  // Y en nuestro navegador, deberemos de acceder a esta dirección IP
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP()); 

  // Registrar la pagina HTML en el servidor
  server.on("/", handleRoot);
  server.on("/guardar", handleGuardar); // Añadir la respuesta HTML del formulario de la pagina HTML
  server.begin(); // Arrancar el servidor


  Serial.println("Servidor iniciado.");
}

void loop() {
  server.handleClient();
}