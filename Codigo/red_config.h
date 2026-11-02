#pragma once
#include <Arduino.h>
#include <string.h>

// ============================================================
// CONFIGURACION DE RED - UN SOLO LUGAR PARA TODO EL PROYECTO
//
// Edite SOLO estas dos lineas. Este mismo archivo lo usan:
//   - el firmware del ESP32 del AGV   (AGV_firmware_v4)
//   - el firmware de la ESP32-CAM     (CameraWebServer)
//
// COMO USARLO:
//   1) Escriba aqui el nombre de la red y su clave.
//   2) Copie ESTE archivo dentro de la carpeta de cada sketch
//      (junto al .ino), con el mismo contenido en las dos.
//   3) Vuelva a subir los dos programas.
//
// REQUISITOS DE LA RED:
//   - Banda de 2.4 GHz (el ESP32 no funciona en 5 GHz).
//   - La PC con la aplicacion debe estar en la MISMA red.
//   - El router no debe tener "aislamiento de clientes" (AP isolation).
//
// No suba este archivo a un repositorio publico: contiene la clave.
// ============================================================
#define WIFI_SSID     "TU_RED_WIFI"
#define WIFI_PASSWORD "TU_CLAVE_WIFI"

// Devuelve true si todavia estan los valores de ejemplo.
inline bool redSinConfigurar() {
  return strcmp(WIFI_SSID, "TU_RED_WIFI") == 0;
}
