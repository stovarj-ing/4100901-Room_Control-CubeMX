#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

#include "main.h"
#include <stdint.h>

// Definición del número de filas y columnas del keypad (4x4 en este caso)
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

/**
 * @brief Estructura que define la configuración de pines del teclado matricial.
 * 
 * Contiene los puertos GPIO y los pines correspondientes a las filas y columnas.
 * De esta forma, el keypad puede configurarse fácilmente sin modificar el código fuente.
 */
typedef struct {
    GPIO_TypeDef* row_ports[KEYPAD_ROWS];  // Arreglo con los puertos GPIO asignados a las filas
    uint16_t row_pins[KEYPAD_ROWS];        // Arreglo con los pines correspondientes a las filas
    GPIO_TypeDef* col_ports[KEYPAD_COLS];  // Arreglo con los puertos GPIO asignados a las columnas
    uint16_t col_pins[KEYPAD_COLS];        // Arreglo con los pines correspondientes a las columnas
} keypad_handle_t;


/**
 * @brief Inicializa el keypad configurando los pines necesarios.
 * 
 * Esta función debe:
 * - Configurar los pines de las filas como salidas.
 * - Configurar los pines de las columnas como entradas (preferiblemente con resistencias pull-down o pull-up).
 * 
 * @param keypad Puntero a la estructura que contiene los puertos y pines del keypad.
 */
void keypad_init(keypad_handle_t* keypad);


/**
 * @brief Escanea el teclado matricial para detectar qué tecla se presiona.
 * 
 * Esta función implementa la técnica de escaneo por columnas y filas:
 * 1. Se activa una fila a la vez (poniéndola en nivel bajo o alto según diseño).
 * 2. Se leen las columnas para ver si alguna tiene una conexión (tecla presionada).
 * 3. Se determina qué tecla fue presionada con base en la posición [fila][columna].
 * 
 * @param keypad Puntero a la estructura del teclado configurado.
 * @param col_pin Pin de la columna que se está leyendo en ese instante.
 * @return El carácter correspondiente a la tecla presionada (por ejemplo, '1', 'A', '*', etc.).
 *         Si no se detecta ninguna tecla presionada, puede retornar un carácter nulo o definido como ‘\0’.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin);

#endif // KEYPAD_DRIVER_H
