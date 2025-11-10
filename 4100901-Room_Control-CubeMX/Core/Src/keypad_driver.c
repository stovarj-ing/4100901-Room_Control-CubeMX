#include "keypad_driver.h"
#include "main.h"

/**
 * @brief Mapa lógico del teclado matricial 4x4.
 * 
 * Este arreglo bidimensional asocia la posición física (fila, columna)
 * con el carácter correspondiente a cada tecla. 
 * 
 * Por ejemplo:
 *  - keypad_map[0][0] = '1'
 *  - keypad_map[3][3] = 'D'
 */
static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};


/**
 * @brief Inicializa las filas del teclado en nivel bajo.
 * 
 * Esta función deja las filas del teclado en estado bajo (0 lógico),
 * preparando el hardware para la detección de una tecla presionada
 * mediante los flancos descendentes que se observan en las columnas.
 * 
 * @param keypad Puntero a la estructura de configuración del teclado.
 */
void keypad_init(keypad_handle_t* keypad) {
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        // Se asegura que todas las filas comiencen en nivel bajo
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_RESET);
    }
}


/**
 * @brief Escanea el teclado y determina qué tecla fue presionada
 *        según la columna activada.
 * 
 * El funcionamiento es el siguiente:
 * 1. Se identifica el índice de la columna que activó la interrupción o evento.
 * 2. Se establecen todas las filas en alto.
 * 3. Se baja una fila a la vez y se verifica si la columna leída se mantiene en bajo.
 * 4. Si la lectura es baja, significa que la tecla en esa fila y columna está presionada.
 * 5. Se espera hasta que la tecla sea liberada (evita rebotes).
 * 
 * @param keypad   Puntero a la estructura que contiene la configuración del keypad.
 * @param col_pin  Pin de la columna que se activó (detectado desde la interrupción o lectura externa).
 * @return El carácter correspondiente a la tecla presionada, o '\0' si no se detectó ninguna.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin) {
    HAL_Delay(5); // Breve retardo para estabilizar la señal y evitar lecturas erróneas (antirrebote simple)

    int col_index = -1;  // Variable para almacenar el índice de la columna detectada

    // Se busca qué columna del arreglo coincide con el pin recibido
    for (int i = 0; i < KEYPAD_COLS; i++) {
        if (keypad->col_pins[i] == col_pin) {
            col_index = i;
            break;
        }
    }

    // Si no se encontró la columna, no se puede determinar ninguna tecla
    if (col_index == -1) 
        return '\0'; 

    char key_pressed = '\0'; // Valor por defecto si no se detecta tecla


    // Se colocan todas las filas en alto antes del escaneo
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }

    // Se analiza cada fila para determinar cuál tiene la tecla presionada
    for (int row = 0; row < KEYPAD_ROWS; row++) {
        // Se pone en bajo una fila a la vez
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_RESET);
        HAL_Delay(1); // Pequeña pausa para estabilizar el cambio de nivel

        // Si la columna está en bajo, se detecta una tecla presionada en esa fila
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
            key_pressed = keypad_map[row][col_index]; // Se obtiene el carácter correspondiente

            // Espera a que la tecla sea liberada (para evitar múltiples lecturas del mismo evento)
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET);
            break;
        }

        // Se vuelve a poner la fila en alto antes de pasar a la siguiente
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET);
    }

    // Se restauran todas las filas a nivel bajo (estado de reposo)
    keypad_init(keypad);

    return key_pressed;
}
