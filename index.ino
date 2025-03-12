#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>

/**
 * Declaramos los motores derecho e izquierdo
 * como una instancia de la librería "Servo".
 */
Servo motor_derecho, motor_izquierdo;

/**
 * Inicia una instancia de la librería "Adafruit MPU6050".
 */
Adafruit_MPU6050 mpu;

/**
 * El giroscopio devuelve datos de 16 bits, 
 * entonces necesitamos crear una variable 
 * de tipo "int 16" para guardar estos datos.
 */
int16_t raw_acc_x_axis, raw_acc_y_axis, raw_acc_z_axis, raw_gyro_x_axis, raw_gyro_y_axis, raw_gyro_z_axis;
 
/**
 * Variable para almacenar las aceleraciones de los ejes X, Y.
 */
float angulo_aceleracion[2];

/**
 * Variable para almacenar los datos del giroscopio en X, Y.
 */
float angulo_gyro[2];
float angulo_total[2];

/**
 * Variables para el control del tiempo.
 */
float tiempo_transcurrido, tiempo, tiempo_anterior;
int i;

/**
 * Constante de conversión de radianes a grados.
 */
float rad_a_grad = 180/3.141592654;

/**
 * Variables para almacenar la acción de control PID, el ancho de
 * pulso para cada motor, el error actual y el error anterior.
 */
float PID, pwm_izquierda, pwm_derecha, error, error_anterior;

float pid_p=0;
float pid_i=0;
float pid_d=0;

/**
 * CONSTANTES DEL PID
 */
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05

// Representa el valor inicial del throttle ("aceleración") de los motores.
double throttle=1300;

// Representa el valor deseado (en grados) para que la balanza tenga como punto de ajuste.
float set_point = 0;

// Variables para guardar los datos del sensor.
sensors_event_t a, g, temp;

void setup() {
  Serial.begin(115200);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  motor_derecho.attach(3);    // Asociamos el motor derecho al PIN 3 de la ESP32.
  motor_izquierdo.attach(5);  // Asociamos el motor derecho al PIN 5 de la ESP32.

  // Comienza a contar el tiempo en milisegundos.
  tiempo = millis();

  /**
   * Para poner en marcha los ESC tenemos que enviar un valor mínimo de PWM a ellos
   * antes de conectar la batería. De lo contrario, los ESCs no se iniciarán ni
   * entrarán en el modo de configuración.
   * En valor mínimo es: 1000us y el valor máximo es: 2000us.
   */
  motor_izquierdo.writeMicroseconds(1000); 
  motor_derecho.writeMicroseconds(1000);

  // Esperamos 10 segundos para conectar los motores y que todo funcione correctamente.
  delay(10000);
}

void loop() {
  /* SECCIÓN DEL IMU */
  tiempo_anterior = tiempo; // Guardamos el tiempo anterior antes de proceder con la lectura actual.
  tiempo = millis();        // Actualizamos la lectura actual.
  tiempo_transcurrido = (tiempo - tiempo_anterior) / 1000;  // Tiempo total transcurrido.
  /**
   * ^ "tiempo_transcurrido" es el tiempo que ha pasado desde el anterior ciclo del programa.
   * Este es el valor que vamos a usar en formato de segundos.
   * Como estamos trabajando en milisegundos, deberemos dividir 
   * entre 1000 para obtener un tiempo válido en segundos.
   */ 

  /**
   * En esta sección leemos los valores que el acelerómetro mide.
   * Recurriendo a la hoja de datos del sensor, sabemos que el registro
   * esclavo está en la dirección "0x68".
   * Entonces usamos esta dirección en el método "beginTransmission" y "requestFrom".
   */
  /*Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Accedemos al registro "0x3B" que corresponde a AcX.
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);*/
  /**
   * Accedimos al registro "0x3B". La IMU devolverá valores "en crudo" del registro.
   * La cantidad de registros para leer se especifica en el método "requestFrom".
   * En este caso, solicitamos 6 registros. Cada valor de aceleración se crea a partir
   * de 2 registros de 8 bits, con valores altos y bajos. Por esto mismo, se solicitan
   * 6 valores y sólo se suman en pares. Para esto debemos recorrer los valores a la
   * izquierda los valores altos del registro (<<) y realizar una operación OR (|) para
   * añadir los valores bajos.
   */
  /*
  raw_acc_x_axis = Wire.read()<<8|Wire.read(); // Cada valor necesita dos registros.
  raw_acc_y_axis = Wire.read()<<8|Wire.read();
  raw_acc_z_axis = Wire.read()<<8|Wire.read();*/
  
  /* Get new sensor events with the readings */
  mpu.getEvent(&a, &g, &temp);

  raw_acc_x_axis = a.acceleration.x;
  raw_acc_y_axis = a.acceleration.y;
  raw_acc_z_axis = a.acceleration.z;

  /**
   * En esta sección, necesitamos utilizar ecuaciones de Euler para calcular los ángulos.
   *
   * 1. Para obtener los valores de aceleración en unidades "g", tenemos que dividir los valores
   * en crudo que leímos por 16384.0 porque es el valor que el fabricante nos da.
   *
   * 2. Necesitamos calcular el valor de radianes a grados, dividiendo 180 grados entre la
   * constante numérica "PI" que es: 3.141592654 y guardar este valor en la variable "rad_a_grad".
   * Todo esto para no tener que calcular este valor en cada ciclo del programa, haciendo
   * el programa un poco más rápido.
   *
   * 3. Ahora podemos aplicar la fórmula de Euler. La función "atan" calculará el arco tangente del número dado.
   * La función pow(a, b) elevará el número "a" a una potencia "b". Y finalmente la función sqrt() calculará
   * la raiz cuadrada del número dado.
   */
    
  // Valor para X.
  angulo_aceleracion[0] = atan((raw_acc_y_axis/16384.0)/sqrt(pow((raw_acc_x_axis/16384.0),2) + pow((raw_acc_z_axis/16384.0),2)))*rad_a_grad;
  // Valor para Y.
  angulo_aceleracion[1] = atan(-1*(raw_acc_x_axis/16384.0)/sqrt(pow((raw_acc_y_axis/16384.0),2) + pow((raw_acc_z_axis/16384.0),2)))*rad_a_grad;
 
  /**
   * Leemos los datos del giroscopio de la misma forma que con el acelerómetro.
   * La dirección de registro para el giroscopio comienza por "0x43".
   * Este dato, observable en el mapa de registros del MPU 6050.
   * En este caso, sólo solicitamos 4 valores (X, Y) porque no necesitamos
   * valores del eje Z.
   */
  /*
  Wire.beginTransmission(0x68);
  Wire.write(0x43);                // Dirección inicial para obtener los datos del giroscopio.
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true); // Sólo solicitamos 4 valores (registros).
  */
   
  // raw_gyro_x_axis = Wire.read()<<8|Wire.read(); // Nuevamente desplazamos y sumamos.
  // raw_gyro_y_axis = Wire.read()<<8|Wire.read();
  raw_gyro_x_axis = g.gyro.x;
  raw_gyro_y_axis = g.gyro.y;
 
  /**
   * Para obtener los valores del giroscopio en grados/segundos, tenemos que
   * dividir nuestro valor en crudo por 131 porque es el valor que el
   * fabricante nos da en la hoja de datos de la IMU.
   */
  angulo_gyro[0] = raw_gyro_x_axis/131.0; // Para X.
  angulo_gyro[1] = raw_gyro_y_axis/131.0; // Para Y.

  /**
   * - Para obtener los valores en grados, deberemos multiplicar los valores
   * de grados/segundos por el valor de la variable "tiempo_transcurrido".
   * - Finalmente podemos aplicar un filtro de suavizado donde añadimos
   * la aceleración que afecta los ángulos la cual multiplicamos por 0.98
   */
  angulo_total[0] = 0.98 *(angulo_total[0] + angulo_gyro[0]*tiempo_transcurrido) + 0.02*angulo_aceleracion[0]; // Ángulo del eje X.
  angulo_total[1] = 0.98 *(angulo_total[1] + angulo_gyro[1]*tiempo_transcurrido) + 0.02*angulo_aceleracion[1]; // Ángulo del eje Y.

  /**
   * Ahora tenemos nuestro angulos en grados y valores 
   * desde -10 hasta 100 grados aproximadamente.
   */
  // Serial.println(angulo_total[1]);
  Serial.println("Medida del eje X: " + String(g.gyro.x));
  // Serial.println("Medida del eje Y: " + String(g.gyro.y));

  /**
   * SECCIÓN PID
   * El control PID se ha implementado, para este caso, en el eje X del sensor.
   * Eso significa que el eje X del IMU tiene que ser paralelo al equilibrio.
   * 
   * - Primero calculamos el error entre el punto de ajuste (punto deseado) y
   * el ángulo real medido.
   */
  error = angulo_total[1] - set_point;

  /**
   * - Ahora, el valor proporcional del PID es solamente la constante
   * proporcional multiplicada por el error.
   */
  pid_p = kp * error;

  /**
   * La parte integral solo debe actuar si estamos cerca del punto de ajuste.
   * Pero queremos ajustar el error.
   * Por eso se implementa un condicional para un error dentro del rango [-3, 3].
   * Para integrar, sólo sumamos el valor de la integral anterior con el error
   * multiplicado por la constante integral. Esto integrará (aumentará) el valor 
   * cada ciclo del programa hasta que llegue a 0.
   */
  if (-3 < error < 3) {
    pid_i = pid_i + (ki * error);
  }

  /**
   * La última parte es la derivativa. La derivada actúa sobre la velocidad
   * del error. Como sabemos la velocidad es la cantidad de error que se produce 
   * en una cierta cantidad de tiempo dividido por ese tiempo. Para ello 
   * utilizaremos una variable llamada "error_anterior".
   * Restamos ese valor del error real y dividimos todo por el tiempo transcurrido. 
   * Finalmente multiplicamos el resultado por la constante derivada.
   */
  pid_d = kd*((error - error_anterior)/tiempo_transcurrido);

  /**
   * El valor final para la acción de control PID 
   * consta de la suma de las tres partes anteriores.
   */
  PID = pid_p + pid_i + pid_d;

  /**
   * Sabemos que el valor mínimo de la señal PWM es 1000us y el
   * máximo es 2000us. Eso nos dice que el valor PID puede oscilar
   * más de -1000 y 1000 porque cuando tenemos un valor de 2000us
   * el valor máximo que podríamos substraer es 1000 y cuando tenemos
   * un valor de 1000us para la señal PWM, el valor máximo que
   * podríamos agregar es 1000 para alcanzar el máximo 2000us
   */
  if (PID < -1000) {
    PID = -1000;
  } else if (PID > 1000) {
    PID = 1000;
  }

  /**
   * Finalmente calculamos el ancho de pulso.
   * Sumamos el "throttle" deseado con el valor PID.
   */
  pwm_izquierda = throttle + PID;
  pwm_derecha = throttle - PID;

  /**
   * Una vez más mapeamos los valores de PWM para asegurarnos de no pasar 
   * los valores mínimos y máximos del ESC.
   * Sí, ya hemos mapeado los valores PID. Pero por ejemplo, para el valor 
   * del acelerador de 1300, si sumamos el valor PID máximo tendríamos 
   * 2300us y eso volvería loco al ESC.
   */
  if (pwm_derecha < 1000) {
    pwm_derecha = 1000;
  }
  if (pwm_derecha > 2000) {
    pwm_derecha = 2000;
  }
  if (pwm_izquierda < 1000) {
    pwm_izquierda = 1000;
  }
  if (pwm_izquierda > 2000) {
    pwm_izquierda = 2000;
  }

  /**
   * Como paso final, usamos las funciones de servomotores para
   * enviar la señal de control usando el ancho de pulso como parámetro.
   */
  motor_izquierdo.writeMicroseconds(pwm_izquierda);
  motor_derecho.writeMicroseconds(pwm_derecha);

  // Recuerda guardar el error anterior, jaja.
  error_anterior = error;
}
