// #define button 10
#include <Arduino_FreeRTOS.h>

// Motor A Left
int pwmA = 9;
int in1A = 7;
int in2A = 8;

// Motor B Right
int pwmB = 10;
int in1B = 11;
int in2B = 12;

// MotorSpeed
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;

// Nilai Error
int errorValue = 0;

// int sensor[6] = { A5, A4, A3, A2, A1, A0 };                 // Pin Sensor
int sensor[6] = { A5, A4, A3, A2, A1, A0 };  // Pin Sensor

int sensorMin[6] = { 1023, 1023, 1023, 1023, 1023, 1023 };  // Nilai Min awal
int sensorMax[6] = { 0, 0, 0, 0, 0, 0 };                    // Nilai Maks awal
int averageSensor[6] = { 0, 0, 0, 0, 0, 0 };                // Nilai rata-rata
int riilSensor[6] = { 0, 0, 0, 0, 0, 0 };                   // Nilai riil sensor
int outputSensor[6] = { 0, 0, 0, 0, 0, 0 };
// int calibrateButton = 0;
void readSensors();

//  Sensor Array   Error Value
//  0 0 0 0 0 1         -5
//  0 0 0 0 1 1         -4
//  0 0 0 0 1 0         -3
//  0 0 0 1 1 0         -2
//  0 0 0 1 0 0          1
//  0 0 1 1 0 0          0
//  0 0 1 0 0 0          1
//  0 1 1 0 0 0          2
//  0 1 0 0 0 0          3
//  1 1 0 0 0 0          4
//  1 0 0 0 0 0          5
//  0 0 0 0 0 0          Error

// Function
void speed(int a, int b) {
  // int spda = (a / 100 * 255);
  // int spdb = (b / 100 * 255);
  analogWrite(pwmA, a);
  analogWrite(pwmB, b);
};

void maju() {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  speed(70, 70);
};

void maju(int a, int b) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  speed(a, b);
};

void mundur() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  speed(70, 70);
};

void stop() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  speed(0, 0);
};

void kanan(int a, int b) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  speed(a, b);
};

void kiri(int a, int b) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  speed(a, b);
};

void setup() {
  Serial.begin(9600);
  // pinMode(button, INPUT_PULLUP);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  // PROSES KALIBRASI
  int sensorValue[6] = { 0, 0, 0, 0, 0, 0 };

  // Tanda Mulai Kalibrasi
  Serial.println("Mulai Kalibrasi!");

  // loop untuk mencari nilai maks dan min dari sensor selama 5 detik
  unsigned long waktuMulaiKalibrasi = millis();
  while (millis() - waktuMulaiKalibrasi < 5000) {
    // Serial.println("Baca Sensor");
    for (int i = 0; i < 6; i++) {
      sensorValue[i] = analogRead(sensor[i]);

      // Cari Nilai Maks
      if (sensorValue[i] > sensorMax[i]) {
        sensorMax[i] = sensorValue[i];
      }

      // Cari Nilai Min
      if (sensorValue[i] < sensorMin[i]) {
        sensorMin[i] = sensorValue[i];
      }
    }
  }

  // Hitung rata-rata sensor
  for (int i = 0; i < 6; i++) {
    averageSensor[i] = (sensorMax[i] + sensorMin[i]) * 0.5;
  }

  // Tampilkan nilai rata-rata sensor
  for (int i = 0; i < 6; i++) {
    Serial.print("Sensor ke - ");
    Serial.print(i + 1);
    Serial.print("\t");
    Serial.println(averageSensor[i]);
  }

  // Bersihkan nilai maks dan min sensor untuk kalibrasi berikutnya
  // for (int i = 0; i < 6; i++) {
  //   sensorMax[i] = 0;
  //   sensorMin[i] = 1023;
  // }
}

void loop() {
  readSensors();
  errorValue = detectLine();
  if (errorValue == 0) {
    maju(40, 40);
    // Serial.println("Majuuuuuu");
  } else if (errorValue == 1) {
    maju(40, 40 - 15 * 1);
    // Serial.println("kanan Dikit");
  } else if (errorValue == 2 || errorValue == 3) {
    maju(40, 40 - 15 * 2);
    // Serial.println("kanan");
  } else if (errorValue == -1) {
    maju(40 - 15 * 1, 40);
    // Serial.println("kiri Dikit");
  } else if (errorValue == -2 || errorValue == -3) {
    maju(40 - 15 * 2, 40);
    // Serial.println("kiri");
  }
  if (errorValue == 4) {
    // maju(5, 5);
    // while (true) {
    // readSensors();
    maju(150, 0);
    // if (errorValue != 4 or errorValue != 5) {
    //   break;
    // }
    // }
    // Serial.println("kanan bgt");
  } else if (errorValue == 5) {
    maju(150, 0);
  }

  // Serial.println("kanan bgt");
  if (errorValue == -4) {
    // maju(5, 5);
    // while (true) {
    //   readSensors();
    maju(0, 150);
    // if (errorValue != 4 or errorValue != 5) {
    //   break;
    // }
  }
  else if (errorValue == -5) {
    maju(0, 200);
  }
  // Serial.println("kanan bgt");
}
// else if (errorValue == -5) {
//   maju(0, 200);
//   // Serial.println("kiri banyakk");
// }
// else {
//   maju(20, 30);
//   // stop();
//   // Serial.println("Berhenti");
// }


//  Sensor Array   Error Value
//  0 0 0 0 0 1         -5
//  0 0 0 0 1 1         -4
//  0 0 0 0 1 0         -3
//  0 0 0 1 1 0         -2
//  0 0 0 1 0 0         -1
//  0 0 1 1 0 0          0
//  0 1 1 0 0 0          1
//  0 0 1 0 0 0          2
//  0 1 0 0 0 0          3
//  1 1 0 0 0 0          4
//  1 0 0 0 0 0          5
//  0 0 0 0 0 0          Error

void readSensors() {
  delay(12);
  for (int i = 0; i < 6; i++) {
    riilSensor[i] = analogRead(sensor[i]);
  }

  for (int i = 0; i < 6; i++) {
    if (riilSensor[i] >= (averageSensor[i])) {
      outputSensor[i] = 0;
    } else if (riilSensor[i] < (averageSensor[i])) {
      outputSensor[i] = 1;
    }
  }

  // for (int i = 0; i < 6; i++) {
  //   if (i < 5) {
  //     // Serial.print(outputSensor[i]);
  //     // Serial.print("\t");
  //   } else {
  //     // Serial.println(outputSensor[i]);
  //   }
  // }

  // for (int i = 0; i < 6; i++) {
  //   if (i < 5) {
  //     Serial.print(riilSensor[i]);
  //     Serial.print("\t");
  //   } else {
  //     Serial.println(riilSensor[i]);
  //   }
  // }

  // Tentukan Nilai Error
}

int detectLine() {
  if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 0) && (outputSensor[3] == 0) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 0;  //  1 1 0 0 1 1  Error Value = 0
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 0) && (outputSensor[5] == 1)) {
    return -3;  //  1 1 1 1 0 1   Error Value = -3
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 0) && (outputSensor[4] == 0) && (outputSensor[5] == 1)) {
    return -2;  //  1 1 1 0 0 1   Error Value = -2
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 0) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return -1;  //  1 1 1 0 1 1   Error Value = -1
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 0) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 1;  //  1 1 0 1 1 1          1
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 0) && (outputSensor[2] == 0) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 2;  //  1 0 0 1 1 1          2
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 0) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 3;  //  1 0 1 1 1 1          3
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 0) && (outputSensor[5] == 0)) {
    return -4;  //  1 1 1 1 0 0   Error Value = -4
  } else if ((outputSensor[0] == 1) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 0)) {
    return -5;  //  1 1 1 1 1 0   Error Value = -5
  } else if ((outputSensor[0] == 0) && (outputSensor[1] == 0) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 4;  //  0 0 1 1 1 1          4
  } else if ((outputSensor[0] == 0) && (outputSensor[1] == 1) && (outputSensor[2] == 1) && (outputSensor[3] == 1) && (outputSensor[4] == 1) && (outputSensor[5] == 1)) {
    return 5;  //  0 1 1 1 1 1          5
  }
  // else {
  //   return 10;  // Stop
  // }
}
