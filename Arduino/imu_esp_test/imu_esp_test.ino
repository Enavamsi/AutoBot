#include <Wire.h>
#include <Adafruit_BNO08x.h>

#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_BNO08x bno08x(-1);   // No reset pin
sh2_SensorValue_t sensorValue;

void setReports()
{
  Serial.println("Enabling Rotation Vector...");

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
  {
    Serial.println("Could not enable Rotation Vector");
    while (1);
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk)
{
  float ysqr = qj * qj;

  // Roll (X-axis)
  float t0 = +2.0 * (qr * qi + qj * qk);
  float t1 = +1.0 - 2.0 * (qi * qi + ysqr);
  float roll = atan2(t0, t1);

  // Pitch (Y-axis)
  float t2 = +2.0 * (qr * qj - qk * qi);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  float pitch = asin(t2);

  // Yaw (Z-axis)
  float t3 = +2.0 * (qr * qk + qi * qj);
  float t4 = +1.0 - 2.0 * (ysqr + qk * qk);
  float yaw = atan2(t3, t4);

  Serial.print("Yaw: ");
  Serial.print(yaw * 180.0 / PI, 2);

  Serial.print("  Pitch: ");
  Serial.print(pitch * 180.0 / PI, 2);

  Serial.print("  Roll: ");
  Serial.println(roll * 180.0 / PI, 2);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing BNO085...");

  if (!bno08x.begin_I2C())
  {
    Serial.println("BNO085 not detected!");
    while (1);
  }

  Serial.println("BNO085 Found!");

  setReports();
}

void loop()
{
  if (bno08x.wasReset())
  {
    Serial.println("Sensor reset");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue))
    return;

  if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
  {
    quaternionToEuler(
      sensorValue.un.rotationVector.real,
      sensorValue.un.rotationVector.i,
      sensorValue.un.rotationVector.j,
      sensorValue.un.rotationVector.k
    );
  }

  delay(10);
}