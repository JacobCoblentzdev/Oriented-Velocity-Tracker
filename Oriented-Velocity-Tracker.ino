#include "Arduino_BMI270_BMM150.h"
#include "SimpleKalmanFilter.h"
#include "MahonyAHRS.h"
Mahony filter;
bool programStarted = false; // Flag to track if the program has started
unsigned long previousMillis = 0; // Variable to store the previous time
const unsigned long interval = 100; // Interval for calculating velocity (milliseconds)
float velocityZ = 0;
float positionZ = 0;
float maxVelocityZ = 0; // Variable to store the maximum velocity
float pretime = 0;
float previousaccel = 0;
float fastaccel = 0;
int repCounter = 0;
bool inRep = false;
float pz = 0;
const float alpha = .5;
float filteredPitch = 0;
float previousGyroPitchRate;
float gx= 0, gy= 0, gz= 0;
  float magx, magy, magz;
  float x=0,y=0,z=0;
  float X, Y, Z;
 float fastestaccel;
float filterConstant = .98;
 // Weight for the accelerometer
float filgz = 0;
 // Previous velocity along y-axis
SimpleKalmanFilter simpleKalmanFilterX(1, 1, 0.05); // Kalman filter instance
SimpleKalmanFilter simpleKalmanFilterY(1, 1, 0.05); // Kalman filter instance
SimpleKalmanFilter simpleKalmanFilterZ(1, 1, 0.05); // Kalman filter instance
SimpleKalmanFilter simpleKalmanFiltergX(4, 1, .05); // Kalman filter instance
SimpleKalmanFilter simpleKalmanFiltergY(4, 1, .05); // Kalman filter instance
SimpleKalmanFilter simpleKalmanFiltergZ(4, 1, .05);
SimpleKalmanFilter postionfill(5, 2, .05);


void setup() {
 
  Serial.begin(9600);
   filter.begin(100);
   //Serial.print("started");
   

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
 

}

void euler_to_quaternion(double pitch, double* quaternion) {
   
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    quaternion[0] = cp;  // Quaternion scalar (real part)
    quaternion[1] = 0;   // Quaternion i-component (imaginary part)
    quaternion[2] = sp;  // Quaternion j-component (imaginary part)
    quaternion[3] = 0;   // Quaternion k-component (imaginary part)
}

double rotate_z_by_quaternion(const double* vector, const double* quaternion) {
  // Simplified rotation for only the z component
  double q_w = quaternion[0];
  double q_x = quaternion[1];
  double q_y = quaternion[2];
  double q_z = quaternion[3];

  double v_x = vector[0];
  double v_y = vector[1];
  double v_z = vector[2];

  // Calculate the rotated z component
  return (2 * (q_x * q_z - q_w * q_y)) * v_x +
         (2 * (q_y * q_z + q_w * q_x)) * v_y +
         (q_w * q_w - q_x * q_x - q_y * q_y + q_z * q_z) * v_z;
}
void loop() {

 
 
   
  if (Serial.available()) { // Check if there is data available to read and the program hasn't started yet
    char key = Serial.read();
     
    if (key == 'a') {
      startProgram();
      programStarted = true; // Set the flag to indicate that the program has started
    }
    if (key == 'b') {
      programStarted = false;
      stopProgram(); // Call your function to stop the program
    }
  }
 
 
 
  if (programStarted == true) {
   
    if(IMU.magneticFieldAvailable()){
    IMU.readMagneticField(magx, magy, magz);
    if(magx == 0){
      delay(50);
    }
    if(magy == 0){
      delay(50);
    }
    if(magz == 0){
      delay(50);
    }
 
   
  }
    if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
     gy = simpleKalmanFiltergY.updateEstimate(gy);
   
   
     gx = simpleKalmanFiltergX.updateEstimate(gx);
   
     
     gz = simpleKalmanFiltergZ.updateEstimate(gz);
     float gyroScale = 3.14159f / 180.0f;
    gx = gx * gyroScale;
    gy = gy * gyroScale;
    gz = gz * gyroScale;
     if(gx <.02 && gx >-.02){
      gx = 0;
     }
     if(gy <.02 && gy >-.02){
      gy = 0;
     }
     
     if(gz <.02 && gz >-.02){
      gz = 0;
     }
     
     
     //filgx = alpha * filgx + (1 - alphag) * gx;
 // filgy = alpha * filgy + (1 - alphag) * gy;
  //filgz = alpha * filgz + (1 - alphag) * gz;
  }
  if(IMU.accelerationAvailable()){
    IMU.readAcceleration(X, Y, Z);
    //float x = simpleKalmanFilterY.updateEstimate(Y);
    //float y = sim simpleKalmanFilterZ.updateEstimate(Z);
      x = simpleKalmanFilterX.updateEstimate(X);
     
      y = simpleKalmanFilterY.updateEstimate(Y);
     z = simpleKalmanFilterZ.updateEstimate(Z);
   // filx = alpha * filx + (1 - alpha) * x;
  //fily = alpha * fily + (1 - alpha) * y;
  //filz = alpha * filz + (1 - alpha) * z;
  }

    double quaternion[4];
    double acceleration[3] = {x, y, z};
    double rotated_z_acceleration;
    double gravity_vector_z = 0;
 
   
    //float yaw = filter.getYaw();


    float gyroS = 3.14159f / 180.0f;
   


    float time = millis();
    float timeDelta = time - pretime;
    float gyroPitchRate = gy; // Assuming 'gy' is the pitch rate from the gyroscope
    float accelPitch = atan2(x, sqrt(y * y + z * z)) * (180.0 / PI); // Calculate pitch from accelerometer

    // Apply the complementary filter to combine the data
    filteredPitch = filterConstant * (filteredPitch + gyroPitchRate * timeDelta) + (1 - filterConstant) * accelPitch;

 
    filteredPitch = filteredPitch * gyroS;
   
   

    if(filteredPitch <.01&&filteredPitch>-.01){filteredPitch = 0;}
     
   
    euler_to_quaternion(filteredPitch, quaternion);
   
   
 
  // Rotate the gravity vector to match the sensor's frame of reference
       rotated_z_acceleration = rotate_z_by_quaternion(acceleration, quaternion);
  rotated_z_acceleration = rotated_z_acceleration +gravity_vector_z;
  if (rotated_z_acceleration > fastaccel){
    fastaccel = rotated_z_acceleration;
  }
  // Subtract the rotated gravity vector from the acceleration to get gravity-compensated acceleration

   
   
    Serial.print(time);
  Serial.print(",");
  Serial.println(rotated_z_acceleration);
//Serial.print("    ");
 

     // Gravity in Earth reference frame
   
   
    // Calculate velocity along y-axis
    // Convert milliseconds to seconds
    // Update this line
    time = millis();
       timeDelta = time - pretime;
      timeDelta = timeDelta /1000;
     float currentVelocityZ = rotated_z_acceleration * timeDelta; // Update this line
     
      // Update maxVelocity if current velocity is greater
     
      if (currentVelocityZ > maxVelocityZ) {
        maxVelocityZ = currentVelocityZ;
      }
     
     // Simple integration
   



if (rotated_z_acceleration > 1.05 && !inRep && fastaccel > 0.85) {
      inRep = true;
      repCounter++;
    } else if (rotated_z_acceleration < 0.8) {
      inRep = false;
      fastaccel = 0; // Reset fast acceleration after each repetition
    }
   
    // Update fast acceleration
    if (rotated_z_acceleration > fastaccel) {
      fastaccel = rotated_z_acceleration;
     
    }
      if ( rotated_z_acceleration > fastestaccel){
        fastestaccel = rotated_z_acceleration;
      }
      // Update the previous time
   
 
   

    pretime = time;
 
}
}
void startProgram() {
  // Any initialization for your program logic goes here
}

void stopProgram() {
  // Report the maximum velocity over time
  Serial.print("Max Velocity: ");
  Serial.print(maxVelocityZ);

  Serial.print("Rep Count: ");
  Serial.println(repCounter);
  Serial.print("fast accel");
  Serial.println(fastestaccel);
 
  // Any cleanup for stopping the program goes here
}


