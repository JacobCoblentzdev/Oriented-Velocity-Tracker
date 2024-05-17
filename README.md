This is a project I worked on in my Engineering class. I used a complemtary filter to get orientation and from that data I convert euler angles to quaternion and rotate the z acceleration value by using quaternions

Now having a stable z acceleration value, it means I can get the following values

Velocity
The velocity was calculated by subtrackting the previous time elapsed by the current

float currentVelocityZ = rotated_z_acceleration * timeDelta;

Rep count
if (rotated_z_acceleration > 1.05 && !inRep && fastaccel > 0.85) {
      inRep = true;
      repCounter++;
    } else if (rotated_z_acceleration < 0.8) {
      inRep = false;
      fastaccel = 0; // Reset fast acceleration after each repetition
    }
Max acceleration

 if ( rotated_z_acceleration > fastestaccel){
        fastestaccel = rotated_z_acceleration;
      } 
![image](https://github.com/JacobCoblentzdev/Oriented-Velocity-Tracker/assets/170132280/669eb7ef-2598-4278-a930-c02ea360af27)
