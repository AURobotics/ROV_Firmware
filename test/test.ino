#include <Arduino.h>


// Output thrust values for horizontal 
float outputHorizontalThrusters[4] = {0, 0, 0, 0};

// output thrust values for vertical 
float outputVerticalThrusters[2] = {0, 0};



// Control Code for ROV Motion
// using 6 thruster configurations (4 horizontal, 2 vertical)

float Fx, Fy, Ty; // ROV forces and torque (Surge force, sway force, Yaw torque)
float F1, F2, F3, F4; // Thruster forces (FR,FL,BR,BL)

float Fz,Tp ; // ROV forces and torque (Heave force, Pitch torque)
float F5, F6; // Thruster forces


// incomming Data from PI  
unsigned char incoming [10];

// end byte 
unsigned char term = 255 ;

// turn light 

// setup horizontal thrusters 



// control horizontal motors 

// Pseudoinverse matrix T_inverse for FX , FY ,YAW
double T_inverse_Horizontal[4][3] = {
    {0.25 , -0.25,  1.4706}, 
    {0.25 ,  0.25, -1.4706}, 
    {-0.25,  0.25,  1.4706},
    {-0.25, -0.25, -1.4706}
};

// Pseudoinverse matrix T_inverse for FZ , PITCH
double T_inverse_Vertical[2][2] = {
    {0.5,  2.5},
    {0.5, -2.5}
};
  
// Output thruster forces  

// Apply constraints to the thruster forces
// if the maximum absolute force exceeds the max allowed force, scale down
void applyConstraints(float* thruster_forces, int size, float max_force) {
    float max_abs_force = 0;
    
    // Find the maximum absolute value of the thruster forces
    for (int i = 0; i < size; i++) {
        if (abs(thruster_forces[i]) > max_abs_force) {
            max_abs_force = abs(thruster_forces[i]);
        }
    }

    // If the maximum absolute force exceeds the max allowed force, scale down
    if (max_abs_force > max_force) {
        float scaling_factor = max_force / max_abs_force;
        for (int i = 0; i < size; i++) {
            thruster_forces[i] *= scaling_factor; // Apply the scaling factor
          }   
    }
}

// compute horizontal forces 
void ComputeHorrizontalThrustForces(double* input, double T_inverse[4][3], float* outputThrusters){
  
  // Perform matrix multiplication outputThrusters = T_inverse * input
  
  for (int i = 0; i < 4; i++) {
    outputThrusters[i] = 0;
    for (int j = 0; j < 3; j++) {
      outputThrusters[i] += T_inverse[i][j] * input[j];
    }

  }

  float max_force=255 ;
  applyConstraints(outputThrusters, 4,max_force);
    
}

// compute vertical forces 
void ComputeVerticalThrustForces(double* input, double T_inverse[2][2], float* outputThrusters){
  
  // Perform matrix multiplication outputThrusters = T_inverse * input
  
  for (int i = 0; i < 2; i++) {
    outputThrusters[i] = 0;
    for (int j = 0; j < 2; j++) {
      outputThrusters[i] += T_inverse[i][j] * input[j];
    }

  }

  float max_force=255 ;
  applyConstraints(outputThrusters, 2,max_force);    
}



void setup() {
  Serial.begin(115200);

   
 

}



void loop() {

  float fx = 0  , fy=0 , tau=0 ;
  float fz=0 , tp=0 ;

  double inputH[3] = {fx*(1020.0/254.0), fy*(1020.0/254.0), tau*(173.4/254) };
  double inputV[2] = {fz*(512.0/254.0), tp*(102.0/254.0)};

  // if (Serial.available()) {
  //   /*
  //     l-stick vert: surge (forward)
  //     l-stick horz: sway (right)

  //     r-stick vert: pitch
  //     r-stick horz: yaw

  //     L2, R2: elevation

  //   */  

  //   Serial.readBytesUntil(term, incoming, 9);

  //   inputH [0] = (float) incoming [0] *(1020/254) * ((incoming [5] & 1)? -1 : 1);  // Fx
  //   inputH [1] = (float) incoming [1] *(1020/254) * ((incoming [5] & 2)? -1 : 1);  // Fy
  //   inputH [2] = (float) incoming [3] *(173.4/254) * ((incoming [5] & 8)? -1 : 1);  // Tau

  //   inputV [0] = (float) incoming [4] *(512/254) * ((incoming [5] & 16)? -1 : 1); // Fz
  //   inputV [1] = (float) incoming [2] *(102/254) * ((incoming [5] & 4)? -1 : 1);  // Tpitch

  //   for (int counter = 0 ; counter <=4  ; counter++){
  //     Serial.print(incoming [counter]) ;
  //     Serial.print(" ");
  //   }

  // ComputeHorrizontalThrustForces(inputH, T_inverse_Horizontal, outputHorizontalThrusters);
  // ComputeVerticalThrustForces(inputV, T_inverse_Vertical, outputVerticalThrusters);

  // // Print the results
  // Serial.println("Thruster Forces:");
  // for (int i = 0; i < 4; i++) {
  //   Serial.print(" F");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print(outputHorizontalThrusters[i], 2); // Print with 2 decimal places
  // }

  // for (int i = 0; i < 2; i++) {
  //   Serial.print(" F");
  //   Serial.print(i + 5);
  //   Serial.print(": ");
  //   Serial.print(outputVerticalThrusters[i], 2); // Print with 2 decimal places
  // }

  // Serial.println();

  // }

  ComputeHorrizontalThrustForces(inputH, T_inverse_Horizontal, outputHorizontalThrusters);
  ComputeVerticalThrustForces(inputV, T_inverse_Vertical, outputVerticalThrusters);

  // Print the results
  Serial.println("Thruster Forces:");
  for (int i = 0; i < 4; i++) {
    Serial.print(" F");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(int(outputHorizontalThrusters[i])); // Print with 2 decimal places
  }

  for (int i = 0; i < 2; i++) {
    Serial.print(" F");
    Serial.print(i + 5);
    Serial.print(": ");
    Serial.print( int(outputVerticalThrusters[i])); // Print with 2 decimal places
  }

  Serial.println();
  // Input vector R
  // Compute the thruster forces
  
   delay(1000); // Wait for 1 second before repeating

}