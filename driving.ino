/********************************************************/
/***          Constants and global variables          ***/
/********************************************************/

/*ARDUINO LEONARDO PIN CONFIGURATION*/

#define LEFT_MOTOR                  5
#define LEFT_ENCODER                2
#define RIGHT_MOTOR                 6
#define RIGHT_ENCODER               3

/************************************/

#define RXLED                       17
#define TXLED                       30

#define SAMPLING_INTERVAL           100
#define NUM_COMMANDS                4
int sample_lens[NUM_COMMANDS] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1

#define NUM_COMMANDS                4
#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;

volatile int left_count = 0;
volatile int right_count = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*---------------------------*/

float theta_left = 0.1105;
float theta_right = 0.1139;
float beta_left = -14.52;
float beta_right = -12.28;
float v_star = 26.9;

// PWM inputs to jolt the car straight
int left_jolt = 220;
int right_jolt = 240;

// Control gains
float f_left = 0.5;
float f_right = 0.5;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*---------------------------*/
float driveStraight_left(float v_star, float delta) {
   return (1/theta_left)*(v_star - (f_left*delta) + beta_left);
}

float driveStraight_right(float v_star, float delta) {
    return (1/theta_right)*(v_star + (f_right*delta) + beta_right);
}

/*---------------------------*/
/*      CODE BLOCK CON3      */
/*---------------------------*/

float delta_ss = 0;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*---------------------------*/
#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 450 // in cm - 6 feet diameter
//CW = 15 & TR = 450, PRETTY GOOD! use w run times: 5500, 3800, 1900, 3800
// CW = 15 & TR = 200, wasn't really turning, veering off to left for all commands
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter

/*---------------------------*/
/*      PROGRAMMED PATH      */
/*---------------------------*/
int run_times[NUM_COMMANDS] = {5500, 3800, 1900, 3800}; // length of commands roughly in ms
int drive_modes[NUM_COMMANDS] = {DRIVE_FAR, DRIVE_LEFT, DRIVE_CLOSE, DRIVE_RIGHT};

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*---------------------------*/
float delta_reference(int i) {
  // YOUR CODE HERE
  // Remember to divide the v* value you use in this function by 5 because of sampling interval differences!
  if (drive_mode == DRIVE_RIGHT) { // Return a NEGATIVE expression
    return -(v_star * CAR_WIDTH * i)/TURN_RADIUS;
  }
  else if (drive_mode == DRIVE_LEFT) { // Return a POSITIVE expression
    return (v_star * CAR_WIDTH * i)/TURN_RADIUS;
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}
/*---------------------------*/
/*      CODE BLOCK CON6      */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             INFINITY

float straight_correction(int i) {
  //no straight correction needed
  return 0;
}

/*** Constants and global variables from classify.ino  ***/

#define SIZE                        5504
#define ADC_TIMER_MS                0.35
#define AVG_SHIFT                   5
#define AVG_SIZE                    (int) pow(2, AVG_SHIFT)
#define SIZE_AFTER_FILTER           (int) SIZE / AVG_SIZE

#define MIC_INPUT                   A2

/***************************************** VOICE DATA PCA SECTION *****************************************/

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

#define SNIPPET_SIZE                80
#define PRELENGTH                   5
#define THRESHOLD                   0.5
#define BASIS_DIM                   3

#define EUCLIDEAN_THRESHOLD         .035
#define LOUDNESS_THRESHOLD          200

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[80] = {0.04646671826193498, 0.02835912973242749, 0.03368384614887787, 0.015143853919810102, 0.0316368761869146, 0.05996249653438628, 0.05478484878029775, 0.033180627204144594, 0.018980474874241496, 0.0013238884985075179, -0.018910984247967957, -0.03026122118417523, -0.03996383306141263, -0.06943030787875001, -0.11131908419843943, -0.1534714911890173, -0.18349244674124282, -0.20242667931797767, -0.20633445248634633, -0.20029446798610492, -0.18334065010703196, -0.1711278207511175, -0.15211793172533886, -0.12219582112924555, -0.08635893547555723, -0.05867910104551384, -0.012042629032603516, 0.0606118608302532, 0.13477875548450316, 0.1971643326853053, 0.26951892562224883, 0.3131143630681042, 0.3148391326634056, 0.28956392742867254, 0.2434079123252037, 0.21114590272377304, 0.1766939065124337, 0.1543918996461288, 0.12636930249247424, 0.10975572780638418, 0.09476067980185515, 0.07519523455934043, 0.06075320227837227, 0.04572824664169026, 0.027344844036176413, 0.012100752532940691, -0.0014886534897240468, -0.007401605066749838, -0.010069329610805502, -0.014882995916044492, -0.017327732984076298, -0.021278472040280404, -0.03411011139293071, -0.031750437466492844, -0.038314404598795634, -0.0419240639314486, -0.038396051095959394, -0.04025792604954059, -0.038512317267154705, -0.04938547513642159, -0.04961303265928655, -0.052649790586094425, -0.049460132894771365, -0.050728815003872024, -0.05208077550953265, -0.05056208616774929, -0.04880465596081293, -0.04661835119371407, -0.04788035175515953, -0.04532643320443366, -0.04286204211222158, -0.038686296435583745, -0.037478978653277, -0.03766332405385047, -0.03688403873301872, -0.032969315685747326, -0.034046242755985556, -0.034301782578787986, -0.03332662636153161, -0.031951163371110494};
float pca_vec2[80] = {-0.01772380268361149, 0.02766243833722304, 0.028370224398059807, 0.04325309070396255, 0.011464460825158451, -0.17327273231929036, -0.1617175460043342, -0.1469375185491041, -0.07878037566345382, -0.07364410645036312, -0.062234756146303714, -0.06763939354980546, -0.07173297863662305, -0.08861363978309791, -0.10703010009762237, -0.11306832172291725, -0.11856813535063, -0.13002283780642615, -0.16702216740939915, -0.1835672134931573, -0.20077269387516006, -0.2212968273296897, -0.22905635793468973, -0.2152018661700998, -0.18116556278339757, -0.16292030089550952, -0.13709315900226157, -0.13013307228346796, -0.1264673268497611, -0.11631056061583525, -0.12364337884365163, -0.12265396514374663, -0.11965432475895986, -0.07485344110378368, -0.047362535662559754, 0.0034707850446452824, 0.004237739950938028, 0.0026814105802099486, 0.0376759645241746, 0.0359107754463172, 0.03586133482471521, 0.016970536630545825, 0.03284623832157009, 0.04593150040873679, 0.07369839410541418, 0.08940861845691656, 0.08685213446624691, 0.08000667748710605, 0.08246844777142069, 0.08810992730682035, 0.09682604056521657, 0.11355491353385397, 0.1438491482470421, 0.12958719979903285, 0.13528745721933455, 0.1457834110919327, 0.14270911614319415, 0.1367282920545564, 0.12308635016459449, 0.12742386702431382, 0.13837036975396766, 0.12927877983074443, 0.12349373919506967, 0.11614560922644324, 0.11376520937443872, 0.11240813111623346, 0.11148599323062207, 0.11141608584222769, 0.0972044560191539, 0.09129686016509755, 0.08614501569888217, 0.09105765469244421, 0.08500472868617857, 0.07602863221488088, 0.07024317136000136, 0.06297717438780195, 0.055777228130528406, 0.06796250285967792, 0.060274930607161804, 0.04810823109390417};
float pca_vec3[80] = {0.07300727924816483, 0.09117550593472601, 0.03323625511856276, -0.03426907723792433, 0.04668797466527519, -0.40486874719036614, -0.09858454645391435, 0.047720772403369596, 0.3608797150014341, 0.3624008780712406, 0.2866568045576972, 0.17375335650736218, 0.11161182981397613, 0.0021513810953044735, -0.058580255840550946, -0.0681869896276942, -0.08084617086692852, -0.07526783531811124, -0.026634428113216712, -0.012229394838313947, 0.01737716080849498, 0.029237112684063857, 0.005335556593929802, -0.034624305871932305, -0.03987436829668164, -0.10270965032299881, -0.10969891074782367, -0.09725618442686582, -0.027872599424062446, 0.00017887298954002897, 0.061386887931005996, 0.11047941912542748, 0.11341372293007573, 0.08179696907600305, 0.004769853905277279, -0.0631869997049271, -0.06531125658419523, -0.08195867444956219, -0.12696067383340648, -0.12319347051137072, -0.10164820777898112, -0.09456803380241428, -0.11659077242668538, -0.1117759140434966, -0.1059779400376724, -0.11850130620346665, -0.09954297917171154, -0.0880873508337308, -0.08704126700813174, -0.09584895378279425, -0.0987485732891817, -0.10908064008718571, -0.08245504952446976, -0.09691232721682994, -0.09175221638314295, -0.07855643607019094, -0.07600797927378702, -0.04497426477380353, -0.031974779632956855, -0.012658520008318307, 0.02370502152272214, 0.03905401383098828, 0.04178626605396707, 0.05953330049251051, 0.07110610851784249, 0.08399562994417642, 0.07798021271725913, 0.08305753870202656, 0.09261824532084739, 0.09189017772645079, 0.08070287115243605, 0.0741538304233074, 0.0740465572809808, 0.07503541915821592, 0.07101977311974567, 0.07793195169106928, 0.08241249499422638, 0.08361398424469584, 0.09135197769353577, 0.08656536796186254};
float projected_mean_vec[3] = {-0.0054216448500394605, -0.05558031368041079, 0.00682116232281162};
float centroid1[3] = {-0.03949634963591237, 0.033180710580592125, 0.013925866315313801};
float centroid2[3] = {-0.04078577934806287, -0.037327648044751145, 0.008116647650604993};
float centroid3[3] = {0.07762826119952748, 0.0007941995929702189, 0.008428747141431166};
float centroid4[3] = {0.003078486629959391, 0.0038891759305790113, -0.035346662884525995};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

//data array and index pointer
int16_t out[SIZE_AFTER_FILTER] = {0};
volatile int re_pointer = 0;

int16_t re0[AVG_SIZE] = {0};
int16_t re1[AVG_SIZE] = {0};
int write_arr = 0;

int16_t * get_re(int loc){
  switch(loc){
    case 0:
      return re0;
    case 1:
      return re1;
    default:
      return re0;
  }
}

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
 
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);
  delay(500);

  re_pointer = 0;
     
  cli();
  //set timer1 interrupt at 1Hz * SAMPLING INTERVAL / 1000
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15.624 * ADC_TIMER_MS;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();

  write_pwm(0, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(TXLED, LOW);
  digitalWrite(RXLED, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(TXLED, HIGH);
  delay(1000);
  digitalWrite(RXLED, HIGH);
  delay(1000);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), flag_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), flag_left, CHANGE);
 
  start_listen_mode();
}

void loop(void) {
  if (re_pointer%AVG_SIZE == 0){
    write_arr = !write_arr;
    envelope_small(get_re(!write_arr), out, re_pointer>>AVG_SHIFT);
  }
  if (re_pointer == (int) (SIZE / 3)) {
    digitalWrite(TXLED, LOW);
  }
  if (re_pointer == (int) (SIZE * 2 / 3)) {
    digitalWrite(RXLED, LOW);
  }
  if (loop_mode == MODE_LISTEN && re_pointer == SIZE) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(TXLED, HIGH);
    digitalWrite(RXLED, HIGH);
    write_pwm(0, 0);
   
    // if enveloped data is above some preset value
    if(envelope(out, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: the principal components are unit norm
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      // multiply
      for (int i = 0; i < SNIPPET_SIZE; i++) {
        proj1 += result[i]*pca_vec1[i];
        proj2 += result[i]*pca_vec2[i];
        proj3 += result[i]*pca_vec3[i];
      }
      //subtract the average
      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      proj3 -= projected_mean_vec[2];
      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
      
      // Project 'result' onto the principal components
      // Classification
      // jth centroid: centroids[j]
      float best_dist = 999999;
      int best_index = -1;
      for (int j = 0; j <= 3; j += 1) {
        float dist = l2_norm3(proj1, proj2, proj3, centroids[j]);
        if (dist < best_dist) {
          best_dist = dist;
          best_index = j;
        }
      }

      /***** DEBUGGING CODE *****/
      // drive_mode = FILL IN; 
      // start_drive_mode(); 
      // Serial.println(drive_mode);
      
   } else { //THIS 'ELSE' IS IN REFERENCE TO THE 'IF' ON LINE 275
     Serial.println("Below LOUDNESS_THRESHOLD.");
   }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
   
  } else if (loop_mode == MODE_DRIVE) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    } else {
      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_count;
      int right_position = right_count;

       /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta + delta_reference(step_num) + straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(v_star, delta);
      int right_cur_pwm = driveStraight_right(v_star, delta);
      write_pwm(left_cur_pwm, right_cur_pwm);
      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
     
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;
    digitalWrite(RXLED, (!(((drive_mode == DRIVE_FAR) || (drive_mode == DRIVE_CLOSE) || (drive_mode == DRIVE_RIGHT)) && ((step_num / 5) % 2))));
    digitalWrite(TXLED, (!(((drive_mode == DRIVE_FAR) || (drive_mode == DRIVE_CLOSE) || (drive_mode == DRIVE_LEFT)) && ((step_num / 5) % 2))));

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }
    delay(SAMPLING_INTERVAL);
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void envelope_small(int16_t* data, int16_t* data_out, int index){
  int32_t avg = 0;
  for (int i = 0; i < AVG_SIZE; i++) {
      avg += data[i];
  }
 
  avg = avg >> AVG_SHIFT;
  data_out[index] = abs(data[0] - avg);  
 
  for (int i = 1; i < AVG_SIZE; i++) {
      data_out[index] += abs(data[i] - avg);
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    Serial.println(maximum);
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres && block < SIZE_AFTER_FILTER);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void flag_left() {
  if(digitalRead(LEFT_ENCODER)) {
    left_count++;
  }
}

void flag_right() {
  if(digitalRead(RIGHT_ENCODER)) {
    right_count++;
  }
}

void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  step_num = 0;
  left_count = 0;
  right_count = 0;
}

void start_listen_mode(void) {
  write_pwm(0, 0);
  delay(3000);
  loop_mode = MODE_LISTEN;
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

ISR(TIMER1_COMPA_vect){//timer1 interrupt 8Khz toggles pin 13 (LED)
  if (re_pointer < SIZE && loop_mode != MODE_DRIVE) {
    digitalWrite(RXLED, LOW);
    get_re(write_arr)[re_pointer%AVG_SIZE] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}
