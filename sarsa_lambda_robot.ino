#include <Wire.h>
#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 accelgyro;
MPU6050 initialize;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define Gry_offset 0  //The offset of the gyro
#define Gyr_Gain 131
#define Angle_offset 0  // The offset of the accelerator
#define RMotor_offset 0  // The offset of the Motor
#define LMotor_offset 0  // The offset of the Motor
#define pi 3.14159

#define bound = pow(2, 16)
#define basis = 3
#define alpha = 0.001
#define lambda_ = 0.9
#define gamma = 1.0
#define coef_length = pow((basis+1), 5)
#define num = 2 * coef_length


float Angle_Delta, Angle_Recursive, Angle_Confidence;

float kp, ki, kd;
float Angle_Raw, Angle_Filtered, omega, dt;
float Turn_Speed = 0, Run_Speed = 0;
float LOutput, ROutput, Input, Output;

unsigned long preTime, lastTime;
float errSum, dErr, error, lastErr;
int timeChange;

float Sum_Right, Sum_Right_Temp, Sum_Left, Sum_Left_Temp, Distance, Distance_Right, Distance_Left, Speed;

double coefficients[coef_length][5];
int i = 0;
int j = 0;
int current = 0;

double theta[num];

double q_state[] = {0, 0, 0, 0};
int q_action = 0;
double q_x[] = {0, 0, 0, 0, 0}
double q_val = 0

int TN1 = 23;
int TN2 = 22;
int ENA = 5;
int TN3 = 24;
int TN4 = 25;
int ENB = 4;

struct Axis  // Datas from remote control
{
  uint16_t axis_1;
  uint16_t axis_2;
  uint16_t axis_3;
  uint16_t axis_4;
  uint16_t axis_5;
  uint16_t axis_6;
  uint16_t axis_7;
  uint16_t axis_8;
};
Axis axis_x;

struct Gesture  // Datas send back to remote control
{
  float angle;
  float omega;
  int speed;
  uint16_t P;
  uint16_t I;
  uint16_t D;
  uint16_t null_1;
  uint16_t null_2;
};
Gesture data;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  TCCR3A = _BV(COM3A1) | _BV(WGM31) | _BV(WGM30); // TIMER_3 @1K Hz, fast pwm
  TCCR3B = _BV(CS31);
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // TIMER_0 @1K Hz, fast pwm
  TCCR0B = _BV(CS01) | _BV(CS00);

  /* If the robot was turned on with the angle over 45(-45) degrees,the wheels
   will not spin until the robot is in right position. */
  accelgyro.initialize();
  for (int i = 0; i < 200; i++) // Looping 200 times to get the real gesture when starting
  {
    Filter();
  }
  if (abs(Angle_Filtered) < 45)  // Start to work after cleaning data
  {
    omega = Angle_Raw = Angle_Filtered = 0;
    Output = error = errSum = dErr = 0;
    Filter();
    myPID();
  }

  pinMode(TN1, OUTPUT);
  pinMode(TN2, OUTPUT);
  pinMode(TN3, OUTPUT);
  pinMode(TN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(18, INPUT);
  pinMode(2, INPUT);

  attachInterrupt(4, State_A, FALLING);
  attachInterrupt(1, State_B, FALLING);

  // 24L01 initialization
  Mirf.cePin = 53;
  Mirf.csnPin = 48;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"serv1");
  Mirf.payload = 16;
  Mirf.config();
}

void loop()
{
  while (1)
  {
    Recive();
    Filter();
    if ((micros() - lastTime) > 10000)
    {
      // If angle > 45 or < -45 then stop the robot
      if (abs(Angle_Filtered) < 45)
      {
        myPID();
        PWMControl();
      }
      else
      {
        digitalWrite(TN1, HIGH);
        digitalWrite(TN2, HIGH);
        digitalWrite(TN3, HIGH);
        digitalWrite(TN4, HIGH);
      }
      lastTime = micros();
    }
  }
}

void Recive()
{
  if (!Mirf.isSending() && Mirf.dataReady())
  {
    // Read datas from the romote controller
    Mirf.getData((byte *) &axis_x);
    /*Serial.print("axis_1=");
    Serial.print(axis_x.axis_1);
    Serial.print("  axis_2=");
    Serial.print(axis_x.axis_2);
    Serial.print("  axis_3=");
    Serial.print(axis_x.axis_3);
    Serial.print("  axis_4=");
    Serial.print(axis_x.axis_4);
    Serial.print("  axis_5=");
    Serial.print(axis_x.axis_5);
    Serial.print("  axis_6=");
    Serial.print(axis_x.axis_6);
    Serial.print("  axis_7=");
    Serial.print(axis_x.axis_7);
    Serial.print("  axis_8=");
    Serial.println(axis_x.axis_8);*/

    Mirf.setTADDR((byte *)"clie1");
    Mirf.send((byte *) &data);  // Send datas back to the controller

    if (axis_x.axis_1 >= 520) // Y axis datas from joystick_1
    {
      Turn_Speed = map(axis_x.axis_1, 520, 1023, 0, 120);
    }
    else if (axis_x.axis_1 <= 480)
    {
      Turn_Speed = map(axis_x.axis_1, 480 , 0, 0, -120);
    }
    else
    {
      Turn_Speed = 0;
    }

    if (axis_x.axis_4 >= 520) // X axis datas from joystick_2
    {
      Run_Speed = map(axis_x.axis_4, 520, 1023, 0, 100);
    }
    else if (axis_x.axis_4 <= 480)
    {
      Run_Speed = map(axis_x.axis_4, 480, 0, 0, -100);
    }
    else
    {
      Run_Speed = 0;
    }

  }
  else
  {
    axis_x.axis_1 = axis_x.axis_4 = 500;
  }
  data.omega = omega;
  data.angle = Angle_Filtered;
  data.speed = Sum_Right;
  data.P = analogRead(A0);
  data.I = analogRead(A1);
  data.D = analogRead(A2);
}

void Filter()
{
  // Raw datas
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Angle_Raw = (atan2(ay, az) * 180 / pi + Angle_offset);
  omega = gx / Gyr_Gain + Gry_offset;
  // Filter datas to get the real gesture
  unsigned long now = micros();
  timeChange = now - preTime;
  preTime = now;
  dt = timeChange * 0.000001;
  Angle_Delta = (Angle_Raw - Angle_Filtered) * 0.64;
  Angle_Recursive = Angle_Delta * dt + Angle_Recursive;
  Angle_Confidence = Angle_Recursive + (Angle_Raw - Angle_Filtered) * 1.6 + omega;
  Angle_Filtered = Angle_Confidence * dt + Angle_Filtered;
  Serial.println(Angle_Filtered);
}

void getC()
{
  for (i=0; i < coef_length; i++) {
    for(j=0; j < 5; j++){
      coefficients[i][j] = current % pow((basis + 1), j);
      current++;
    }
  }
}

double randomDouble(double minf, double maxf)
{
  return minf + random(1UL << 31) * (maxf - minf) / (1UL << 31);  // use 1ULL<<63 for max double values)
}

void initializeTheta()
{
  for (i=0; i < num; i++) {
    theta[i] = randomDouble(0.0, 1.0);
  }
}

double dot_product(double v[], double u[], int n)
{
    double result = 0.0;
    for (int i = 0; i < n; i++)
        result += v[i]*u[i];
    return result;
}

double qValue()
{
  q_val = 0;
  for (i=0; i < num/2; i++) {
    q_val += theta[2*i] * cos(pi * dot_product(coefficients[i], q_x, 5)) + theta[2*i+1] * sin(pi * dot_product(coefficients[i], q_x, 5))
  }
  return q_val
}

void myPID()
{
  kp = 26.000;
  ki = 0;
  kd = 2.6;
  // Calculating the output values using the gesture values and the PID values.
  error = Angle_Filtered;
  errSum += error;
  dErr = error - lastErr;
  Output = kp * error + ki * errSum + kd * omega;
  lastErr = error;
  noInterrupts();
  Sum_Right = (Sum_Right + Sum_Right_Temp) / 2;
  Sum_Left = (Sum_Left + Sum_Left_Temp) / 2;
  Speed = (Sum_Right + Sum_Left) / 2;
  Distance += Speed + Run_Speed;
  Distance = constrain(Distance, -300, 300);
  Output += Speed * 70 + Distance * 0.6;
  Sum_Right_Temp = Sum_Right;
  Sum_Left_Temp = Sum_Right;
  Sum_Right = 0;
  Sum_Left = 0;
  ROutput = Output + Turn_Speed;
  LOutput = Output - Turn_Speed;
  interrupts();
}

void PWMControl()
{
  if (LOutput > 0)
  {
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, LOW);
  }
  else if (LOutput < 0)
  {
    digitalWrite(TN1, LOW);
    digitalWrite(TN2, HIGH);
  }
  else
  {
    OCR3A = 0;
  }
  if (ROutput > 0)
  {
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, LOW);
  }
  else if (ROutput < 0)
  {
    digitalWrite(TN3, LOW);
    digitalWrite(TN4, HIGH);
  }
  else
  {
    OCR0B = 0;
  }
  OCR3A = min(1023, (abs(LOutput * 4) + LMotor_offset * 4)); // Timer/Counter3 is a general purpose 16-bit Timer/Counter module
  OCR0B = min(255, (abs(ROutput) + RMotor_offset)); // Timer/Counter0 is a general purpose 8-bit Timer/Counter module
}

void State_A()
{
  if (digitalRead(18))
  {
    Sum_Right ++;
  }
  else
  {
    Sum_Right --;
  }
}

void State_B()
{
  if (!digitalRead(2))
  {
    Sum_Left ++;
  }
  else
  {
    Sum_Left --;
  }
}











def best_action(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N

    :return: the action (either 0 or 1) with the highest q score.
    """
    left_val = q_value(s, 0, c, theta)
    right_val = q_value(s, 1, c, theta)
    if left_val > right_val:
        return 0
    elif left_val == right_val and random.random() < 0.5:
        return 0
    else:
        return 1


def best_q(s, c, theta):
    """
    :param s: list of state variables
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N

    :return: the highest q score possible given the possible actions.
    """
    return max(q_value(s, 0, c, theta), q_value(s, 1, c, theta))


def update_coef(new_s, new_a, s, a, c, theta, r):
    """
    :param new_s: list of new state variables
    :param new_a: best action available from new_s
    :param s: list of old state variables
    :param a: variable defining action taken to get from s to new_s
    :param c: list of N/2 vectors of dimension 5
    :param theta: vector of dimension N
    :param r: reward given

    :return: the updated theta vector
    """
    x = np.append(s, [a])
    theta_new = np.zeros(N)

    temp = alpha * (r + gamma * q_value(new_s, new_a, c, theta) - q_value(s, a, c, theta))
    for i in xrange(N):
        if i % 2 == 0:
            deriv = math.cos(math.pi * np.dot(c[i/2], x))
        else:
            deriv = math.sin(math.pi * np.dot(c[i/2], x))
        theta_new[i] = min(max(0, theta[i] + temp * deriv), 100)
    return theta_new


def main():
    env.monitor.start(outdir, force=True)
    print "start environment"

    # epsilon = initial_epsilon
    intialize_theta...
    c = get_c(5, basis+1)
    print "setup done"

    # Run 2000 episodes
    for episode_num in xrange(2000):
        print "begin episode:", episode_num
        # print episode_num, episode(epsilon, theta, env.spec.timestep_limit)
        last_state = normalize(env.reset())
        last_action = best_action(last_state, c, theta)
        total_reward = 0
        done = False
        while not done:
            state, reward, done, _ = env.step(last_action)
            state = normalize(state)
            action = best_action(state, c, theta)
            theta = update_coef(state, action, last_state, last_action, c, theta, reward)
            total_reward += reward
            last_state = state
            last_action = action
        print "end episode:", episode_num, total_reward

    env.monitor.close()

main()
